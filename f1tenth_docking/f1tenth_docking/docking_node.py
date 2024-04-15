import do_mpc
import casadi as ca
import numpy as np
import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion

from f1tenth_docking_interfaces.msg import StateVector
from geometry_msgs.msg import PoseStamped


class BicycleModel(do_mpc.model.Model):
    def __init__(self, L: float) -> None:
        super().__init__("continuous")

        self.L = L

        self.set_variable(var_type="_x", var_name="x_pos", shape=(1, 1))
        self.set_variable(var_type="_x", var_name="y_pos", shape=(1, 1))
        self.set_variable(var_type="_x", var_name="theta", shape=(1, 1))
        self.set_variable(var_type="_x", var_name="delta", shape=(1, 1))

        self.set_variable(var_type="_u", var_name="v")
        self.set_variable(var_type="_u", var_name="phi")

        self.set_variable(var_type="_tvp", var_name="set_x_pos")
        self.set_variable(var_type="_tvp", var_name="set_y_pos")
        self.set_variable(var_type="_tvp", var_name="set_theta")
        self.set_variable(var_type="_tvp", var_name="set_delta")

        d_x_pos = self.u["v"] * ca.cos(self.x["theta"])
        d_y_pos = self.u["v"] * ca.sin(self.x["theta"])
        d_theta = self.u["v"] * ca.tan(self.x["delta"]) / self.L
        d_delta = self.u["phi"]

        self.set_rhs("x_pos", d_x_pos)
        self.set_rhs("y_pos", d_y_pos)
        self.set_rhs("theta", d_theta)
        self.set_rhs("delta", d_delta)

        self.setup()


class BicycleMPC(do_mpc.controller.MPC):
    def __init__(
        self, gains, model, n_horizon, t_step, n_robust, env_size
    ) -> None:  # TODO add type hints

        self.gains = gains
        self.model = model
        self.n_horizon = n_horizon
        self.t_step = t_step
        self.n_robust = n_robust
        self.env_size = env_size

        self.setpoint = np.zeros((4, 1))

        super().__init__(self.model)

        self.set_param(
            n_horizon=self.n_horizon,
            t_step=self.t_step,
            n_robust=self.n_robust,
            store_full_solution=True,
            nlpsol_opts={"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": 0},
        )

        self.bounds["lower", "_x", "x_pos"] = 0
        self.bounds["upper", "_x", "y_pos"] = self.env_size["width"]

        self.bounds["lower", "_x", "y_pos"] = 0
        self.bounds["upper", "_x", "y_pos"] = self.env_size["height"]

        self.bounds["lower", "_x", "delta"] = -ca.pi / 4
        self.bounds["upper", "_x", "delta"] = ca.pi / 4

        self.bounds["lower", "_u", "v"] = -120
        self.bounds["upper", "_u", "v"] = 120

        self.bounds["lower", "_u", "phi"] = -2
        self.bounds["upper", "_u", "phi"] = 2

        norm = lambda x, min, max: (x - min) / (max - min)
        lterm = (
            (
                norm(
                    (self.model.x["x_pos"] - self.model.tvp["set_x_pos"]),
                    0,
                    self.env_size["width"],
                )
                * self.gains["pos"]
            )
            ** 2
            + (
                norm(
                    (self.model.x["y_pos"] - self.model.tvp["set_y_pos"]),
                    0,
                    self.env_size["height"],
                )
                * self.gains["pos"]
            )
            ** 2
            + (
                (self.model.x["theta"] - self.model.tvp["set_theta"])
                * self.gains["theta"]
            )
            ** 2
            + (
                (self.model.x["delta"] - self.model.tvp["set_delta"])
                * self.gains["delta"]
            )
            ** 2
        )
        mterm = lterm
        self.set_objective(mterm=mterm, lterm=lterm)
        self.set_rterm(v=1e-2, phi=1e-2)

        self.set_tvp_fun(lambda _: self.get_tvp_template())
        self.setup()

    def set_setpoint_horizon(self) -> None:
        """Set the setpoint for the entire horizon"""
        tvp_template = self.get_tvp_template()
        tvp_template["_tvp", 0 : self.n_horizon + 1, "set_x_pos"] = self.setpoint[0]
        tvp_template["_tvp", 0 : self.n_horizon + 1, "set_y_pos"] = self.setpoint[1]
        tvp_template["_tvp", 0 : self.n_horizon + 1, "set_theta"] = self.setpoint[2]
        tvp_template["_tvp", 0 : self.n_horizon + 1, "set_delta"] = self.setpoint[3]


class DockingNode(Node):

    # Move to a config file (YAML)
    T_STEP = 0.1
    ENV_SIZE = {"width": 1200, "height": 800}
    L = 30

    GAINS = {
        "pos": 22,
        "theta": 1,
        "delta": 0.5,
    }

    # below will be received from topics
    SETPOINT = {
        "x_pos": 10,
        "y_pos": 700,
        "theta": 0,
        "delta": 0,
    }
    DELTA = 0

    # below stay untouched
    NODE_NAME = "docking_node"

    def __init__(self):
        super().__init__(self.NODE_NAME)

        self.x = StateVector()
        self.bicycle_model = BicycleModel(self.L)
        self.mpc = BicycleMPC(
            gains=self.GAINS,
            model=self.bicycle_model,
            n_horizon=20,
            t_step=self.T_STEP,
            n_robust=1,
            env_size=self.ENV_SIZE,
        )

        # TODO: Add vecs callback and timer for MPC
        # Remember to add set_setpoint_horizon() to the timer callback

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            "optitrack/rigid_body_0",
            self._set_pose_callback,
            1,
        )

        self.setpoint_subscription = self.create_subscription(
            StateVector,
            f"{self.NODE_NAME}/setpoint",
            self._set_setpoint_callback,
            1,
        )

        self.initial_guess_subscription = self.create_subscription(
            StateVector,
            f"{self.NODE_NAME}/initial_guess",
            self._set_initial_guess_callback,
            1,
        )

    def _set_pose_callback(self, msg: PoseStamped) -> None:
        """Callback function to receive the currnet pose of the car"""
        self.get_logger().info("New pose received")

        orientation_list = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

        self.x.x_pos = msg.pose.position.x
        self.x.y_pos = msg.pose.position.y
        self.x.theta = euler_from_quaternion(orientation_list)[2]

    def _set_setpoint_callback(self, msg: StateVector) -> None:
        """Callback function to receive the desired setpoint of the car"""
        self.get_logger().info("New setpoint received")
        self.mpc.setpoint = np.array(
            [[msg.x_pos], [msg.y_pos], [msg.theta], [msg.delta]]
        )

    def _set_initial_guess_callback(self, msg: StateVector) -> None:
        """Callback function to receive the initial guess of the car pose"""
        self.get_logger().info("New initial guess received")
        self.mpc.x0 = np.array([[msg.x_pos], [msg.y_pos], [msg.theta], [msg.delta]])
        self.mpc.set_initial_guess()


def main(args=None):
    rclpy.init(args=args)
    controller = DockingNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
