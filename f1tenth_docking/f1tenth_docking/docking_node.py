"""Node for controlling the docking procedure of the F1/10th car"""

import time

import rclpy
import do_mpc
import numpy as np
import casadi as ca
from rclpy.node import Node

from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped
from vesc_msgs.msg import VescStateStamped
from f1tenth_docking_interfaces.msg import DockingState, DockingControlOutput


class BicycleModel(do_mpc.model.Model):
    """Bicycle model for the F1/10th car"""

    def __init__(self, L: float) -> None:
        super().__init__("continuous")

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
        d_theta = self.u["v"] * ca.tan(self.x["delta"]) / L
        d_delta = self.u["phi"]

        self.set_rhs("x_pos", d_x_pos)
        self.set_rhs("y_pos", d_y_pos)
        self.set_rhs("theta", d_theta)
        self.set_rhs("delta", d_delta)

        self.setup()


class BicycleMPC(do_mpc.controller.MPC):
    """Model Predictive Controller with objective function and constraints for the F1/10th car"""

    def __init__(
        self,
        model: BicycleModel,
        t_step: float,
        n_robust: int,
        n_horizon: int,
        solver_max_iter: int,
        gains: dict,
        bounds: dict,
    ) -> None:

        super().__init__(model)

        self.set_param(
            n_horizon=n_horizon,
            t_step=t_step,
            n_robust=n_robust,
            store_full_solution=False,
            nlpsol_opts={
                "ipopt.max_iter": solver_max_iter,
                "ipopt.print_level": 0,
                "ipopt.sb": "yes",
                "print_time": 0,
            },
        )

        self.bounds["lower", "_x", "x_pos"] = bounds["x_pos"]["lower"]
        self.bounds["upper", "_x", "y_pos"] = bounds["x_pos"]["upper"]

        self.bounds["lower", "_x", "y_pos"] = bounds["y_pos"]["lower"]
        self.bounds["upper", "_x", "y_pos"] = bounds["y_pos"]["upper"]

        self.bounds["lower", "_x", "delta"] = bounds["delta"]["lower"]
        self.bounds["upper", "_x", "delta"] = bounds["delta"]["upper"]

        self.bounds["lower", "_u", "v"] = bounds["v"]["lower"]
        self.bounds["upper", "_u", "v"] = bounds["v"]["upper"]

        self.bounds["lower", "_u", "phi"] = bounds["phi"]["lower"]
        self.bounds["upper", "_u", "phi"] = bounds["phi"]["upper"]

        norm = lambda x, min, max: (x - min) / (max - min)
        lterm = (
            (
                norm(
                    (model.x["x_pos"] - model.tvp["set_x_pos"]),
                    bounds["x_pos"]["lower"],
                    bounds["x_pos"]["upper"],
                )
                * gains["pos"]
            )
            ** 2
            + (
                norm(
                    (model.x["y_pos"] - model.tvp["set_y_pos"]),
                    bounds["y_pos"]["lower"],
                    bounds["y_pos"]["upper"],
                )
                * gains["pos"]
            )
            ** 2
            + ((model.x["theta"] - model.tvp["set_theta"]) * gains["theta"]) ** 2
            + ((model.x["delta"] - model.tvp["set_delta"]) * gains["delta"]) ** 2
        )
        mterm = lterm
        self.set_objective(mterm=mterm, lterm=lterm)
        self.set_rterm(v=1e-2, phi=1e-2)

        self.set_tvp_fun(lambda _: self.get_tvp_template())
        self.setup()

    def choose_setpoint(
        self, x_pos: float, y_pos: float, theta: float, delta: float
    ) -> None:
        """Choose setpoint for the MPC controller"""
        tvp_template = self.get_tvp_template()
        tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_x_pos"] = x_pos
        tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_y_pos"] = y_pos
        tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_theta"] = theta
        tvp_template["_tvp", 0 : self.settings.n_horizon + 1, "set_delta"] = delta


class DockingNode(Node):
    """Node for controling the docking procedure of the F1/10th car"""

    NODE_NAME = "docking_node"

    def __init__(self):
        super().__init__(self.NODE_NAME)

        self.setpoint = None
        self.pose_stamped = None
        self.vesc_state_stamped = None
        self.is_initial_guess_set = False

        self.declare_parameters(
            namespace="",
            parameters=[
                ("L", rclpy.Parameter.Type.DOUBLE),
                ("t_step", rclpy.Parameter.Type.DOUBLE),
                ("n_robust", rclpy.Parameter.Type.INTEGER),
                ("n_horizon", rclpy.Parameter.Type.INTEGER),
                ("solver_max_iter", rclpy.Parameter.Type.INTEGER),
                ("gains.pos", rclpy.Parameter.Type.DOUBLE),
                ("gains.theta", rclpy.Parameter.Type.DOUBLE),
                ("gains.delta", rclpy.Parameter.Type.DOUBLE),
                ("bounds.x_pos.lower", rclpy.Parameter.Type.DOUBLE),
                ("bounds.x_pos.upper", rclpy.Parameter.Type.DOUBLE),
                ("bounds.y_pos.lower", rclpy.Parameter.Type.DOUBLE),
                ("bounds.y_pos.upper", rclpy.Parameter.Type.DOUBLE),
                ("bounds.delta.lower", rclpy.Parameter.Type.DOUBLE),
                ("bounds.delta.upper", rclpy.Parameter.Type.DOUBLE),
                ("bounds.v.lower", rclpy.Parameter.Type.DOUBLE),
                ("bounds.v.upper", rclpy.Parameter.Type.DOUBLE),
                ("bounds.phi.lower", rclpy.Parameter.Type.DOUBLE),
                ("bounds.phi.upper", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        gains = {
            "pos": self.get_parameter("gains.pos").value,
            "theta": self.get_parameter("gains.theta").value,
            "delta": self.get_parameter("gains.delta").value,
        }
        bounds = {
            "x_pos": {
                "lower": self.get_parameter("bounds.x_pos.lower").value,
                "upper": self.get_parameter("bounds.x_pos.upper").value,
            },
            "y_pos": {
                "lower": self.get_parameter("bounds.y_pos.lower").value,
                "upper": self.get_parameter("bounds.y_pos.upper").value,
            },
            "delta": {
                "lower": self.get_parameter("bounds.delta.lower").value,
                "upper": self.get_parameter("bounds.delta.upper").value,
            },
            "v": {
                "lower": self.get_parameter("bounds.v.lower").value,
                "upper": self.get_parameter("bounds.v.upper").value,
            },
            "phi": {
                "lower": self.get_parameter("bounds.phi.lower").value,
                "upper": self.get_parameter("bounds.phi.upper").value,
            },
        }

        self.mpc = BicycleMPC(
            model=BicycleModel(L=self.get_parameter("L").value),
            t_step=self.get_parameter("t_step").value,
            n_robust=self.get_parameter("n_robust").value,
            n_horizon=self.get_parameter("n_horizon").value,
            solver_max_iter=self.get_parameter("solver_max_iter").value,
            gains=gains,
            bounds=bounds,
        )

        self.mpc_timer = self.create_timer(
            self.mpc.settings.t_step, self.mpc_timer_callback
        )

        self.control_output_publisher = self.create_publisher(
            DockingControlOutput,
            f"{self.NODE_NAME}/control_output",
            1,
        )

        self.setpoint_subscription = self.create_subscription(
            DockingState,
            f"{self.NODE_NAME}/setpoint",
            self.setpoint_callback,
            1,
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            "optitrack/rigid_body_0",
            self.pose_callback,
            1,
        )

        self.vesc_subscription = self.create_subscription(
            VescStateStamped,
            "vesc/core",
            self.vesc_callback,
            1,
        )

    def mpc_timer_callback(self) -> None:
        """Main control loop"""
        if None not in (self.pose_stamped, self.vesc_state_stamped, self.setpoint):
            start_time = time.time()

            orientation_list = [
                self.pose_stamped.pose.orientation.x,
                self.pose_stamped.pose.orientation.y,
                self.pose_stamped.pose.orientation.z,
                self.pose_stamped.pose.orientation.w,
            ]
            x_pos = self.pose_stamped.pose.position.x
            y_pos = self.pose_stamped.pose.position.y
            theta = euler_from_quaternion(orientation_list)[2]
            delta = self.vesc_state_stamped.state.servo_pose

            x0 = np.array([[x_pos], [y_pos], [theta], [delta]])

            if self.is_initial_guess_set:
                self.mpc.choose_setpoint(
                    self.setpoint.x_pos,
                    self.setpoint.y_pos,
                    self.setpoint.theta,
                    self.setpoint.delta,
                )
                u0 = self.mpc.make_step(x0).reshape(-1)
                self.control_output_publisher.publish(
                    DockingControlOutput(v=u0[0], phi=u0[1])
                )
                self.get_logger().info("Control output published")

            else:
                self.mpc.x0 = x0
                self.mpc.set_initial_guess()
                self.is_initial_guess_set = True
                self.get_logger().info("Initial guess set")

            end_time = time.time()
            execution_time = end_time - start_time
            if execution_time > self.mpc.settings.t_step:
                self.get_logger().error(
                    f"Execution time exceeded time step: {execution_time} > {self.mpc.settings.t_step}"
                )

    def pose_callback(self, msg: PoseStamped) -> None:
        """Callback for the stamped pose message"""
        self.pose_stamped = msg
        self.get_logger().info("Updated pose")

    def vesc_callback(self, msg: VescStateStamped) -> None:
        """Callback for the stamped vesc state message"""
        self.vesc_state_stamped = msg
        self.get_logger().info("Updataed vesc state")

    def setpoint_callback(self, msg: DockingState) -> None:
        """Callback for changing the setpoint"""
        self.setpoint = msg
        self.get_logger().info("Updated setpoint")


def main(args=None):
    rclpy.init(args=args)
    docking_node = DockingNode()
    rclpy.spin(docking_node)
    docking_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
