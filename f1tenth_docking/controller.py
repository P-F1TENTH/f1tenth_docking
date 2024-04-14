import do_mpc
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf_transformations import euler_from_quaternion


class DockingMPC(Node):

    # Move to a config file (YAML)
    T_STEP = 0.1
    ENV_SIZE = {"width": 1200, "height": 800}
    L = 30
    POS_GAIN = 22
    THETA_GAIN = 1
    DELTA_GAIN = 0.5

    # below will be received from topics
    SETPOINT = {
        "x_pos": 10,
        "y_pos": 700,
        "theta": 0,
        "delta": 0,
    }

    def __init__(self):
        super().__init__("controller")

        self.subscription = self.create_subscription(
            PoseStamped,
            "optitrack/rigid_body_0",
            self.pose_callback,
            10,
        )
        self._setup_model()
        self._setup_mpc()

    def pose_callback(self, msg):
        self.get_logger().info("Received Pose: \n{}".format(msg))
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            orientation = msg.pose.orientation
            orientation_list = [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
            _, _, yaw = euler_from_quaternion(orientation_list)

            self.get_logger().info(f"Converted Pose: x={x}, y={y}, yaw={yaw}\n")

        except Exception as e:
            self.get_logger().error(f"Error processing pose: {e}")

    def _setup_model(self):
        """Setup the model for the MPC controller"""
        self.model = do_mpc.model.Model("continuous")

        self.x_pos = self.model.set_variable(
            var_type="_x", var_name="x_pos", shape=(1, 1)
        )
        self.y_pos = self.model.set_variable(
            var_type="_x", var_name="y_pos", shape=(1, 1)
        )
        self.theta = self.model.set_variable(
            var_type="_x", var_name="theta", shape=(1, 1)
        )
        self.delta = self.model.set_variable(
            var_type="_x", var_name="delta", shape=(1, 1)
        )

        self.v = self.model.set_variable(var_type="_u", var_name="v")
        self.phi = self.model.set_variable(var_type="_u", var_name="phi")

        d_x_pos = self.v * np.cos(self.theta)
        d_y_pos = self.v * np.sin(self.theta)
        d_theta = self.v * np.tan(self.delta) / self.L
        d_delta = self.phi

        self.model.set_rhs("x_pos", d_x_pos)
        self.model.set_rhs("y_pos", d_y_pos)
        self.model.set_rhs("theta", d_theta)
        self.model.set_rhs("delta", d_delta)

        self.model.setup()

    def _setup_mpc(self):
        """Setup the MPC controller"""
        self.mpc = do_mpc.controller.MPC(self.model)
        self.mpc.set_param(
            n_horizon=20,
            t_step=DockingMPC.T_STEP,
            n_robust=1,
            store_full_solution=True,
        )

        self.mpc.bounds["lower", "_x", "x_pos"] = 0
        self.mpc.bounds["upper", "_x", "y_pos"] = self.ENV_SIZE["width"]

        self.mpc.bounds["lower", "_x", "y_pos"] = 0
        self.mpc.bounds["upper", "_x", "y_pos"] = self.ENV_SIZE["height"]

        self.mpc.bounds["lower", "_x", "delta"] = -np.pi / 4
        self.mpc.bounds["upper", "_x", "delta"] = np.pi / 4

        self.mpc.bounds["lower", "_u", "v"] = -120
        self.mpc.bounds["upper", "_u", "v"] = 120

        self.mpc.bounds["lower", "_u", "phi"] = -2
        self.mpc.bounds["upper", "_u", "phi"] = 2

        norm = lambda x, min, max: (x - min) / (max - min)
        lterm = (
            (
                norm((self.x_pos - self.SETPOINT["x_pos"]), 0, self.ENV_SIZE["width"])
                * self.POS_GAIN
            )
            ** 2
            + (
                norm((self.y_pos - self.SETPOINT["y_pos"]), 0, self.ENV_SIZE["height"])
                * self.POS_GAIN
            )
            ** 2
            + ((self.theta - self.SETPOINT["theta"]) * self.THETA_GAIN) ** 2
            + ((self.delta - self.SETPOINT["delta"]) * self.DELTA_GAIN) ** 2
        )
        mterm = lterm
        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        self.mpc.set_rterm(v=1e-2, phi=1e-2)

        self.mpc.setup()


def main(args=None):
    rclpy.init(args=args)
    controller = DockingMPC()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
