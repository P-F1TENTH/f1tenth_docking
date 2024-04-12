import warnings

# Suppress warnings from do-mpc
warnings.filterwarnings("ignore", message="The .* feature is not available.*")

import do_mpc
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

T_STEP = 0.1
SETPOINT = np.array([10, 700, 0, 0])
ENV_SIZE = (1200, 800)
POS_GAIN = 22
THETA_GAIN = 1
DELTA_GAIN = 0.5
L = 30


class OptitrackSubscriber(Node):
    def __init__(self):
        super().__init__("controller")
        self.subscription = self.create_subscription(
            PoseStamped,
            "optitrack/rigid_body_0",
            self.pose_callback,
            10,
        )

        model = do_mpc.model.Model("continuous")

        x_pos = model.set_variable(var_type="_x", var_name="x_pos", shape=(1, 1))
        y_pos = model.set_variable(var_type="_x", var_name="y_pos", shape=(1, 1))
        theta = model.set_variable(var_type="_x", var_name="theta", shape=(1, 1))
        delta = model.set_variable(var_type="_x", var_name="delta", shape=(1, 1))

        v = model.set_variable(var_type="_u", var_name="v")
        phi = model.set_variable(var_type="_u", var_name="phi")

        d_x_pos = v * np.cos(theta)
        d_y_pos = v * np.sin(theta)
        d_theta = v * np.tan(delta) / L
        d_delta = phi

        model.set_rhs("x_pos", d_x_pos)
        model.set_rhs("y_pos", d_y_pos)
        model.set_rhs("theta", d_theta)
        model.set_rhs("delta", d_delta)

        model.setup()

        mpc = do_mpc.controller.MPC(model)

        mpc.set_param(
            n_horizon=20,
            t_step=T_STEP,
            n_robust=1,
            store_full_solution=True,
        )

        set_x_pos = SETPOINT[0]
        set_y_pos = SETPOINT[1]
        set_theta = SETPOINT[2]
        set_delta = SETPOINT[3]

        mpc.bounds["lower", "_x", "x_pos"] = 0
        mpc.bounds["upper", "_x", "y_pos"] = ENV_SIZE[0]

        mpc.bounds["lower", "_x", "y_pos"] = 0
        mpc.bounds["upper", "_x", "y_pos"] = ENV_SIZE[1]

        mpc.bounds["lower", "_x", "delta"] = -np.pi / 4
        mpc.bounds["upper", "_x", "delta"] = np.pi / 4

        mpc.bounds["lower", "_u", "v"] = -120
        mpc.bounds["upper", "_u", "v"] = 120

        # TODO: RANDOM VALUES FOR NOW
        mpc.bounds["lower", "_u", "phi"] = -np.pi / 4
        mpc.bounds["upper", "_u", "phi"] = np.pi / 4

        lterm = (
            (self.norm((x_pos - set_x_pos), 0, ENV_SIZE[0]) * POS_GAIN) ** 2
            + (self.norm((y_pos - set_y_pos), 0, ENV_SIZE[1]) * POS_GAIN) ** 2
            + ((theta - set_theta) * THETA_GAIN) ** 2
            + ((delta - set_delta) * DELTA_GAIN) ** 2
        )

        mterm = lterm

        mpc.set_objective(mterm=mterm, lterm=lterm)
        mpc.set_rterm(v=1e-2, phi=1e-2)

        mpc.settings.supress_ipopt_output()  # Suppress any output from the optimizer
        mpc.setup()

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
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

            self.get_logger().info(f"Converted Pose: x={x}, y={y}, yaw={yaw}\n")

        except Exception as e:
            self.get_logger().error(f"Error processing pose: {e}")

    def norm(self, x, min, max):
        return (x - min) / (max - min)


def main(args=None):
    rclpy.init(args=args)
    controller = OptitrackSubscriber()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
