"""Node for controlling the docking procedure of the F1/10th car"""

import time
import threading

import rclpy
import do_mpc
import numpy as np
import casadi as ca
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from tf_transformations import euler_from_quaternion

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from vesc_msgs.msg import VescStateStamped
from control_interfaces.msg import Control

from f1tenth_docking_interfaces.action import Docking
from f1tenth_docking_interfaces.msg import DockingState


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

        d_x_pos = self.u["v"] * ca.sin(self.x["theta"])
        d_y_pos = self.u["v"] * ca.cos(self.x["theta"])
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
        solver_max_cpu_time: float,
        n_horizon: int,
        gains: dict,
        bounds: dict,
    ) -> None:

        super().__init__(model)

        self.set_param(
            n_horizon=n_horizon,
            t_step=t_step,
            store_full_solution=False,
            nlpsol_opts={
                "ipopt.max_cpu_time": solver_max_cpu_time,
                "ipopt.print_level": 0,
                "ipopt.sb": "yes",
                "print_time": 0,
            },
        )

        self.bounds["lower", "_x", "x_pos"] = bounds["x_pos"]["lower"]
        self.bounds["upper", "_x", "y_pos"] = bounds["x_pos"]["upper"]

        self.bounds["lower", "_x", "y_pos"] = bounds["y_pos"]["lower"]
        self.bounds["upper", "_x", "y_pos"] = bounds["y_pos"]["upper"]

        self.bounds["lower", "_x", "delta"] = -bounds["delta"]["radians"]
        self.bounds["upper", "_x", "delta"] = bounds["delta"]["radians"]

        self.bounds["lower", "_u", "v"] = bounds["v"]["lower"] * t_step
        self.bounds["upper", "_u", "v"] = bounds["v"]["upper"] * t_step

        self.bounds["lower", "_u", "phi"] = bounds["phi"]["lower"] * t_step
        self.bounds["upper", "_u", "phi"] = bounds["phi"]["upper"] * t_step

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


class DockingActionServer(Node):
    """Node for controling the docking procedure of the F1/10th car"""

    NAME = "docking_action_server"

    def __init__(self):
        super().__init__(self.NAME)

        self.pose_stamped = None
        self.vesc_state_stamped = None

        self.goal_handle = None
        self.goal_lock = threading.Lock()

        self.declare_parameters(
            namespace="",
            parameters=[
                ("L", rclpy.Parameter.Type.DOUBLE),
                ("t_step", rclpy.Parameter.Type.DOUBLE),
                ("solver_max_cpu_time", rclpy.Parameter.Type.DOUBLE),
                ("n_horizon", rclpy.Parameter.Type.INTEGER),
                ("gains.pos", rclpy.Parameter.Type.DOUBLE),
                ("gains.theta", rclpy.Parameter.Type.DOUBLE),
                ("gains.delta", rclpy.Parameter.Type.DOUBLE),
                ("bounds.x_pos.lower", rclpy.Parameter.Type.DOUBLE),
                ("bounds.x_pos.upper", rclpy.Parameter.Type.DOUBLE),
                ("bounds.y_pos.lower", rclpy.Parameter.Type.DOUBLE),
                ("bounds.y_pos.upper", rclpy.Parameter.Type.DOUBLE),
                ("bounds.delta.normalized", rclpy.Parameter.Type.DOUBLE),
                ("bounds.delta.radians", rclpy.Parameter.Type.DOUBLE),
                ("bounds.v.lower", rclpy.Parameter.Type.DOUBLE),
                ("bounds.v.upper", rclpy.Parameter.Type.DOUBLE),
                ("bounds.phi.lower", rclpy.Parameter.Type.DOUBLE),
                ("bounds.phi.upper", rclpy.Parameter.Type.DOUBLE),
                ("accepted_absolute_errors.pos", rclpy.Parameter.Type.DOUBLE),
                ("accepted_absolute_errors.theta", rclpy.Parameter.Type.DOUBLE),
                ("accepted_absolute_errors.delta", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        self.gains = {
            "pos": self.get_parameter("gains.pos").value,
            "theta": self.get_parameter("gains.theta").value,
            "delta": self.get_parameter("gains.delta").value,
        }

        self.bounds = {
            "x_pos": {
                "lower": self.get_parameter("bounds.x_pos.lower").value,
                "upper": self.get_parameter("bounds.x_pos.upper").value,
            },
            "y_pos": {
                "lower": self.get_parameter("bounds.y_pos.lower").value,
                "upper": self.get_parameter("bounds.y_pos.upper").value,
            },
            "delta": {
                "normalized": self.get_parameter("bounds.delta.normalized").value,
                "radians": self.get_parameter("bounds.delta.radians").value,
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

        self.accepted_absolute_errors = {
            "pos": self.get_parameter("accepted_absolute_errors.pos").value,
            "theta": self.get_parameter("accepted_absolute_errors.theta").value,
            "delta": self.get_parameter("accepted_absolute_errors.delta").value,
        }

        self.mpc = BicycleMPC(
            model=BicycleModel(L=self.get_parameter("L").value),
            t_step=self.get_parameter("t_step").value,
            solver_max_cpu_time=self.get_parameter("solver_max_cpu_time").value,
            n_horizon=self.get_parameter("n_horizon").value,
            gains=self.gains,
            bounds=self.bounds,
        )

        self.control_publisher = self.create_publisher(
            Control,
            "mpc/control",
            1,
        )

        self.predicted_states_publisher = self.create_publisher(
            PoseArray,
            "mpc/predicted_states",
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
            qos_profile_sensor_data,
        )

        self.control_loop_rate = self.create_rate(1 / self.mpc.settings.t_step)
        self.action_server = ActionServer(
            self,
            Docking,
            self.NAME,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
        )

    def destroy(self):
        """Destroy the node and the action server"""
        self._action_server.destroy()
        super().destroy_node()

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Docking.Result:
        """Callback that executes the docking procedure"""
        self.get_logger().info("Executing goal...")

        if None in (self.pose_stamped, self.vesc_state_stamped):
            self.get_logger().error("Either pose or vesc state was not received")
            goal_handle.abort()
            return Docking.Result()

        setpoint = np.array(
            [
                goal_handle.request.setpoint.x_pos,
                goal_handle.request.setpoint.y_pos,
                goal_handle.request.setpoint.theta,
                goal_handle.request.setpoint.delta,
            ]
        )
        docking_state = self.get_docking_state()
        error = setpoint - docking_state

        self.mpc.x0 = docking_state
        self.mpc.set_initial_guess()
        self.get_logger().info("Initial guess set")

        feedback_msg = Docking.Feedback()
        result_msg = Docking.Result()

        while (
            np.abs(error[0]) > self.accepted_absolute_errors["pos"]
            or np.abs(error[1]) > self.accepted_absolute_errors["pos"]
            or np.abs(error[2]) > self.accepted_absolute_errors["theta"]
            or np.abs(error[3]) > self.accepted_absolute_errors["delta"]
        ):
            start = time.time()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn("Goal canceled")
                return Docking.Result()

            self.mpc.choose_setpoint(*setpoint)
            docking_state = self.get_docking_state()

            u0 = self.mpc.make_step(np.vstack(docking_state)).flatten()

            error = setpoint - docking_state
            feedback_msg.error = DockingState(
                x_pos=error[0], y_pos=error[1], theta=error[2], delta=error[3]
            )
            control = Control(
                set_speed=u0[0],
                # obtain the first predicted delta state after applaying the control
                steering_angle=float(self.mpc.opt_x_num["_x", 1, 0, -1, "delta"])
                * (
                    self.bounds["delta"]["normalized"] / self.bounds["delta"]["radians"]
                ),
                control_mode=Control.SPEED_MODE,
            )
            predicted_states = PoseArray(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id="predicted_states",
                ),
                poses=[
                    Pose(
                        position=Point(
                            x=float(self.mpc.opt_x_num["_x", i, 0, -1, "x_pos"]),
                            y=float(self.mpc.opt_x_num["_x", i, 0, -1, "y_pos"]),
                        ),
                        orientation=Quaternion(
                            x=0.0,
                            y=0.0,
                            z=np.sin(
                                float(self.mpc.opt_x_num["_x", i, 0, -1, "theta"])
                            ),
                            w=np.cos(
                                float(self.mpc.opt_x_num["_x", i, 0, -1, "theta"])
                            ),
                        ),
                    )
                    for i in range(self.mpc.settings.n_horizon + 1)
                ],
            )

            goal_handle.publish_feedback(feedback_msg)
            self.control_publisher.publish(control)
            self.predicted_states_publisher.publish(predicted_states)

            self.get_logger().info("Control output published")

            end = time.time()
            execution_time = end - start

            if execution_time > self.mpc.settings.t_step:
                self.get_logger().warn(
                    f"Execution time exceeded time step: {execution_time} > {self.mpc.settings.t_step}"
                )

            self.control_loop_rate.sleep()

        error = setpoint - self.get_docking_state()
        result_msg.steady_state_error = DockingState(
            x_pos=error[0], y_pos=error[1], theta=error[2], delta=error[3]
        )

        goal_handle.succeed()
        return result_msg

    def goal_callback(self, _) -> GoalResponse:
        """Callback that rejects new goals if the current goal is active"""
        self.get_logger().info("Received new goal request")
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.get_logger().warn("Goal rejected")
            return GoalResponse.REJECT
        self.get_logger().info("Goal accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _) -> CancelResponse:
        """Callback that cancels the current goal"""
        self.get_logger().warn("Received cancel request")
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        """Callback that handles the accepted goal"""
        self.goal_handle = goal_handle
        self.goal_handle.execute()

    def pose_callback(self, msg: PoseStamped) -> None:
        """Callback for the stamped pose message"""
        self.pose_stamped = msg
        self.get_logger().info("Updated pose")

    def vesc_callback(self, msg: VescStateStamped) -> None:
        """Callback for the stamped vesc state message"""
        self.vesc_state_stamped = msg
        self.get_logger().info("Updated vesc state")

    def get_docking_state(self) -> np.ndarray:
        """Get the current state of the docking procedure"""
        orientation_list = [
            self.pose_stamped.pose.orientation.x,
            self.pose_stamped.pose.orientation.y,
            self.pose_stamped.pose.orientation.z,
            self.pose_stamped.pose.orientation.w,
        ]
        x_pos = self.pose_stamped.pose.position.x
        y_pos = self.pose_stamped.pose.position.y
        theta = euler_from_quaternion(orientation_list)[2]
        delta = self.vesc_state_stamped.state.servo_pose * (
            self.bounds["delta"]["radians"] / self.bounds["delta"]["normalized"]
        )
        return np.array([x_pos, y_pos, theta, delta])


def main(args=None):
    """Main function for the docking action server"""
    rclpy.init(args=args)
    docking_action_server = DockingActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(docking_action_server, executor=executor)
    docking_action_server.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
