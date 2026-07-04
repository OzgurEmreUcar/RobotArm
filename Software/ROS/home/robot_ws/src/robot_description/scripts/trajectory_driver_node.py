#!/usr/bin/env python3
"""
ROS2 node that drives a 6-DOF robotic arm over a serial link to an
Arduino-based low-level motor controller.

Exposes:
  - A FollowJointTrajectory action server compatible with MoveIt.
  - /robot_arm/freeze and /robot_arm/engage Trigger services.
  - A /joint_states publisher reflecting the last known joint positions.
  - A subscription to /display_planned_path that logs the duration of
    plans previewed in RViz (informational only).
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor 

from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory

from constants import JOINT_NAMES, SCALES, SERIAL_BAUD, SERIAL_PORT
from serial_bridge import SerialBridge


class TrajectoryDriver(Node):
    """Bridges MoveIt trajectory goals to Arduino motor-control commands."""

    def __init__(self):
        super().__init__('trajectory_driver')

        self.cb_group = ReentrantCallbackGroup()

        self.joint_names = JOINT_NAMES
        self.current_positions = [0.0] * 6
        self._accum_steps = [0.0] * 6

        # Synchronization primitives shared with the serial bridge.
        self._cancel_flag = threading.Event()
        self._trajectory_done_event = threading.Event()
        self._target_segments = 0
        self._segments_finished = 0

        # Serial bridge owns all low-level I/O with the Arduino.
        self.bridge = SerialBridge(self.get_logger(), self._cancel_flag)
        self.bridge.on_segment_done = self._on_segment_done

        # ROS interfaces.
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.05, self.publish_current_states, callback_group=self.cb_group)

        self.create_service(Trigger, '/robot_arm/freeze', self._srv_freeze, callback_group=self.cb_group)
        self.create_service(Trigger, '/robot_arm/engage', self._srv_engage, callback_group=self.cb_group)

        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self._on_display_planned_path,
            10,
            callback_group=self.cb_group,
        )

        # Start the serial bridge's background threads and connection attempt.
        self.bridge.start()
        self.bridge.connect_async(SERIAL_PORT, SERIAL_BAUD)

        self.get_logger().info('TrajectoryDriver v9.4 ready. Lightweight timing comparison active.')

    # ------------------------------------------------------------------ #
    # Serial bridge callbacks (called from the reader thread)
    # ------------------------------------------------------------------ #
    def _on_segment_done(self, pos_rad):
        """Invoked when the Arduino reports a completed motion segment."""
        if pos_rad is not None:
            self.current_positions = pos_rad

        self._segments_finished += 1
        if 0 < self._target_segments <= self._segments_finished:
            self._trajectory_done_event.set()

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #
    def _on_display_planned_path(self, msg: DisplayTrajectory):
        """Log the duration of a MoveIt-previewed plan (informational only)."""
        if not msg.trajectory:
            return

        for traj in msg.trajectory:
            points = traj.joint_trajectory.points
            if len(points) < 2:
                continue

            t_first = points[0].time_from_start.sec + points[0].time_from_start.nanosec / 1e9
            t_last = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec / 1e9
            duration_s = t_last - t_first

            self.get_logger().info(
                f'[PLAN PREVIEW] The plan generated in RViz will take {duration_s:.3f} seconds '
                f'({len(points)} points).'
            )

    # ------------------------------------------------------------------ #
    # Services
    # ------------------------------------------------------------------ #
    def _srv_freeze(self, _, response):
        self._cancel_flag.set()
        self._trajectory_done_event.set()
        self.bridge.clear_send_queue()

        response.success = self.bridge.send_control('FREEZE')
        response.message = 'FREEZE OK' if response.success else 'FREEZE FAILED'
        return response

    def _srv_engage(self, _, response):
        self._cancel_flag.clear()
        response.success = self.bridge.send_control('ENGAGE')
        response.message = 'ENGAGE OK' if response.success else 'ENGAGE FAILED'
        return response

    # ------------------------------------------------------------------ #
    # Periodic publishing
    # ------------------------------------------------------------------ #
    def publish_current_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(self.current_positions)
        self.joint_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Action server callbacks
    # ------------------------------------------------------------------ #
    def goal_callback(self, _):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self._cancel_flag.set()
        self._trajectory_done_event.set()
        self.bridge.send_control('ABORT')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.bridge.clear_queues()

        self._segments_finished = 0
        self._trajectory_done_event.clear()
        self.get_logger().info('Processing new trajectory...')
        result = FollowJointTrajectory.Result()
        self._cancel_flag.clear()

        if not self.bridge.send_control('ENGAGE'):
            self.get_logger().error('Could not engage the drivers (ENGAGE error)')
            goal_handle.abort()
            return result
        time.sleep(0.05)

        points = goal_handle.request.trajectory.points
        recv_joints = goal_handle.request.trajectory.joint_names

        try:
            idx = [recv_joints.index(name) for name in self.joint_names]
        except ValueError as e:
            self.get_logger().error(f'URDF/trajectory mismatch: {e}')
            goal_handle.abort()
            return result

        if len(points) < 2:
            goal_handle.abort()
            return result

        self._target_segments = len(points) - 1
        self._trajectory_done_event.clear()

        # ---- [TIMING - INPUT] Total planned duration from MoveIt ----
        t_first = points[0].time_from_start.sec + points[0].time_from_start.nanosec / 1e9
        t_last = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec / 1e9
        planned_total_s = t_last - t_first
        self.get_logger().info(f'[TIMING] MoveIt plan should take: {planned_total_s:.3f} seconds.')

        # Synchronize the step accumulator with the current robot position.
        for j in range(6):
            self._accum_steps[j] = math.degrees(self.current_positions[j]) * SCALES[j]

        # ---- [TIMING - STOPWATCH] Physical motion start time ----
        exec_start_time = time.monotonic()

        last_calculated_pos = list(self.current_positions)
        for i in range(1, len(points)):
            pt_prev, pt_curr = points[i - 1], points[i]

            t_prev = pt_prev.time_from_start.sec + pt_prev.time_from_start.nanosec / 1e9
            t_curr = pt_curr.time_from_start.sec + pt_curr.time_from_start.nanosec / 1e9
            dur_ms = max(int(round((t_curr - t_prev) * 1000.0)), 2)

            pos_rad = [pt_curr.positions[j] for j in idx]
            deltas = []

            for j in range(6):
                target_steps = math.degrees(pos_rad[j]) * SCALES[j]
                delta_int = int(round(target_steps - self._accum_steps[j]))
                self._accum_steps[j] += delta_int
                deltas.append(delta_int)

            self.bridge.enqueue_segment(deltas, dur_ms, pos_rad)
            last_calculated_pos = pos_rad

        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = self.joint_names
        feedback_msg.actual.positions = list(self.current_positions)
        goal_handle.publish_feedback(feedback_msg)

        # Wait for the physical motion to complete.
        finished_physically = self._trajectory_done_event.wait(
            timeout=max(planned_total_s * 1.20, 10.0)
        )

        # ---- [TIMING - OUTPUT] Compare planned vs. actual duration ----
        exec_elapsed_s = time.monotonic() - exec_start_time

        if self._cancel_flag.is_set() or not finished_physically:
            self.get_logger().error(
                f'[TIMING] Motion aborted or timed out! Elapsed time: {exec_elapsed_s:.3f} seconds.'
            )
            self.bridge.send_cmd("ABORT\n")
            self.bridge.clear_queues()

            for j in range(6):
                self._accum_steps[j] = math.degrees(self.current_positions[j]) * SCALES[j]
        else:
            self.current_positions = list(last_calculated_pos)
            diff_s = exec_elapsed_s - planned_total_s
            percent_diff = (abs(diff_s) / planned_total_s * 100.0) if planned_total_s > 0 else 0.0
            self.get_logger().info(
                f'\n=== [TIMING REPORT] ===\n'
                f'-> Planned duration : {planned_total_s:.3f} s\n'
                f'-> Actual duration  : {exec_elapsed_s:.3f} s\n'
                f'-> Deviation        : {diff_s:+.3f} s (%{percent_diff:.1f})\n'
                f'======================'
            )

        if self._cancel_flag.is_set():
            goal_handle.canceled()
        elif not finished_physically:
            self.get_logger().error('Trajectory physically timed out!')
            goal_handle.abort()
        else:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info('Trajectory execution completed successfully.')

        return result

    # ------------------------------------------------------------------ #
    # Shutdown
    # ------------------------------------------------------------------ #
    def shutdown(self):
        """Freeze the arm and release the serial port. Call before node teardown."""
        self.bridge.send_control('FREEZE')
        self.bridge.close()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()