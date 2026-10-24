import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty
from turtlesim_interfaces.action import DrawSquare, GoToPose


class SquareActionServerNode(Node):
    def __init__(self):
        super().__init__('square_action_server_node')

        self.cb_group = ReentrantCallbackGroup()

        # Client to our custom Reset service
        self.reset_cli = self.create_client(
            Empty,
            'reset_turtle',
            callback_group=self.cb_group
        )

        # Client to our custom GoToPose action server
        self.goto_cli = ActionClient(
            self,
            GoToPose,
            'go_to_pose',
            callback_group=self.cb_group
        )

        # Action Server for DrawSquare
        self._action_server = ActionServer(
            self,
            DrawSquare,
            'draw_square',
            execute_callback=self.execute_square_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('Square Action Server Node initialized (action: /draw_square).')

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received DrawSquare goal: side_length={goal_request.side_length}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for DrawSquare.')
        return CancelResponse.ACCEPT

    def execute_square_callback(self, goal_handle):
        side_len = goal_handle.request.side_length
        # Fallback to sensible default if side_length is unset or zero
        if side_len <= 0.0:
            side_len = 3.0
            self.get_logger().info(f'Using default side length: {side_len}')

        # Prevent square from exceeding turtlesim boundary (0 to 11)
        if side_len > 4.5:
            self.get_logger().warn(f'Side length {side_len} is too large for canvas. Clamping to 4.5.')
            side_len = 4.5

        feedback_msg = DrawSquare.Feedback()
        result = DrawSquare.Result()

        # Step 6.2: Call Reset service first so canvas is clean and turtle is at center (5.54, 5.54)
        self.get_logger().info('Calling reset service before drawing square...')
        if not self.reset_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Reset service /reset_turtle not available!')
            goal_handle.abort()
            result.success = False
            return result

        reset_future = self.reset_cli.call_async(Empty.Request())
        while not reset_future.done() and rclpy.ok():
            time.sleep(0.05)

        # Ensure GoToPose action server is online
        if not self.goto_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('GoToPose action server /go_to_pose not available!')
            goal_handle.abort()
            result.success = False
            return result

        # The reset pose in Turtlesim
        x0 = 5.544445
        y0 = 5.544445

        # 4 corners defining the 4 sides of the square
        # Corner 1: bottom-right
        # Corner 2: top-right
        # Corner 3: top-left
        # Corner 4: back to bottom-left (start)
        corners = [
            (x0 + side_len, y0,            math.pi / 2.0),
            (x0 + side_len, y0 + side_len, math.pi),
            (x0,            y0 + side_len, -math.pi / 2.0),
            (x0,            y0,            0.0)
        ]

        # Step 6.3 & 6.4: Execute 4 separate goToPose goals sequentially
        for side_idx, (target_x, target_y, target_yaw) in enumerate(corners, start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                self.get_logger().info('Square action canceled.')
                return result

            # Publish feedback: which side we are currently drawing
            feedback_msg.current_side = side_idx
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Drawing Side {side_idx}/4: Moving to ({target_x:.2f}, {target_y:.2f}, yaw={target_yaw:.2f})...')

            # Send goal to GoToPose server
            goto_goal = GoToPose.Goal()
            goto_goal.target_pose.x = float(target_x)
            goto_goal.target_pose.y = float(target_y)
            goto_goal.target_pose.yaw = float(target_yaw)

            send_future = self.goto_cli.send_goal_async(goto_goal)
            while not send_future.done() and rclpy.ok():
                time.sleep(0.05)

            client_goal_handle = send_future.result()
            if not client_goal_handle.accepted:
                self.get_logger().error(f'GoToPose goal for side {side_idx} was rejected!')
                goal_handle.abort()
                result.success = False
                return result

            # Wait for this corner to complete before moving to the next
            result_future = client_goal_handle.get_result_async()
            while not result_future.done() and rclpy.ok():
                time.sleep(0.05)

            action_res = result_future.result().result
            if not action_res.success:
                self.get_logger().error(f'Failed to reach corner for side {side_idx}!')
                goal_handle.abort()
                result.success = False
                return result

        # Step 6.5: All 4 sides completed successfully
        goal_handle.succeed()
        result.success = True
        self.get_logger().info('Square drawing completed successfully with 4 goToPose goals!')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SquareActionServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
