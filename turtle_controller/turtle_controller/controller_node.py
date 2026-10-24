import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rcl_interfaces.msg import ParameterDescriptor

from turtlesim_interfaces.srv import SetSpeedLimits
from turtlesim_interfaces.action import GoToPose


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller_node')

        # Use reentrant callback group so callbacks don't block each other
        self.cb_group = ReentrantCallbackGroup()

        # Step 3: Parameters
        self.declare_parameter('max_linear_speed', 2.0, ParameterDescriptor(description='Max linear velocity'))
        self.declare_parameter('max_angular_speed', 1.5, ParameterDescriptor(description='Max angular velocity'))

        # Step 3: Speed limit service
        self.speed_srv = self.create_service(
            SetSpeedLimits,
            'set_speed_limits',
            self.set_speed_limits_callback,
            callback_group=self.cb_group
        )

        # Step 4: Pose subscriber & cmd_vel publisher
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.cb_group
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Step 4: GoToPose Action Server
        self._action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            execute_callback=self.execute_goto_pose_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('Turtle Controller Node running with GoToPose Action Server and Speed Service.')

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def set_speed_limits_callback(self, request, response):
        if request.max_linear_speed <= 0.0 or request.max_angular_speed <= 0.0:
            response.success = False
            response.message = 'Error: Speeds must be strictly positive (> 0).'
            self.get_logger().warn(f'Rejected invalid speed limits: {request.max_linear_speed}, {request.max_angular_speed}')
            return response

        p_lin = rclpy.parameter.Parameter('max_linear_speed', rclpy.Parameter.Type.DOUBLE, float(request.max_linear_speed))
        p_ang = rclpy.parameter.Parameter('max_angular_speed', rclpy.Parameter.Type.DOUBLE, float(request.max_angular_speed))
        self.set_parameters([p_lin, p_ang])

        response.success = True
        response.message = f'Updated: max_linear={request.max_linear_speed}, max_angular={request.max_angular_speed}'
        self.get_logger().info(response.message)
        return response

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: x={goal_request.target_pose.x:.2f}, y={goal_request.target_pose.y:.2f}, yaw={goal_request.target_pose.yaw:.2f}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    @staticmethod
    def normalize_angle(angle):
        """Wrap angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop_turtle(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def execute_goto_pose_callback(self, goal_handle):
        target = goal_handle.request.target_pose
        feedback_msg = GoToPose.Feedback()
        result = GoToPose.Result()

        # Wait until we have received at least one pose message
        rate = 20.0  # 20 Hz control loop
        while self.current_pose is None and rclpy.ok():
            self.get_logger().info('Waiting for turtle pose...')
            time.sleep(0.1)

        # Control gains
        k_linear = 1.5
        k_angular = 4.0
        pos_tolerance = 0.05
        yaw_tolerance = 0.03

        self.get_logger().info('Phase 1: Navigating to target (x, y)...')

        # PHASE 1: Move towards (target.x, target.y)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_turtle()
                result.success = False
                return result

            # Read parameters dynamically each loop so live changes take effect immediately
            max_lin = self.get_parameter('max_linear_speed').value
            max_ang = self.get_parameter('max_angular_speed').value

            dx = target.x - self.current_pose.x
            dy = target.y - self.current_pose.y
            dist = math.sqrt(dx * dx + dy * dy)

            target_heading = math.atan2(dy, dx)
            heading_err = self.normalize_angle(target_heading - self.current_pose.theta)

            # Publish feedback
            feedback_msg.distance_remaining = dist
            feedback_msg.angle_remaining = abs(heading_err)
            goal_handle.publish_feedback(feedback_msg)

            # Check if position is reached
            if dist <= pos_tolerance:
                break

            cmd = Twist()
            # If heading error is large, turn first; otherwise drive and turn concurrently
            if abs(heading_err) > 0.5:
                cmd.linear.x = 0.0
                raw_ang = k_angular * heading_err
                cmd.angular.z = max(-max_ang, min(max_ang, raw_ang))
            else:
                raw_lin = k_linear * dist
                raw_ang = k_angular * heading_err
                cmd.linear.x = max(-max_lin, min(max_lin, raw_lin))
                cmd.angular.z = max(-max_ang, min(max_ang, raw_ang))

            self.cmd_vel_pub.publish(cmd)
            time.sleep(1.0 / rate)

        self.stop_turtle()
        self.get_logger().info('Phase 2: Aligning to target yaw...')

        # PHASE 2: In-place rotation to target.yaw
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_turtle()
                result.success = False
                return result

            max_ang = self.get_parameter('max_angular_speed').value

            yaw_err = self.normalize_angle(target.yaw - self.current_pose.theta)

            feedback_msg.distance_remaining = 0.0
            feedback_msg.angle_remaining = abs(yaw_err)
            goal_handle.publish_feedback(feedback_msg)

            if abs(yaw_err) <= yaw_tolerance:
                break

            cmd = Twist()
            raw_ang = k_angular * yaw_err
            cmd.angular.z = max(-max_ang, min(max_ang, raw_ang))
            self.cmd_vel_pub.publish(cmd)
            time.sleep(1.0 / rate)

        # Stop and finish
        self.stop_turtle()
        goal_handle.succeed()
        result.success = True
        self.get_logger().info('Target pose successfully reached!')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
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
