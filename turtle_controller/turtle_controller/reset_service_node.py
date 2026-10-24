import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen


class ResetServiceNode(Node):
    def __init__(self):
        super().__init__('reset_service_node')

        self.cb_group = ReentrantCallbackGroup()

        # Service clients to Turtlesim's native services
        self.clear_cli = self.create_client(Empty, '/clear', callback_group=self.cb_group)
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=self.cb_group)
        self.set_pen_cli = self.create_client(SetPen, '/turtle1/set_pen', callback_group=self.cb_group)

        # Custom reset service provided to the system
        self.reset_srv = self.create_service(
            Empty,
            'reset_turtle',
            self.reset_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('Reset Service Node initialized (service: /reset_turtle).')

    async def reset_callback(self, request, response):
        self.get_logger().info('Reset requested: Teleporting turtle to origin and clearing screen...')

        # Ensure all required Turtlesim services are available
        for name, cli in [('/turtle1/set_pen', self.set_pen_cli),
                          ('/turtle1/teleport_absolute', self.teleport_cli),
                          ('/clear', self.clear_cli)]:
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f'Service {name} is not available!')
                return response

        # 1. Turn pen OFF so no trace is drawn during teleportation
        pen_off = SetPen.Request()
        pen_off.off = 1
        await self.set_pen_cli.call_async(pen_off)

        # 2. Teleport turtle back to the starting center (5.544445, 5.544445, 0.0)
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = 5.544445
        teleport_req.y = 5.544445
        teleport_req.theta = 0.0
        await self.teleport_cli.call_async(teleport_req)

        # 3. Clear all trails from the screen
        clear_req = Empty.Request()
        await self.clear_cli.call_async(clear_req)

        # 4. Turn pen back ON (default Turtlesim pen: whitish line, width 3)
        pen_on = SetPen.Request()
        pen_on.r = 179
        pen_on.g = 184
        pen_on.b = 255
        pen_on.width = 3
        pen_on.off = 0
        await self.set_pen_cli.call_async(pen_on)

        self.get_logger().info('Reset complete: Turtle at center facing 0.0, screen cleared cleanly.')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetServiceNode()
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
