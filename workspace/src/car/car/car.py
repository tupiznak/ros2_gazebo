import rclpy
from rclpy.node import Node

from car_msgs.msg import Kinematic, Airbag


class Car(Node):

    def __init__(self):
        super().__init__('car')

        self.create_subscription(Kinematic, '/sensor/kinematic',
                                 self.kinematic_callback, 1)
        self.airbag_command_pub = \
            self.create_publisher(Airbag, '/command/airbag', 1)

        self.get_logger().info('Нода запущена')

    def kinematic_callback(self, msg: Kinematic):
        a = msg.acceleration
        self.get_logger().info(f'Получено ускорение: {a}')
        if a > 10:
            self.get_logger() \
                .warn('Критическое ускорение. Выброс подушки безопасноcти')
            self.airbag_command_pub.publish(Airbag(drop=True))


def main():
    rclpy.init()
    executor = rclpy.get_global_executor()
    node = Car()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
