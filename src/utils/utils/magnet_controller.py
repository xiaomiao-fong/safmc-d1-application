# TODO drone_id???

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from agent_msgs.msg import Magnet
from gpiozero import LED


class MagnetController(Node):

    def __init__(self):
        super().__init__('magnet_controller')
        self.create_subscription(String, '/drone_1/in/magnet',self.activate_magnet, 10)

        # 5: mid
        # 6: front
        # 16: back
        self.leds = [LED(pin) for pin in [5, 6, 16]]

    def activate_magnet(self, msg: Magnet):
        for led, state in zip(self.leds, [msg.magnet1, msg.magnet2, msg.magnet3]):
            led.on() if state else led.off()


def main(args=None):
    rclpy.init(args=args)
    node = MagnetController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

