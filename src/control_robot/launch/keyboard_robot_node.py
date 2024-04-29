

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial 
from geometry_msgs.msg import Twist

class KeyboardRobotNode(Node):    # 繼承 Node 類別
    def __init__(self):
        super().__init__('keyboard_robot_node')    # 初始化 Node 類別
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.move_callback, 10)
        self.subscription  # prevent unused variable warning
        self.robot = serial.Serial('/dev/ttyACM0', 115200, timeout=1)    # 設定串列通訊的設定


    def speed(self,v):
        a = [0, 0]
        if v >= 0:
            a[0] = int(v / 256)
            a[1] = int(v % 256)
        else:
            v = -v
            a[0] = 255 - int(v / 256)
            a[1] = 255 - int(v % 256)
        return a

    def moter_vmode(self, robot, Vx, Vy, Vz):
        x, y, z = self.speed(Vx), self.speed(Vy), self.speed(Vz)
        BCC = 0x7B ^ x[0] ^ x[1] ^ y[0] ^ y[1] ^ z[0] ^ z[1]
        robot.write(
            bytearray([0x7B, 0x00, 0x00, x[0], x[1], y[0], y[1], z[0], z[1], BCC, 0x7D])
        )  # cmd = [0x7B, 0x00, Setzero, x[0], x[1], y[0], y[1], z[0], z[1], BCC, 0x7D]

    def move_forward(self, robot, speed):
        self.moter_vmode(robot, speed, 0, 0)

    def move_left(self, robot, speed):
        self.moter_vmode(robot, 0, speed, 0)

    def move_right(self, robot, speed):
        self.moter_vmode(robot, 0, -speed, 0)

    def move_back(self, robot, speed):
        self.moter_vmode(robot, -speed, 0, 0)


    def move_callback(self, msg):
        x_speed = msg.linear.x
        z_speed = msg.angular.z

        if x_speed > 0:
            self.move_forward(self.robot, abs(x_speed) * 10)
        elif x_speed < 0:
            self.move_back(self.robot, abs(x_speed) * 10)

        if z_speed > 0:
            self.move_left(self.robot, abs(z_speed) * 10)
        elif z_speed < 0:
            self.move_right(self.robot, abs(z_speed) * 10)
    
    



def main(args=None):    # 主程式
    rclpy.init(args=args)
    keyboard_robot_node = KeyboardRobotNode()
    rclpy.spin(keyboard_robot_node)
    keyboard_robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':    # 程式進入點
    main()    # 呼叫主程式