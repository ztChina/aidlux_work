
from std_msgs.msg import String
import zmq
import threading
import rclpy
from rclpy.node import Node

class pub(Node):
    def __init__(self):
        super().__init__('FaBu_CheShi')
        self.get_logger().error("发布节点测试开始")
        self.pub =self.create_publisher(String,'ros_topic',10)
        self.timer=self.create_timer(1,self.publish_callback)

    def publish_callback(self):
        self.msg=String()
        self.msg.data="turn_right(30.0)"
        self.pub.publish(self.msg)

        # self.msg.data="status 移动通知:方向：R 距离：1.0 角度：30"
        # self.msg1.data="follow_point([[3.0,-1.0],[0.0,0.0]])"
        # self.msg2=String()
        # self.msg2.data="status 移动通知:方向：A 角度：30.0"
        # self.pub.publish(self.msg2)



def main():
    rclpy.init()
    rclpy.spin(pub())
    rclpy.shutdown()

if __name__ == "__main__":
    main()  