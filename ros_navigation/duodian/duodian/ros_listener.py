
from std_msgs.msg import String
import zmq
import threading
import rclpy
from rclpy.node import Node


PUBLISHER_IP = "192.168.111.234"
SUBSCRIBER_IP = "192.168.111.23"
PUB_PORT = "5556"
REP_PORT = "5557"

class ROSListener(Node):  
    def __init__(self, publisher_ip=PUBLISHER_IP, pub_port=PUB_PORT, rep_port=REP_PORT, topic="status"):
        super().__init__('rps_listener_node')  
        self.rqm_context = zmq.Context()
        self.pub=self.create_publisher(String,'ros_topic',10)


        address = f"tcp://{publisher_ip}:{pub_port}"
        self.socket = self.rqm_context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.get_logger().info(f"ROS监听器已连接到 {address}，订阅主题 '{topic}'")
        rep_bind_address = f"tcp://{SUBSCRIBER_IP}:{rep_port}"
        self.rep_socket = self.rqm_context.socket(zmq.REP)
        try:
            self.rep_socket.bind(rep_bind_address)
            self.get_logger().info(f"ROS监听器应答端口绑定到 {rep_bind_address}")
        except zmq.ZMQError as e:
            self.get_logger().error(f"绑定失败：{e}")
            raise
        self.listen()
        self.timer=self.create_timer(0.01,self.timer_callback)
    def listen(self):
        def sub_thread():
            self.get_logger().info("订阅线程已启动")
            while True:
                message = self.socket.recv_string()
                self.get_logger().info(f"{message}")

        def rep_thread():
            self.get_logger().info("应答线程已启动")
            while True:
                req = self.rep_socket.recv_string()
                self.get_logger().info(f"收到请求：{req}")
                self.rep_socket.send_string("已收到你的消息！")

        threading.Thread(target=sub_thread, daemon=True).start()
        threading.Thread(target=rep_thread, daemon=True).start()

    def timer_callback(self):
        self.msg=String()
        self.msg.data=self.socket.recv_string()
        self.pub.publish(self.msg)
        self.get_logger().info('%s' %self.msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ROSListener()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()  