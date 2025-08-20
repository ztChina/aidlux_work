import serial
import struct
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import math
from queue import Queue

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.declare_parameter('port', '/dev/ttyUSB0')  #  sudo chmod 666 /dev/ttyUSB0 
        # self.declare_parameter('port', '/dev/pts/20')
        self.port = self.get_parameter('port').value

        self.serial_init()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_recv_time = None
        self.last_cmd = [0.0, 0.0]
        self.last_sent_cmd = [None, None] # ????????????????
        self.last_time = self.get_clock().now()
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.temp_pub = self.create_publisher(Float32, 'sensor/temperature', 10)
        self.humi_pub = self.create_publisher(Float32, 'sensor/humidity', 10)
        self.water_pub = self.create_publisher(Float32, 'sensor/water_level', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # self.read_buffer = bytearray()
        self.buffer = bytearray()
        self.frame_queue = Queue()

        self.processor_thread = threading.Thread(target=self.frame_processing_loop)
        self.processor_thread.daemon = True
        self.processor_thread.start()

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # self.create_timer(0.02, self.send_velocity)

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.create_timer(0.01, self.serial_read_timer_callback)
        self.create_timer(0.1, self.serial_write_timer_callback)
        # self.create_timer(0.01, self.frame_handler_timer_callback)

    def serial_init(self):
        try:
            self.ser = serial.Serial(self.port, baudrate=115200, timeout=0, write_timeout=0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"???????{self.port}")
        except Exception as e:
            self.get_logger().error(f"???????: {e}")
            raise

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        wheel_base = 0.234
        v_l = self.linear_x - (self.angular_z * wheel_base / 2.0)
        v_r = self.linear_x + (self.angular_z * wheel_base / 2.0)
        self.last_cmd = [v_l, v_r]  # ???????
        # if [v_l, v_r] != getattr(self, 'last_cmd', [None, None]):# ????????????????????????????????
        #     self.last_cmd = [v_l, v_r]
        #     frame = self.build_frame(0x02, [v_l, v_r])
        #     try:
        #         self.ser.write(frame)
        #     except serial.SerialTimeoutException:
        #         self.get_logger().warn("????????????")
        #     except Exception as e:
        #         self.get_logger().error(f"?????: {e}")
        


    def build_frame(self, func_code, floats):
        frame = bytearray([0xAA, 0x55])
        data_len = 1 + len(floats) * 4 + 1
        frame.append(data_len)
        frame.append(func_code)
        for f in floats:
            frame.extend(struct.pack('<f', f))
        checksum = 0
        for b in frame[2:]:
            checksum ^= b
        frame.append(checksum)
        return frame

    def serial_write_timer_callback(self):
        v_l, v_r = self.last_cmd
        if [v_l, v_r] != self.last_sent_cmd:# ????????????????????????????????
            self.last_sent_cmd = [v_l, v_r]
            frame = self.build_frame(0x02, [v_l, v_r])
            try:
                self.ser.write(frame)
            except serial.SerialTimeoutException:
                self.get_logger().warn("????????????")
            except Exception as e:
                self.get_logger().error(f"?????: {e}")


    def serial_read_timer_callback(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        if data:

            now = self.get_clock().now()
            if self.last_recv_time is not None:
                dt = (now - self.last_recv_time).nanoseconds / 1e6
                self.get_logger().info(f"? ??????: {dt:.1f} ms")
            self.last_recv_time = now

            self.buffer.extend(data)
            while True:
                head_idx = self.buffer.find(b'\xAA\x55')
                if head_idx == -1:
                    self.buffer.clear()
                    break
                if head_idx > 0:
                    self.buffer = self.buffer[head_idx:]
                if len(self.buffer) < 3:
                    break
                length = self.buffer[2]
                total_len = length + 3
                if len(self.buffer) < total_len:
                    break
                frame = self.buffer[:total_len]
                self.buffer = self.buffer[total_len:]

                checksum = 0
                for b in frame[2:-1]:
                    checksum ^= b
                if checksum == frame[-1]:
                    func_code = frame[3]
                    data_bytes = frame[4:-1]
                    self.frame_queue.put((func_code, data_bytes))  # ? ??
                    # self.handle_frame(func_code, data_bytes)  # ? ???????
                else:
                    self.get_logger().warn("????????")

    def frame_processing_loop(self):
        while rclpy.ok():
            try:
                func_code, data_bytes = self.frame_queue.get(timeout=0.1)
                self.handle_frame(func_code, data_bytes)
            except Exception:
                continue

    def handle_frame(self, func_code, data_bytes):
        # now = self.get_clock().now()
        # if self.last_recv_time is not None:
        #     dt = (now - self.last_recv_time).nanoseconds / 1e6
        #     # self.get_logger().info(f"? ??????: {dt:.1f} ms")
        # self.last_recv_time = now
        if len(data_bytes) % 4 != 0:
            self.get_logger().warn("??????float????")
            return
        floats = [struct.unpack('<f', data_bytes[i:i+4])[0] for i in range(0, len(data_bytes), 4)]
        # self.get_logger().info(f"???: 0x{func_code:02X}, ??: {floats}")

        if func_code == 0x01:
            floats = [struct.unpack('<f', data_bytes[i:i+4])[0] for i in range(0, len(data_bytes), 4)]
            if len(floats) >= 5:
                left, right, temp, humi, water = floats[:5]
                self.get_logger().info(
                    f"????????? - ??: {left:.3f} m/s, ??: {right:.3f} m/s, ??: {temp:.3f} ?, "
                    f"??: {humi:.3f}s %, ??: {water:.3f}"
                )
                self.temp_pub.publish(Float32(data=temp))
                self.humi_pub.publish(Float32(data=humi))
                self.water_pub.publish(Float32(data=water))
                self.update_odometry(left, right)

    def update_odometry(self, v_l, v_r):

        wheel_base=0.234
        current_time = self.get_clock().now()
        # dt = (current_time - self.last_time).nanoseconds / 1e9
        dt = 0.1
        # if dt <= 0 or dt > 0.12:
        #     self.last_time = current_time
        #     return
        self.get_logger().error(f"dt={dt}")
        # ?????????m?
        d_left = v_l * dt
        d_right = v_r * dt

        # ???????????
        d_center = (d_left + d_right) / 2.0
        phi = (d_right - d_left) / wheel_base  # ??

        # ????
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)
        self.th += phi  # ????????

        # ??????[-?, ?]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # ??TF?Odom
        self.publish_odom_and_tf(d_center / dt, phi / dt, current_time)
        # self.last_time =self.get_clock().now()
        # self.get_logger().warning(f"lase_time={self.last_time}")

    def publish_odom_and_tf(self, v, w, current_time):
    # ???
        odom_quat = Quaternion()
        odom_quat.x = 0.0
        odom_quat.y = 0.0
        odom_quat.z = math.sin(self.th / 2.0)
        odom_quat.w = math.cos(self.th / 2.0)

        # TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom_quat
        self.tf_broadcaster.sendTransform(t)

        # Odometryaidlux
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = odom_quat
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        # ?????????????????????
        odom.pose.covariance = [
            0.1, 0, 0, 0, 0, 0,    # x
            0, 0.1, 0, 0, 0, 0,      # y
            0, 0, 1e6, 0, 0, 0,     # z (2D?????)
            0, 0, 0, 1e6, 0, 0,      # roll (??2D??????)
            0, 0, 0, 0, 1e6, 0,      # pitch (??2D??????)
            0, 0, 0, 0, 0, 0.05      # yaw/th
        ]

        # ?????
        odom.twist.covariance = [
            0.2, 0, 0, 0, 0, 0,     # vx
            0, 0, 0, 0, 0, 0,      # vy (??????????0)
            0, 0, 1e6, 0, 0, 0,      # vz
            0, 0, 0, 1e6, 0, 0,      # vroll
            0, 0, 0, 0, 1e6, 0,      # vpitch
            0, 0, 0, 0, 0, 0.1       # vyaw
        ]

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
