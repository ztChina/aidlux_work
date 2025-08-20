from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


class YoloFollower(Node):
    def __init__(self):
        super().__init__("cheshi")
        self.get_logger().info("YOLO 导航节点启动")

        # 初始化导航器
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.current_pose = None
        self.latest_feedback = None
        self.nav_started = False
        self.nav_complete = True
        self.person_area = 2  # 面积阈值（根据你的像素调整）
        self.current_goal_handle = None  # 跟踪当前目标句柄

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 订阅当前位置
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # 订阅 YOLO 目标检测
        self.person_sub = self.create_subscription(
            PointStamped,
            '/person_box',
            self.person_box_callback,
            10
        )

        # 发布"靠近人"的标志
        self.close_pub = self.create_publisher(Bool, '/close_to_person', 10)

        # 控制导航启动的服务
        self.srv = self.create_service(SetBool, '/start_nav', self.start_nav_callback)

    def pose_init(self):
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 0.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.orientation.w = 1.0
        self.navigator.goToPose(self.goal_pose) 
        self.get_logger().info('初始化完成')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def start_nav_callback(self, request, response):
        if request.data:
            self.get_logger().info("收到启动导航请求")
            self.nav_started = True
            response.success = True
            response.message = "导航开始"
        else:
            self.get_logger().info("收到停止导航请求")
            self.nav_started = False
            response.success = True
            response.message = "导航停止"
        return response

    def person_box_callback(self, msg: PointStamped):
        if not self.nav_started:
            return

        box_area = msg.point.z
        self.get_logger().info(f"接收到目标框面积: {box_area:.2f}")

        if box_area >= self.person_area:
            self.get_logger().info("已靠近人，发布 close_to_person")
            self.nav_started = False
            self.nav_complete = True
            msg_out = Bool()
            msg_out.data = True
            self.close_pub.publish(msg_out)
            return

        self.get_logger().info(f"导航靠近 ({msg.point.x:.2f}, {msg.point.y:.2f}) 中...")

        # 创建导航目标
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.point.x
        goal_pose.pose.position.y = msg.point.y
        goal_pose.pose.orientation.w = 1.0  # 朝向简单设定

        # 取消当前导航（如果有）
        self.cancel_current_navigation()

        # 发送新导航目标
        self.send_navigation_goal(goal_pose)

    def cancel_current_navigation(self):
        if self.current_goal_handle is not None:
            self.get_logger().info("取消当前导航...")
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        try:
            future.result()
            self.get_logger().info("成功取消当前导航")
        except Exception as e:
            self.get_logger().error(f"取消导航失败: {str(e)}")

    def send_navigation_goal(self, goal_pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info("发送导航目标...")
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().warning("导航目标被拒绝")
            self.nav_complete = True
            self.current_goal_handle = None
            return

        self.get_logger().info("目标已接受，开始导航")
        self.nav_complete = False
        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        try:
            result = future.result().result

            if result == TaskResult.SUCCEEDED or self.latest_feedback <= 0.3:
                self.get_logger().info("导航成功")
            elif result == TaskResult.CANCELED:
                self.get_logger().info("导航被取消")
            else:
                self.get_logger().warning("导航失败")

        except Exception as e:
            self.get_logger().error(f"导航异常: {str(e)}")
        
        self.nav_complete = True
        self.current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.distance_remaining
        self.latest_feedback = feedback
        self.get_logger().info(f"剩余距离: {self.latest_feedback:.2f} m")
        return self.latest_feedback

def main(args=None):
    rclpy.init(args=args)
    node = YoloFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()