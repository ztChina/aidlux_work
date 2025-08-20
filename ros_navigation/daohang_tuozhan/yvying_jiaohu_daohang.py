

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler
 


class YoloFollower(Node):
    def __init__(self):
        super().__init__("cheshi")
        self.get_logger().info("YOLO 导航节点启动")

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.goal_pose=PoseStamped()
        self.goal_pose.header.frame_id='map'
        self.goal_pose.header.stamp=self.navigator.get_clock().now().to_msg()
        self.pose_init()
        self.current_pose = None
        self.latest_feedback = None
        self.nav_started = False
        self.nav_complete = True
        self.localization_ready = False
        self.person_area = 2  # 面积阈值（根据你的像素调整）

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.localization_timer = self.create_timer(1.0, self.check_localization)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.topic_sub=self.create_subscription(
            String,
            'ros_topic',
            self.topic_callback,
            10
        )

    def topic_callback(self,msg):

        if "follow_point" in msg.data:
            try:
                # 解析路径点，例如 "follow_point([[3.0,-1.0],[0.0,0.0]])"
                points_str = msg.data.split('(', 1)[1].rstrip(')')
                points = eval(points_str)  # 将字符串转换为列表
                self.follow_point(points)
            except Exception as e:
                self.get_logger().error(f"解析指令失败: {e}")

        if "move_to" in msg.data:
            try:
                point_str = msg.data.split('(',1)[1].rstrip(')')
                point_x = float(point_str.split(',')[0])
                point_y = float(point_str.split(',')[1])
                self.move_to(point_x,point_y)
            except Exception as e:
                self.get_logger().error(f"解析指令失败: {e}")

        if "move_forward" in msg.data:
            try:
                distance_str=msg.data.split('(',1)[1].rstrip(')')
                distance=float(distance_str)
                self.move_forward(distance)
            except:
                self.get_logger().error(f"解析指令失败: {e}")

        if "move_backward" in msg.data:
            try:
                distance_str=msg.data.split('(',1)[1].rstrip(')')
                distance=float(distance_str)
                self.move_backward(distance)
            except:
                self.get_logger().error(f"解析指令失败: {e}")

        if "turn_right" in msg.data:
            try:
                angle_str=msg.data.split('(',1)[1].rstrip(')')
                angle=float(angle_str)
                self.turn_right(angle)
            except:
                self.get_logger().error(f"解析指令失败: {e}")

        if "turn_left" in msg.data:
            try:
                angle_str=msg.data.split('(',1)[1].rstrip(')')
                angle=float(angle_str)
                self.turn_left(angle)
            except:
                self.get_logger().error(f"解析指令失败: {e}")



    def check_localization(self):
        if self.current_pose is not None and not self.localization_ready:
            self.get_logger().info("定位系统准备就绪")
            self.localization_ready = True
            self.localization_timer.cancel()

    def pose_init(self):
    # 初始化目标位姿 (PoseStamped)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0  # 必填
        initial_pose.pose.orientation.x = 0.0  # 必填
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0

    
    # 设置初始位姿
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('初始化完成')


    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose


    def move_to(self,point_x,point_y):
        if  self.nav_complete:
            self.nav_complete = False
            
            self.goal_pose.pose.position.x=point_x
            self.goal_pose.pose.position.y=point_y
            self.goal_pose.pose.orientation.w=1.0
            self.send_navigation_goal(self.goal_pose)
    
    def move_forward(self,distance):

        if self.current_pose is None:
            self.get_logger().warn("当前位姿不可用，等待定位...")
            return False

        if self.nav_complete:
            self.nav_complete = False

            self.goal_pose.pose.position.x=self.current_pose.position.x + distance
            self.goal_pose.pose.position.y=self.current_pose.position.y
            self.goal_pose.pose.orientation.w=1.0
            self.send_navigation_goal(self.goal_pose)

    def move_backward(self,distance):
        if self.current_pose is None:
            self.get_logger().warn("当前位姿不可用，等待定位...")
            return False

        if self.nav_complete:
            self.nav_complete = False

            self.goal_pose.pose.position.x=self.current_pose.position.x - distance
            self.goal_pose.pose.position.y=self.current_pose.position.y
            self.goal_pose.pose.orientation.w=1.0
            self.send_navigation_goal(self.goal_pose)

    def turn_right(self,angle):
        if self.current_pose is None:
            self.get_logger().warn("当前位姿不可用，等待定位...")
            return False

        if self.nav_complete:
            self.nav_complete = False
            rotation_rota = quaternion_from_euler(0,0,angle)
            self.goal_pose.pose.position.x=self.current_pose.position.x 
            self.goal_pose.pose.position.y=self.current_pose.position.y
            self.goal_pose.pose.orientation.x=rotation_rota[0]
            self.goal_pose.pose.orientation.y=rotation_rota[1]
            self.goal_pose.pose.orientation.z=rotation_rota[2]
            self.goal_pose.pose.orientation.w=rotation_rota[3]
            self.send_navigation_goal(self.goal_pose)

    def turn_left(self,angle):
        if self.current_pose is None:
            self.get_logger().warn("当前位姿不可用，等待定位...")
            return False

        if self.nav_complete:
            self.nav_complete = False
            rotation_rota = quaternion_from_euler(0,0,-angle)
            self.goal_pose.pose.position.x=self.current_pose.position.x 
            self.goal_pose.pose.position.y=self.current_pose.position.y
            self.goal_pose.pose.orientation.x=rotation_rota[0]
            self.goal_pose.pose.orientation.y=rotation_rota[1]
            self.goal_pose.pose.orientation.z=rotation_rota[2]
            self.goal_pose.pose.orientation.w=rotation_rota[3]
            self.send_navigation_goal(self.goal_pose)
    
    def follow_point(self, points):
        if self.current_pose is None:
            self.get_logger().warn("当前位姿不可用，等待定位...")
            return False

        if self.nav_complete:
            self.nav_complete = False
            self.path_points = points  # 存储所有路径点
            self.current_point_index = 0  # 当前路径点索引
            self._send_next_point()  # 开始发送第一个点
        return True
    
    def _send_next_point(self):

        if self.current_point_index < len(self.path_points):
            point = self.path_points[self.current_point_index]
            self.goal_pose = PoseStamped()
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            
            self.goal_pose.pose.position.x = float(point[0])
            self.goal_pose.pose.position.y = float(point[1])
            self.goal_pose.pose.orientation.w = 1.0
            
            self.send_navigation_goal(self.goal_pose)
            self.current_point_index += 1
        else:
            self.nav_complete = True
            self.get_logger().info("所有路径点导航完成")
                                    

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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("导航目标被拒绝")
            self.nav_complete = True
            return

        self.get_logger().info("目标已接受，开始导航")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self,future):
        try:
            result = future.result().result
            feedback = self.latest_feedback if self.latest_feedback is not None else float('inf')
            
            if result.result == TaskResult.SUCCEEDED or feedback <= 0.3:
                self.get_logger().info("导航到当前点成功")
                if hasattr(self, 'current_point_index') and self.current_point_index < len(getattr(self, 'path_points', [])):
                    self._send_next_point()
                else:
                    self.nav_complete = True
            elif result.result == TaskResult.CANCELED:
                self.get_logger().info("导航被取消")
                self.nav_complete = True
            else:
                self.get_logger().warning("导航失败")
                self.nav_complete = True
        except Exception as e:
            self.get_logger().error(f"导航异常: {str(e)}")
            self.nav_complete = True
        self.nav_started = True
        

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


#ros2 service call /start_yolo std_srvs/srv/SetBool "{data: true}"
