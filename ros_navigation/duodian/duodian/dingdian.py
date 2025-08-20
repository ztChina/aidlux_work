from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

class dinlian(Node):
    def __init__(self):
        super().__init__("cheshi")
        self.get_logger().info("定点导航节点建立")
        self.navigater=BasicNavigator()
        self.navigater.waitUntilNav2Active()
    def init(self):
        
        self.goal_pose=PoseStamped()
        self.goal_pose.header.frame_id='map'
        self.goal_pose.header.stamp=self.navigater.get_clock().now().to_msg()
        self.goal_pose.pose.position.x=0.0
        self.goal_pose.pose.position.y=0.0
        self.goal_pose.pose.orientation.w=0.0
        self.navigater.goToPose(self.goal_pose)
        self.get_logger().info("初始化完成")
        
        
    def way_pont(self):
        
        self.goal_poses = []
        self.goal_poses1=PoseStamped()
        self.goal_poses1.header.frame_id='map'
        self.goal_poses1.header.stamp=self.navigater.get_clock().now().to_msg()
        self.goal_poses1.pose.position.x=4.0
        self.goal_poses1.pose.position.y=0.0
        self.goal_poses1.pose.orientation.w=1.0
        self.goal_poses.append(self.goal_poses1)
        self.goal_poses2=PoseStamped()
        self.goal_poses2.header.frame_id='map'
        self.goal_poses2.header.stamp=self.navigater.get_clock().now().to_msg()
        self.goal_poses2.pose.position.x=-4.0
        self.goal_poses2.pose.position.y=1.0
        self.goal_poses2.pose.orientation.w=1.0
        self.goal_poses.append(self.goal_poses2)
        self.navigater.followWaypoints(self.goal_poses)
    # 判断结束及获取反馈
        while not self.navigater.isTaskComplete():
            feedback = self.navigater.getFeedback()
            self.navigater.get_logger().info(
                f'当前目标编号：{feedback.current_waypoint}')
            
    # 最终结果判断
        result = self.navigater.getResult()
        if result == TaskResult.SUCCEEDED:
            self.navigater.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            self.navigater.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.navigater.get_logger().error('导航结果：失败')
        else:
            self.navigater.get_logger().error('导航结果：返回状态无效')

    def check_pose_reachable(self, pose):
        """检查目标点是否可达"""
        try:
            self.navigater.goToPose(pose)
            result = self.navigater.getResult()
            return result == TaskResult.SUCCEEDED
        except Exception as e:
            self.get_logger().error(f"目标点检查失败: {str(e)}")
            return False


def main():
    rclpy.init()
    shiyan=dinlian()
    shiyan.way_pont()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
