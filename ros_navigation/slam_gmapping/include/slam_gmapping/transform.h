

#ifndef SLAM_GMAPPING_SLAM_GMAPPING_H_
#define SLAM_GMAPPING_SLAM_GMAPPING_H_

#include <mutex>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"
#include "message_filters/subscriber.h"
#include "tf2_ros/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"
#include "gmapping/sensor/sensor_range/rangesensor.h"
#include "gmapping/sensor/sensor_odometry/odometrysensor.h"

using namespace std;

class Transform : public rclcpp::Node{
public:
    Transform();
    void publishTransform();
    void publishLoop();

private:
    string parents_frame, child_frame;
    double x, y, z;
    double yaw, pitch, roll;
    double rotation_w, rotation_x, rotation_y, rotation_z;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfB_;
    std::shared_ptr<std::thread> transform_thread_;
    double tf_delay_;
};

#endif //SLAM_GMAPPING_SLAM_GMAPPING_H_
