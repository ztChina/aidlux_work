

#include "slam_gmapping/transform.h"

#include "tf2_ros/create_timer_ros.h"

using std::placeholders::_1;

Transform::Transform() : Node("transform_node"),
                         transform_thread_(nullptr) {
    this->declare_parameter("parents_frame",parents_frame);
    this->declare_parameter("child_frame",child_frame);
    this->declare_parameter("x",0.0);
    this->declare_parameter("y",0.0);
    this->declare_parameter("z",0.0);
    this->declare_parameter("roll",0.0);
    this->declare_parameter("pitch",0.0);
    this->declare_parameter("yaw",0.0);
    this->get_parameter_or<std::string>("parents_frame", parents_frame, "odom");
    this->get_parameter_or<std::string>("child_frame", child_frame, "laser");
    this->get_parameter_or<std::double_t>("x", x, 0.0);
    this->get_parameter_or<std::double_t>("y", y, 0.0);
    this->get_parameter_or<std::double_t>("z", z, 0.0);
    this->get_parameter_or<std::double_t>("roll", roll, 0.0);
    this->get_parameter_or<std::double_t>("pitch", pitch, 0.0);
    this->get_parameter_or<std::double_t>("yaw", yaw, 0.0);

    //Degree to radius:
    yaw = yaw * M_PI / 180;
    pitch = pitch * M_PI / 180;
    roll = roll * M_PI / 180;
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    rotation_w = cy * cp * cr + sy * sp * sr;
    rotation_x = cy * cp * sr - sy * sp * cr;
    rotation_y = sy * cp * sr + cy * sp * cr;
    rotation_z = sy * cp * cr - cy * sp * sr;

    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(),
            get_node_timers_interface());
    buffer_->setCreateTimerInterface(timer_interface);
    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    transform_thread_ = std::make_shared<std::thread>
            (std::bind(&Transform::publishLoop, this));
}

void Transform::publishLoop() {
    rclcpp::Rate r(1.0 / 0.05);
    while (rclcpp::ok()) {
        publishTransform();
        r.sleep();
    }
}


void Transform::publishTransform() {
    if (parents_frame.empty() || child_frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Empty frame_id or child_frame_id!");
        return;
    }
    rclcpp::Time tf_expiration = get_clock()->now() + rclcpp::Duration(
            static_cast<int32_t>(static_cast<rcl_duration_value_t>(tf_delay_)), 0);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parents_frame;
    transform.header.stamp = tf_expiration;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation.w = rotation_w;
    transform.transform.rotation.x = rotation_x;
    transform.transform.rotation.y = rotation_y;
    transform.transform.rotation.z = rotation_z;
    try {
        tfB_->sendTransform(transform);
    }
    catch (tf2::LookupException &te) {
        RCLCPP_INFO(this->get_logger(), te.what());
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto transform = std::make_shared<Transform>();
    rclcpp::spin(transform);
    return (0);
}
