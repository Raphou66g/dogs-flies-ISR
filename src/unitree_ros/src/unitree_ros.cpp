#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <memory>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <unitree_ros/unitree_ros.hpp>

#include "unitree_ros/serializers.hpp"

UnitreeRosNode::UnitreeRosNode() : Node("unitree_ros_node") {
    read_parameters_();
    init_subscriptions_();
    init_publishers_();
    init_timers_();
    unitree_driver_ = std::make_unique<UnitreeDriver>(robot_ip_, robot_target_port_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    unitree_driver_->enable_obstacle_avoidance(use_obstacle_avoidance_);

    sleep(1);
    check_robot_battery_callback_();

    RCLCPP_INFO(get_logger(), "Unitree ROS node initialized!");
}

UnitreeRosNode::~UnitreeRosNode() {
    RCLCPP_INFO(get_logger(), "Shutting down Unitree ROS node...");
}

void UnitreeRosNode::apply_namespace_to_topic_names_() {
    cmd_vel_topic_name_ = ns_ + cmd_vel_topic_name_;
    odom_topic_name_ = ns_ + odom_topic_name_;
    imu_topic_name_ = ns_ + imu_topic_name_;
    bms_topic_name_ = ns_ + bms_topic_name_;

    if (ns_ != "") {
        imu_frame_id_ = ns_ + '/' + imu_frame_id_;
        odom_frame_id_ = ns_ + '/' + odom_frame_id_;
        odom_child_frame_id_ = ns_ + '/' + odom_child_frame_id_;
    }
}

// -----------------------------------------------------------------------------
// -                                Init Methods                               -
// -----------------------------------------------------------------------------

void UnitreeRosNode::init_subscriptions_() {
    RCLCPP_INFO(get_logger(), "Initializing ROS subscriptions...");

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_name_,
        qos,
        std::bind(&UnitreeRosNode::cmd_vel_callback_, this, std::placeholders::_1));

    stand_up_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "stand_up",
        qos,
        std::bind(&UnitreeRosNode::stand_up_callback_, this, std::placeholders::_1));

    stand_down_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "stand_down",
        qos,
        std::bind(&UnitreeRosNode::stand_down_callback_, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Finished initializing ROS subscriptions!");
}

void UnitreeRosNode::init_publishers_() {
    RCLCPP_INFO(get_logger(), "Initializing ROS publishers...");

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, qos);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name_, qos);
    bms_pub_ = this->create_publisher<unitree_ros::msg::BmsState>(bms_topic_name_, qos);
    sensor_ranges_pub_ = this->create_publisher<unitree_ros::msg::SensorRanges>(
        sensor_ranges_topic_name, qos);

    RCLCPP_INFO(get_logger(), "Finished initializing ROS publishers!");
}

void UnitreeRosNode::init_timers_() {
    RCLCPP_INFO(get_logger(), "Initializing ROS timers...");

    robot_state_timer_ = this->create_wall_timer(
        20ms, std::bind(&UnitreeRosNode::robot_state_callback_, this));

    cmd_vel_reset_timer_ = this->create_wall_timer(
        1ms, std::bind(&UnitreeRosNode::cmd_vel_reset_callback_, this));

    check_robot_battery_timer_ = this->create_wall_timer(
        1min, std::bind(&UnitreeRosNode::check_robot_battery_callback_, this));

    RCLCPP_INFO(get_logger(), "Finished initializing ROS timers!");
}

// -----------------------------------------------------------------------------
// -                              Callback Methods                             -
// -----------------------------------------------------------------------------

void UnitreeRosNode::cmd_vel_callback_(const geometry_msgs::msg::Twist::UniquePtr msg) {
    unitree_driver_->walk_w_vel(msg->linear.x, msg->linear.y, msg->angular.z);
    prev_cmd_vel_sent_ = clock_.now();
}

void UnitreeRosNode::check_robot_battery_callback_() {
    auto batt_level = unitree_driver_->get_battery_percentage();
    if (batt_level < low_batt_shutdown_threshold_) {
        // Battery is low, Stand down the robot
        RCLCPP_ERROR(this->get_logger(),
                     "Robot battery level is low. Currently at: %hhu%%",
                     batt_level);
        unitree_driver_->stop();
        throw std::runtime_error("Robot battery low. Shutting down Driver Node");
    }
    RCLCPP_INFO(this->get_logger(), "Robot battery level is at: %hhu%%", batt_level);
}

void UnitreeRosNode::robot_state_callback_() {
    auto now = this->get_clock()->now();
    publish_imu_(now);
    publish_bms_();
    publish_sensor_ranges_();
    publish_odom_(now);
}

void UnitreeRosNode::cmd_vel_reset_callback_() {
    auto delta_t = clock_.now() - prev_cmd_vel_sent_;
    if (delta_t >= 400ms && delta_t <= 402ms) {
        unitree_driver_->walk_w_vel(0, 0, 0);
    }
}

void UnitreeRosNode::stand_up_callback_(const std_msgs::msg::Empty::UniquePtr msg) {
    msg.get();  // Just to ignore linter warning
    unitree_driver_->stand_up();
}
void UnitreeRosNode::stand_down_callback_(const std_msgs::msg::Empty::UniquePtr msg) {
    msg.get();  // Just to ignore linter warning
    unitree_driver_->stand_down();
}

// -----------------------------------------------------------------------------
// -                             Publish Methods                               -
// -----------------------------------------------------------------------------

void UnitreeRosNode::publish_odom_(rclcpp::Time time) {
    odom_t odom = unitree_driver_->get_odom();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = odom_child_frame_id_;
    serialize(odom_msg, odom);
    odom_pub_->publish(odom_msg);
    publish_odom_tf_(time, odom);
}

void UnitreeRosNode::publish_imu_(rclcpp::Time time) {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = time;
    imu_msg.header.frame_id = imu_frame_id_;
    serialize(imu_msg, unitree_driver_->get_imu());
    imu_pub_->publish(imu_msg);
}

void UnitreeRosNode::publish_bms_() {
    unitree_ros::msg::BmsState bms_msg;
    serialize(bms_msg, unitree_driver_->get_bms());
    bms_pub_->publish(bms_msg);
}

void UnitreeRosNode::publish_sensor_ranges_() {
    unitree_ros::msg::SensorRanges ranges_msg;
    serialize(ranges_msg, unitree_driver_->get_radar_ranges());
    sensor_ranges_pub_->publish(ranges_msg);
}

void UnitreeRosNode::publish_odom_tf_(rclcpp::Time time, odom_t odom) {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = time;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = odom_child_frame_id_;

    transform.transform.translation.x = odom.pose.position.x;
    transform.transform.translation.y = odom.pose.position.y;
    transform.transform.translation.z = odom.pose.position.z;

    transform.transform.rotation.x = odom.pose.orientation.x;
    transform.transform.rotation.y = odom.pose.orientation.y;
    transform.transform.rotation.z = odom.pose.orientation.z;
    transform.transform.rotation.w = odom.pose.orientation.w;

    tf_broadcaster_->sendTransform(transform);
}

// -----------------------------------------------------------------------------
// -                              ROS Parameters                               -
// -----------------------------------------------------------------------------

void UnitreeRosNode::read_parameters_() {
    RCLCPP_INFO(get_logger(), "Reading ROS parameters...");

    // Robot params
    declare_parameter<std::string>("robot_ip", robot_ip_);
    declare_parameter<int>("robot_target_port", robot_target_port_);
    // --------------------------------------------------------
    get_parameter("robot_ip", robot_ip_);
    get_parameter("robot_target_port", robot_target_port_);

    // Namespace
    declare_parameter<std::string>("ns", ns_);
    // --------------------------------------------------------
    get_parameter("ns", ns_);

    // Topic Names
    declare_parameter<std::string>("cmd_vel_topic_name", cmd_vel_topic_name_);
    declare_parameter<std::string>("imu_topic_name", imu_topic_name_);
    declare_parameter<std::string>("odom_topic_name", odom_topic_name_);
    declare_parameter<std::string>("bms_state_topic_name", bms_topic_name_);
    // --------------------------------------------------------
    get_parameter("cmd_vel_topic_name", cmd_vel_topic_name_);
    get_parameter("odom_topic_name", odom_topic_name_);
    get_parameter("imu_topic_name", imu_topic_name_);
    get_parameter("bms_state_topic_name", bms_topic_name_);

    // Frame Ids
    declare_parameter<std::string>("imu_frame_id", imu_frame_id_);
    declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
    declare_parameter<std::string>("odom_child_frame_id", odom_child_frame_id_);
    // --------------------------------------------------------
    get_parameter("odom_frame_id", odom_frame_id_);
    get_parameter("odom_child_frame_id", odom_child_frame_id_);
    get_parameter("imu_frame_id", imu_frame_id_);

    // Flags
    declare_parameter<uint8_t>("low_batt_shutdown_threshold",
                               low_batt_shutdown_threshold_);
    declare_parameter<bool>("use_obstacle_avoidance", use_obstacle_avoidance_);
    // --------------------------------------------------------
    get_parameter("low_batt_shutdown_threshold", low_batt_shutdown_threshold_);
    get_parameter("use_obstacle_avoidance", use_obstacle_avoidance_);

    apply_namespace_to_topic_names_();
    RCLCPP_INFO(get_logger(), "Finished reading ROS parameters!");
}
