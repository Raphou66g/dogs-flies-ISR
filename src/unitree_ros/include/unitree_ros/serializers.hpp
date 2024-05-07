#ifndef SERIALIZERS_HPP
#define SERIALIZERS_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree_ros/common_defines.hpp>
#include <unitree_ros/msg/bms_state.hpp>
#include <unitree_ros/msg/sensor_ranges.hpp>

#include "unitree_legged_sdk/comm.h"

void serialize(nav_msgs::msg::Odometry& msg, const odom_t odom);
void serialize(sensor_msgs::msg::Imu& msg, const UNITREE_LEGGED_SDK::IMU imu);
void serialize(unitree_ros::msg::BmsState& msg, const UNITREE_LEGGED_SDK::BmsState bms);
void serialize(unitree_ros::msg::SensorRanges& msg, const sensor_ranges_t ranges);

#endif  // !#ifndef SERIALIZERS_HPP
