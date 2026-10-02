#include <path_planning/sub_movement_modeling.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

void func(rclcpp::Node::SharedPtr sub_to_point_node) {
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription = sub_to_point_node->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", rclcpp::SystemDefaultsQoS(),[](const nav_msgs::msg::Odometry &msg){

	});

	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr commanded_velocity_subscription = sub_to_point_node->create_subscription<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", rclcpp::SystemDefaultsQoS(),[](const geometry_msgs::msg::TwistStamped &msg){

        });
}
