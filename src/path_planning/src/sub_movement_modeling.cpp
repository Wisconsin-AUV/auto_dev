#include <atomic>

#include <path_planning/sub_movement_modeling.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

void func(rclcpp::Node::SharedPtr path_planning_node) {

	nav_msgs::msg::Odometry previous_odometry_message = nav_msgs::msg::Odometry();
	nav_msgs::msg::Odometry odometry_message = previous_odometry_message;
	geometry_msgs::msg::TwistStamped commanded_velocity_message = geometry_msgs::msg::TwistStamped();

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription = path_planning_node->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", rclcpp::SystemDefaultsQoS(),[&previous_odometry_message, &odometry_message, &commanded_velocity_message](const nav_msgs::msg::Odometry &msg){
		if(odometry_message == previous_odometry_message) {
			odometry_message = msg;
			return;
		}

		previous_odometry_message = odometry_message;
		odometry_message = msg;

		float delta_time = std::fmax(DELTA_TIME_EPSILON, (rclcpp::Time(odometry_message.header.stamp) - rclcpp::Time(previous_odometry_message.header.stamp)).seconds());
		tf2::Quaternion current_orientation = tf2::Quaternion(odometry_message.pose.pose.orientation.x, odometry_message.pose.pose.orientation.y, odometry_message.pose.pose.orientation.z, odometry_message.pose.pose.orientation.w);
		tf2::Quaternion previous_orientation = tf2::Quaternion(previous_odometry_message.pose.pose.orientation.x, previous_odometry_message.pose.pose.orientation.y, previous_odometry_message.pose.pose.orientation.z, previous_odometry_message.pose.pose.orientation.w);
		tf2::Quaternion delta_orientation = current_orientation * previous_orientation.inverse();

		float function_inputs[INPUT_BITS] = {
			current_orientation.getX(), current_orientation.getY(), current_orientation.getZ(), current_orientation.getW(),
			odometry_message.twist.twist.linear.x, odometry_message.twist.twist.linear.y, odometry_message.twist.twist.linear.z,
			odometry_message.twist.twist.angular.x, odometry_message.twist.twist.angular.y, odometry_message.twist.twist.angular.z,
			(odometry_message.twist.twist.linear.x - previous_odometry_message.twist.twist.linear.x) / delta_time, (odometry_message.twist.twist.linear.y - previous_odometry_message.twist.twist.linear.y) / delta_time, (odometry_message.twist.twist.linear.z - previous_odometry_message.twist.twist.linear.z) / delta_time,
			(odometry_message.twist.twist.angular.x - previous_odometry_message.twist.twist.angular.x) / delta_time, (odometry_message.twist.twist.angular.y - previous_odometry_message.twist.twist.angular.y) / delta_time, (odometry_message.twist.twist.angular.z - previous_odometry_message.twist.twist.angular.z) / delta_time,
			commanded_velocity_message.twist.linear.x, commanded_velocity_message.twist.linear.y, commanded_velocity_message.twist.linear.z,
			commanded_velocity_message.twist.angular.x, commanded_velocity_message.twist.angular.y, commanded_velocity_message.twist.angular.y,
			delta_time
		};

		float function_outputs[OUTPUT_BITS] = {
			previous_odometry_message.twist.twist.linear.x, previous_odometry_message.twist.twist.linear.y, previous_odometry_message.twist.twist.linear.z,
                        previous_odometry_message.twist.twist.angular.x, previous_odometry_message.twist.twist.angular.y, previous_odometry_message.twist.twist.angular.z,
			odometry_message.pose.pose.position.x - previous_odometry_message.pose.pose.position.x, odometry_message.pose.pose.position.y - previous_odometry_message.pose.pose.position.y, odometry_message.pose.pose.position.z - previous_odometry_message.pose.pose.position.z,
			delta_orientation.getX(), delta_orientation.getY(), delta_orientation.getZ(), delta_orientation.getW()
		};

		std::bitset<INPUT_BITS> function_inputs_bits = calc_inputs_to_bits(function_inputs);
		std::bitset<OUTPUT_BITS> function_outputs_bits = calc_outputs_to_bits(function_outputs);
	});

	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr commanded_velocity_subscription = path_planning_node->create_subscription<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", rclcpp::SystemDefaultsQoS(),[&commanded_velocity_message](const geometry_msgs::msg::TwistStamped &msg){
		commanded_velocity_message = msg;
	});
}
