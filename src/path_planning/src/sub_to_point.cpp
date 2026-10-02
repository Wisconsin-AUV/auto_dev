#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

int main(int argc, char ** argv) {
	double goal_position[7] = {atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7])};

	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr path_planning_node = std::make_shared<rclcpp::Node>("path_planning");

	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_velocity_publisher = path_planning_node->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", rclcpp::SystemDefaultsQoS());

	auto setpoint_velocity_publisher_timer = path_planning_node->create_wall_timer(std::chrono::milliseconds(5), [path_planning_node, setpoint_velocity_publisher](){
		geometry_msgs::msg::TwistStamped setpoint_velocity_message = geometry_msgs::msg::TwistStamped();

		setpoint_velocity_publisher->publish(setpoint_velocity_message);
	});

	//Previous iteration of velocity publishing, just a simple PID controller
	/*auto setpoint_velocity_publisher_timer = path_planning_node->create_wall_timer(std::chrono::milliseconds(5), [path_planning_node, setpoint_velocity_publisher, &global_position_local_message, &previous_global_position_local_message, &setpoint_position_global_message]() {
		geometry_msgs::msg::TwistStamped setpoint_velocity_message = geometry_msgs::msg::TwistStamped();

                setpoint_velocity_message.header = std_msgs::msg::Header();
                setpoint_velocity_message.header.stamp = path_planning_node->get_clock()->now();
                setpoint_velocity_message.header.frame_id = "global";

                setpoint_velocity_message.twist = geometry_msgs::msg::Twist();
                setpoint_velocity_message.twist.linear = geometry_msgs::msg::Vector3();
                
		setpoint_velocity_message.twist.linear.x = setpoint_position_global_message.pose.position.longitude - global_position_local_message.pose.pose.position.x;
                setpoint_velocity_message.twist.linear.y = setpoint_position_global_message.pose.position.latitude - global_position_local_message.pose.pose.position.y;
                setpoint_velocity_message.twist.linear.z = setpoint_position_global_message.pose.position.altitude - global_position_local_message.pose.pose.position.z;
		
		setpoint_velocity_message.twist.linear.x = kp_linear_xy * setpoint_velocity_message.twist.linear.x + kd_linear_xy * (global_position_local_message.pose.pose.position.x - previous_global_position_local_message.pose.pose.position.x) / std::max(epsilon, ((global_position_local_message.header.stamp.sec - previous_global_position_local_message.header.stamp.sec) + ((global_position_local_message.header.stamp.nanosec - previous_global_position_local_message.header.stamp.nanosec) / 1000000000.0)));
	       setpoint_velocity_message.twist.linear.y = kp_linear_xy * setpoint_velocity_message.twist.linear.y + kd_linear_xy * (global_position_local_message.pose.pose.position.y - previous_global_position_local_message.pose.pose.position.y) / std::max(epsilon, ((global_position_local_message.header.stamp.sec - previous_global_position_local_message.header.stamp.sec) + ((global_position_local_message.header.stamp.nanosec - previous_global_position_local_message.header.stamp.nanosec) / 1000000000.0)));
	       setpoint_velocity_message.twist.linear.z = kp_linear_z * setpoint_velocity_message.twist.linear.z + kd_linear_z * (global_position_local_message.pose.pose.position.z - previous_global_position_local_message.pose.pose.position.z) / std::max(epsilon, ((global_position_local_message.header.stamp.sec - previous_global_position_local_message.header.stamp.sec) + ((global_position_local_message.header.stamp.nanosec - previous_global_position_local_message.header.stamp.nanosec) / 1000000000.0)));

                setpoint_velocity_message.twist.angular = geometry_msgs::msg::Vector3();

		tf2::Quaternion setpoint_position_global_quaternion = tf2::Quaternion(setpoint_position_global_message.pose.orientation.x, setpoint_position_global_message.pose.orientation.y, setpoint_position_global_message.pose.orientation.z, setpoint_position_global_message.pose.orientation.w);
		tf2::Quaternion global_position_local_quaternion = tf2::Quaternion(global_position_local_message.pose.pose.orientation.x, global_position_local_message.pose.pose.orientation.y, global_position_local_message.pose.pose.orientation.z, global_position_local_message.pose.pose.orientation.w);
		tf2::Matrix3x3 delta_rotation_matrix = tf2::Matrix3x3(setpoint_position_global_quaternion * global_position_local_quaternion.inverse());
		double delta_roll, delta_pitch, delta_yaw;
		delta_rotation_matrix.getRPY(delta_roll, delta_pitch, delta_yaw);

		setpoint_velocity_message.twist.angular.x = delta_roll;
		setpoint_velocity_message.twist.angular.y = delta_pitch;
                setpoint_velocity_message.twist.angular.z = delta_yaw;

		tf2::Quaternion previous_global_position_local_quaternion = tf2::Quaternion(previous_global_position_local_message.pose.pose.orientation.x, previous_global_position_local_message.pose.pose.orientation.y, previous_global_position_local_message.pose.pose.orientation.z, previous_global_position_local_message.pose.pose.orientation.w);
                tf2::Matrix3x3 previous_delta_rotation_matrix = tf2::Matrix3x3(setpoint_position_global_quaternion * previous_global_position_local_quaternion.inverse());
                double previous_delta_roll, previous_delta_pitch, previous_delta_yaw;
                previous_delta_rotation_matrix.getRPY(previous_delta_roll, previous_delta_pitch, previous_delta_yaw);

		
		setpoint_velocity_message.twist.angular.x = kp_angular_xy * setpoint_velocity_message.twist.angular.x + kd_angular_xy * (delta_roll - previous_delta_roll) / std::max(epsilon, ((global_position_local_message.header.stamp.sec - previous_global_position_local_message.header.stamp.sec) + ((global_position_local_message.header.stamp.nanosec - previous_global_position_local_message.header.stamp.nanosec) / 1000000000.0)));
		setpoint_velocity_message.twist.angular.y = kp_angular_xy * setpoint_velocity_message.twist.angular.y + kd_angular_xy * (delta_pitch - previous_delta_pitch) / std::max(epsilon, ((global_position_local_message.header.stamp.sec - previous_global_position_local_message.header.stamp.sec) + ((global_position_local_message.header.stamp.nanosec - previous_global_position_local_message.header.stamp.nanosec) / 1000000000.0)));
		setpoint_velocity_message.twist.angular.z = kp_angular_z * setpoint_velocity_message.twist.angular.z + kd_angular_z * (delta_yaw - previous_delta_yaw) / std::max(epsilon, ((global_position_local_message.header.stamp.sec - previous_global_position_local_message.header.stamp.sec) + ((global_position_local_message.header.stamp.nanosec - previous_global_position_local_message.header.stamp.nanosec) / 1000000000.0)));


                setpoint_velocity_publisher->publish(setpoint_velocity_message);
	});*/

	rclcpp::spin(path_planning_node);

	rclcpp::shutdown();
	return 0;
}
