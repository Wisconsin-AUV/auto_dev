#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <path_planning/msg/odometry_simple.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#define EPSILON 1e-6
#define KI_DISTANCE_FACTOR 1e1

/*
There's two different PID controllers used here:
-One for xyz control
-One for yaw control
*/

//TODO: Change OdometrySimple message to have different modes for either pose control or velocity control

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr path_planning_node = std::make_shared<rclcpp::Node>("path_planning");

	//The constants to use for the different PID calculations
	double POSITION_KP = path_planning_node->declare_parameter<double>("position_kp", 1.3612832);
	double POSITION_KI = path_planning_node->declare_parameter<double>("position_ki", 0.0011702376);
	double POSITION_KD = path_planning_node->declare_parameter<double>("position_kd", 0.13313131);
	double ORIENTATION_KP = path_planning_node->declare_parameter<double>("orientation_kp", 1.0574531);
	double ORIENTATION_KI = path_planning_node->declare_parameter<double>("orientation_ki", 1.0799867e-06);
	double ORIENTATION_KD = path_planning_node->declare_parameter<double>("orientation_kd", 0.19184868);

	//Callback to update the pid constants using ros2 param
	auto parameter_callback = path_planning_node->add_on_set_parameters_callback(
			[&POSITION_KP, &POSITION_KI, &POSITION_KD, &ORIENTATION_KP, &ORIENTATION_KI, &ORIENTATION_KD](
            const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto &param : parameters) {
                if (param.get_name() == "position_kp") {
                    POSITION_KP = param.as_double();
                } else if (param.get_name() == "position_ki") {
                    POSITION_KI = param.as_double();
                } else if (param.get_name() == "position_kd") {
                    POSITION_KD = param.as_double();
                } else if (param.get_name() == "orientation_kp") {
                    ORIENTATION_KP = param.as_double();
                } else if (param.get_name() == "orientation_ki") {
                    ORIENTATION_KI = param.as_double();
                } else if (param.get_name() == "orientation_kd") {
                    ORIENTATION_KD = param.as_double();
                }
            }

            return result;
        });

	//Messages for the current and previous odometry of the robot
	nav_msgs::msg::Odometry previous_odometry_message = nav_msgs::msg::Odometry();
	nav_msgs::msg::Odometry odometry_message = previous_odometry_message;

	//The error integral for all 9 variables (3 xyz, yaw)
	double error_integrals[4] = {0.0, 0.0, 0.0, 0.0};

	//The end pose we want the robot to be in
	path_planning::msg::OdometrySimple goal_odometry_message = path_planning::msg::OdometrySimple();

	//Used to keep track of if odometry is updated so we don't recalculate control for the same state
	bool odometry_updated = false;

	//Updates the odometry the robot is in
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription = path_planning_node->create_subscription<nav_msgs::msg::Odometry>("/mavros/local_position/odom", rclcpp::SystemDefaultsQoS(),[&previous_odometry_message, &odometry_message, &odometry_updated](const nav_msgs::msg::Odometry &msg){
		previous_odometry_message = odometry_message;
		odometry_message = msg;
		odometry_updated = true;
	});

	//Updates the end odometry we want to be in
	rclcpp::Subscription<path_planning::msg::OdometrySimple>::SharedPtr goal_odometry_subscription = path_planning_node->create_subscription<path_planning::msg::OdometrySimple>("/wauv/goal_odometry", rclcpp::SystemDefaultsQoS(),[&goal_odometry_message, &error_integrals](const path_planning::msg::OdometrySimple &msg){
		if(msg != goal_odometry_message) {
			for(int i = 0; i < 4; ++i) {
				error_integrals[i] = 0.0;
			}
		}

                goal_odometry_message = msg;
        });

	//Publisher for the linear and angular velocity control commands
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_velocity_publisher = path_planning_node->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", rclcpp::SystemDefaultsQoS());

	//Runs this publisher function at 50 hz
	auto setpoint_velocity_publisher_timer = path_planning_node->create_wall_timer(std::chrono::milliseconds(2), [&path_planning_node, &setpoint_velocity_publisher, &previous_odometry_message, &odometry_message, &goal_odometry_message, &error_integrals, &odometry_updated, &POSITION_KP, &POSITION_KI, &POSITION_KD, &ORIENTATION_KP, &ORIENTATION_KI, &ORIENTATION_KD](){
		//If there's no new odometry, don't recalculate control (especially since then integrals would be double added)
		if(!odometry_updated) {
			return;
		}

		geometry_msgs::msg::TwistStamped setpoint_velocity_message = geometry_msgs::msg::TwistStamped();
		setpoint_velocity_message.header = std_msgs::msg::Header();
		setpoint_velocity_message.header.stamp = path_planning_node->get_clock()->now();
		setpoint_velocity_message.header.frame_id = "global";

		//TODO: Change this once OdometrySimple is changed
		//If the goal odometry message contains any values in the velocity fields, then we will use those velocities (e.g. so we can spin through gate task) instead of applying PID calculations
		if(goal_odometry_message.twist.linear.x != 0 || goal_odometry_message.twist.linear.y != 0 || goal_odometry_message.twist.linear.z != 0 ||
		goal_odometry_message.twist.angular.x != 0 || goal_odometry_message.twist.angular.y != 0 || goal_odometry_message.twist.angular.z != 0) {
			setpoint_velocity_message.twist = goal_odometry_message.twist;
			setpoint_velocity_publisher->publish(setpoint_velocity_message);
			return;
		}

		setpoint_velocity_message.twist = geometry_msgs::msg::Twist();
		setpoint_velocity_message.twist.linear = geometry_msgs::msg::Vector3();
		setpoint_velocity_message.twist.angular = geometry_msgs::msg::Vector3();

		//Shorter variable names for each of the poses
		geometry_msgs::msg::Pose go_pose = goal_odometry_message.pose;
		geometry_msgs::msg::Pose po_pose = previous_odometry_message.pose.pose;
		geometry_msgs::msg::Pose o_pose = odometry_message.pose.pose;
		//The time elapsed from previous odometry to current odometry
		double dt = std::max(EPSILON, (rclcpp::Duration{rclcpp::Time{odometry_message.header.stamp} - rclcpp::Time{previous_odometry_message.header.stamp}}).seconds());

		//Quaternions of each of the odometries
		tf2::Quaternion goal_quaternion = tf2::Quaternion(go_pose.orientation.x, go_pose.orientation.y, go_pose.orientation.z, go_pose.orientation.w);
		tf2::Quaternion current_quaternion = tf2::Quaternion(o_pose.orientation.x, o_pose.orientation.y, o_pose.orientation.z, o_pose.orientation.w);
		tf2::Quaternion previous_quaternion = tf2::Quaternion(po_pose.orientation.x, po_pose.orientation.y, po_pose.orientation.z, po_pose.orientation.w);
		//Calculates the rotation matrix from current odometry to goal odometry
		tf2::Matrix3x3 delta_rotation_matrix = tf2::Matrix3x3(goal_quaternion * current_quaternion.inverse());
		//Calculates the rotation matrix from previous odometry to goal odometry
		tf2::Matrix3x3 previous_delta_rotation_matrix = tf2::Matrix3x3(goal_quaternion * previous_quaternion.inverse());
		//Gets the difference in roll, putch, and yaw based on the rotation matrices. Only need yaw, but getRPY requires three variables to work
		double d_roll, d_pitch, d_yaw, pd_roll, pd_pitch, pd_yaw;
		delta_rotation_matrix.getRPY(d_roll, d_pitch, d_yaw);
		previous_delta_rotation_matrix.getRPY(pd_roll, pd_pitch, pd_yaw);

		//Decreases the how much the erorr integrals increase based on the inverse distance, to prevent the startup adding a ton to error integrals and messing them up
		double distance = std::sqrt((go_pose.position.x - o_pose.position.x) * (go_pose.position.x - o_pose.position.x) + (go_pose.position.y - o_pose.position.y) * (go_pose.position.y - o_pose.position.y) + (go_pose.position.z - po_pose.position.z) * (go_pose.position.z - po_pose.position.z));
		double integral_factor = std::min(KI_DISTANCE_FACTOR, 1.0 / distance);

		error_integrals[0] += POSITION_KI * (go_pose.position.x - po_pose.position.x) * dt * integral_factor;
		error_integrals[1] += POSITION_KI * (go_pose.position.y - po_pose.position.y) * dt * integral_factor;
		error_integrals[2] += POSITION_KI * (go_pose.position.z - po_pose.position.z) * dt * integral_factor;
		error_integrals[3] += ORIENTATION_KI * (pd_yaw) * dt * integral_factor;

		//PID 1
		setpoint_velocity_message.twist.linear.x = POSITION_KP * (go_pose.position.x - o_pose.position.x) + error_integrals[0] + POSITION_KD * ((go_pose.position.x - o_pose.position.x) - (go_pose.position.x - po_pose.position.x)) / dt;
		setpoint_velocity_message.twist.linear.y = POSITION_KP * (go_pose.position.y - o_pose.position.y) + error_integrals[1] + POSITION_KD * ((go_pose.position.y - o_pose.position.y) - (go_pose.position.y - po_pose.position.y)) / dt;
		setpoint_velocity_message.twist.linear.z = POSITION_KP * (go_pose.position.z - o_pose.position.z) + error_integrals[2] + POSITION_KD * ((go_pose.position.z - o_pose.position.z) - (go_pose.position.z - po_pose.position.z)) / dt;
		setpoint_velocity_message.twist.angular.z = ORIENTATION_KP * d_yaw + error_integrals[3] + ORIENTATION_KD * (d_yaw - pd_yaw) / dt;

		odometry_updated = false;
		setpoint_velocity_publisher->publish(setpoint_velocity_message);
	});

	rclcpp::spin(path_planning_node);

	rclcpp::shutdown();
	return 0;
}
