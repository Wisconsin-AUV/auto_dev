#include <string>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#define EPSILON 1e-6
#define GATE_SCORE_THRESHOLD 0.75
#define GATE_DISTANCE_THRESHOLD 0.15

int main(int argc, char ** argv) {
        rclcpp::init(argc, argv);
        rclcpp::Node::SharedPtr gate_testing_node = std::make_shared<rclcpp::Node>("gate_testing");
	//The constants to use for the different PID calculations
	double POSITION_KP = gate_testing_node->declare_parameter<double>("position_kp", 1.3612832);
	double POSITION_KI = gate_testing_node->declare_parameter<double>("position_ki", 0.0011702376);
	double POSITION_KD = gate_testing_node->declare_parameter<double>("position_kd", 0.13313131);
	double ORIENTATION_KP = gate_testing_node->declare_parameter<double>("orientation_kp", 1.0574531);
	double ORIENTATION_KI = gate_testing_node->declare_parameter<double>("orientation_ki", 1.0799867e-06);
	double ORIENTATION_KD = gate_testing_node->declare_parameter<double>("orientation_kd", 0.19184868);
	double GOAL_X = gate_testing_node->declare_parameter<double>("goal_x", -1.0);
	double GOAL_Y = gate_testing_node->declare_parameter<double>("goal_y", -1.0);
	double GOAL_Z = gate_testing_node->declare_parameter<double>("goal_z", -1.0);

	bool at_goal = false;

	//The error integral for all 4 variables (3 xyz, yaw)
        double error_integrals[4] = {0.0, 0.0, 0.0, 0.0};

	//Callback to update the pid constants using ros2 param
	auto parameter_callback = gate_testing_node->add_on_set_parameters_callback(
			[&POSITION_KP, &POSITION_KI, &POSITION_KD, &ORIENTATION_KP, &ORIENTATION_KI, &ORIENTATION_KD, &GOAL_X, &GOAL_Y, &GOAL_Z, &error_integrals](
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
                } else if(param.get_name() == "goal_x") {
			GOAL_X = param.as_double();
		} else if(param.get_name() == "goal_y") {
                        GOAL_Y = param.as_double();
                } else if(param.get_name() == "goal_z") {
                        GOAL_Z = param.as_double();
                }

		for(int i = 0; i < 4; ++i) {
			error_integrals[i] = 0.0;
		}
            }

            return result;
        });

	//Messages for the current and previous detections of the robot
	vision_msgs::msg::Detection3D previous_detections_message = vision_msgs::msg::Detection3D();
	vision_msgs::msg::Detection3D detections_message = previous_detections_message;

	//Used to keep track of if detections is updated so we don't recalculate control for the same state
	bool gate_updated = false;

	//Keeps track of last ten distances we were from goal
	double previous_distances[10] = {1e10, 1e10, 1e10, 1e10, 1e10, 1e10, 1e10, 1e10, 1e10, 1e10};

	//Updates the detections the robot is in
	rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detections_subscription = gate_testing_node->create_subscription<vision_msgs::msg::Detection3DArray>("/wauv/perception/objects", rclcpp::SystemDefaultsQoS(),[&previous_detections_message, &detections_message, &gate_updated](const vision_msgs::msg::Detection3DArray &msg){
		vision_msgs::msg::Detection3D best_gate_detection = vision_msgs::msg::Detection3D();
		double best_gate_score = -1.0;

		for(const auto &detection : msg.detections) {
			for(const auto &result : detection.results) {
        			if(result.hypothesis.class_id == "gate" && result.hypothesis.score > best_gate_score) {
        		    		best_gate_score = result.hypothesis.score;
	        	    		best_gate_detection = detection;
        			}
    			}
		}

		if(best_gate_score > GATE_SCORE_THRESHOLD) {
			previous_detections_message = detections_message;
                        detections_message = best_gate_detection;
                        gate_updated = true;
		}
	});

	//Publisher for the linear and angular velocity control commands
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_velocity_publisher = gate_testing_node->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", rclcpp::SystemDefaultsQoS());

	//Runs this publisher function at 50 hz
	auto setpoint_velocity_publisher_timer = gate_testing_node->create_wall_timer(std::chrono::milliseconds(20), [&gate_testing_node, &setpoint_velocity_publisher, &previous_detections_message, &detections_message, &error_integrals, &gate_updated, &POSITION_KP, &POSITION_KI, &POSITION_KD, &ORIENTATION_KP, &ORIENTATION_KI, &ORIENTATION_KD, &GOAL_X, &GOAL_Y, &GOAL_Z, &previous_distances, &at_goal](){
		//If there's no new detections, don't recalculate control (especially since then integrals would be double added)
		if(!gate_updated || at_goal) {
			return;
		}

		geometry_msgs::msg::TwistStamped setpoint_velocity_message = geometry_msgs::msg::TwistStamped();
		setpoint_velocity_message.header = std_msgs::msg::Header();
		setpoint_velocity_message.header.stamp = gate_testing_node->get_clock()->now();
		setpoint_velocity_message.header.frame_id = "global";

		setpoint_velocity_message.twist = geometry_msgs::msg::Twist();
		setpoint_velocity_message.twist.linear = geometry_msgs::msg::Vector3();
		setpoint_velocity_message.twist.angular = geometry_msgs::msg::Vector3();

		//Shorter variable names for each of the poses
		geometry_msgs::msg::Pose po_pose = previous_detections_message.bbox.center;
		geometry_msgs::msg::Pose o_pose = detections_message.bbox.center;
		//The time elapsed from previous detections to current detections
		double dt = std::max(EPSILON, (rclcpp::Duration{rclcpp::Time{detections_message.header.stamp} - rclcpp::Time{previous_detections_message.header.stamp}}).seconds());

		//Decreases the how much the error integrals increase based on the inverse distance, to prevent the startup adding a ton to error integrals and messing them up
		double distance = std::sqrt((GOAL_X - o_pose.position.x) * (GOAL_X - o_pose.position.x) + (GOAL_Y - o_pose.position.y) * (GOAL_Y - o_pose.position.y) + (GOAL_Z - po_pose.position.z) * (GOAL_Z - po_pose.position.z));
		double max_distance = distance;

		for(int i = 0; i < 9; ++i) {
			previous_distances[i] = previous_distances[i + 1];

			if(previous_distances[i] > max_distance) {
				max_distance = previous_distances[i];
			}
		}

		previous_distances[9] = distance;

		if(max_distance < GATE_DISTANCE_THRESHOLD) {
			at_goal = true;
			RCLCPP_INFO(gate_testing_node->get_logger(),"Gate alignment complete!");

			if(std::abs(o_pose.position.x) > std::abs(o_pose.position.y)) {
				setpoint_velocity_message.twist.linear.x = o_pose.position.x > 0 ? 1.0 : -1.0;
			} else {
				setpoint_velocity_message.twist.linear.y = o_pose.position.y > 0 ? 1.0 : -1.0;
			}

			setpoint_velocity_publisher->publish(setpoint_velocity_message);

			return;
		}

		error_integrals[0] += POSITION_KI * (GOAL_X - po_pose.position.x) * dt;
		error_integrals[1] += POSITION_KI * (GOAL_Y - po_pose.position.y) * dt;
		error_integrals[2] += POSITION_KI * (GOAL_Z - po_pose.position.z) * dt;
		//error_integrals[3] += ORIENTATION_KI * (pd_yaw) * dt;

		//PID 1
		setpoint_velocity_message.twist.linear.x = POSITION_KP * (GOAL_X - o_pose.position.x) + error_integrals[0] + POSITION_KD * ((GOAL_X - o_pose.position.x) - (GOAL_X - po_pose.position.x)) / dt;
		setpoint_velocity_message.twist.linear.y = POSITION_KP * (GOAL_Y - o_pose.position.y) + error_integrals[1] + POSITION_KD * ((GOAL_Y - o_pose.position.y) - (GOAL_Y - po_pose.position.y)) / dt;
		setpoint_velocity_message.twist.linear.z = POSITION_KP * (GOAL_Z - o_pose.position.z) + error_integrals[2] + POSITION_KD * ((GOAL_Z - o_pose.position.z) - (GOAL_Z - po_pose.position.z)) / dt;
		//setpoint_velocity_message.twist.angular.z = ORIENTATION_KP * d_yaw + error_integrals[3] + ORIENTATION_KD * (d_yaw - pd_yaw) / dt;

		gate_updated = false;
		setpoint_velocity_publisher->publish(setpoint_velocity_message);
	});

	rclcpp::spin(gate_testing_node);
        rclcpp::shutdown();
        return 0;
}
