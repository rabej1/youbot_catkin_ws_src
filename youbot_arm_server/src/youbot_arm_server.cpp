#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <angles/angles.h>

#include <actionlib/server/simple_action_server.h>
#include <youbot_arm_server/YouBotGoalAction.h>
#include <actionlib/client/simple_action_client.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <boost/bind.hpp>

double pi = 3.1415926;
double joint1_val = 1.06109995904;
double joint2_val = 1.69981100958;
double joint3_val = -2.38776596459;
double joint4_val = 1.87102626524;
double joint5_val = 5.42331512024;

//double alpha = 0;
//double beta = pi/4;
//double gm = 0;
//bool proceed = false;
//const double time_factor = 1;

//Robot specific data, from URDF file
const double l1 = 0.155;
const double l2 = 0.135;
const double l3 = 0.130;
const int no_joints = 5;

//const std::string base_footprint = "base_footprint";
//const std::string arm_link_2 = "arm_link_2";
//const std::string arm_link_1 = "arm_link_1";

ros::Publisher joint_pub_;
ros::Publisher gripp_pub_;

//tf::StampedTransform tf_base_arm_1;
//tf::StampedTransform tf_arm_1_2;

//static const std::string arr[] = { "arm_joint_1", "arm_joint_2", "arm_joint_3","arm_joint_4", "arm_joint_5" };
//std::vector<std::string> joint_names(arr, arr + sizeof(arr) / sizeof(arr[0]));

static const double arr_offset[] = { 2.967, 1.134, -2.5485, 1.7885, 1.2 };
std::vector<double> joint_offset(arr_offset,
		arr_offset + sizeof(arr_offset) / sizeof(arr_offset[0]));

static const double arr_upper_limits[] = { 5.84, 2.617, -0.15, 3.429, 5.641 };
static const double arr_lower_limits[] = { 0.01007, 0.01007, -5.0266, 0.02213,
		0.1107 };

static const double base_to_arm1[] = {0.167, 0, 0.142};
static const double arm1_to_arm2[] = {0.033, 0, 0.019};

std::vector<double> upper_limits(arr_upper_limits,
		arr_upper_limits
		+ sizeof(arr_upper_limits) / sizeof(arr_upper_limits[0]));
std::vector<double> lower_limits(arr_lower_limits,
		arr_lower_limits
		+ sizeof(arr_lower_limits) / sizeof(arr_lower_limits[0]));

tf::TransformListener *listener;
tf::TransformBroadcaster *broadcaster;
//std::string frame_target = "/arm_link_1";
//std::string frame_int = "/arm_link_0";
std::string frame_source = "/base_footprint";

typedef actionlib::SimpleActionServer<youbot_arm_server::YouBotGoalAction> Server;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

typedef std::vector<double> JointAngles;
typedef std::pair<JointAngles, JointAngles> JointAnglesPair;

std::string action_name_;

JointAngles joint_states;

youbot_arm_server::YouBotGoalFeedback feedback_;
youbot_arm_server::YouBotGoalResult result_;


/*
 * Callback to read in Joint States
 */

void read_joint_stateCB(const sensor_msgs::JointState &states){
	joint_states = states.position;
}


/*
 * Calculate rotation matrix from euler zyx
 */
Eigen::Matrix3d euler_zyx(const double alpha, // around z
		const double beta, // around y
		const double gm)  // around x
{
	const double c1 = cos(alpha);
	const double s1 = sin(alpha);
	const double c2 = cos(beta);
	const double s2 = sin(beta);
	const double c3 = cos(gm);
	const double s3 = sin(gm);

	Eigen::Matrix3d rot;
	rot << c1 * c2, c1 * s2 * s3 - c3 * s1, s1 * s3 + c1 * c3 * s2, c2 * s1, c1
			* c3 + s1 * s2 * s3, c3 * s1 * s2 - c1 * s3, -s2, c2 * s3, c2 * c3;
	return rot;
}

/*
 * Calculate rotation matrix from euler zyx
 */
Eigen::Matrix3d euler_zyx(const Eigen::Vector3d &angle) {
	return euler_zyx(static_cast<double>(angle(0)),
			static_cast<double>(angle(1)), static_cast<double>(angle(2)));
}

Eigen::Vector3d inv_orientation_kuka(const Eigen::Matrix3d &rot) {
	// order of angles is 1st 1st element 2nd 2nd element 3rd 3rd element
	// specialization of inv_euler_zyx
	// where gimbal lock is resolved and angle (2) is used angle (0) is 0
	Eigen::Vector3d angle;
	const double c2 = sqrt(
			static_cast<double>(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));

	if (fabs(c2) > 1e-6) {
		angle(0) = atan2(static_cast<double>(rot(1, 0)) / c2,
				static_cast<double>(rot(0, 0)) / c2);
		angle(1) = atan2(-static_cast<double>(rot(2, 0)), c2);
		angle(2) = atan2(static_cast<double>(rot(2, 1)) / c2,
				static_cast<double>(rot(2, 2)) / c2);
	} else {
		// we set angle (0) to zero
		angle(0) = 0.0;
		angle(2) = 0.0;
		const double s1 = 1;
		const double c1 = 0;
		// with:
		// c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2,
		// c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3,
		// -s2, c2*s3, c2*c3;
		// yields a rotation matrix:
		// 0,   -c3, s3,
		// c2,  s2*s3, c3*s2,
		// -s2, c2*s3, c2*c3;

		// sign of angle (1) is unknown +/- 90 deg.
		angle(1) = -(static_cast<double>(-rot(2, 0)) > 0) ? M_PI_2 : -M_PI_2;
		//angle (2) = atan2 (static_cast <double> (rot (0,2)), -static_cast <double> (rot (0,1)));
		//angle (2) = atan2 (static_cast <double> (-rot (1,1) / rot (2,0)),
		//		static_cast <double> (-rot (1,2) / rot (2,0))) +  M_PI_2;
		const double psi13 = atan2(static_cast<double>(-rot(0, 1)),
				static_cast<double>(rot(1, 1)));
		if (rot(2, 0) < 0) {
			angle(2) = -psi13;
			//angle (0) = psi13;
		} else {
			angle(2) = psi13;
			//angle (0) = -psi13;
		}
		std::cout << rot(0, 1) << ',' << rot(1, 1) << " -> " << angle(2);
		std::cout << std::endl;
	}

	return angle;
}

/*
 * Convert rotation matrix to euler angles
 */
Eigen::Vector3d inv_euler_zyx(const Eigen::Matrix3d &rot) {
	// order of angles is 1st 1st element 2nd 2nd element 3rd 3rd element
	Eigen::Vector3d angle;
	const double c2 = sqrt(
			static_cast<double>(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0)));

	if (fabs(c2) > 1e-6) {
		angle(0) = atan2(static_cast<double>(rot(1, 0)) / c2,
				static_cast<double>(rot(0, 0)) / c2);
		angle(1) = atan2(-static_cast<double>(rot(2, 0)), c2);
		angle(2) = atan2(static_cast<double>(rot(2, 1)) / c2,
				static_cast<double>(rot(2, 2)) / c2);
	} else {
		throw std::invalid_argument("Gimbal lock: cannot determine angle 2");
	}
	return angle;
}

/*
 * Test resulting euler angles
 */
bool test_euler_zyx(const double step = M_PI / 18) {
	bool ok = true;
	size_t count_all = 0;
	size_t count_diverging = 0;
	size_t count_gimbal_lock = 0;
	const double max_diff = 0.01;
	for (double alpha = -M_PI; alpha < M_PI; alpha += step) {
		for (double beta = -M_PI; beta < M_PI; beta += step) {
			for (double gm = -M_PI; gm < M_PI; gm += step) {
				Eigen::Matrix3d rot = euler_zyx(alpha, beta, gm);
				try {
					//Eigen::Vector3d angle = inv_euler_zyx (rot);
					Eigen::Vector3d angle = inv_orientation_kuka(rot);
					Eigen::Matrix3d tmp = euler_zyx(
							static_cast<double>(angle(0)),
							static_cast<double>(angle(1)),
							static_cast<double>(angle(2)));
					//Eigen::Vector3d angle2 = inv_euler_zyx (tmp);
					Eigen::Vector3d angle2 = inv_orientation_kuka(tmp);
					Eigen::Vector3d diff = angle - angle2;

					tmp *= rot.inverse();
					bool fail = (fabs(static_cast<double>(tmp(0, 0)) - 1)
							> max_diff)
																			|| (fabs(static_cast<double>(tmp(1, 1)) - 1)
																					> max_diff)
																					|| (fabs(static_cast<double>(tmp(2, 2)) - 1)
																							> max_diff);
					/*
					 bool fail = (fabs(static_cast <double> (angle (0)) - alpha) > max_diff) ||
					 (fabs(static_cast <double> (angle (1)) - beta) > max_diff) ||
					 (fabs(static_cast <double> (angle (2)) - gm) > max_diff);
					 */

					bool fail2 = (fabs(static_cast<double>(diff(0))) > max_diff)
																			|| (fabs(static_cast<double>(diff(1))) > max_diff)
																			|| (fabs(static_cast<double>(diff(2))) > max_diff);
					fail2 = false;

					if (fail || fail2) {
						std::cout << "---" << std::endl << rot << std::endl
								<< tmp << std::endl;
						std::cout << alpha << ',' << beta << ',' << gm << " vs "
								<< angle(0) << ',' << angle(1) << ','
								<< angle(2);
						std::cout << std::endl;
						++count_diverging;
					}
				} catch (std::invalid_argument &e) {
					std::cout << e.what() << std::endl;
					++count_gimbal_lock;
				}

				++count_all;
			}
		}
	}

	std::cout << "failed " << count_diverging << " out of " << count_all
			<< " checks." << std::endl;
	std::cout << count_gimbal_lock << " gimbal locks" << std::endl;
	return ok;
}

/*
 * Transform poses
 */
/*double transform_pose(const Eigen::Vector3d &source_point,
		const std::string source_frame, const std::string target_frame) {

	tf::StampedTransform tf_source_int;
	tf::StampedTransform tf_int_target;
	double angle;


	try {
		listener->waitForTransform(target_frame, frame_int, ros::Time::now(),
				ros::Duration(1.0));

		listener->lookupTransform(source_frame, frame_int, ros::Time(0),
				tf_source_int);

		listener->waitForTransform(frame_int, source_frame, ros::Time::now(),
				ros::Duration(1.0));

		listener->lookupTransform(frame_int, target_frame, ros::Time(0),
				tf_int_target);

		double offset = tf_int_target.getOrigin().getX()
				+ tf_source_int.getOrigin().getX();

		angle = atan2(source_point.y(), source_point.x() - offset);

	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

	return angle;

}*/

/*
 * Display goal in rviz in tf tree
 */
geometry_msgs::PoseStamped set_goal(youbot_arm_server::YouBotGoalGoalConstPtr goal , const std::string frame_id) {
	geometry_msgs::PoseStamped result_pose;

	//Eigen::Matrix3d rot_mat = euler_zyx(angle);
	//Eigen::Quaternion<double> q(rot_mat);
	//tf::Quaternion quat;
	//tf::quaternionEigenToTF(q, quat);

	result_pose.header.stamp = ros::Time::now();
	result_pose.header.frame_id = frame_id;
	result_pose.pose.position.x = goal->goal.pose.position.x;
	result_pose.pose.position.y = goal->goal.pose.position.y;
	result_pose.pose.position.z = goal->goal.pose.position.z;
	result_pose.pose.orientation.x = goal->goal.pose.orientation.x;
	result_pose.pose.orientation.y = goal->goal.pose.orientation.y;
	result_pose.pose.orientation.z = goal->goal.pose.orientation.z;
	result_pose.pose.orientation.w = goal->goal.pose.orientation.w;

	//broadcaster->sendTransform(
	//		tf::StampedTransform(
	//				tf::Transform(quat,
	//						tf::Vector3(position(0), position(1), position(2))),
	//				ros::Time::now(), "/base_footprint", "/target_link"));

	return result_pose;
}



/*
 * Construct joint message from angles
 */
/*sensor_msgs::JointState create_joint_msg(const std::vector<double> &angles) {

	if (angles.size() != joint_names.size()) {
		throw std::invalid_argument(
				"Something went wrong. Size of angles!= size of joint_names!");
	}

	sensor_msgs::JointState js;

	js.name = joint_names;
	js.position = angles;

	return js;
}*/

/*
 * Get transforms
 */
/*void get_transforms() {

	try {
		//Lookup transform from base_footprint to arm_lnk_0
		listener->waitForTransform(base_footprint, arm_link_1, ros::Time::now(),
				ros::Duration(1.0));

		listener->lookupTransform(base_footprint, arm_link_1, ros::Time(0),
				tf_base_arm_1);

		//Lookup transform from arm_link_0 to arm_link_1
		listener->waitForTransform(arm_link_1, arm_link_2, ros::Time::now(),
				ros::Duration(1.0));

		listener->lookupTransform(arm_link_1, arm_link_2, ros::Time(0),
				tf_arm_1_2);
	}

	catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
}*/

/*
 * Map angles from IK space to KUKA space
 */
std::vector<double> angle_mapping(std::vector<double> angles) {

	std::vector<double> angles_KUKA;
	angles_KUKA.resize(5);
	//for (int ite = 0; ite < 5; ite++){
	//	angles_KUKA.at(ite) = joint_offset.at(ite) +angles.at(ite);
	//}

	if (angles.at(0) < 0) {
		angles_KUKA.at(0) = joint_offset.at(0) + std::fabs(angles.at(0));
	} else {
		angles_KUKA.at(0) = joint_offset.at(0) - std::fabs(angles.at(0));
	}

	if (angles.at(1) < 0) {
		angles_KUKA.at(1) = (joint_offset.at(1) + M_PI / 2) + fabs(angles.at(1));
	} else {
		angles_KUKA.at(1) = (joint_offset.at(1) + M_PI / 2) - fabs(angles.at(1));
	}

	if (angles.at(2) < 0) {
		angles_KUKA.at(2) = joint_offset.at(2) + fabs(angles.at(2));
	} else {
		angles_KUKA.at(2) = joint_offset.at(2) - fabs(angles.at(2));
	}

	if (angles.at(3) < 0) {
		angles_KUKA.at(3) = joint_offset.at(3) + fabs(angles.at(3));
	} else {
		angles_KUKA.at(3) = joint_offset.at(3) - fabs(angles.at(3));
	}

	angles_KUKA.at(4) = joint_offset.at(4) + angles.at(4);

	return angles_KUKA;
}

/*
 * Check if solution does not violate joint limits
 */
bool check_joint_values(const JointAngles &angles) {

	bool valid;

	if (((angles[0] < lower_limits[0]) || (angles[0] > upper_limits[0]))
			|| ((angles[1] < lower_limits[1]) || (angles[1] > upper_limits[1]))
			|| ((angles[2] < lower_limits[2]) || (angles[2] > upper_limits[2]))
			|| ((angles[3] < lower_limits[3]) || (angles[3] > upper_limits[3]))
			|| ((angles[4] < lower_limits[4]) || (angles[4] > upper_limits[4]))) {
		valid = false;
	} else {
		valid = true;
	}

	return valid;
}
/*
geometry_msgs::PoseStamped wrist_transform(geometry_msgs::PoseStamped tooltip_pose, double theta_z){
	geometry_msgs::PoseStamped wrist_pose;

	wrist_pose = tooltip_pose;
	wrist_pose.pose.position.z = tooltip_pose.pose.position.z-(sin(tooltip_pose.pose.orientation.y))*l3;
	wrist_pose.pose.position.x = tooltip_pose.pose.position.x-(cos(tooltip_pose.pose.orientation.y)*cos(theta_z))*l3;
	wrist_pose.pose.position.y = tooltip_pose.pose.position.y-(cos(tooltip_pose.pose.orientation.y)*sin(theta_z))*l3;

	return wrist_pose;
}*/

/*
 * Calculate two IK solutions for given goal
 */
JointAnglesPair ik_youbot_arm(const geometry_msgs::PoseStamped &target_pose) {

	bool valid = false;
	JointAngles result_angles_1;
	JointAngles result_angles_2;
	JointAnglesPair result;
	geometry_msgs::PoseStamped wrist_pose;

	result_angles_1.resize(5);
	result_angles_2.resize(5);

	//tf::Vector3 base_arm_1 = tf_base_arm_1.getOrigin();
	//tf::Vector3 arm_1_2 = tf_arm_1_2.getOrigin();

	//transform goal position in 2D space
	double x = sqrt(pow(target_pose.pose.position.y - base_to_arm1[1]-arm1_to_arm2[1], 2)+ pow(target_pose.pose.position.x - base_to_arm1[0]-arm1_to_arm2[0],2));
	double z = target_pose.pose.position.z - base_to_arm1[2]-arm1_to_arm2[2];

	//transform tooltip position into wrist
	double x_wrist = x - l3 * cos(target_pose.pose.orientation.y);
	double z_wrist = z - l3 * sin(target_pose.pose.orientation.y);

	//get first angle out of Postition
	if (target_pose.pose.position.x  < 0){
		ROS_INFO("Position on back of Robot");
		result_angles_1.at(0) = atan2(
				target_pose.pose.position.y - base_to_arm1[1],
				-(target_pose.pose.position.x - base_to_arm1[0]));
		x_wrist = -x_wrist;
	}else{
		result_angles_1.at(0) = atan2(
				target_pose.pose.position.y - base_to_arm1[1],
				target_pose.pose.position.x - base_to_arm1[0]);
	}

	result_angles_2.at(0) = result_angles_1.at(0);

	//calc second angle
	double phi = atan2(z_wrist,x_wrist);
	double psi = acos((((x_wrist * x_wrist + z_wrist * z_wrist) + l1 * l1 - l2 * l2)) / (2 * l1 * sqrt(x_wrist * x_wrist + z_wrist * z_wrist)));
	result_angles_1.at(1) = atan2(sin(phi + psi),cos(phi + psi));
	result_angles_2.at(1) = atan2(sin(phi - psi),cos(phi - psi));

	//calc third angle
	double gamma = acos((l1*l1+l2*l2-(x_wrist * x_wrist + z_wrist * z_wrist))/(2*l1*l2));
	result_angles_1.at(2) = atan2(sin(M_PI + gamma),cos(M_PI + gamma));
	result_angles_2.at(2) = atan2(sin(M_PI - gamma),cos(M_PI - gamma));

	if (target_pose.pose.position.x  < 0){
		result_angles_1.at(3) = M_PI-target_pose.pose.orientation.y - result_angles_1.at(1) - result_angles_1.at(2);
		result_angles_2.at(3) = M_PI-target_pose.pose.orientation.y - result_angles_2.at(1) - result_angles_2.at(2);
	}else{
		result_angles_1.at(3) = target_pose.pose.orientation.y - result_angles_1.at(1) - result_angles_1.at(2);
		result_angles_2.at(3) = target_pose.pose.orientation.y - result_angles_2.at(1) - result_angles_2.at(2);
	}


	//Check if soulution is valid: Nans = solution not valid
	/*for (std::size_t it = 1; it < 3; ++it) {
		if ((result_angles_1.at(it) != result_angles_1.at(it))
				|| (result_angles_2.at(it) != result_angles_2.at(it))) {
			valid = false;
		} else {
			valid = true;
		}
	}*/
	//set angle of gripper

	result_angles_1.at(4) = target_pose.pose.orientation.z-result_angles_1.at(0)+M_PI/2;
	result_angles_2.at(4) = target_pose.pose.orientation.z-result_angles_2.at(0)+M_PI/2;

	valid = true;

	if (valid) {
		//result.first = result_angles_1;
		//result.second = result_angles_2;
		result.first = angle_mapping(result_angles_1);
		result.second = angle_mapping(result_angles_2);

		if(!check_joint_values(result.first)){
			//			ROS_INFO("first solution cleared");
			result.first.clear();
		}
		if(!check_joint_values(result.second)){
			//			ROS_INFO("second solution cleared");
			result.second.clear();
		}

	} else {
		result.first.clear();
		result.second.clear();
	}
	return result;
}

brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
	brics_actuator::JointPositions msg;

	brics_actuator::JointValue joint;
	joint.timeStamp = ros::Time::now();
	joint.unit = "m";
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);
	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);

	return msg;
}

brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions, int numberOfJoints) {

	brics_actuator::JointPositions msg;

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided

	for (int i = 0; i < numberOfJoints; i++) {
		// Set all values for one joint, i.e. time, name, value and unit
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = "rad";

		// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		joint.joint_uri = jointName.str();

		// add joint to message
		msg.positions.push_back(joint);
	}

	return msg;
}



void executeCB(const youbot_arm_server::YouBotGoalGoalConstPtr &gl,
		Server* as) {

	ros::NodeHandle nh_;
	bool success = false;
	bool cont = true;
	brics_actuator::JointPositions grippermsg;

	//	ROS_INFO("Goal recieved");
	if (gl->gripperposstart >= 0){
		grippermsg = createGripperPositionCommand(gl->gripperposstart);
		gripp_pub_.publish(grippermsg);
		ros::Duration(3).sleep();
	}


	//get_transforms();

	//	ROS_INFO("Desired goal:");
	//	//ROS_INFO("Reference frame is: %s", armTF.frame_id_.c_str());
	//	//ROS_INFO("End effector link is: %s", armTF.child_frame_id_.c_str());
	//	ROS_INFO("x: %f", gl->goal.pose.position.x);
	//	ROS_INFO("y: %f", gl->goal.pose.position.y);
	//	ROS_INFO("z: %f", gl->goal.pose.position.z);
	//	ROS_INFO("theta1: %f", gl->goal.pose.orientation.y);
	//	ROS_INFO("theta2: %f", gl->goal.pose.orientation.z);

	//Set goal from input
	geometry_msgs::PoseStamped goal = set_goal(gl, frame_source);
	// TODO why 1 degree deviation in IK??

	//	ROS_INFO("calculating IK for new goal");
	JointAnglesPair ik_result = ik_youbot_arm(goal);

//	if (ik_result.first.empty() && ik_result.second.empty()) {
//		throw std::invalid_argument("inverse kinematics has no solution");
//		cont = false;
//	}
	if (as->isPreemptRequested() || !ros::ok()) {
		ROS_INFO("%s: Preempted", action_name_.c_str());
		// set the action state to preempted
		as->setPreempted();
		cont = false;
	}

	feedback_.current_pose = goal;
	as->publishFeedback(feedback_);
	if (cont) {
		double result1, result2 = 0;

		JointAngles solution;
		if (ik_result.first.empty()){
			ROS_INFO("first empty");
			if (ik_result.second.empty()){
				ROS_INFO("second empty");
			}else{
				solution = ik_result.second;
			}
		}else{
			solution =ik_result.first;
		}


		//		ROS_INFO("got IK, set Joint angles");

		brics_actuator::JointPositions joint_pos_init;
		joint_pos_init = createArmPositionCommand(solution,no_joints);

		/*for (int i = 0; i < no_joints; i++) {
			// Set all values for one joint, i.e. time, name, value and unit
			brics_actuator::JointValue joint;
			joint.timeStamp = ros::Time::now();
			joint.value = solution.at(i);
			joint.unit = "rad";

			// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
			std::stringstream jointName;
			jointName << "arm_joint_" << (i + 1);
			joint.joint_uri = jointName.str();

			// add joint to positionmessage
			joint_pos_init.positions.push_back(joint);
		}*/
		if(!solution.empty()){
			joint_pub_.publish(joint_pos_init);
			//		ROS_INFO("new position published");
			ros::Duration(4).sleep();
		}else{
			ROS_WARN("No valid endposition for arm");
		}


		if (gl->gripperposend >= 0){
			grippermsg = createGripperPositionCommand(gl->gripperposend);
			gripp_pub_.publish(grippermsg);
		}

		result_.success = true;
	}
	as->setSucceeded(result_);

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "arm_server");
	ros::NodeHandle nh_;
	//listener = new(tf::TransformListener);
	std::string name = "move_arm";
	Server arm_server(nh_, name, boost::bind(&executeCB, _1, &arm_server),
			false);
	arm_server.start();
	ROS_INFO("Youbot_arm_server up and running");
	ros::Subscriber sub =nh_.subscribe("/joint_state",20,read_joint_stateCB);
	joint_pub_ = nh_.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command",1);
	gripp_pub_ = nh_.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);

	ros::spin();
	return 0;
}
