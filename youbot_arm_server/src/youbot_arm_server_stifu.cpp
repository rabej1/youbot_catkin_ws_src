#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <angles/angles.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/JointLimits.h>
#include <moveit_msgs/DisplayTrajectory.h>

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

double alpha = 0;
double beta = pi/4;
double gm = 0;
bool proceed = false;
const double time_factor = 1;

//Robot specific data, from URDF file
const double l1 = 0.155;
const double l2 = 0.135;
const double l3 = 0.130;
const int no_joints = 5;

const std::string base_footprint = "base_footprint";
const std::string arm_link_2 = "arm_link_2";
const std::string arm_link_1 = "arm_link_1";

tf::StampedTransform tf_base_arm_1;
tf::StampedTransform tf_arm_1_2;

static const std::string arr[] = {"arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"};
std::vector<std::string> joint_names (arr, arr + sizeof(arr) / sizeof(arr[0]) );

static const double arr_offset[] = {2.967, 1.134, -2.5485, 1.7885, 1.2};
std::vector<double> joint_offset (arr_offset, arr_offset + sizeof(arr_offset) / sizeof(arr_offset[0]) );

static const double arr_upper_limits[] = {5.84, 2.617, -0.15, 3.429, 5.641};
static const double arr_lower_limits[] = {0.01007, 0.01007, -5.0266, 0.02213, 0.1107};

std::vector<double> upper_limits (arr_upper_limits,
		arr_upper_limits + sizeof(arr_upper_limits) / sizeof(arr_upper_limits[0]) );
std::vector<double> lower_limits (arr_lower_limits,
		arr_lower_limits + sizeof(arr_lower_limits) / sizeof(arr_lower_limits[0]) );

moveit_msgs::DisplayTrajectory result_trajectory;
control_msgs::FollowJointTrajectoryGoal goal_trajectory;

tf::TransformListener *listener;
tf::TransformBroadcaster *broadcaster;
std::string frame_target = "/arm_link_1";
std::string frame_int = "/arm_link_0";
std::string frame_source = "base_footprint";

//Action servers and clients
//actionlib::SimpleActionServer *as;


moveit::planning_interface::MoveGroup *group;

class YoubotArm {

protected:

	ros::NodeHandle nh_;

	actionlib::SimpleActionServer<youbot_arm_server::YouBotGoalAction> as_;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
	std::string action_name_;
	youbot_arm_server::YouBotGoalFeedback feedback_;
	youbot_arm_server::YouBotGoalResult result_;
	moveit::planning_interface::PlanningSceneInterface psi;



public:

	typedef std::vector<double> JointAngles;
	typedef std::pair <JointAngles,JointAngles> JointAnglesPair;

	typedef moveit::planning_interface::MoveGroup::Plan MotionPlan;
	typedef std::pair <MotionPlan,MotionPlan> MotionPlanPair;

	tf::StampedTransform tf_bfp_al1;



	YoubotArm(std::string name):
		as_(nh_,name, boost::bind(&YoubotArm::executeCB, this, _1),false),
		ac_("/arm_1/arm_controller/follow_joint_trajectory", true),
		action_name_(name) {

		as_.start();
		listener = new tf::TransformListener(nh_);
		broadcaster = new tf::TransformBroadcaster;
		

		ros::AsyncSpinner spinner(1);
		spinner.start();

		group = new moveit::planning_interface::MoveGroup("manipulator");
		
		
		group->setPlanningTime (60.0);
		group->setPlannerId ("RRTConnectkConfigDefault");
		group->allowReplanning (true);
		group->allowLooking (false);


	}

	~YoubotArm(void) {

	}

	
	/*
	 * Calculate rotation matrix from euler zyx
	 */
	Eigen::Matrix3d euler_zyx (const double alpha, // around z
			const double beta, // around y
			const double gm)  // around x
	{
		const double c1 = cos (alpha);
		const double s1 = sin (alpha);
		const double c2 = cos (beta);
		const double s2 = sin (beta);
		const double c3 = cos (gm);
		const double s3 = sin (gm);

		Eigen::Matrix3d rot;
		rot << c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2,
				c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3,
				-s2, c2*s3, c2*c3;
		return rot;
	}

	
	
	/*
	 * Calculate rotation matrix from euler zyx
	 */
	Eigen::Matrix3d euler_zyx (const Eigen::Vector3d &angle) {
		return euler_zyx(static_cast<double>(angle(0)),
				static_cast<double>(angle(1)),
				static_cast<double>(angle(2)));
	}

	
	
	Eigen::Vector3d inv_orientation_kuka (const Eigen::Matrix3d &rot)
	{
		// order of angles is 1st 1st element 2nd 2nd element 3rd 3rd element
		// specialization of inv_euler_zyx
		// where gimbal lock is resolved and angle (2) is used angle (0) is 0
		Eigen::Vector3d angle;
		const double c2 = sqrt (static_cast <double> (rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0)));

		if (fabs (c2) > 1e-6)
		{
			angle (0) = atan2 (static_cast <double> (rot (1,0))/c2, static_cast <double> (rot (0,0))/c2);
			angle (1) = atan2 (-static_cast <double> (rot (2,0)), c2);
			angle (2) = atan2 (static_cast <double> (rot (2,1))/c2, static_cast <double> (rot (2,2))/c2);
		}
		else
		{
			// we set angle (0) to zero
			angle (0) = 0.0;
			angle (2) = 0.0;
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
			angle (1) = -(static_cast <double> (-rot (2,0)) > 0) ? M_PI_2 : -M_PI_2;
			//angle (2) = atan2 (static_cast <double> (rot (0,2)), -static_cast <double> (rot (0,1)));
			//angle (2) = atan2 (static_cast <double> (-rot (1,1) / rot (2,0)),
			//		static_cast <double> (-rot (1,2) / rot (2,0))) +  M_PI_2;
			const double psi13 = atan2 (static_cast <double> (-rot (0,1)),
					static_cast <double> (rot (1,1)));
			if (rot (2,0) < 0) {
				angle (2) = -psi13;
				//angle (0) = psi13;
			} else {
				angle (2) = psi13;
				//angle (0) = -psi13;
			}
			std::cout << rot (0,1) << ',' << rot (1,1) << " -> " << angle (2);
			std::cout << std::endl;
		}

		return angle;
	}

	
	
	/*
	 * Convert rotation matrix to euler angles
	 */
	Eigen::Vector3d inv_euler_zyx (const Eigen::Matrix3d &rot)
	{
		// order of angles is 1st 1st element 2nd 2nd element 3rd 3rd element
		Eigen::Vector3d angle;
		const double c2 = sqrt (static_cast <double> (rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0)));

		if (fabs (c2) > 1e-6)
		{
			angle (0) = atan2 (static_cast <double> (rot (1,0))/c2, static_cast <double> (rot (0,0))/c2);
			angle (1) = atan2 (-static_cast <double> (rot (2,0)), c2);
			angle (2) = atan2 (static_cast <double> (rot (2,1))/c2, static_cast <double> (rot (2,2))/c2);
		}
		else
		{
			throw std::invalid_argument ("Gimbal lock: cannot determine angle 2");
		}
		return angle;
	}
	
	

	/*
	 * Test resulting euler angles
	 */
	bool test_euler_zyx (const double step=M_PI/18) {
		bool ok = true;
		size_t count_all = 0;
		size_t count_diverging = 0;
		size_t count_gimbal_lock = 0;
		const double max_diff = 0.01;
		for (double alpha=-M_PI; alpha<M_PI; alpha+=step) {
			for (double beta=-M_PI; beta<M_PI; beta+=step) {
				for (double gm=-M_PI; gm<M_PI; gm+=step) {
					Eigen::Matrix3d rot = euler_zyx (alpha, beta, gm);
					try {
						//Eigen::Vector3d angle = inv_euler_zyx (rot);
						Eigen::Vector3d angle = inv_orientation_kuka (rot);
						Eigen::Matrix3d tmp = euler_zyx (
								static_cast <double> (angle (0)),
								static_cast <double> (angle (1)),
								static_cast <double> (angle (2)));
						//Eigen::Vector3d angle2 = inv_euler_zyx (tmp);
						Eigen::Vector3d angle2 = inv_orientation_kuka (tmp);
						Eigen::Vector3d diff = angle - angle2;

						tmp *= rot.inverse ();
						bool fail = (fabs (static_cast <double> (tmp(0,0)) - 1) > max_diff) ||
								(fabs (static_cast <double> (tmp(1,1)) - 1) > max_diff) ||
								(fabs (static_cast <double> (tmp(2,2)) - 1) > max_diff);
						/*
						bool fail = (fabs(static_cast <double> (angle (0)) - alpha) > max_diff) ||
								(fabs(static_cast <double> (angle (1)) - beta) > max_diff) ||
								(fabs(static_cast <double> (angle (2)) - gm) > max_diff);
						 */

						bool fail2 = (fabs(static_cast <double> (diff (0))) > max_diff) ||
								(fabs(static_cast <double> (diff (1))) > max_diff) ||
								(fabs(static_cast <double> (diff (2))) > max_diff);
						fail2 = false;

						if (fail || fail2) {
							std::cout << "---" << std::endl
									<< rot << std::endl
									<< tmp << std::endl;
							std::cout << alpha << ',' << beta << ',' << gm
									<< " vs "
									<< angle (0) << ',' << angle (1) << ',' << angle (2);
							std::cout << std::endl;
							++count_diverging;
						}
					}
					catch (std::invalid_argument &e) {
						std::cout << e.what () << std::endl;
						++count_gimbal_lock;
					}

					++count_all;
				}
			}
		}

		std::cout << "failed " << count_diverging << " out of " << count_all << " checks." << std::endl;
		std::cout << count_gimbal_lock << " gimbal locks" << std::endl;
		return ok;
	}
	
	

	/*
	 * Transform poses
	 */
	double transform_pose (const Eigen::Vector3d &source_point,
			const std::string source_frame,
			const std::string target_frame) {

		tf::StampedTransform tf_source_int;
		tf::StampedTransform tf_int_target;
		double angle;

		try{
			listener->waitForTransform(target_frame,
					frame_int,
					ros::Time::now(),
					ros::Duration(1.0));

			listener->lookupTransform(source_frame, frame_int,
					ros::Time(0), tf_source_int);

			listener->waitForTransform(frame_int,
					source_frame,
					ros::Time::now(),
					ros::Duration(1.0));

			listener->lookupTransform(frame_int, target_frame,
					ros::Time(0), tf_int_target);

			double offset = tf_int_target.getOrigin().getX() + tf_source_int.getOrigin().getX();

			angle = atan2(source_point.y(),source_point.x() - offset);



		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}




		return angle;

	}
	
	

	/*
	 * Display goal in rviz in tf tree
	 */
	geometry_msgs::PoseStamped set_goal (const Eigen::Vector3d position,
			const Eigen::Vector3d angle,
			const std::string frame_id) {
		geometry_msgs::PoseStamped result_pose;

		Eigen::Matrix3d rot_mat = euler_zyx(angle);
		Eigen::Quaternion<double> q(rot_mat);
		tf::Quaternion quat;
		tf::quaternionEigenToTF(q,quat);

		result_pose.header.stamp = ros::Time::now();
		result_pose.header.frame_id = frame_id;
		result_pose.pose.position.x = position(0);
		result_pose.pose.position.y = position(1);
		result_pose.pose.position.z = position(2);
		result_pose.pose.orientation.x = quat.x();
		result_pose.pose.orientation.y = quat.y();
		result_pose.pose.orientation.z = quat.z();
		result_pose.pose.orientation.w = quat.w();

		broadcaster->sendTransform(tf::StampedTransform(tf::Transform(quat, tf::Vector3(position(0),
				position(1),
				position(2))),
				ros::Time::now(),"/base_footprint", "/target_link"));


		return result_pose;
	}

	/*
	 * Calculate two IK solutions for given goal
	 */
	JointAnglesPair ik_youbot_arm (
			const geometry_msgs::PoseStamped &target_pose,
			const double theta) {

		bool valid = false;
		geometry_msgs::PoseStamped result_pose;
		JointAngles result_angles_1;
		JointAngles result_angles_2;
		JointAnglesPair result;

		result_angles_1.resize(5);
		result_angles_2.resize(5);

		tf::Vector3 base_arm_1 = tf_base_arm_1.getOrigin();
		tf::Vector3 arm_1_2 = tf_arm_1_2.getOrigin();

		result_angles_1.at(0) = atan2 (target_pose.pose.position.y - base_arm_1.getY(),
				target_pose.pose.position.x - base_arm_1.getX());
		result_angles_2.at(0) = result_angles_1.at(0);


		double x = sqrt (pow(target_pose.pose.position.y - base_arm_1.getY() - arm_1_2.getY(),2) +
				pow(target_pose.pose.position.x - base_arm_1.getX() - arm_1_2.getX(),2));
		double z = target_pose.pose.position.z - arm_1_2.getZ() - base_arm_1.getZ();


		double x_ = x - l3*cos(theta);
		double z_ = z - l3*sin(theta);

		double gamma = atan2 (-z_/sqrt (x_*x_ + z_*z_),-x_/sqrt (x_*x_ + z_*z_));

		result_angles_1.at(1) = gamma + acos ((-(x_*x_ + z_*z_ +l1*l1 - l2*l2))/(2*l1*sqrt(x_*x_ + z_*z_)));
		result_angles_2.at(1) = gamma - acos ((-(x_*x_ + z_*z_ +l1*l1 - l2*l2))/(2*l1*sqrt(x_*x_ + z_*z_)));

		if ((fabs(result_angles_1.at(1)) > M_PI)) {
			result_angles_1.at(1) = 2*M_PI - fabs(result_angles_1.at(1));
		}
		if ((fabs(result_angles_2.at(1)) > M_PI)) {
			result_angles_2.at(1) = 2*M_PI - fabs(result_angles_2.at(1));
		}

		result_angles_1.at(2) = atan2 ((z_-l1*sin((result_angles_1.at(1))))/l2,
				(x_-l1*cos((result_angles_1.at(1))))/l2) - result_angles_1.at(1);
		result_angles_2.at(2) = atan2 ((z_-l1*sin((result_angles_2.at(1))))/l2,
				(x_-l1*cos((result_angles_2.at(1))))/l2) - result_angles_2.at(1);

		if ((fabs(result_angles_1.at(2)) > M_PI)) {
			result_angles_1.at(2) = 2*M_PI - fabs(result_angles_1.at(2));
		}
		if ((fabs(result_angles_2.at(2)) > M_PI)) {
			result_angles_2.at(2) = 2*M_PI - fabs(result_angles_2.at(2));
		}

		result_angles_1.at(3) = theta - (result_angles_1.at(1) + result_angles_1.at(2));
		result_angles_2.at(3) = theta - (result_angles_2.at(1) + result_angles_2.at(2));



		//Check if soulution is valid: Nans = solution not valid
		for (std::size_t it = 0; it < 3; ++it) {
			if ((result_angles_1.at(it) != result_angles_1.at(it))
					|| (result_angles_2.at(it) != result_angles_2.at(it) )) {
				valid = false;
			}
			else {
				valid = true;
			}
		}

		result_angles_1.at(4) = 2.95;
		result_angles_2.at(4) = 2.95;

		if(valid) {

			result.first = result_angles_1;
			result.second = result_angles_2;

			angle_mapping (result.first);
			angle_mapping (result.second);

		}
		else
		{
			result.first.clear ();
			result.second.clear ();
		}

		// TODO: remove this


		return result;
	}

	/*
	 * Construct joint message from angles
	 */
	sensor_msgs::JointState create_joint_msg (const std::vector<double> &angles) {


		if (angles.size() != joint_names.size()) {
			throw std::invalid_argument("Something went wrong. Size of angles!= size of joint_names!");
		}

		sensor_msgs::JointState js;

		js.name = joint_names;
		js.position = angles;

		return js;
	}

	/*
	 * Get transforms
	 */
	void get_transforms () {

		try {
			//Lookup transform from base_footprint to arm_lnk_0
			listener->waitForTransform(base_footprint,
					arm_link_1,
					ros::Time::now(),
					ros::Duration(1.0));

			listener->lookupTransform(base_footprint, arm_link_1,
					ros::Time(0), tf_base_arm_1);

			//Lookup transform from arm_link_0 to arm_link_1
			listener->waitForTransform(arm_link_1,
					arm_link_2,
					ros::Time::now(),
					ros::Duration(1.0));

			listener->lookupTransform(arm_link_1, arm_link_2,
					ros::Time(0), tf_arm_1_2);
		}

		catch (tf::TransformException &ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	/*
	 * Map angles from IK space to KUKA space
	 */
	void angle_mapping ( std::vector<double> &angles) {

		if(angles.at(0) < 0) {
			angles.at(0) = joint_offset.at(0) + fabs(angles.at(0));
		}
		else {
			angles.at(0) = joint_offset.at(0) - fabs(angles.at(0));
		}

		if (angles.at(1) < 0) {
			angles.at(1) = (joint_offset.at(1) + M_PI/2) + fabs(angles.at(1));
		}
		else {
			angles.at(1) = (joint_offset.at(1) + M_PI/2) - fabs(angles.at(1));
		}

		if (angles.at(2) < 0) {
			angles.at(2) = joint_offset.at(2) + fabs(angles.at(2));
		}
		else {
			angles.at(2) = joint_offset.at(2) - fabs(angles.at(2));
		}

		if (angles.at(3) < 0) {
			angles.at(3) = joint_offset.at(3) + fabs(angles.at(3));
		}
		else {
			angles.at(3) = joint_offset.at(3) - fabs(angles.at(3));
		}
	}

	/*
	 * Check if solution does not violate joint limits
	 */
	bool check_joint_values (const JointAngles &angles) {

		bool valid;

		if (((angles[0] < lower_limits[0]) || (angles[0] > upper_limits[0])) ||
				((angles[1] < lower_limits[1]) || (angles[1] > upper_limits[1])) ||
				((angles[2] < lower_limits[2]) || (angles[2] > upper_limits[2])) ||
				((angles[3] < lower_limits[3]) || (angles[3] > upper_limits[3])) ||
				((angles[4] < lower_limits[4]) || (angles[4] > upper_limits[4])) ) {
			valid = false;
		}
		else
		{
			valid = true;
		}

		return valid;
	}

	/*
	 * Construct brics message from moveit trajectory
	 */
	void execute_trajectory (const moveit::planning_interface::MoveGroup::Plan &plan) {

		int size_trajectory = plan.trajectory_.joint_trajectory.points.size();
		//std::cout << "Number of points in trajectory: " << size_trajectory << std::endl;
		goal_trajectory.trajectory.points.resize(size_trajectory);
		goal_trajectory.trajectory.joint_names = joint_names;

		for (std::size_t itp = 0; itp < size_trajectory; itp++) {
			goal_trajectory.trajectory.points[itp].positions.resize(no_joints);
			goal_trajectory.trajectory.points[itp].velocities.resize(no_joints);
			goal_trajectory.trajectory.points[itp].accelerations.resize(no_joints);
			for (std::size_t itj = 0; itj < no_joints; itj++) {
				goal_trajectory.trajectory.points[itp].time_from_start = plan.trajectory_.joint_trajectory.points[itp].time_from_start * (1/time_factor);
				goal_trajectory.trajectory.points[itp].positions[itj] = plan.trajectory_.joint_trajectory.points[itp].positions[itj];
				goal_trajectory.trajectory.points[itp].velocities[itj] = plan.trajectory_.joint_trajectory.points[itp].velocities[itj]*time_factor;
				goal_trajectory.trajectory.points[itp].accelerations[itj] = plan.trajectory_.joint_trajectory.points[itp].accelerations[itj]*time_factor;
			}
		}

		goal_trajectory.trajectory.header.stamp = ros::Time::now();
		ac_.sendGoal(goal_trajectory);
		
			ac_.waitForResult();
	}

	void executeCB(const youbot_arm_server::YouBotGoalGoalConstPtr &gl) {

		bool success = false;
		bool cont = true;

		ros::Time now = ros::Time::now();

		get_transforms();

		ROS_INFO("Desired goal:");
		ROS_INFO("Reference frame is: %s",group->getPlanningFrame().c_str());
		ROS_INFO("End effector link is: %s",group->getEndEffectorLink().c_str());
		ROS_INFO("x: %f",gl->goal.pose.position.x);
		ROS_INFO("y: %f",gl->goal.pose.position.y);
		ROS_INFO("z: %f",gl->goal.pose.position.z);
		ROS_INFO("theta1: %f",gl->goal.pose.orientation.x);
		ROS_INFO("theta2: %f",gl->goal.pose.orientation.x);

		Eigen::Vector3d position;
		//position << params [0], params [1], params [2];

		position << gl->goal.pose.position.x, gl->goal.pose.position.y, gl->goal.pose.position.z;
		Eigen::Vector3d angle;
		angle << transform_pose(position,frame_source, frame_target), gl->goal.pose.orientation.x, gl->goal.pose.orientation.y;

		//Start moveit stuff



		geometry_msgs::PoseStamped current_pose = group->getCurrentPose();

		//Set goal from input
		geometry_msgs::PoseStamped goal = set_goal(position, angle, frame_source);
		// TODO why 1 degree deviation in IK??


		JointAnglesPair ik_result = ik_youbot_arm (
				goal,
				static_cast<double> (angle (1)));
		if (ik_result.first.empty () || ik_result.second.empty ()) {
			throw std::invalid_argument ("inverse kinematics has no solution");
		}


		// generate motion plan
		MotionPlanPair plan_pair;
		group->setGoalOrientationTolerance (0.05);
		group->setGoalPositionTolerance (0.05);

		if (as_.isPreemptRequested() || !ros::ok())
		{
			ROS_INFO("%s: Preempted", action_name_.c_str());
			// set the action state to preempted
			as_.setPreempted();
			cont = false;

		}

		if (cont) {
			group->setJointValueTarget (create_joint_msg (ik_result.first));
			bool success_1 = group->plan (plan_pair.first);

			group->setJointValueTarget (create_joint_msg (ik_result.second));
			bool success_2 = group->plan (plan_pair.second);

			// select best plan
			MotionPlan the_plan;
			if (!success_1 && !success_2) {
				success = false;
				result_.success = success;
				as_.setSucceeded(result_);
				ROS_INFO("Kinematic solution does not yield a valid plan. Check for collisions with sensors");
				//throw std::invalid_argument ("inverse kinematics solution does not yield a valid plan (self-collision ?)");
			}

			if (success_1 && success_2) {
				if (plan_pair.first.trajectory_.joint_trajectory.points.size () <=
						plan_pair.first.trajectory_.joint_trajectory.points.size ()) {
					the_plan = plan_pair.first;
					ROS_INFO("Plan 1 shorter than plan 2 ->  Choosing plan 1");
				} else {
					the_plan = plan_pair.second;
					ROS_INFO("Plan 2 shorter than plan 1 -> Choosing plan 2");

				}
			}
			if(success_1 || success_2){
				success = true;
				if (check_joint_values(ik_result.first) && success_1) {
					the_plan = plan_pair.first;
					ROS_INFO("Plan 1");


				} else if (check_joint_values(ik_result.second) && success_2)
				{
					the_plan = plan_pair.second;
					ROS_INFO("Plan 2");

				}
				result_.success = success;
				as_.setSucceeded(result_);
			}
			else {
				success = false;
				ROS_INFO("Plan theoretically valid. However, joint angles exceed joint limits");
				result_.success = success;
				as_.setSucceeded(result_);
			}

			result_trajectory.trajectory_start = the_plan.start_state_;
			result_trajectory.trajectory.push_back(the_plan.trajectory_);
			
			execute_trajectory(the_plan);

			ROS_INFO("Planning ok");

		}

	}

};

int main(int argc, char** argv){

	//TODO: change name
	ros::init(argc, argv, "youbot_arm_server");


	/*// order is x, y, z, alpha, beta, gamma
	const double dist = 0.25;
	double params [] = {dist, 0.0, dist, 0.0, 0.0};
	double temp_x = 0;
	const size_t n_params = sizeof (params) / sizeof (double);
	if (argc > 6) {
		std::cerr << "ERROR: cannot take more than 5 command line arguments" << std::endl;
		return EXIT_FAILURE;
	}
	for (size_t idx=0; idx<argc-1; ++ idx) {
		float tmp;
		int n = sscanf (argv [idx+1], "%f", &tmp);
		params [idx] = tmp;
		if (n!=1) {
			std::cerr << "ERROR: could not parse " << argv [idx+1] << std::endl;
			return EXIT_FAILURE;
		}
	}*/

	YoubotArm youbot_arm(ros::this_node::getName());

	ROS_INFO("Youbot_arm_tf node up and running");




ros::spin();
	
	ROS_WARN("Following error message can be ignored. Error is related to a bug in MoveIt.");

	return 0;
}
