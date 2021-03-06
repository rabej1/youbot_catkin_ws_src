#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <sstream>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>

#include <actionlib/server/simple_action_server.h>
#include <youbot_arm_server/YouBotGoalAction.h>
#include <actionlib/client/simple_action_client.h>

ros::Publisher joint_pub_;
std::string arm1 = "arm_joint_1";
std::string arm2 = "arm_joint_2";
std::string arm3 = "arm_joint_3";
std::string arm4 = "arm_joint_4";
std::string arm5 = "arm_joint_5";

typedef actionlib::SimpleActionClient<youbot_arm_server::YouBotGoalAction> Client;

//double pi = 3.1415926;

void initialize_arm();

int main(int argc, char **argv) {

	ros::init(argc, argv, "arm_client");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();
	joint_pub_ = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command",1);
	//initialize_arm();

	//actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient("/arm_1/arm_controller/follow_joint_trajectory", true);

	Client ac_("move_arm", true);
	ROS_INFO("Waiting for action server to start.");
	ac_.waitForServer();

	ROS_INFO("Action server started.");

	youbot_arm_server::YouBotGoalGoal goal;






//	ROS_INFO("Reference frame is: %s", group->getPlanningFrame().c_str());
//	ROS_INFO("Endeffector is: %s", group.getEndEffectorLink().c_str());

//	std::vector<double> tolerance_pose(3, 0.05);
//	std::vector<double> tolerance_angle(3, 0.5);

	tf::TransformListener tfListener(nh);
	tf::StampedTransform armTF;
	ros::Time now = ros::Time::now();

	tfListener.waitForTransform("/base_link", "/gripper_finger_link_l", now,
			ros::Duration(1.0));
	tfListener.lookupTransform("/base_link", "/gripper_finger_link_l", ros::Time(0),
			armTF);

	goal.goal.pose.position.x = armTF.getOrigin().x();
	goal.goal.pose.position.y = armTF.getOrigin().y();
	goal.goal.pose.position.z = armTF.getOrigin().z();

	//goal.goal.pose.orientation.x = armTF.getOrigin().x();
	//goal.goal.pose.orientation.y = armTF.getOrigin().y();
	//goal.goal.pose.orientation.z = armTF.getOrigin().z();
	goal.goal.pose.orientation.w = armTF.getOrigin().w();

//	double yawi = atan2(target.position.y - armTF.getOrigin().y(),
//			target.position.x - armTF.getOrigin().x()) - 3.14159;
//	std::cout << "yaw = " << angles::to_degrees(yawi) << std::endl;

//	Set the pose orientation
//	tf::quaternionTFToMsg(tf::createQuaternionFromRPY(rolli, pitchi, yawi),
//			target.orientation);
//	*/

//	geometry_msgs::PoseStamped start_pose;
//	start_pose = group.getCurrentPose();
//	tf::Quaternion q(start_pose.pose.orientation.x,
//			start_pose.pose.orientation.y, start_pose.pose.orientation.z,
//			start_pose.pose.orientation.w);
//
//	brics_actuator::JointPositions joint_pos;
//	brics_actuator::JointValue joint_val;
	youbot_arm_server::YouBotGoalGoal goal2;
	goal2.goal.pose.position.x = 0.45;
	goal2.goal.pose.position.y = 0.1;
	goal2.goal.pose.position.z = 0.23;
	goal2.goal.pose.orientation.x = 0;
	goal2.goal.pose.orientation.y = 0;
	goal2.goal.pose.orientation.z = 3.2;
	goal2.goal.pose.orientation.w = 0.002;

	for (int it = 0; it < 5; it++) {

		std::cout << goal2.goal.header.frame_id << std::endl;
		goal2.goal.pose.position.x += 0.02;
		//goal2.goal.pose.orientation.x += 0.02;

		/*orientation_constraints_.orientation.x = target.orientation.x;
		 orientation_constraints_.orientation.y = target.orientation.y;
		 orientation_constraints_.orientation.z = target.orientation.z;
		 orientation_constraints_.orientation.w = target.orientation.w;
		 //constraints
		 orientation_constraints_.absolute_x_axis_tolerance = 3.14;
		 orientation_constraints_.absolute_y_axis_tolerance = 3.14;
		 orientation_constraints_.absolute_z_axis_tolerance = 3.14;*/

		//sleep(1);
		//startversuch

		goal2.goal.header.stamp = ros::Time::now();

		//endeversuch
		ROS_INFO("Sending Goal to Server");
		goal.goal.header.stamp = ros::Time::now();
		ac_.sendGoal(goal2);
		bool finished_before_timeout = ac_.waitForResult(ros::Duration(30.0));
		if (finished_before_timeout)
		  {
		    actionlib::SimpleClientGoalState state = ac_.getState();
		    ROS_INFO("Action finished: %s",state.toString().c_str());
		  }
		  else
		    ROS_INFO("Action did not finish before the time out.");
	}

return 0;

}

void initialize_arm()
{

	std::cout << "Initializing arm" <<std::endl;

	brics_actuator::JointPositions joint_pos_init;
	brics_actuator::JointValue joint_val_init;
	joint_val_init.joint_uri=arm1;
	joint_val_init.unit = "rad";
	joint_val_init.value =  2.945;
	joint_pos_init.positions.push_back(joint_val_init);

	joint_val_init.joint_uri=arm2;
	joint_val_init.unit = "rad";
	joint_val_init.value =  1.71007;
	joint_pos_init.positions.push_back(joint_val_init);

	joint_val_init.joint_uri=arm3;
	joint_val_init.unit = "rad";
	joint_val_init.value = -1.31571;
	joint_pos_init.positions.push_back(joint_val_init);

	joint_val_init.joint_uri=arm4;
	joint_val_init.unit = "rad";
	joint_val_init.value =  3.12212;
	joint_pos_init.positions.push_back(joint_val_init);

	joint_val_init.joint_uri=arm5;
	joint_val_init.unit = "rad";
	joint_val_init.value =  1.57;
	joint_pos_init.positions.push_back(joint_val_init);


	joint_pub_.publish(joint_pos_init);

	sleep(5);

}
