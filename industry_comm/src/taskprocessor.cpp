#include <iostream>
#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "industry_comm_msg/TaskProcessor.h"
#include "industry_comm_msg/feedback.h"

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <youbot_arm_server/YouBotGoalAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Twist.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>

#include <std_srvs/Empty.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<youbot_arm_server::YouBotGoalAction> ArmClient;

move_base_msgs::MoveBaseGoal goal;

bool last_task_diceroll = false;
//std::vector<int> tagid;
int tagid;
double big_x, big_y;
bool big_new;
int finalposetagnbr;

class Processor {

protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
	actionlib::SimpleActionClient<youbot_arm_server::YouBotGoalAction> ac_arm_;
	ros::Subscriber sub;
	ros::Subscriber bigtag_sub;
	ros::Publisher pub;
	ros::Publisher vel_pub_;
	ros::ServiceServer service;
	std::string action_name_;
	tf::TransformListener listener;
	ros::ServiceClient stop_cam_client;
	ros::ServiceClient start_cam_client;

	move_base_msgs::MoveBaseGoal storage;
	move_base_msgs::MoveBaseGoal dicerollfield, waitingpose;
	industry_comm_msg::feedback task_feedback;

	std_srvs::Empty cam_srv;

public:

	move_base_msgs::MoveBaseGoal goal;
	youbot_arm_server::YouBotGoalGoal arm_goal;


	Processor(std::string name):
		ac_("move_base", true),
		ac_arm_("move_arm", true),
		action_name_(name) {

		sub = nh_.subscribe("/uptag_visualization", 100, &Processor::poseCB, this);
		bigtag_sub = nh_.subscribe("/big_visualization", 100, &Processor::pose_bigCB, this);
		service = nh_.advertiseService("TaskProcessor", &Processor::task, this);
		pub = nh_.advertise<industry_comm_msg::feedback>("taskfeedback",100);
		vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

		stop_cam_client = nh_.serviceClient<std_srvs::Empty>("stop_capture");
		start_cam_client = nh_.serviceClient<std_srvs::Empty>("start_capture");

		task_feedback.task_ID = 0;
		task_feedback.client_ID = "to be completed";
		task_feedback.robot_ID = "to be completed";
		task_feedback.bring = false;
		task_feedback.bring_success = false;
		task_feedback.get = false;
		task_feedback.get_success = false;
		task_feedback.bringback = false;
		task_feedback.bringback_success = false;
		task_feedback.diceroll = false;
		task_feedback.dicevalue = 0;

		//defined poses for the BFH robotics lab in july 2016.
		storage.target_pose.header.frame_id = "map";
		storage.target_pose.pose.position.x = 12.2;
		storage.target_pose.pose.position.y = 8.9;
		storage.target_pose.pose.orientation.z = 0.739;
		storage.target_pose.pose.orientation.x = 0;
		storage.target_pose.pose.orientation.y = 0;
		storage.target_pose.pose.orientation.w = sqrt(1-pow(storage.target_pose.pose.orientation.z,2));

		waitingpose.target_pose.header.frame_id = "map";
		waitingpose.target_pose.pose.position.x = 11;
		waitingpose.target_pose.pose.position.y = 3;
		waitingpose.target_pose.pose.orientation.z = 0;
		waitingpose.target_pose.pose.orientation.x = 0;
		waitingpose.target_pose.pose.orientation.y = 0;
		waitingpose.target_pose.pose.orientation.w = sqrt(1-pow(waitingpose.target_pose.pose.orientation.z,2));

		dicerollfield.target_pose.header.frame_id = "map";
		dicerollfield.target_pose.pose.position.x = 11.245;
		dicerollfield.target_pose.pose.position.y = 3.357;
		dicerollfield.target_pose.pose.orientation.z = 0;
		dicerollfield.target_pose.pose.orientation.x = 0;
		dicerollfield.target_pose.pose.orientation.y = 0;
		dicerollfield.target_pose.pose.orientation.w = sqrt(1-pow(dicerollfield.target_pose.pose.orientation.z,2));

		//At every endpose there has to be a April Tag with the defined number.
		finalposetagnbr = 210;

		ros::AsyncSpinner spinner(1);
		spinner.start();
	}

	void publishresult(void){
		pub.publish(task_feedback);
		task_feedback.task_ID = 0;
		task_feedback.client_ID = "to be completed";
		task_feedback.robot_ID = "to be completed";
		task_feedback.bring = false;
		task_feedback.bring_success = false;
		task_feedback.get = false;
		task_feedback.get_success = false;
		task_feedback.bringback = false;
		task_feedback.bringback_success = false;
		task_feedback.diceroll = false;
		task_feedback.dicevalue = 0;
	}

	void poseCB(const visualization_msgs::Marker msg){
		if(msg.ns == "basic_shapes")
			tagid = ((int)msg.id);
	}

	void pose_bigCB(const visualization_msgs::Marker msg){
		if(msg.id == finalposetagnbr){
			big_x = msg.pose.position.z;
			big_y = msg.pose.position.x;
			big_new=true;
		}
	}


	void publishArmPos(youbot_arm_server::YouBotGoalGoal gripgoal){
		while(!ac_arm_.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the arm server to come up");
		}
		ac_arm_.sendGoal(gripgoal);
		ac_arm_.waitForResult();
		if(ac_arm_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the arm reached its destination!");
		else
			ROS_INFO("The Arm failed to reach the goal");
	}

	/*
	 * Pick up the object with the given April tag number
	 */
	void pickup(int apriltagnbr){
		//std::cout << "ar_start" << std::endl;
		//sleep(20);

		ROS_INFO("Pick up object with april-tag number: [%d]", apriltagnbr);

		bool foundtag = false;
		int pos = 0;

		tf::StampedTransform transform, transform_uptag;
		std::stringstream apriltagtf, up_tag_tf;
		double offset;
		apriltagtf << "ar_marker_" << (apriltagnbr);
		ROS_INFO("April TF Name: %s", apriltagtf.str().c_str());


		arm_goal.goal.pose.orientation.x = 0;
		arm_goal.goal.pose.orientation.y = -1.35;
		arm_goal.goal.pose.orientation.w = 1;
		arm_goal.goal.header.stamp = ros::Time::now();

		arm_goal.gripperposstart = -1;
		arm_goal.gripperposend = -1;

		while ((!foundtag) && (pos < 3)){
			switch (pos){
			case 0:
				arm_goal.goal.pose.position.x = 0.17;
				arm_goal.goal.pose.position.y = 0.18;
				arm_goal.goal.pose.position.z = 0.21;
				arm_goal.goal.pose.orientation.z = 0.2;
				offset = M_PI/2;
				break;
			case 1:
				arm_goal.goal.pose.position.x = 0.35;
				arm_goal.goal.pose.position.y = 0;
				arm_goal.goal.pose.position.z = 0.21;
				arm_goal.goal.pose.orientation.z = 0.2-1.57;
				offset = 0;
				break;
			case 2:
				arm_goal.goal.pose.position.x = 0.17;
				arm_goal.goal.pose.position.y = -0.18;
				arm_goal.goal.pose.position.z = 0.21;
				arm_goal.goal.pose.orientation.z = 0.2-3.14;
				offset = -M_PI/2;
				break;
			default:
				ROS_WARN("Error");
			}

			publishArmPos(arm_goal);
			sleep(3);

			try {
				listener.waitForTransform("/base_footprint", apriltagtf.str(), ros::Time(0), ros::Duration(3.0) );
				listener.lookupTransform("/base_footprint", apriltagtf.str(), ros::Time(0), transform);
				ROS_INFO("Transformationage: [%d]", ros::Time::now().sec-transform.stamp_.sec);

				if((ros::Time::now().sec-transform.stamp_.sec) < 3)
					foundtag = true;
			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
				foundtag = false;
			}
			++pos;
		}

		if (foundtag){

			ROS_INFO("Tag Position x: [%f], y: [%f], z: [%f]", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

			tagid = 0;
			ros::spinOnce();
			if(tagid)
				up_tag_tf << "ar_marker_" <<  tagid;
			try {
				listener.waitForTransform("/base_footprint", up_tag_tf.str(), ros::Time(0), ros::Duration(3.0) );
				listener.lookupTransform("/base_footprint", up_tag_tf.str(), ros::Time(0), transform_uptag);
			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
			}


			double Y,P,R;
			tf::Matrix3x3 m(transform_uptag.getRotation());
			m.getEulerYPR(Y, P, R);

			ROS_INFO("Tag Orientation z: [%f]", Y);

			if (pos == 2){
				arm_goal.goal.pose.position.x = transform.getOrigin().x()*0.99;
				arm_goal.goal.pose.position.y = transform.getOrigin().y()*0.95;
			}else{
				arm_goal.goal.pose.position.x = transform.getOrigin().x()+0.007;
				arm_goal.goal.pose.position.y = transform.getOrigin().y()*0.87;
			}

			arm_goal.goal.pose.position.z = transform.getOrigin().z()+0.106;

			arm_goal.goal.pose.position.z = transform.getOrigin().z()+0.05;

			arm_goal.goal.pose.orientation.x = 0;
			arm_goal.goal.pose.orientation.y = -1.56;
			arm_goal.goal.pose.orientation.z = fmod(Y,(M_PI/2))+offset;
			arm_goal.goal.pose.orientation.w = 1;
			arm_goal.goal.header.stamp = ros::Time::now();

			arm_goal.gripperposstart = -1;
			arm_goal.gripperposend = 0.011;

			publishArmPos(arm_goal);
			sleep(3);
			arm_goal.goal.pose.position.z = arm_goal.goal.pose.position.z-0.005;
			publishArmPos(arm_goal);
			arm_goal.goal.pose.position.z = arm_goal.goal.pose.position.z-0.0025;
			publishArmPos(arm_goal);
			arm_goal.goal.pose.position.z = arm_goal.goal.pose.position.z-0.0025;
			arm_goal.gripperposend = 0.006;

			publishArmPos(arm_goal);
			sleep(5);
		}
	}

	void pickup_diceroll(int apriltagnbr){
		//		std::cout << "ar_start" << std::endl;
		//		sleep(10);

		ROS_INFO("Pick up object with april-tag number: [%d]", apriltagnbr);

		bool foundtag = false;
		int pos = 0;

		tf::StampedTransform transform, transform_uptag;
		std::stringstream apriltagtf, up_tag_tf;
		double offset;
		apriltagtf << "ar_marker_" << (apriltagnbr);
		ROS_INFO("April TF Name: %s", apriltagtf.str().c_str());


		arm_goal.goal.pose.orientation.x = 0;
		arm_goal.goal.pose.orientation.y = -1.35;
		arm_goal.goal.pose.orientation.w = 1;
		arm_goal.goal.header.stamp = ros::Time::now();

		arm_goal.gripperposstart = -1;
		arm_goal.gripperposend = -1;

		arm_goal.goal.pose.position.x = 0.17;
		arm_goal.goal.pose.position.y = -0.18;
		arm_goal.goal.pose.position.z = 0.21;
		arm_goal.goal.pose.orientation.z = 0.2-3.14;
		publishArmPos(arm_goal);

		while ((!foundtag) && (pos < 3)){
			sleep(3);

			try {
				listener.waitForTransform("/base_footprint", apriltagtf.str(), ros::Time(0), ros::Duration(3.0) );
				listener.lookupTransform("/base_footprint", apriltagtf.str(), ros::Time(0), transform);
				ROS_INFO("Transformationage: [%d]", ros::Time::now().sec-transform.stamp_.sec);

				if((ros::Time::now().sec-transform.stamp_.sec) < 3)
					foundtag = true;
			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
				foundtag = false;
			}
			++pos;
		}

		if (foundtag){
			ROS_INFO("Tag Position x: [%f], y: [%f], z: [%f]", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

			tagid = 0;

			ros::spinOnce();
			if(tagid)
				up_tag_tf << "ar_marker_" <<  tagid;
			try {
				listener.waitForTransform("/base_footprint", up_tag_tf.str(), ros::Time(0), ros::Duration(3.0) );
				listener.lookupTransform("/base_footprint", up_tag_tf.str(), ros::Time(0), transform_uptag);
			} catch (tf::TransformException ex) {
				ROS_ERROR("%s",ex.what());
			}
			ROS_INFO("Tag Position x: [%f], y: [%f], z: [%f]", transform_uptag.getOrigin().x(), transform_uptag.getOrigin().y(), transform_uptag.getOrigin().z());

			double Y,P,R;
			tf::Matrix3x3 m(transform_uptag.getRotation());
			m.getEulerYPR(Y, P, R);

			ROS_INFO("Tag Orientation z: [%f]", Y);

			arm_goal.goal.pose.position.x = transform_uptag.getOrigin().x()*0.81+0.021;
			arm_goal.goal.pose.position.y = transform_uptag.getOrigin().y()+0.022+0.0255-0.13*arm_goal.goal.pose.position.x;

			arm_goal.goal.pose.position.z = transform.getOrigin().z()+0.088;

			arm_goal.goal.pose.orientation.x = 0;
			arm_goal.goal.pose.orientation.y = -1.56;
			arm_goal.goal.pose.orientation.z = fmod((Y+M_PI+M_PI/4),(M_PI/2))-M_PI/4-M_PI/2;
			arm_goal.goal.pose.orientation.w = 1;
			arm_goal.goal.header.stamp = ros::Time::now();

			arm_goal.gripperposstart = -1;
			arm_goal.gripperposend = 0.011;

			publishArmPos(arm_goal);
			sleep(3);
			arm_goal.goal.pose.position.z = arm_goal.goal.pose.position.z-0.005;
			publishArmPos(arm_goal);
			//sleep(0.5);
			arm_goal.goal.pose.position.z = arm_goal.goal.pose.position.z-0.0025;
			publishArmPos(arm_goal);
			//sleep(1);
			arm_goal.goal.pose.position.z = arm_goal.goal.pose.position.z-0.0025;
			publishArmPos(arm_goal);
			//sleep(1);
			arm_goal.goal.pose.position.z = arm_goal.goal.pose.position.z-0.0025;
			arm_goal.gripperposend = 0.005;

			publishArmPos(arm_goal);
			sleep(5);
		}
	}

	/*
	 * Drop off the object actually hold with the gripper
	 */
	void drop(void){
		arm_goal.goal.pose.position.x = 0.38;
		arm_goal.goal.pose.position.y = -0.07;
		arm_goal.goal.pose.position.z = 0.015;
		arm_goal.goal.pose.orientation.x = 0;
		arm_goal.goal.pose.orientation.y = -1.56;
		arm_goal.goal.pose.orientation.z = 0.2;
		arm_goal.goal.pose.orientation.w = 0;
		arm_goal.gripperposstart = -1;
		arm_goal.gripperposend = 0.011;

		arm_goal.goal.header.stamp = ros::Time::now();

		publishArmPos(arm_goal);
		sleep(2);
	}

	/*
	 * Set the arm Position to to note crash during movement
	 */
	void transportpose(void){
		arm_goal.goal.pose.position.x = -0.04;
		arm_goal.goal.pose.position.y = 0;
		arm_goal.goal.pose.position.z = 0.18;
		arm_goal.goal.pose.orientation.x = 0;
		arm_goal.goal.pose.orientation.y = -1.5;
		arm_goal.goal.pose.orientation.z = 0;
		arm_goal.goal.pose.orientation.w = 0;
		arm_goal.gripperposstart = -1;
		arm_goal.gripperposend = -1;

		arm_goal.goal.header.stamp = ros::Time::now();

		publishArmPos(arm_goal);
		sleep(3);
	}

	/*
	 * Set the arm Position to to note crash during movement
	 */
	void ar_tag_pose(void){
		arm_goal.goal.pose.position.x = 0.18;
		arm_goal.goal.pose.position.y = 0;
		arm_goal.goal.pose.position.z = 0.38;
		arm_goal.goal.pose.orientation.x = 0;
		arm_goal.goal.pose.orientation.y = 0;
		arm_goal.goal.pose.orientation.z = 0.2-1.57;
		arm_goal.goal.pose.orientation.w = 0;
		arm_goal.gripperposstart = -1;
		arm_goal.gripperposend = -1;

		arm_goal.goal.header.stamp = ros::Time::now();

		publishArmPos(arm_goal);
		sleep(3);
	}

	/*
	 * Move the robot to the given position
	 */
	bool movebasetogoal(move_base_msgs::MoveBaseGoal goal){
		int found;
		tf::StampedTransform pos_transform;
		double dist = 0.5;
		double wait_time;
		geometry_msgs::Twist vel_new;

		//std::cout << "ar_kill" << std::endl;
		stop_cam_client.call(cam_srv);
		transportpose();

		goal.target_pose.header.stamp = ros::Time::now();
		while(!ac_.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		ac_.sendGoal(goal);

		ac_.waitForResult();
		if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Hooray, the base moved to the desired goal");

			//std::cout << "ar_start" << std::endl;
			start_cam_client.call(cam_srv);
			ar_tag_pose();
			//ros::Duration(10).sleep();
			sleep(2);
			ROS_INFO("TEST");
			for(int qn = 0; qn < 6; qn++){
				sleep(1);
				ros::spinOnce();
				ROS_INFO("Got new position: [%d]", (int)big_new);
				ROS_INFO("Distance x: [%f], y: [%f]", big_x, big_y);
				if(big_new){
					qn = 6;
					if(big_x > 0.05 || big_y > 0.05){

						big_new = false;
						vel_new.linear.x = 0.05;
						vel_new.linear.y = vel_new.linear.x*big_y/(big_x-dist);
						wait_time = (big_x-dist)/vel_new.linear.x;

						vel_pub_.publish(vel_new);
						ROS_INFO("wait time: [%f]", wait_time);
						//ros::Duration(wait_time).sleep();
						sleep(wait_time);
						vel_new.linear.x = 0;
						vel_new.linear.y = 0;
						vel_pub_.publish(vel_new);
					}
				}
			}
			return true;
		}
		else{
			ROS_INFO("The base failed to move to goal for some reason");
			return false;
		}
	}

	/*
		 * Move the robot to the given position
		 */
		bool movebasetogoal_no_tag(move_base_msgs::MoveBaseGoal goal){

			//std::cout << "ar_kill" << std::endl;
			stop_cam_client.call(cam_srv);
			transportpose();

			goal.target_pose.header.stamp = ros::Time::now();
			while(!ac_.waitForServer(ros::Duration(5.0))){
				ROS_INFO("Waiting for the move_base action server to come up");
			}
			ac_.sendGoal(goal);

			ac_.waitForResult();
			start_cam_client.call(cam_srv);
			if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("Hooray, the base moved to the desired goal");
				return true;
			}
			else{
				ROS_INFO("The base failed to move to goal for some reason");
				return false;
			}
		}


	bool task(industry_comm_msg::TaskProcessor::Request  &req,
			industry_comm_msg::TaskProcessor::Response &res)
	{

		ROS_INFO("bring: [%s]", req.bring ? "true":"false");
		//
		if(req.get){
			movebasetogoal(storage);
			pickup_diceroll((int)req.april_tag_number_get);

			task_feedback.get = true;
			task_feedback.get_success = true;
			publishresult(); 
			last_task_diceroll = false;
		}
		ROS_INFO("test");
		//
		if(req.bring){
			ROS_INFO("test2");
			ROS_INFO("Move to position x: [%f], y: [%f]", req.x_coord_bring, req.y_coord_bring);

			goal.target_pose.header.frame_id = "map";

			goal.target_pose.pose.position.x = req.x_coord_bring;
			goal.target_pose.pose.position.y = req.y_coord_bring;
			goal.target_pose.pose.orientation.z = req.orientation_bring;
			goal.target_pose.pose.orientation.x = 0;
			goal.target_pose.pose.orientation.y = 0;
			goal.target_pose.pose.orientation.w = sqrt(1-pow(goal.target_pose.pose.orientation.z,2));

			movebasetogoal(goal);
			drop();
			movebasetogoal_no_tag(waitingpose);
			task_feedback.bring = true;
			task_feedback.bring_success = true;
			publishresult();
			last_task_diceroll = false;
		}

		//Roll the dice
		if(req.diceroll){
			bool su = true;
			if(!last_task_diceroll)
				su = movebasetogoal(dicerollfield);
			if(su){

				//std::cout << "ar_start" << std::endl;
				//sleep(10);

				pickup_diceroll(0);


				arm_goal.goal.pose.position.x = 0.17;
				arm_goal.goal.pose.position.y = -0.21;
				arm_goal.goal.pose.position.z = 0.09;
				arm_goal.goal.pose.orientation.x = 0;
				arm_goal.goal.pose.orientation.y = -1.2;
				arm_goal.goal.pose.orientation.z = 0.2-1.57;
				arm_goal.goal.pose.orientation.w = 0;
				arm_goal.gripperposstart = -1;
				arm_goal.gripperposend = 0.011;

				arm_goal.goal.header.stamp = ros::Time::now();

				publishArmPos(arm_goal);
				sleep(2);

				arm_goal.goal.pose.position.x = 0.17;
				arm_goal.goal.pose.position.y = -0.18;
				arm_goal.goal.pose.position.z = 0.21;
				arm_goal.goal.pose.orientation.z = 0.2-3.14;
				arm_goal.goal.pose.orientation.x = 0;
				arm_goal.goal.pose.orientation.y = -1.35;
				arm_goal.goal.pose.orientation.w = 1;
				arm_goal.gripperposstart = -1;
				arm_goal.gripperposend = -1;

				arm_goal.goal.header.stamp = ros::Time::now();

				publishArmPos(arm_goal);
				sleep(3);
				tagid = 0;

				ros::spinOnce();
				if(tagid){
					ROS_INFO("Diceroll: [%d]",tagid);

					task_feedback.diceroll = true;
					task_feedback.dicevalue = tagid;
				}else{
					task_feedback.diceroll = false;
					task_feedback.dicevalue = 0;
				}


			}else{
				task_feedback.diceroll = false;
				task_feedback.dicevalue = 0;
			}
			publishresult();
			last_task_diceroll = true;
		}

		//Put item back into storage
		if(req.bringback){
			goal.target_pose.header.frame_id = "map";

			goal.target_pose.pose.position.x = req.x_coord_bring;
			goal.target_pose.pose.position.y = req.y_coord_bring;
			goal.target_pose.pose.orientation.z = req.orientation_bring;
			goal.target_pose.pose.orientation.x = 0;
			goal.target_pose.pose.orientation.y = 0;
			goal.target_pose.pose.orientation.w = sqrt(1-pow(goal.target_pose.pose.orientation.z,2));

			movebasetogoal(goal);
			pickup((int)req.april_tag_number_bringback);
			movebasetogoal(storage);
			drop();

			task_feedback.bringback = true;
			task_feedback.bringback_success = true;
			publishresult();
			last_task_diceroll = false;
		}

		res.success = true;
		return true;
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TaskProcessor");

	Processor task_processor(ros::this_node::getName());

	ros::spin();

	return 0;
}
