#include "ros/ros.h"
#include "industry_comm_msg/TaskProcessor.h"

#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<youbot_arm_server::YouBotGoalAction> ArmClient;

move_base_msgs::MoveBaseGoal goal;

std::vector<int> tagid;

//static const std::string apriltagtf[] = {"ar_marker_1","ar_marker_2","ar_marker_3","ar_marker_4","ar_marker_5","ar_marker_6","ar_marker_7","ar_marker_8","ar_marker_9"};



class Processor {

protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
	actionlib::SimpleActionClient<youbot_arm_server::YouBotGoalAction> ac_arm_;
	ros::Subscriber sub;
	ros::ServiceServer service;
	std::string action_name_;
	tf::TransformListener listener;

public:

	move_base_msgs::MoveBaseGoal goal;
	youbot_arm_server::YouBotGoalGoal arm_goal;


	Processor(std::string name):
		ac_("move_base", true),
		ac_arm_("move_arm", true),
		action_name_(name) {

		sub = nh_.subscribe("/visualization_marker", 100, &Processor::poseCB, this);
		service = nh_.advertiseService("TaskProcessor", &Processor::task, this);

		ros::AsyncSpinner spinner(1);
		spinner.start();
	}

	void poseCB(const visualization_msgs::Marker msg){

		//		ar_track_alvar_msgs::AlvarMarkersConstPtr
		//		ar_track_alvar_msgs::AlvarMarker bb = msg.markers;
		//		bb.id;
		if(msg.ns == "basic_shapes")
			tagid.push_back((int)msg.id);
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

	bool task(industry_comm_msg::TaskProcessor::Request  &req,
			industry_comm_msg::TaskProcessor::Response &res)
	{
		bool foundtag = false;
		int pos = 0;
		//moving the Robot to the given Position
		if(req.move){
			ROS_WARN("This Robot cant move");
		}

		//Pick up the object with the given April-Tag
		if(req.grip){
			ROS_INFO("Pick up object with april-tag number: [%d]", req.april_tag_number);

			tf::StampedTransform transform, transform_uptag;
			std::stringstream apriltagtf, up_tag_tf;
			int apriltagnbr = req.april_tag_number;
			double offset;
			//apriltagtf.clear();
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
				//ROS_INFO("Tag Orientation x: [%f], y: [%f], z: [%f], w: [%f]", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());

				tagid.clear();
				ros::spinOnce();
				up_tag_tf << "ar_marker_" <<  tagid.at(tagid.size()-1);
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

				arm_goal.goal.pose.position.z = transform.getOrigin().z()+0.097;


				//				arm_goal.goal.pose.position.x = transform.getOrigin().x();
				//				arm_goal.goal.pose.position.y = transform.getOrigin().y();
				//				arm_goal.goal.pose.position.z = transform.getOrigin().z()+0.05;

				arm_goal.goal.pose.orientation.x = 0;
				arm_goal.goal.pose.orientation.y = -1.56;
				//arm_goal.goal.pose.orientation.z = atan(tan(Y))+offset;
				arm_goal.goal.pose.orientation.z = fmod(Y,(M_PI/2))+offset;
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
//				arm_goal.goal.pose.position.z = arm_goal.goal.pose.position.z-0.0025;
//				publishArmPos(arm_goal);
				//sleep(1);
				arm_goal.goal.pose.position.z = arm_goal.goal.pose.position.z-0.0025;
				arm_goal.gripperposend = 0.006;

				publishArmPos(arm_goal);
				sleep(5);

			}
		}

		//Place the Object at the back
		if(req.carry){
			//			arm_goal.goal.pose.position.x = -0.04;
			//			arm_goal.goal.pose.position.y = 0;
			//			arm_goal.goal.pose.position.z = 0.18;
			//			arm_goal.goal.pose.orientation.x = 0;
			//			arm_goal.goal.pose.orientation.y = -1.5;
			//			arm_goal.goal.pose.orientation.z = 0;
			//			arm_goal.goal.pose.orientation.w = 0;

			arm_goal.goal.pose.position.x = 0.38;
			arm_goal.goal.pose.position.y = 0;
			arm_goal.goal.pose.position.z = 0.18;
			arm_goal.goal.pose.orientation.x = 0;
			arm_goal.goal.pose.orientation.y = -0.8;
			arm_goal.goal.pose.orientation.z = 0.2;
			arm_goal.goal.pose.orientation.w = 0;
			arm_goal.gripperposstart = -1;
			arm_goal.gripperposend = 0.011;

			arm_goal.goal.header.stamp = ros::Time::now();

			publishArmPos(arm_goal);
			sleep(2);


			arm_goal.goal.pose.position.x = 0.35;
			arm_goal.goal.pose.position.y = 0;
			arm_goal.goal.pose.position.z = 0.21;
			arm_goal.goal.pose.orientation.z = 0.2-1.57;
			arm_goal.goal.pose.orientation.x = 0;
			arm_goal.goal.pose.orientation.y = -1.35;
			arm_goal.goal.pose.orientation.w = 1;
			arm_goal.gripperposstart = -1;
			arm_goal.gripperposend = -1;
			arm_goal.goal.header.stamp = ros::Time::now();

			publishArmPos(arm_goal);
			sleep(3);
			tagid.clear();
			ros::spinOnce();

			ROS_INFO("Diceroll: [%d]",tagid.at(tagid.size()-1));
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
