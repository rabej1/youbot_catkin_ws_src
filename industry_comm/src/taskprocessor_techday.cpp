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

typedef actionlib::SimpleActionClient<youbot_arm_server::YouBotGoalAction> ArmClient;


bool last_task_diceroll = false;
//std::vector<int> tagid;
int tagid;


class Processor {

protected:

	ros::NodeHandle nh_;
	actionlib::SimpleActionClient<youbot_arm_server::YouBotGoalAction> ac_arm_;
	ros::Subscriber sub;

	ros::Publisher pub;
	ros::ServiceServer service;
	std::string action_name_;
	tf::TransformListener listener;


	industry_comm_msg::feedback task_feedback;


public:

	move_base_msgs::MoveBaseGoal goal;
	youbot_arm_server::YouBotGoalGoal arm_goal;


	Processor(std::string name):
		ac_arm_("move_arm", true),
		action_name_(name) {

		sub = nh_.subscribe("/uptag_visualization", 100, &Processor::poseCB, this);

		service = nh_.advertiseService("TaskProcessor", &Processor::task, this);
		pub = nh_.advertise<industry_comm_msg::feedback>("taskfeedback",100);


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

	bool task(industry_comm_msg::TaskProcessor::Request  &req,
			industry_comm_msg::TaskProcessor::Response &res)
	{
		bool foundtag = false;
		int pos = 0;
		bool notag = false;
		tf::StampedTransform transform_uptag;
		std::stringstream  up_tag_tf;

		up_tag_tf << "ar_marker_0";
		while(ros::ok()){
			ros::spinOnce();
			sleep(1);

			try {
				listener.waitForTransform("/usb_cam", up_tag_tf.str(), ros::Time(0), ros::Duration(3.0) );
				listener.lookupTransform("/usb_cam", up_tag_tf.str(), ros::Time(0), transform_uptag);
				//ROS_INFO("Transformationage: [%d]", ros::Time::now().sec-transform.stamp_.sec);

				if((ros::Time::now().sec-transform_uptag.stamp_.sec) < 3){
					foundtag = true;
				}
				else{
					if(!notag)
						ROS_INFO("ready to roll the dice");
					foundtag = false;
					notag = true;
				}
			} catch (tf::TransformException ex) {
				ROS_INFO("catch");
				ROS_ERROR("%s",ex.what());
				foundtag = false;
				notag = true;
			}


			if (foundtag && notag){
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
				notag = false;
				foundtag = false;
				publishresult();
			}
		}
		return 1;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "TaskProcessor");

	ROS_INFO("Task Processor up and running");
	Processor task_processor(ros::this_node::getName());

	ros::spin();

	return 0;
}
