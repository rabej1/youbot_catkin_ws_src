#include <ros/ros.h>
#include <string>

#include <industry_comm_msg/task.h>
#include <industry_comm_msg/ack.h>

#include "industry_comm_msg/TaskProcessor.h"

#include <dynamic_reconfigure/server.h>
#include <industry_comm/robotparamConfig.h>

std::string robot;
bool carry;
bool roboticarm;
bool mobile;

bool update;
bool ocp;

industry_comm_msg::ack task_ack;

industry_comm_msg::TaskProcessor srv;


void callback(industry_comm::robotparamConfig &config, uint32_t level)
{
	carry = config.carry;
	roboticarm = config.roboticarm;
	mobile = config.move;
	robot = config.robotname;
}

void taskCallback(const industry_comm_msg::task::ConstPtr& msg)
{
	if (msg->client_ID == robot){
	}
	else{
		if (ocp == false && update == false){
			if(msg->robot_ID == robot){
				ROS_INFO("Task [%d] accepted", msg->task_ID);

				srv.request.bring = msg->bring;
				srv.request.x_coord_bring = msg->x_coord_bring;
				srv.request.y_coord_bring = msg->y_coord_bring;
				srv.request.orientation_bring = msg->orientation_bring;
				srv.request.bringback = msg->bringback;
				srv.request.april_tag_number_bringback = msg->april_tag_number_bringback;
				srv.request.get = msg->get;
				srv.request.april_tag_number_get = msg->april_tag_number_get;
				srv.request.diceroll = msg->diceroll;

				ocp = true;
			}
			else{
				if(((msg->bring == false && msg->get == false && msg->bringback == false) || (carry == true && mobile == true)) && (msg->diceroll == false || roboticarm == true)){
					ROS_INFO("I heard: [%s] on task: [%d]", msg->robot_ID.c_str(),msg->task_ID);
					task_ack.client_ID = msg->client_ID;
					task_ack.robot_ID = robot;
					task_ack.task_ID = msg->task_ID;

					update = true;
				}
			}
		}
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "taskmanager");

	ros::NodeHandle n;
	ros::Publisher task_ack_pub = n.advertise<industry_comm_msg::ack>("task_ack", 1000);
	ros::Subscriber sub = n.subscribe("task_call", 1000, taskCallback);

	dynamic_reconfigure::Server<industry_comm::robotparamConfig> srv_p;
	dynamic_reconfigure::Server<industry_comm::robotparamConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	srv_p.setCallback(f);

	ros::ServiceClient client = n.serviceClient<industry_comm_msg::TaskProcessor>("TaskProcessor");
	update = false;
	ocp = false;


	ros::Rate loop_rate(10);

	while (ros::ok())
	{

		if (update == true){
			task_ack_pub.publish(task_ack);
			update = false;
		}
		if (ocp == true){
			if (client.call(srv))
			{
				//feedback
			}
			else
			{
				ROS_ERROR("Failed to call service TaskProcessor");
				return 1;
			}
			ocp = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
