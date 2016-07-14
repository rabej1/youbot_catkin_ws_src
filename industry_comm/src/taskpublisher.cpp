#include <ros/ros.h>
//#include <actionlib/server/simple_action_server.h>
//#include <industry_comm/TaskPublishAction.h>
#include <string>

#include <industry_comm_msg/task.h>
#include <industry_comm_msg/ack.h>

#include "industry_comm_msg/TaskPublisher.h"

#include <dynamic_reconfigure/server.h>
#include <industry_comm/robotnameConfig.h>


std::string robot;
const std::string robot_open = "open";

bool answer[100];
std::string answer_robot;

ros::ServiceServer service;

std::vector<int> it;
std::vector<int> publishcounter;
std::vector<industry_comm_msg::task> task;
int taskcounter = 0;

void callback(industry_comm::robotnameConfig &config, uint32_t level)
{
	robot = config.robotname;
}

void ackCallback(const industry_comm_msg::ack::ConstPtr& msg)
{
	if (msg->client_ID == robot){
		if (msg->robot_ID == robot_open)
			ROS_INFO("got answer without robot id");
		else{
			ROS_INFO("Robot answered: [%s]", msg->robot_ID.c_str());
			ROS_INFO("Task ID: [%d]", msg->task_ID);
			answer[msg->task_ID] = true;
			answer_robot = msg->robot_ID;
		}
	}
}

bool addtask(industry_comm_msg::TaskPublisher::Request  &req,
		industry_comm_msg::TaskPublisher::Response &res){

	industry_comm_msg::task settask;

	settask.client_ID = robot;
	settask.priority = 1;
	settask.robot_ID = robot_open;
	settask.task_ID = taskcounter;
	
	settask.bring = req.bring;
	settask.x_coord_bring = req.x_coord_bring;
	settask.y_coord_bring = req.y_coord_bring;
	settask.orientation_bring = req.orientation_bring;
	settask.get = req.get;
	settask.april_tag_number_get = req.april_tag_number_get;
	settask.bringback = req.bringback;
	settask.april_tag_number_bringback = req.april_tag_number_bringback;
	settask.diceroll = req.diceroll;

	task.push_back(settask);
	publishcounter.push_back(0);
	it.push_back(1000);

	++taskcounter;

	res.success = true;
	return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "taskpublisher");

	ros::NodeHandle n;

	service = n.advertiseService("Taskadder", addtask);
	//   ros::spin();

	ros::Publisher task_call_pub = n.advertise<industry_comm_msg::task>("task_call", 1000);
	ros::Subscriber sub = n.subscribe("task_ack", 1000, ackCallback);
	ros::Rate loop_rate(10);

	dynamic_reconfigure::Server<industry_comm::robotnameConfig> srv_p;
	dynamic_reconfigure::Server<industry_comm::robotnameConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	srv_p.setCallback(f);

//	industry_comm_msg::task ettask;

//	ettask.client_ID = robot;
//	ettask.priority = 1;
//	ettask.robot_ID = robot_open;
//	ettask.task_ID = 20;
//	ettask.bring = false;
//	ettask.x_coord_bring = 10.207;
//	ettask.y_coord_bring = 2.646;
//	ettask.orientation_bring = -0.647;
//	ettask.get = false;
//	ettask.april_tag_number_get = 0;
//	ettask.bringback = false;
//	ettask.april_tag_number_bringback = 0;
//	ettask.diceroll = true;

//	task.push_back(ettask);
//	publishcounter.push_back(0);
//	it.push_back(1000);

	while (ros::ok())
	{
		//ROS_INFO("Number of tasks: [%ld]", task.size());
		for(long i = 0; i < task.size(); i++){
			if (it.at(i) > 40){
				task_call_pub.publish(task.at(i));
				it.at(i) = 0;
				++publishcounter.at(i);
				ROS_INFO("Task [%d] published", task.at(i).task_ID);
			}
			++it.at(i);
			if (answer[task.at(i).task_ID]){
				task.at(i).robot_ID = answer_robot;
				task_call_pub.publish(task.at(i));
				answer[task.at(i).task_ID] = false;
				ROS_INFO("Task with ID [%d] assigned", task.at(i).task_ID);
				publishcounter.at(i) = 101;
				//publishcounter.at(i) = 1;
			}
			if (publishcounter.at(i) > 100){
				ROS_INFO("Task [%d] deleted", task.at(i).task_ID);
				task.erase(task.begin()+i);
				it.erase(it.begin()+i);
				publishcounter.erase(publishcounter.begin()+i);

			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}




/*class TaskPublishAction
  {
  protected:

    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<industry_comm::TaskPublishAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    industry_comm::TaskPublishAction feedback_;
    industry_comm::TaskPublishAction result_;

  public:

    TaskPublishAction(std::string name) :
      as_(nh_, name, boost::bind(&TaskPublishAction::executeCB, this, _1), false),
      action_name_(name)
    {
      as_.start();
    }



    void executeCB(const industry_comm::TaskPublishGoalConstPtr &goal)
    {

  	  TaskType task1;
  	  task1.x_coord = 2;

      // helper variables
      ros::Rate r(1);
      bool success = true;


      if(success)
      {
        result_.sequence = feedback_.sequence;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
    }


  };


  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "fibonacci");

    FibonacciAction fibonacci(ros::this_node::getName());
    ros::spin();

    return 0;
  }*/
