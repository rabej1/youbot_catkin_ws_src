#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <youbot_arm_server/YouBotGoalAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_youbot_arm");

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
	}

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<youbot_arm_server::YouBotGoalAction> ac("youbot_arm_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  youbot_arm_server::YouBotGoalGoal goal;
  goal.goal.pose.position.x = params[0];
  goal.goal.pose.position.y = params[1];
  goal.goal.pose.position.z = params[2];
  goal.goal.pose.orientation.y = params[3];
  goal.goal.pose.orientation.z = params[4];

  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();

    ROS_INFO("Action finished: %s",state.toString().c_str());

  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
