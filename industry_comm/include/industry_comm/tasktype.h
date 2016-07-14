/*
 * tasktype.h
 *
 *  Created on: May 24, 2016
 *      Author: jonas
 */

#include <string>

#ifndef TASKTYPE_H_
#define TASKTYPE_H_

class TaskType
{
public:

	int task_ID;
	double x_coord;
	double y_coord;
	std::string client_ID;
	bool mobile;
	bool roboticarm;
	bool carry;
	int priority;
	std::string robot_ID;

private:

};



#endif /* TASKTYPE_H_ */
