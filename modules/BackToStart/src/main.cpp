#include <iostream>
#include <unistd.h>
#include <math.h>
#include "trajectory.hpp"
#include "logging.h"
#include "util.h"

#define MODULE_NAME "BackToStart"

int main()
{
	COMMAND command = COMM_INV;
	char mqRecvData[MQ_MSG_SIZE];
	const struct timespec sleepTimePeriod = {0,10000000};
	struct timespec remTime;

	LogInit(MODULE_NAME,LOG_LEVEL_DEBUG);
	LogMessage(LOG_LEVEL_INFO, "Task running with PID: %u", getpid());

	// Initialize message bus connection
	while (iCommInit()) {
		nanosleep(&sleepTimePeriod,&remTime);
	}

	while (true) {
		if (iCommRecv(&command,mqRecvData,MQ_MSG_SIZE,nullptr) < 0) {
			util_error("Message bus receive error");
		}

		switch (command) {
		case COMM_INV:
			nanosleep(&sleepTimePeriod,&remTime);
			break;
		case COMM_OBC_STATE:
			break;
		case COMM_INIT:{
			Trajectory traj;
			traj.initializeFromFile("VO20kmIntersection");
			std::cout << "TRAJ UNREVERSED: \n";
			std::cout << traj.toString();
			Trajectory reversedTraj;
			traj.reverse(reversedTraj);
			std::cout << "TRAJ REVERSED: \n";
			std::cout <<  reversedTraj.toString();
			break;
		}
		default:
			LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
		}
	}

	return 0;
}
