#include <iostream>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include "logging.h"
#include "util.h"

#define MODULE_NAME "BackToStart"

int reverseTraj()
{
	std::vector<std::string> fileVector;
	std::vector<std::string> timeVector;

	std::ifstream in("/home/jesper/Documents/gitz/Maestro/core/traj/GarageplanInnerring.traj");
	std::string str;

	while (std::getline(in, str))
	{
		// Line contains string of length > 0 then save it in vector
		std::cout << str << "\n";
		if(str.size() > 0){
			fileVector.push_back(str);
		}
	}

	std::string header = fileVector.front();
	fileVector.erase(fileVector.begin());
	std::string endLine = fileVector.back();
	fileVector.pop_back();

	std::string delimiter = ";";

	for(int i=0; i < fileVector.size(); i++){
		std::string str = fileVector.at(i);
		size_t pos = 0;
		std::string token;
		while ((pos = str.find(delimiter)) != std::string::npos) {
			token = str.substr(0, pos);
			if(pos == 5){
				std::cout << token << std::endl;
				timeVector.push_back(token);
			}
			str.erase(0, pos + delimiter.length());
		}
	}

	std::cout << "REVERSED: \n";

	std::cout << header << "\n";
	while (!fileVector.empty())
	  {
		std::cout << fileVector.back() << "\n";
		fileVector.pop_back();
	  }
	std::cout << endLine << "\n";

}

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
		case COMM_INIT:
			reverseTraj();

		default:
			LogMessage(LOG_LEVEL_INFO,"Received command %u",command);
		}
	}

	return 0;
}
