#include <iostream>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <math.h>
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
			//if(str.find("LINE"))
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
		int count = 0;
		while ((pos = str.find(delimiter)) != std::string::npos) {
			token = str.substr(0, pos);
			std::cout << "pos: " << pos << " Token: " << token << std::endl;

			if(count == 1){
				timeVector.push_back(token);
			}
			count ++;
			str.erase(0, pos + delimiter.length());
		}
	}

	std::cout << "REVERSED: \n";

	std::cout << header << "\n";
	while (!fileVector.empty())
	  {
		//std::cout << "Running: " << str << std::endl;

		size_t pos = 0;
		int count = 0;
		std::string token;
		std::string newTimeStr;
		std::string line = fileVector.back();
		while ((pos = line.find(delimiter)) != std::string::npos) {
			token = line.substr(0, pos);
			if(count == 1){
				//std::cout << token << std::endl;
				token = timeVector.front();
				timeVector.erase(timeVector.begin());
			}
			if(count == 5){
				double newAngle = std::stof(token) + (M_PI);
				std::ostringstream ss;
				ss << newAngle;
				std::string s(ss.str());
				token = s;
			}
			newTimeStr.append(token);
			newTimeStr.append(";");
			count++;
			line.erase(0, pos + delimiter.length());

		}
		std::cout << newTimeStr << "\n";

		//std::cout << fileVector.back() << "\n";
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
