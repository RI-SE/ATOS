#ifndef MQTTTOPICHANDLERS_H
#define MQTTTOPICHANDLERS_H
#include <string>

using namespace std;

namespace MQTTTopicHandlers {
	typedef int (*MQTTTopicHandler)(void*, string&);

	int handleMessage(void* message, string &topic);
	int monrHandler(void* message, string &topic);
	int motorRPMHandler(void* message, string &topic);
}

#endif // TOPICHANDLERS_H
