#ifndef MQTTTOPICHANDLERS_H
#define MQTTTOPICHANDLERS_H
#include <string>
#include <vector>

using namespace std;

namespace MQTTTopicHandlers {
	typedef int (*MQTTTopicHandler)(void*, string&);

	int handleMessage(void* message, string &topic);
	int monrHandler(void*, string &topic);
	int motorRPMHandler(void* message, string &topic);

	vector<string> getSubscriptions(void);
}

#endif // MQTTTOPICHANDLERS_H
