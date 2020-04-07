#include <string>
#include <sstream>
#include <unordered_map>
#include <iterator>

#include "mqtttopichandlers.hpp"
#include "logging.h"


using namespace std;
using namespace MQTTTopicHandlers;

static bool containsWildcards(const string &topic);

/*!
 *  \brief split String splitter template function: splits input string at delimiter
 *		and outputs substrings to result iterator
 */
template <typename Out>
void split(const string &s, char delim, Out result) {
	istringstream iss(s);
	string item;
	while(getline(iss, item, delim)) {
		*result++ = item;
	}
}

/*!
 * \brief The Comparator struct is used as a replacement predicate by unordered_map to determine
 *		if two elements are equal.
 * \details The default behaviour for strings is the regular string
 *		compare to determine if they represent the same data. Here, this behaviour is
 *		replaced such that e.g. the map key "astazero/projectA/vehicle1/velocity" matches
 *		the key "astazero/+/vehicle1/#".
 */
struct Comparator {
	bool operator()(const string &lhs, const string &rhs) const {
		if (!lhs.compare(rhs)){
			// Topic expressions identical: equivalent
			return true;
		}

		bool lhsIsExpression = containsWildcards(lhs);
		bool rhsIsExpression = containsWildcards(rhs);
		if (lhsIsExpression && rhsIsExpression) {
			// Expressions not equal and both contain wildcards: not equivalent
			return false;
		}
		else if (!lhsIsExpression && !rhsIsExpression) {
			// Both of the expressions are topic names and not equal: not equivalent
			return false;
		}

		// One of the strings is an expression and the other a topic name
		const string &expr = lhsIsExpression ? lhs : rhs;
		const string &topic = lhsIsExpression ? rhs : lhs;

		vector<string> exprTokens, topicTokens;
		split(expr, '/', back_inserter(exprTokens));
		split(topic, '/', back_inserter(topicTokens));

		if (topicTokens.size() < exprTokens.size()) {
			// Topic contains fewer levels than the expression thus they cannot match
			return false;
		}

		for (unsigned int i = 0; i < exprTokens.size(); ++i) {
			if (!exprTokens[i].compare("#") && topicTokens[i].size() != 0) {
				return true;
			}
			else if (!exprTokens[i].compare("+") && topicTokens[i].size() != 0) {
				continue;
			}
			else if (!exprTokens[i].compare(topicTokens[i])) {
				continue;
			}
			else {
				return false;
			}
		}

		return true;
	}
};



typedef unordered_map<string, MQTTTopicHandler, hash<string>, Comparator> HandlerMap;
/*!
 * \brief handlerMap maps MQTT topics to their respective handlers
 */
static HandlerMap handlerMap {
	{"astazero/+/+/motor-rpm/#", MQTTTopicHandlers::motorRPMHandler},
	{"astazero/+/+/MONR/#", MQTTTopicHandlers::monrHandler}
};

/*!
 * \brief containsWildcards Checks if a string contains any MQTT wildcard symbols
 * \param topic Topic string
 * \return True if the string contained any wildcards, false otherwise
 */
bool containsWildcards(const string &topic) {
	 bool retval = false;
	 const string singleLevelWildCard = "/+";
	 const string multiLevelWildCard = "/#";
	 // Check if topic contains any single level wildcards
	 retval = retval || topic.find(singleLevelWildCard + "/") != string::npos;
	 // Check if topic ends with a wildcard
	 retval = retval || !topic.compare(topic.length() - singleLevelWildCard.length(),
									  singleLevelWildCard.length(), singleLevelWildCard);
	 retval = retval || !topic.compare(topic.length() - multiLevelWildCard.length(),
									  multiLevelWildCard.length(), multiLevelWildCard);
	 return retval;
}



/*!
 * \brief MQTTTopicHandlers::handleMessage Delegates handling of a message to one of the
 *			handler functions specified in ::handlerMap depending on the input topic.
 * \param message Message to be handled
 * \param topic Topic on which the message was received
 * \return 0 on success, -1 otherwise
 */
int MQTTTopicHandlers::handleMessage(void* message, string& topic) {
	try {
		return handlerMap.at(topic)(message, topic);
	}
	catch (out_of_range) {
		LogMessage(LOG_LEVEL_ERROR, "No handler specified for topic %s", topic.c_str());
		return -1;
	}
}

/*! ********************************************************************
 *    Topic handlers
 *  ********************************************************************
 */

int MQTTTopicHandlers::monrHandler(void*, string& topic) {
	LogPrint("MONR handler not implemented. Topic %s", topic.c_str());
	return -1;
}

int MQTTTopicHandlers::motorRPMHandler(void* message, string &topic) {
	LogPrint("Received %s on topic %s", message, topic.c_str());
	return 0;
}
