#include <string>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

using std_msgs::Empty;
using std_msgs::String;
/** 
 *  Topic for publishing and subscribing to messages
 * 
 *  @tparam Msg Type of the message to be sent / recevied on this topic
 *  @tparam Node ROS node that should subscribe / publish on this topic
 * 
*/
template <typename Msg, class Node> 
class Topic {
public:
   	/**
    * Default constructor
    * @param topicName name of topic
	* @param queueSize maximum number of messages the topic can buffer before deleting new messages
	* @param cb callback function 
	* @param n ROS node subscribing/publishing to this topic
    */
    Topic(const std::string topicName,int queueSize,void(Node::*cb)(const Msg&),Node* n) {
        this->pub = n->template advertise<Msg>(topicName, queueSize);
        this->sub = n->subscribe(topicName, queueSize, cb, n);
    }
    void publish(Msg message){
        pub.publish(message);
    }
private:
    ros::Publisher pub;
    ros::Subscriber sub;
};

class Module : public ros::NodeHandle {
    private:        
        void getStatusCB(const Empty&) { 
            String message;
            message.data=this->name;
            getStatusOKTopic.publish(message);
            };
        
        void getStatusOKCB(const String&) { };
        void failureCB(const Empty&){ };
        virtual void initCB(const Empty&) { };
        virtual void connectCB(const Empty&){  };
        virtual void armCB(const Empty&) { };
        virtual void startCB(const Empty&) { };

    public:
        //ros::NodeHandle nh_;
        //ros::NodeHandle nh_;
        Module(const std::string name) :  name(name) {};
        //Topic<Empty, Module> strtTopic = Topic<Empty, Module> (std::string("/start"),1000,&Module::startCB,this);
        //Topic<Empty, Module> getStatusTopic = Topic<Empty, Module> ("/getStatus",1000,&Module::getStatusCB,this);
        Topic<String, Module> getStatusOKTopic = Topic<String, Module> (std::string("/getStatus"),1000,&Module::getStatusOKCB,this);
        Topic<Empty, Module> failureTopic = Topic<Empty, Module> (std::string("/failure"),1000,&Module::failureCB,this);
        Topic<Empty, Module> initTopic = Topic<Empty, Module>("/init",1000,&Module::initCB,this);
		Topic<Empty, Module> connectTopic = Topic<Empty, Module>("/connect",1000,&Module::connectCB,this);
		Topic<Empty, Module> armTopic = Topic<Empty, Module>("/arm",1000,&Module::armCB,this);
        Topic<Empty, Module> startTopic = Topic<Empty, Module>("/start",1000,&Module::startCB,this);
        std::string name;

};
