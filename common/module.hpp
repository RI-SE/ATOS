#include <string>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Empty.h"

using std_msgs::Empty;
using std_msgs::String;
template <typename T, class C> 
class Topic {
public:
    // *** ska modulerna subscriba och publisha till alla kanaler? Ger liknande funktionalitet som tidigare, 
    // *** en modul "jackas in" och kan börja mottaga och skicka vilka meddelanden som helst
    Topic(const std::string topicName,int queueSize,void(C::*fp)(const T&),C* m) {
        this->pub=m->nh_. template advertise<T>(topicName, queueSize);
        this->sub=m->nh_.subscribe(topicName.c_str(), queueSize, fp, m);
    }
    void publish(T message){
        pub.publish(message);
    }
private:
    ros::Publisher pub;
    ros::Subscriber sub;
};

class Module {
    private:
        //void startCB(const Empty&) { };
        
        void getStatusCB(const Empty&) { 
            String message;
            message.data=this->name.c_str();
            getStatusOKTopic.publish(message);
            };
        
        void getStatusOKCB(const String&) { };
        void failureCB(const Empty&){ };
        virtual void initCB(const Empty&) { };
        virtual void connectCB(const Empty&){std::cout << "here!!!" << std::endl; };
        virtual void armCB(const Empty&) { };
        virtual void startCB(const Empty&) { };
    public:
        //ros::NodeHandle nh_;
        ros::NodeHandle nh_;
        Module(const std::string name );
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
