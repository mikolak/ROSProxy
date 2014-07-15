/*!
 * \file
 * \brief
 * \author Maciej,,,
 */

#include <memory>
#include <string>

#include "IntProxy.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace IntProxy {

IntProxy::IntProxy(const std::string & name) :
		Base::Component(name) , 
		ros_topic_name("ros.topic_name", std::string("int")), 
		ros_namespace("ros.namespace", std::string("discode")) {
		registerProperty(ros_topic_name);
		registerProperty(ros_namespace);

}

IntProxy::~IntProxy() {
}

void IntProxy::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_data", &in_data);

	// Register handlers
	h_onNewData.setup(boost::bind(&IntProxy::onNewData, this));
	registerHandler("onNewData", &h_onNewData);
//	addDependency("onNewData", &in_data);
	addDependency("onNewData", NULL);

}

bool IntProxy::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
	nh = new ros::NodeHandle;
	pub = nh->advertise<std_msgs::Int32>(ros_topic_name, 1000);
	sub = nh->subscribe("my_topic", 1, &IntProxy::callback, this);
	return true;
}

bool IntProxy::onFinish() {
	delete nh;
	return true;
}

bool IntProxy::onStop() {
	return true;
}

bool IntProxy::onStart() {
	return true;
}

void IntProxy::onNewData() {
	std_msgs::Int32 msg;
//	msg.data = in_data.read();
	msg.data = 5;
	pub.publish(msg);
	ros::spinOnce();
}

void IntProxy::callback(const std_msgs::Int32ConstPtr& msg) {
	CLOG(LNOTICE) << "Received from ROS: " << msg->data;
}


} //: namespace IntProxy
} //: namespace Processors
