/*!
 * \file
 * \brief 
 * \author Maciej,,,
 */

#ifndef INTPROXY_HPP_
#define INTPROXY_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "ros/ros.h"
#include "std_msgs/Int32.h"

namespace Processors {
namespace IntProxy {

/*!
 * \class IntProxy
 * \brief IntProxy processor class.
 *
 * IntProxy processor.
 */
class IntProxy: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	IntProxy(const std::string & name = "IntProxy");

	/*!
	 * Destructor
	 */
	virtual ~IntProxy();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


// Input data streams

		Base::DataStreamIn<int> in_data;

// Output data streams

	// Handlers
	Base::EventHandler2 h_onNewData;
		Base::Property<std::string> ros_topic_name;
		Base::Property<std::string> ros_namespace;

	
	// Handlers
	void onNewData();

	ros::Publisher pub;
	ros::Subscriber sub;
	ros::NodeHandle * nh;

	void callback(const std_msgs::Int32ConstPtr& msg);
};

} //: namespace IntProxy
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("IntProxy", Processors::IntProxy::IntProxy)

#endif /* INTPROXY_HPP_ */
