#ifndef OROCOS_TFROBOTSUBSCRIBER_COMPONENT_HPP
#define OROCOS_TFROBOTSUBSCRIBER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/typekit/Types.hpp>
#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <tfRobotSubscriber/tfRobotSubscriber-types.hpp>

class TfRobotSubscriber : public RTT::TaskContext
{
	RTT::InputPort< std::vector<DirectKinematicsData> > entree;
	tf::StampedTransform  robot_transform;
	std::vector<DirectKinematicsData> dataCState;
	RTT::InputPort <unsigned int> size ;
	unsigned int datasize;
	public:
		TfRobotSubscriber(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
		void addPorts();
};
#endif
