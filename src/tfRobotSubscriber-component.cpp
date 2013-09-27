#include "tfRobotSubscriber-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

TfRobotSubscriber::TfRobotSubscriber(std::string const& name) 
	: TaskContext(name)
{
	//tell the RTT the name and type of this struct
	RTT::types::Types()->addType(new DirectKinematicsDataTypeInfo());
	RTT::types::Types()->type("DirectKinematicsData")->addConstructor( RTT::types::newConstructor(&createCD) );
	
	//register a std::vector (or compatible) of some type
	RTT::types::Types()->addType(new RTT::types::SequenceTypeInfo< std::vector<DirectKinematicsData> > ("std::vector<DirectKinematicsData>"));

	this->addProperty("dataCState", dataCState)
		.doc("dataCState : vector that contrains the received data");

	this->addProperty("datasize", datasize)
		.doc("data size");

	this->addPort("inCartesianState",entree)
		.doc("kinematic chain of the robot");

	this->addEventPort("inCartesianState", entree);
	this->addPort("size",size);
	this->addEventPort("size",size);

	std::cout << "TfRobotSubscriber constructed !" <<std::endl;

}
//-----------------------------------
bool TfRobotSubscriber::configureHook()
{	
	datasize = 0;
	std::cout << "TfRobotSubscriber configured !" <<std::endl;
	return true;
}
//-----------------------------------
bool TfRobotSubscriber::startHook()
{
	std::cout << "TfRobotSubscriber started !" <<std::endl;
	return true;
}
//-----------------------------------
void TfRobotSubscriber::updateHook()
{
	size.read(datasize);
	std::vector<double> vec(3,100),rot(4,100);
	entree.read(dataCState);

	for(unsigned int i = 1 ;  i< dataCState.size() ; i++)
	{
		dataCState[i].frame_id = "idOfTheRobotSegment" ;
		dataCState[i].vector = vec;
		dataCState[i].rotation = rot;
	}

	RTT::FlowStatus fs = entree.read(dataCState);
/*
	if(fs == RTT::NewData) 
	{
		std::cout << "Got New Data " << std::endl;
	}
	else
	{
		std::cout << "No new Data " << std::endl;
	}

*/
	for(unsigned int i=1 ; i < dataCState.size(); i++)
	{
		robot_transform.setRotation( tf::Quaternion(dataCState[i].rotation[0], dataCState[i].rotation[1], dataCState[i].rotation[2], dataCState[i].rotation[3] ) );
		robot_transform.setOrigin( tf::Vector3(dataCState[i].vector[0],dataCState[i].vector[1],dataCState[i].vector[2]) );

		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(robot_transform, ros::Time::now(), "world",dataCState[i].frame_id ));
	}

}

void TfRobotSubscriber::stopHook() 
{
	std::cout << "TfRobotSubscriber executes stopping !" <<std::endl;
}

void TfRobotSubscriber::cleanupHook() 
{
	std::cout << "TfRobotSubscriber cleaning up !" <<std::endl;
}
/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(TfRobotSubscriber)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(TfRobotSubscriber)
