/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
//======================================================================================
// Brazilian Institute of Robotics 
// Authors: Thomio Watanabe
// Date: December 2014
//====================================================================================== 
// add joints, position and velocity
//======================================================================================
#include "ModelTask.hpp"

using namespace gazebo;
using namespace rock_gazebo;

//======================================================================================
ModelTask::ModelTask(std::string const& name)
	: ModelTaskBase(name)
{
}
//======================================================================================
ModelTask::ModelTask(std::string const& name, RTT::ExecutionEngine* engine)
	: ModelTaskBase(name, engine)
{
}
//======================================================================================
void ModelTask::setGazeboModel(WorldPtr _world,  ModelPtr _model)
{
    std::string name = "gazebo:" + _world->GetName() + ":" + _model->GetName();
    provides()->setName(name);
    _name.set(name);

	world = _world;
	model = _model;
//	sdf = model->GetSDF();
	
//	Export Gazebo Joints and Links to Rock-Robotics
	setJointPorts();
	setLinkPorts();
} 
//======================================================================================
void ModelTask::setJointPorts()
{
    // base::samples::Joints rock_joint; 
	// Get all joints from a model and creates a Rock component InputPort
	joints = model->GetJoints();
	for(Joint_V::iterator joint = joints.begin(); joint != joints.end(); ++joint)
	{
		gzmsg << "RockBridge: found joint: " << world->GetName() + "/" + model->GetName() + 
				"/" + (*joint)->GetName() << std::endl;
		gzmsg << "RockBridge: create joint InputPort in Rock." << std::endl;
		joint_port = new RTT::InputPort<double>( "_joint_" + (*joint)->GetName() + "_in");
		ports()->addPort( *joint_port );
		joint_port_list.push_back( std::make_pair(joint_port,*joint) );			
	}
}
//======================================================================================
void ModelTask::setLinkPorts()
{
    // Get all links from a model and creates a Rock component InputPort
	links = model->GetLinks();
	for(Link_V::iterator link = links.begin(); link != links.end(); ++link)
	{
		gzmsg << "RockBridge: found link: " << world->GetName() + "/" + model->GetName() + 
				"/" + (*link)->GetName() << std::endl;
		gzmsg << "RockBridge: create link InputPort in Rock." << std::endl;
		link_port = new RTT::InputPort<base::Vector3d>( "_link_" + (*link)->GetName() + "_in");
		ports()->addPort( *link_port );
		link_port_list.push_back( std::make_pair(link_port,*link) );
	}
}
//======================================================================================
void ModelTask::updateHook()
{
    updateJoints();
    updateLinks();
}
//======================================================================================
void ModelTask::updateJoints()
{
	for(JointPort_V::iterator it = joint_port_list.begin();it != joint_port_list.end(); ++it)
	{
		double effort = 0;	
		(*it).first->readNewest(effort);
		if(effort != RTT::NoData)
		{
			// Define the limits for the joint effort
			try{
			    if((-1.0 <= effort) && (effort <= 1.0)){
			        (*it).second->SetForce(0,effort);
			    }else{
			        throw;
			    }
			}catch(...){
			    // Joint effort value out of range 
			    gzmsg << "RockBridge: error - effort value out of range (-1 <= effort <= 1)." << std::endl;
			}
		}
	}
}
//======================================================================================
void ModelTask::updateLinks()
{
	// Update all links from gazebo model
	for(LinkPort_V::iterator it = link_port_list.begin(); it != link_port_list.end(); ++it)
	{
		LinkPtr link = (*it).second;

		// Read rock input port and apply a force to gazebo links
		base::Vector3d force(0.0, 0.0, 0.0);
		(*it).first->readNewest(force);				
		if((force(0) != RTT::NoData)||(force(1) != RTT::NoData)||(force(2) != RTT::NoData)){
			link->AddRelativeForce( math::Vector3(force(0),force(1),force(2)) );
		}
	}
}
//======================================================================================
ModelTask::~ModelTask()
{
	delete joint_port;
	delete link_port;
	
	for(JointPort_V::iterator it = joint_port_list.begin();it != joint_port_list.end(); ++it)
		delete (it)->first;

	for(LinkPort_V::iterator it = link_port_list.begin(); it != link_port_list.end(); ++it)
		delete it->first; 

	joint_port_list.clear();
	link_port_list.clear();
}
//======================================================================================
