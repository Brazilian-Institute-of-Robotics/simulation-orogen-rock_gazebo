/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
//======================================================================================
// Brazilian Institute of Robotics 
// Authors: Thomio Watanabe
// Date: December 2014
//====================================================================================== 
// add joints, position and velocity
//======================================================================================
#include "ModelTask.hpp"

//======================================================================================
gazebo::ModelTask::ModelTask(std::string const& name)
	: ModelTaskBase(name)
{
}
//======================================================================================
gazebo::ModelTask::ModelTask(std::string const& name, RTT::ExecutionEngine* engine)
	: ModelTaskBase(name, engine)
{
}
//======================================================================================
void gazebo::ModelTask::setGazeboModel(physics::WorldPtr _world,  physics::ModelPtr _model)
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
void gazebo::ModelTask::setJointPorts()
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
void gazebo::ModelTask::setLinkPorts()
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
void gazebo::ModelTask::updateHook()
{
    updateJoints();
    updateLinks();
}
//======================================================================================
void gazebo::ModelTask::updateJoints()
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
void gazebo::ModelTask::updateLinks()
{
	// Update all links from gazebo model
	for(LinkPort_V::iterator it = link_port_list.begin(); it != link_port_list.end(); ++it)
	{
		physics::LinkPtr link = (*it).second;

		// Read rock input port and apply a force to gazebo links
		base::Vector3d force(0.0, 0.0, 0.0);
		(*it).first->readNewest(force);				
		if((force(0) != RTT::NoData)||(force(1) != RTT::NoData)||(force(2) != RTT::NoData)){
			link->AddRelativeForce( gazebo::math::Vector3(force(0),force(1),force(2)) );
		}
	}
}
//======================================================================================
gazebo::ModelTask::~ModelTask()
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
