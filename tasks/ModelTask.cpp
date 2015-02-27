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
		joint_in_port = new RTT::InputPort<double>( "_joint_" + (*joint)->GetName() + "_in");
		ports()->addPort( *joint_in_port );
		joint_in_port_list.push_back( std::make_pair(joint_in_port,*joint) );

		gzmsg << "RockBridge: create joint OutputPort in Rock." << std::endl;
		joint_out_port = new RTT::OutputPort<base::Vector3d>( "_joint_" + (*joint)->GetName() + "_out");
		ports()->addPort( *joint_out_port);
		joint_out_port_list.push_back( std::make_pair(joint_out_port,*joint)  );
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
		link_in_port = new RTT::InputPort<base::Vector3d>( "_link_" + (*link)->GetName() + "_in");
		ports()->addPort( *link_in_port );
		link_in_port_list.push_back( std::make_pair(link_in_port,*link) );

		gzmsg << "RockBridge: create link OutputPort in Rock." << std::endl;
		link_out_port = new RTT::OutputPort<base::Vector3d>("_link_" + (*link)->GetName() + "_out");
		ports()->addPort( *link_out_port );
		link_out_port_list.push_back( std::make_pair(link_out_port,*link) );
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
    // Apply effort to joint
	for(JointInPort_V::iterator it = joint_in_port_list.begin(); it != joint_in_port_list.end(); ++it)
	{
		double effort = 0;	
		(*it).first->readNewest(effort);
		if(effort != RTT::NoData){
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

    // Read joint angle in each axis (x,y,z)
    for(JointOutPort_V::iterator it = joint_out_port_list.begin(); it != joint_out_port_list.end(); ++it)
    {
        (*it).first->write( base::Vector3d((*it).second->GetAngle(0).Radian(),
                (*it).second->GetAngle(1).Radian(),(*it).second->GetAngle(2).Radian()) );
    }
}
//======================================================================================
void ModelTask::updateLinks()
{
	// Update all links from gazebo model
    // Apply effort to link
	for(LinkInPort_V::iterator it = link_in_port_list.begin(); it != link_in_port_list.end(); ++it)
	{
		// Read rock input port and apply a force to gazebo links
		base::Vector3d force(0.0, 0.0, 0.0);
		(*it).first->readNewest(force);
		if((force(0) != RTT::NoData)||(force(1) != RTT::NoData)||(force(2) != RTT::NoData)){
            // gzmsg << "RockBridge: applying force to link: " << force << std::endl;
			(*it).second->AddRelativeForce( math::Vector3(force(0),force(1),force(2)) );
	    }
	}

    // Read absolute link position in gazebo
    for(LinkOutPort_V::iterator it = link_out_port_list.begin(); it != link_out_port_list.end(); ++it)
    {
        // Update link positon in Rock
	    math::Pose link_pose = (*it).second->GetWorldPose();
	    (*it).first->write( base::Vector3d(link_pose.pos.x,link_pose.pos.y,link_pose.pos.z) );
    }
}
//======================================================================================
ModelTask::~ModelTask()
{
	delete joint_in_port;
	delete joint_out_port;
	delete link_in_port;
	delete link_out_port;
	
	for(JointInPort_V::iterator it = joint_in_port_list.begin();it != joint_in_port_list.end(); ++it)
		delete (it)->first;
    for(JointOutPort_V::iterator it = joint_out_port_list.begin();it != joint_out_port_list.end(); ++it)
        delete (it)->first;
	for(LinkInPort_V::iterator it = link_in_port_list.begin(); it != link_in_port_list.end(); ++it)
		delete it->first;
    for(LinkOutPort_V::iterator it = link_out_port_list.begin(); it != link_out_port_list.end(); ++it )
        delete it->first;

	joint_in_port_list.clear();
	joint_out_port_list.clear();
	link_in_port_list.clear();
	link_out_port_list.clear();
}
//======================================================================================
