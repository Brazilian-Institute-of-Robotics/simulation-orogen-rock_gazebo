/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
//======================================================================================
// Brazilian Institute of Robotics 
// Authors: Thomio Watanabe
// Date: December 2014
//====================================================================================== 
// add joints, position and velocity
//======================================================================================
#include "ModelTask.hpp"

static const int UNDERWATER = 1;

//======================================================================================
gazebo::ModelTask::ModelTask(std::string const& name, TaskCore::TaskState initial_state)
	: ModelTaskBase(name, initial_state),environment(0)
{
}
//======================================================================================
gazebo::ModelTask::ModelTask(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
	: ModelTaskBase(name, engine, initial_state),environment(0)
{
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
void gazebo::ModelTask::setGazeboModel(physics::WorldPtr _world,  physics::ModelPtr _model, int _environment)
{
    std::string name = "gazebo:" + _world->GetName() + ":" + _model->GetName();
    provides()->setName(name);
    _name.set(name);

	world = _world;
	model = _model;
	environment = _environment;
//	sdf = model->GetSDF();
	
//	Export Gazebo Joints and Links to Rock-Robotics
	setJoints();
	setLinks();
} 
//======================================================================================
void gazebo::ModelTask::setJoints()
{
	typedef gazebo::physics::Joint_V Joint_V;
	Joint_V joints;
			
	// Get all joints from a model and creates a Rock component InputPort
	joints = model->GetJoints();
	for(Joint_V::iterator joint = joints.begin(); joint != joints.end(); ++joint)
	{
		gzmsg <<"RockBridge: found joint: "<< world->GetName() + "/" + model->GetName() + 
				"/" + (*joint)->GetName() << std::endl;
		gzmsg << "RockBridge: create joint InputPort in Rock." << std::endl;
		joint_port = new RTT::InputPort<double>( "joint: " + world->GetName() +
				"/" + model->GetName() + "/" + (*joint)->GetName() );
		ports()->addPort( *joint_port );
		joint_port_list.push_back( std::make_pair(joint_port,*joint) );			
	}
	joints.clear();
}
//======================================================================================
void gazebo::ModelTask::setLinks()
{
	typedef gazebo::physics::Link_V Link_V;
	Link_V links; 
			
	// Get all links from a model and creates a Rock component InputPort
	links = model->GetLinks();
	for(Link_V::iterator link = links.begin(); link != links.end(); ++link)
	{
		gzmsg << "RockBridge: found link: "<< world->GetName() + "/" + model->GetName() + 
				"/" + (*link)->GetName() << std::endl;
		
		gzmsg << "RockBridge: create link InputPort in Rock." << std::endl;
		link_port = new RTT::InputPort<base::Vector3d>((*link)->GetName());
		ports()->addPort(*link_port);
		link_port_list.push_back( std::make_pair(link_port,*link) );
	}
	links.clear();
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
			if((-1.0 <= effort) && (effort <= 1.0)) 
			{	
				(*it).second->SetForce(0,effort);
			} else
			{
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

		// Apply bouyancy to links in underwater environment
		if(environment == UNDERWATER){
			math::Vector3 cobPosition;
			math::Vector3 velocityDifference;
			math::Vector3 gravity = world->GetPhysicsEngine()->GetGravity();
			math::Vector3 link_bouyancy;

			double distancetosurface = 0.0; // Distance to surface is given in meters
			double link_length = 15; // dimensions in decimeters
			double link_width = 6; // dimensions in decimeters
			double link_height = 5;	// dimensions in decimeters
			double relative_height = 0.0;			
			// double link_volume = 1; 	// link volume in liters or dm^3.
			double liquid_weight = 1.0; // 1 liter of water ~ 1 kgs.
			double submersed_volume = 0.0;
			double compensation = 1.0;
			double surface_position = 2.2;
			double viscous_damping = 15.0;
//			math::Vector3 viscous_damping(0.5,0.5,0.5); 
			math::Vector3 buoyancy_center(0.0, 0.0, 0.0);
			math::Vector3 fluid_velocity(0.0, 0.0, 0.0);
			math::Vector3 link_friction(0.0, 0.0, 0.0);
			math::Vector3 resultant_force(0.0, 0.0, 0.0);
		
			cobPosition = link->GetWorldPose().pos + 
					link->GetWorldPose().rot.RotateVector(buoyancy_center);		
		
			// link_bouyancy goes to zero when the link is above the surface. 
			// it depends on the volume submersed
			distancetosurface = surface_position - cobPosition.z;
			if(distancetosurface <= 0 )
			{ 
				submersed_volume = 0;
			} else{
				if(distancetosurface <= (link_height/10))  // distancetosurface is in meters and link_height is in dm
				{	
					relative_height = (distancetosurface*10.0);
					submersed_volume = link_length * link_width * relative_height; 
				}else
				{
					submersed_volume = link_length * link_width * link_height;
				}
			}
			// The bouyancy opposes gravity	=> it is negative.		
			link_bouyancy = - compensation * submersed_volume * liquid_weight * gravity;
			
			// Calculates dynamic viscous damp
			velocityDifference = link->GetWorldPose().rot.RotateVectorReverse(link->GetWorldLinearVel() - fluid_velocity);
//			velocityDifference = link->GetWorldLinearVel() + fluid_velocity;	
			link_bouyancy -= link->GetWorldPose().rot.RotateVector(
						viscous_damping * velocityDifference) ;
//			link_friction = - link->GetWorldPose().rot.RotateVector( math::Vector3(
//							  viscous_damping.x * velocityDifference.x,
//							  viscous_damping.y * velocityDifference.y,
//							  viscous_damping.z * velocityDifference.z));			
			
			
			// Gazebo adds the link weight, so there is no need to calculate it. 
			// resultant_force = link_bouyancy + link_friction; 
		
			link->AddForceAtWorldPosition(link_bouyancy, cobPosition);											
//			link->AddForceAtWorldPosition(resultant_force, cobPosition);
//			link->AddForce(link_bouyancy);
		}

		// Read rock input port and apply a force to gazebo links
		base::Vector3d force(0.0, 0.0, 0.0);
		(*it).first->readNewest(force);				
		if((force(0) != RTT::NoData)||(force(1) != RTT::NoData)||(force(2) != RTT::NoData)){
			link->AddRelativeForce( gazebo::math::Vector3(force(0),force(1),force(2)) );
		}
	}
}
//======================================================================================
