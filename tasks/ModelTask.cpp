/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

// add joints, position and velocity


#include "ModelTask.hpp"

namespace gazebo
{

//	ModelActivity::~ModelActivity()
//	{
//	}
//	
//	void ModelActivity::step()
//	{
//		//this->start();
//		//this->provides()->addOperation(_updateModel);
//	}


	ModelTask::ModelTask(std::string const& name, TaskCore::TaskState initial_state)
		: ModelTaskBase(name, initial_state)
	{
	}

	ModelTask::ModelTask(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
		: ModelTaskBase(name, engine, initial_state)
	{
	}

	ModelTask::~ModelTask()
	{
		joints.clear();
		_joint_port_list.clear();
	}

	void ModelTask::setGazeboModel(physics::WorldPtr _world,  physics::ModelPtr _model)
	{
		model = _model;
		world = _world;
				
		// Get all joints from a model and creates an Rock component InputPort
		joints = model->GetJoints();
		for(Joint_V::iterator it = joints.begin(); it != joints.end(); ++it)
		{
			gzmsg <<"RockBridge: found joint: "<< (*it)->GetName() << std::endl;
			
			_axial_joint_port = new RTT::InputPort<double>( world->GetName() +
				"/" + model->GetName() + "/" + (*it)->GetName() );
			ports()->addPort( *_axial_joint_port );
			//_axial_joint_port.clear();
			
			_joint_port_list.push_back( std::make_pair(_axial_joint_port,*it) );	
		}
		
	}


	void ModelTask::updateModel()
	{
		// gzmsg << " calling step() from model:"<< model->GetName() << std::endl;
		
		for(JointPort_V::iterator it = _joint_port_list.begin();
				it != _joint_port_list.end(); ++it)
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


	/// The following lines are template definitions for the various state machine
	// hooks defined by Orocos::RTT. See ModelTask.hpp for more detailed
	// documentation about them.

	bool ModelTask::configureHook()
	{
		if (! ModelTaskBase::configureHook())
		    return false;
		return true;
	}
	bool ModelTask::startHook()
	{
		if (! ModelTaskBase::startHook())
		    return false;

		return true;
	}
	void ModelTask::updateHook()
	{
		ModelTaskBase::updateHook(); 
	}

	void ModelTask::errorHook()
	{
		ModelTaskBase::errorHook();
	}
	void ModelTask::stopHook()
	{
		ModelTaskBase::stopHook();
	}
	void ModelTask::cleanupHook()
	{
		ModelTaskBase::cleanupHook();
	}
}

