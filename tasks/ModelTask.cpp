/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ModelTask.hpp"



using namespace gazebo;

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
}

void ModelTask::setGazeboModel(physics::WorldPtr _world,  physics::ModelPtr _model)
{
	model = _model;
	world = _world;
	
	world->GetModel("my_robot");
//	
//	if( world->GetModel("my_robot") )
//	{	
//		 Load robots joints
//		robot_left_joint = model->GetJoint("left_wheel_hinge");
//		robot_right_joint = model->GetJoint("right_wheel_hinge");
//		if (robot_left_joint && robot_right_joint)
//				gzmsg << "rock: found expected joints" << std::endl;	
//	}

}

void ModelTask::step()
{
//	if( world->GetModel("my_robot") )
//	{	
//		double effort = 0;
//		
//		_joint_effort.read(effort);
//		// readNewest()
//		
//		if (effort != RTT::NoData)
//		{
//			if((0.0 <= effort) && (effort <= 1.0)) // Define the limits for the joint effort
//			{	
//			    robot_left_joint->SetForce(0,effort);
//				robot_right_joint->SetForce(0,effort);
//			} else
//			{
//				gzmsg << "rock: error - effort value out of range." << std::endl;
//			}
//		}	
//	}
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
