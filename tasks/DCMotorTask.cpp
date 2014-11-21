/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DCMotorTask.hpp"

using namespace gazebo;

DCMotorTask::DCMotorTask(std::string const& name, TaskCore::TaskState initial_state)
    : DCMotorTaskBase(name, initial_state)
{
}

DCMotorTask::DCMotorTask(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DCMotorTaskBase(name, engine, initial_state)
{
}

DCMotorTask::~DCMotorTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DCMotorTask.hpp for more detailed
// documentation about them.

bool DCMotorTask::configureHook()
{
    if (! DCMotorTaskBase::configureHook())
        return false;
    return true;
}
bool DCMotorTask::startHook()
{
    if (! DCMotorTaskBase::startHook())
        return false;
    return true;
}
void DCMotorTask::updateHook()
{
    DCMotorTaskBase::updateHook();
}
void DCMotorTask::errorHook()
{
    DCMotorTaskBase::errorHook();
}
void DCMotorTask::stopHook()
{
    DCMotorTaskBase::stopHook();
}
void DCMotorTask::cleanupHook()
{
    DCMotorTaskBase::cleanupHook();
}
