/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BaseTask.hpp"

using namespace rock_gazebo;

BaseTask::BaseTask(std::string const& name)
    : BaseTaskBase(name)
{
}

BaseTask::BaseTask(std::string const& name, RTT::ExecutionEngine* engine)
    : BaseTaskBase(name, engine)
{
}

BaseTask::~BaseTask()
{
}

void BaseTask::setGazeboWorld(WorldPtr _world)
{
    world = _world;
}

base::Time BaseTask::getSimTime() const
{
    gazebo::common::Time sim_time = world->GetSimTime();
    return base::Time::fromSeconds(sim_time.sec) +
        base::Time::fromMicroseconds(sim_time.nsec / 1000);
}

base::Time BaseTask::getCurrentTime() const
{
    if (_use_sim_time)
        return getSimTime();
    else
        return base::Time::now();
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseTask.hpp for more detailed
// documentation about them.

bool BaseTask::configureHook()
{
    if (! BaseTaskBase::configureHook())
        return false;
    return true;
}
bool BaseTask::startHook()
{
    if (! BaseTaskBase::startHook())
        return false;
    return true;
}
void BaseTask::updateHook()
{
    BaseTaskBase::updateHook();
}
void BaseTask::errorHook()
{
    BaseTaskBase::errorHook();
}
void BaseTask::stopHook()
{
    BaseTaskBase::stopHook();
}
void BaseTask::cleanupHook()
{
    BaseTaskBase::cleanupHook();
}
