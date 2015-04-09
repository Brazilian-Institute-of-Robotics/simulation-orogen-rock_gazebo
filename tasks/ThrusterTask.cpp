/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ThrusterTask.hpp"

#include <gazebo/msgs/msgs.hh>

using namespace gazebo;
using namespace rock_gazebo;

ThrusterTask::ThrusterTask(std::string const& name)
    : ThrusterTaskBase(name)
{
}

ThrusterTask::ThrusterTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ThrusterTaskBase(name, engine)
{
}

ThrusterTask::~ThrusterTask()
{
}

bool ThrusterTask::configureHook()
{
    if (! ThrusterTaskBase::configureHook())
        return false;

    node = transport::NodePtr( new transport::Node() );
    node->Init();
    thrusterPublisher = node->Advertise<ThrusterInput>("~/" + model->GetName());
    return true;
}
bool ThrusterTask::startHook()
{
    if (! ThrusterTaskBase::startHook())
        return false;
    return true;
}

void ThrusterTask::updateHook()
{
    ThrusterTaskBase::updateHook();

    // Read Rock input port
    double thrusterFrequency;
    _thruster_frequency.read(thrusterFrequency);

    // Write in gazebo topic
    if(thrusterPublisher->HasConnections())
    {
        ThrusterInput thrusterMsg;
        thrusterMsg.set_frequency(thrusterFrequency);
        thrusterPublisher->Publish(thrusterMsg);
    }else{
        gzthrow("ThrusterTask: publisher has no connections.");
    }
}

//void ThrusterTask::errorHook()
//{
//    ThrusterTaskBase::errorHook();
//}
//void ThrusterTask::stopHook()
//{
//    ThrusterTaskBase::stopHook();
//}
//void ThrusterTask::cleanupHook()
//{
//    ThrusterTaskBase::cleanupHook();
//}

void ThrusterTask::setGazeboModel(WorldPtr _world,  ModelPtr _model)
{
    std::string name = "gazebo:" + _world->GetName() + ":" + _model->GetName() +
            ":thruster_task";
    provides()->setName(name);
    _name.set(name);

    model = _model;
} 


