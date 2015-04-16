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

    // Set gazebo topic to advertise
    node = transport::NodePtr( new transport::Node() );
    node->Init();
    std::string topicName = model->GetName() + "/thrusters";
    thrusterPublisher = node->Advertise<ThrustersMSG>("~/" + topicName);
    gzmsg <<"ThrusterTask: subscribing to gazebo topic /gazebo/"+ model->GetWorld()->GetName()
            + "/" + topicName << std::endl;
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

    // Read Rock input port and update the message
    _thrusters_cmd.readNewest( jointsCMD );
    ThrustersMSG thrustersMSG;
    for(std::vector<std::string>::iterator jointName = jointsCMD.names.begin();
            jointName != jointsCMD.names.end(); ++jointName)
    {
        base::JointState jointState = jointsCMD.getElementByName(*jointName);
        rock_thruster::msgs::Thruster* thruster = thrustersMSG.add_thrusters();
        thruster->set_name( *jointName );
        if( jointState.isRaw() )
            thruster->set_raw( jointState.raw );

        if( jointState.isEffort() )
            thruster->set_effort( jointState.effort );
    }

    // Write in gazebo topic
    if(thrusterPublisher->HasConnections())
    {
        thrusterPublisher->Publish(thrustersMSG);
    }else{
        gzmsg << "ThrusterTask: publisher has no connections. Going into exception" << std::endl;
        exception(NO_TOPIC_CONNECTION);
    }
}

void ThrusterTask::errorHook()
{
    ThrusterTaskBase::errorHook();
}

void ThrusterTask::stopHook()
{
    ThrusterTaskBase::stopHook();
}

void ThrusterTask::cleanupHook()
{
    ThrusterTaskBase::cleanupHook();

    node->Fini();
}

void ThrusterTask::setGazeboModel(WorldPtr _world,  ModelPtr _model)
{
    std::string name = "gazebo:" + _world->GetName() + ":" + _model->GetName() +
            ":thruster_task";
    provides()->setName(name);
    _name.set(name);

    model = _model;
} 


