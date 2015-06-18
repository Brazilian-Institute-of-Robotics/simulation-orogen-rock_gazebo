/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ThrusterTask.hpp"

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

ThrusterTask::ThrusterTask(string const& name)
    : ThrusterTaskBase(name)
{
}

ThrusterTask::ThrusterTask(string const& name, RTT::ExecutionEngine* engine)
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
    thrusterPublisher = node->Advertise<ThrustersMSG>("~/" + topicName);
    gzmsg <<"ThrusterTask: advertising to gazebo topic ~/" + topicName << endl;
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
    if (_thrusters_cmd.readNewest( jointsCMD ) == RTT::NewData)
    {
        gzmsg << "ThrusterTask: new sample" << endl;

        ThrustersMSG thrustersMSG;
        for(vector<string>::iterator jointName = jointsCMD.names.begin();
                jointName != jointsCMD.names.end(); ++jointName)
        {
            base::JointState jointState = jointsCMD.getElementByName(*jointName);
            gazebo_thruster::msgs::Thruster* thruster = thrustersMSG.add_thrusters();
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
            gzmsg << "ThrusterTask: publisher has no connections. Going into exception" << endl;
            exception(NO_TOPIC_CONNECTION);
        }
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

void ThrusterTask::setGazeboThruster( ModelPtr model )
{
    string taskName = "gazebo:" + model->GetWorld()->GetName() + ":" + model->GetName() + ":gazebo_thruster";
    provides()->setName(taskName);
    _name.set(taskName);

    topicName = model->GetName() + "/thrusters";
} 


