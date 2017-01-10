/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SensorTask.hpp"
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

using namespace rock_gazebo;
using namespace std;

SensorTask::SensorTask(std::string const& name)
    : SensorTaskBase(name)
{
}

SensorTask::SensorTask(std::string const& name, RTT::ExecutionEngine* engine)
    : SensorTaskBase(name, engine)
{
}

SensorTask::~SensorTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SensorTask.hpp for more detailed
// documentation about them.

bool SensorTask::configureHook()
{
    if (! SensorTaskBase::configureHook())
        return false;

    // Initialize communication node and subscribe to gazebo topic
    node = gazebo::transport::NodePtr( new gazebo::transport::Node() );
    node->Init();

    return true;
}
bool SensorTask::startHook()
{
    if (! SensorTaskBase::startHook())
        return false;
    gazebo::sensors::SensorPtr sensor = gazebo::sensors::get_sensor(sensorFullName);
    sensor->SetActive(true);
    return true;
}
void SensorTask::updateHook()
{
    SensorTaskBase::updateHook();
}
void SensorTask::errorHook()
{
    SensorTaskBase::errorHook();
}
void SensorTask::stopHook()
{
    gazebo::sensors::SensorPtr sensor = gazebo::sensors::get_sensor(sensorFullName);
    sensor->SetActive(false);
    SensorTaskBase::stopHook();
}
void SensorTask::cleanupHook()
{
    node->Fini();
    SensorTaskBase::cleanupHook();
}
void SensorTask::setGazeboModel(ModelPtr model, sdf::ElementPtr sdfSensor)
{
    sdf::ElementPtr sdfLink = sdfSensor->GetParent();
    this->gazeboModel = model;
    this->sdfSensor = sdfSensor;

    sensorFullName =
        model->GetWorld()->GetName() + "::" +
        model->GetScopedName() + "::" +
        sdfLink->Get<string>("name") + "::" +
        sdfSensor->Get<string>("name");
    baseTopicName = "~/" + model->GetName() + "/" + sdfLink->Get<string>("name") + "/" + sdfSensor->Get<string>("name");

    string taskName = "gazebo:" + model->GetWorld()->GetName() + ":" + model->GetName() + ":" + sdfSensor->Get<string>("name");
    if(!provides())
        throw std::runtime_error("GPSTask::provides returned NULL");
    provides()->setName(taskName);
    _name.set(taskName);
    BaseTask::setGazeboWorld( model->GetWorld() );
}

