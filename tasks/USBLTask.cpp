/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "USBLTask.hpp"
#include <gazebo/common/common.hh>

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;


USBLTask::USBLTask(std::string const& name)
    : USBLTaskBase(name)
{
}


USBLTask::USBLTask(std::string const& name, RTT::ExecutionEngine* engine)
    : USBLTaskBase(name, engine)
{
}


USBLTask::~USBLTask()
{
}


void USBLTask::setGazeboModel(WorldPtr const& _world, ModelPtr const& _model)
{
    string name = "gazebo:" + _world->GetName() + ":" + _model->GetName();
    provides()->setName(name);
    _name.set(name);

    model = _model;
    world = _world;
}


void USBLTask::calcRelPosition()
{
    // Get USBL Pose
    math::Pose usblAbsPose = model->GetWorldPose();
    base::Vector3d usblAbsPos(usblAbsPose.pos.x,usblAbsPose.pos.y,usblAbsPose.pos.z);

    // Read model pose containing USBL modem
    base::samples::RigidBodyState modemRBS;
    _pose_cmd.readNewest( modemRBS );
    
    // Calculate relative position
    base::Vector3d modemDistance( modemRBS.position.x() - usblAbsPos.x(),
            modemRBS.position.y() - usblAbsPos.y(), modemRBS.position.z() - usblAbsPos.z() );
    _modem_distance.write( modemDistance );
}


bool USBLTask::configureHook()
{
    if (! USBLTaskBase::configureHook())
        return false;
    return true;
}
bool USBLTask::startHook()
{
    if (! USBLTaskBase::startHook())
        return false;
    return true;
}
void USBLTask::updateHook()
{
    USBLTaskBase::updateHook();
    calcRelPosition();
}
void USBLTask::errorHook()
{
    USBLTaskBase::errorHook();
}
void USBLTask::stopHook()
{
    USBLTaskBase::stopHook();
}
void USBLTask::cleanupHook()
{
    USBLTaskBase::cleanupHook();
}
