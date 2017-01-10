/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ImuTask.hpp"

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

ImuTask::ImuTask(std::string const& name)
    : ImuTaskBase(name)
{
}

ImuTask::ImuTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ImuTaskBase(name, engine)
{
}

ImuTask::~ImuTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ImuTask.hpp for more detailed
// documentation about them.

bool ImuTask::configureHook()
{
    if (! ImuTaskBase::configureHook())
        return false;

    return true;
}
bool ImuTask::startHook()
{
    if (! ImuTaskBase::startHook())
        return false;

    samples.clear();
    topicSubscribe(&ImuTask::readInput, baseTopicName + "/imu");
    return true;
}
void ImuTask::updateHook()
{
    ImuTaskBase::updateHook();

    Samples samples;
    { lock_guard<mutex> readGuard(readMutex);
        samples = move(this->samples);
    }

    for (auto const& sample : samples)
    {
        _orientation_samples.write(sample.first);
        _imu_samples.write(sample.second);
    }
}
void ImuTask::errorHook()
{
    ImuTaskBase::errorHook();
}
void ImuTask::stopHook()
{
    ImuTaskBase::stopHook();
}
void ImuTask::cleanupHook()
{
    ImuTaskBase::cleanupHook();
}

void ImuTask::readInput( ConstIMUPtr & imuMsg)
{ lock_guard<mutex> readGuard(readMutex);
    const gazebo::msgs::Quaternion &quat = imuMsg->orientation();
    const gazebo::msgs::Vector3d& avel =  imuMsg->angular_velocity();
    const gazebo::msgs::Vector3d& linacc =  imuMsg->linear_acceleration();

    base::Time stamp = getCurrentTime(imuMsg->stamp());

    base::samples::IMUSensors imu_sensors;
    base::samples::RigidBodyState orientation;
    orientation.time = stamp;
    orientation.sourceFrame = _imu_frame.value();
    orientation.targetFrame = _world_frame.value();
    orientation.orientation = base::Orientation(quat.w(),quat.x(),quat.y(),quat.z());
    orientation.angular_velocity = base::Vector3d(avel.x(),avel.y(),avel.z());

    imu_sensors.time = stamp;
    imu_sensors.mag  = base::getEuler(orientation.orientation);
    imu_sensors.gyro = base::Vector3d(avel.x(),avel.y(),avel.z());
    imu_sensors.acc  = base::Vector3d(linacc.x(), linacc.y(), linacc.z());

    samples.push_back(make_pair(orientation, imu_sensors));
}

