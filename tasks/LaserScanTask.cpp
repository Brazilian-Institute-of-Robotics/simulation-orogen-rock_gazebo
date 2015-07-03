/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LaserScanTask.hpp"

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

LaserScanTask::LaserScanTask(std::string const& name)
    : LaserScanTaskBase(name)
{
}

LaserScanTask::LaserScanTask(std::string const& name, RTT::ExecutionEngine* engine)
    : LaserScanTaskBase(name, engine)
{
}

LaserScanTask::~LaserScanTask()
{
}


bool LaserScanTask::configureHook()
{
    if (! LaserScanTaskBase::configureHook())
        return false;

    // Initialize communication node and subscribe to gazebo topic
    node = transport::NodePtr( new transport::Node() );
    node->Init();
    laserScanSubscriber = node->Subscribe("~/" + topicName, &LaserScanTask::readInput, this);
    gzmsg << "LaserScanTask: subscribing to gazebo topic ~/" + topicName << endl;

    return true;
}

bool LaserScanTask::startHook()
{
    if (! LaserScanTaskBase::startHook())
        return false;
    return true;
}

void LaserScanTask::updateHook()
{
    LaserScanTaskBase::updateHook();

    laserScanCMD.time = getCurrentTime();
    _laser_scan_cmd.write( laserScanCMD );
}

void LaserScanTask::errorHook()
{
    LaserScanTaskBase::errorHook();
}

void LaserScanTask::stopHook()
{
    LaserScanTaskBase::stopHook();
}

void LaserScanTask::cleanupHook()
{
    LaserScanTaskBase::cleanupHook();

    node->Fini();
}

void LaserScanTask::setGazeboModel( ModelPtr model, string sensorName, string topicName )
{
    string taskName = "gazebo:" + model->GetWorld()->GetName() + ":" + model->GetName() + ":" + sensorName;
    provides()->setName(taskName);
    _name.set(taskName);

    BaseTask::setGazeboWorld( model->GetWorld() );

    // Set topic name to communicate with Gazebo
    this->topicName = topicName;
}


void LaserScanTask::readInput( ConstLaserScanStampedPtr & laserScanMSG )
{
    for(int i = 0; i < laserScanMSG->scan().ranges_size(); ++i)
        laserScanCMD.ranges.push_back( laserScanMSG->scan().ranges(i) );

    if( laserScanMSG->scan().has_angle_step() )
        laserScanCMD.angular_resolution = laserScanMSG->scan().angle_step();

    if( laserScanMSG->scan().has_range_min() )
        laserScanCMD.minRange = laserScanMSG->scan().range_min();

    if( laserScanMSG->scan().has_range_max() )
        laserScanCMD.maxRange = laserScanMSG->scan().range_max();

    if( laserScanMSG->scan().has_angle_min() )
        laserScanCMD.start_angle = laserScanMSG->scan().angle_min();
}


