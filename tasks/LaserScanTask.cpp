/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LaserScanTask.hpp"

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

#define meters_to_milimeters(x) (x) * 1000

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

    topicSubscribe(&LaserScanTask::readInput, baseTopicName + "/scan");
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
    _laser_scan_samples.write( laserScanCMD );
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

}


void LaserScanTask::readInput( ConstLaserScanStampedPtr & laserScanMSG )
{
    laserScanCMD.ranges.clear();

    if( !laserScanMSG->scan().has_range_max() )
        throw std::runtime_error("rock_gazebo::LaserScanTask requires range_max to be set");

    if( !laserScanMSG->scan().has_range_min() )
        throw std::runtime_error("rock_gazebo::LaserScanTask requires range_min to be set");

    if( !laserScanMSG->scan().has_angle_step() )
        throw std::runtime_error("rock_gazebo::LaserScanTask requires angle_step to be set");

    if( !laserScanMSG->scan().has_angle_min() )
        throw std::runtime_error("rock_gazebo::LaserScanTask requires angle_min to be set");

    double range_min = laserScanMSG->scan().range_min();
    double range_max = laserScanMSG->scan().range_max();

    laserScanCMD.minRange = meters_to_milimeters(range_min);
    laserScanCMD.maxRange = meters_to_milimeters(range_max);
    laserScanCMD.angular_resolution = laserScanMSG->scan().angle_step();
    laserScanCMD.start_angle = laserScanMSG->scan().angle_min();

    for(int i = 0; i < laserScanMSG->scan().ranges_size(); ++i) {
        double range = laserScanMSG->scan().ranges(i);

        if (range >= range_max) {
            laserScanCMD.ranges.push_back(base::samples::TOO_FAR);
        }
        else if (range <= range_min) {
            laserScanCMD.ranges.push_back(base::samples::TOO_NEAR);
        }
        else {
            laserScanCMD.ranges.push_back(meters_to_milimeters(range));
        }
    }
}


