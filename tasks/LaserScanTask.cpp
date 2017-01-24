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
    scans.clear();
    return true;
}

void LaserScanTask::updateHook()
{
    LaserScanTaskBase::updateHook();

    vector<base::samples::LaserScan> scans;
    { lock_guard<mutex> readGuard(readMutex);
        scans = move(this->scans);
    }

    for (auto const& scan : scans)
        _laser_scan_samples.write(scan);
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
{ lock_guard<mutex> readGuard(readMutex);
    double range_min = laserScanMSG->scan().range_min();
    double range_max = laserScanMSG->scan().range_max();

    base::samples::LaserScan scan;
    scan.time = getCurrentTime(laserScanMSG->time());
    scan.minRange = meters_to_milimeters(range_min);
    scan.maxRange = meters_to_milimeters(range_max);
    scan.angular_resolution = laserScanMSG->scan().angle_step();
    scan.start_angle = laserScanMSG->scan().angle_min();

    unsigned int scan_size = laserScanMSG->scan().ranges_size();
    scan.ranges.resize(scan_size);
    for(unsigned int i = 0; i < scan_size; ++i) {
        double range = laserScanMSG->scan().ranges(i);

        if (range >= range_max) {
            scan.ranges[i] = base::samples::TOO_FAR;
        }
        else if (range <= range_min) {
            scan.ranges[i] = base::samples::TOO_NEAR;
        }
        else {
            scan.ranges[i] = meters_to_milimeters(range);
        }
    }
    scans.push_back(scan);
}


