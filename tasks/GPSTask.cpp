/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GPSTask.hpp"
#include <gazebo/transport/transport.hh>

using namespace std;
using namespace rock_gazebo;

GPSTask::GPSTask(std::string const& name)
    : GPSTaskBase(name)
    , deviationHorizontal(base::unknown<double>()), deviationVertical(base::unknown<double>())
{
}

GPSTask::GPSTask(std::string const& name, RTT::ExecutionEngine* engine)
    : GPSTaskBase(name, engine)
    , deviationHorizontal(base::unknown<double>()), deviationVertical(base::unknown<double>())
{
}

GPSTask::~GPSTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See GPSTask.hpp for more detailed
// documentation about them.

bool GPSTask::configureHook()
{
    if (! GPSTaskBase::configureHook())
        return false;

    topicSubscribe(&GPSTask::readInput, baseTopicName);
    return true;
}
bool GPSTask::startHook()
{
    if (! GPSTaskBase::startHook())
        return false;
    utm_converter.setUtmZone(_utm_zone.value());
    utm_converter.setUtmNorth(_utm_north.value());
    utm_converter.setOrigin(_origin.value());
    solutions.clear();
    return true;
}

void GPSTask::updateHook()
{
    vector<gps_base::Solution> solutions;
    { lock_guard<mutex> readGuard(readMutex);
        solutions = move(this->solutions);
    }

    for (auto const& solution : solutions)
    {
        _gps_solution.write(solution);
        base::samples::RigidBodyState rbs;
        rbs.sourceFrame = _gps_frame.value();
        rbs.targetFrame = _world_frame.value();
        utm_converter.convertSolutionToRBS(solution, rbs);
        rbs.time = solution.time;
        _position_samples.write(rbs);
    }
    GPSTaskBase::updateHook();
}
void GPSTask::errorHook()
{
    GPSTaskBase::errorHook();
}
void GPSTask::stopHook()
{
    GPSTaskBase::stopHook();
}
void GPSTask::cleanupHook()
{
    GPSTaskBase::cleanupHook();
}

void GPSTask::setGazeboModel(ModelPtr model, sdf::ElementPtr sdfSensor)
{
    GPSTaskBase::setGazeboModel(model, sdfSensor);
    sdf::ElementPtr gps = sdfSensor->GetElement("gps");

    sdf::ElementPtr h_noise = gps
        ->GetElement("position_sensing")
        ->GetElement("horizontal")
        ->GetElement("noise");
    deviationHorizontal = 1;
    if (h_noise->HasElement("stddev"))
        deviationHorizontal = h_noise->Get<double>("stddev");

    sdf::ElementPtr v_noise = gps
        ->GetElement("position_sensing")
        ->GetElement("vertical")
        ->GetElement("noise");
    deviationVertical = 1;
    if (v_noise->HasElement("stddev"))
        deviationVertical = v_noise->Get<double>("stddev");
}

void GPSTask::readInput( ConstGPSPtr & msg)
{ lock_guard<mutex> readGuard(readMutex);
    gps_base::Solution solution;
    solution.time = getCurrentTime(msg->time());
    solution.latitude = msg->latitude_deg();
    solution.longitude = msg->longitude_deg();
    solution.altitude = msg->altitude();
    solution.positionType = gps_base::AUTONOMOUS;
    solution.noOfSatellites = 5;
    solution.geoidalSeparation = base::unknown<double>();
    solution.ageOfDifferentialCorrections = 0;
    solution.deviationAltitude = deviationVertical;
    solution.deviationLatitude = deviationHorizontal;
    solution.deviationLongitude = deviationHorizontal;
    solutions.emplace_back(move(solution));
}

