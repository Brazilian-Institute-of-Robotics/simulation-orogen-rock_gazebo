/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GPSTask.hpp"
#include <gazebo/transport/transport.hh>

using namespace std;
using namespace rock_gazebo;
using gazebo::common::SphericalCoordinates;

GPSTask::GPSTask(std::string const& name)
    : GPSTaskBase(name)
    , deviationHorizontal(base::unknown<double>())
    , deviationVertical(base::unknown<double>())
{
    _nwu_origin.set(base::Position::Zero());
}

GPSTask::GPSTask(std::string const& name, RTT::ExecutionEngine* engine)
    : GPSTaskBase(name, engine)
    , deviationHorizontal(base::unknown<double>())
    , deviationVertical(base::unknown<double>())
{
    _nwu_origin.set(base::Position::Zero());
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

    if (_use_proper_utm_conversion.get())
    {
        utm_converter.setUTMZone(_utm_zone.value());
        utm_converter.setUTMNorth(_utm_north.value());
        utm_converter.setNWUOrigin(_nwu_origin.value());
    }
    else
    {
        utm_converter.setNWUOrigin(Eigen::Vector3d::Zero());
        gazebo_spherical.SetLatitudeReference(
                ignition::math::Angle(_latitude_origin.value().getRad()));
        gazebo_spherical.SetLongitudeReference(
                ignition::math::Angle(_longitude_origin.value().getRad()));
    }
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
        base::samples::RigidBodyState utm, position;

        if (_use_proper_utm_conversion.get())
        {
            utm = utm_converter.convertToUTM(solution);
            position = utm_converter.convertToNWU(utm);
        }
        else
        {
            ignition::math::Vector3d local = gazebo_spherical.LocalFromSpherical(
                    ignition::math::Vector3d(solution.latitude, solution.longitude, solution.altitude));

            utm.position = Eigen::Vector3d(local.X(), local.Y(), local.Z());
            utm.cov_position = 1.0 * base::Matrix3d::Identity();
            utm.cov_position(0, 0) = solution.deviationLongitude * solution.deviationLongitude;
            utm.cov_position(1, 1) = solution.deviationLatitude * solution.deviationLatitude;
            utm.cov_position(2, 2) = solution.deviationAltitude * solution.deviationAltitude;
            
            position.position = Eigen::Vector3d(local.Y(), -local.X(), local.Z());
            position.cov_position = utm.cov_position;
            std::swap(position.cov_position(0, 0), position.cov_position(1, 1));
        }

        utm.time = solution.time;
        utm.sourceFrame = _gps_frame.value();
        utm.targetFrame = _utm_frame.value();
        _utm_samples.write(utm);

        position.time = solution.time;
        position.sourceFrame = _gps_frame.value();
        position.targetFrame = _nwu_frame.value();
        _position_samples.write(position);
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

