/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROCK_GAZEBO_LASERSCANTASK_TASK_HPP
#define ROCK_GAZEBO_LASERSCANTASK_TASK_HPP

#include "rock_gazebo/LaserScanTaskBase.hpp"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/laserscan_stamped.pb.h>

namespace rock_gazebo{
    class LaserScanTask : public LaserScanTaskBase
    {
	friend class LaserScanTaskBase;
    public:
        LaserScanTask(std::string const& name = "rock_gazebo::LaserScanTask");
        LaserScanTask(std::string const& name, RTT::ExecutionEngine* engine);
    	~LaserScanTask();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();

        typedef gazebo::physics::ModelPtr ModelPtr;

        void setGazeboModel( ModelPtr model, std::string sensorName );
        void readInput( ConstLaserScanStampedPtr &laserScanMSG );

    private:
        std::string topicName;
        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr laserScanSubscriber;

        base::samples::LaserScan laserScanCMD;
    };
}

#endif
