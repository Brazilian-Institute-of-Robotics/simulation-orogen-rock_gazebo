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
        typedef gazebo::physics::ModelPtr ModelPtr;

        LaserScanTask(std::string const& name = "rock_gazebo::LaserScanTask");
        LaserScanTask(std::string const& name, RTT::ExecutionEngine* engine);
    	~LaserScanTask();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();

        void setGazeboLaserScan( ModelPtr model, std::string sensorName );

        typedef const boost::shared_ptr<const gazebo::msgs::LaserScanStamped> LaserScanStamped;
        void readInput(LaserScanStamped const& laserScanMSG);
    private:
        std::string topicName;
        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr laserScanSubscriber;
    };
}

#endif
