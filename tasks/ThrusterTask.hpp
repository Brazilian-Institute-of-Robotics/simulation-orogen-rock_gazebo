/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROCK_GAZEBO_THRUSTERTASK_TASK_HPP
#define ROCK_GAZEBO_THRUSTERTASK_TASK_HPP

#include "rock_gazebo/ThrusterTaskBase.hpp"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_thruster/msgs.pb.h>

namespace rock_gazebo {
    class ThrusterTask : public ThrusterTaskBase
    {
	friend class ThrusterTaskBase;
    public:
        typedef gazebo::physics::ModelPtr ModelPtr;
        typedef gazebo_thruster::msgs::Thrusters ThrustersMSG;

        ThrusterTask(std::string const& name = "rock_gazebo::ThrusterTask");
        ThrusterTask(std::string const& name, RTT::ExecutionEngine* engine);
        ~ThrusterTask();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();

        void setGazeboThruster( ModelPtr );

    private:
        std::string topicName;
        gazebo::transport::NodePtr node;
        gazebo::transport::PublisherPtr thrusterPublisher;
        base::samples::Joints jointsCMD;
    };
}

#endif

