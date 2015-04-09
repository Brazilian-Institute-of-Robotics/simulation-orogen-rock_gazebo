/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROCK_GAZEBO_THRUSTERTASK_TASK_HPP
#define ROCK_GAZEBO_THRUSTERTASK_TASK_HPP

#include "rock_gazebo/ThrusterTaskBase.hpp"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <thruster_types/thruster_types.pb.h>

namespace rock_gazebo {
    class ThrusterTask : public ThrusterTaskBase
    {
	friend class ThrusterTaskBase;
    public:
        typedef gazebo::physics::ModelPtr ModelPtr;
        typedef gazebo::physics::WorldPtr WorldPtr;

        typedef rock_thruster::msgs::ThrusterInput ThrusterInput;

        ThrusterTask(std::string const& name = "rock_gazebo::ThrusterTask");
        ThrusterTask(std::string const& name, RTT::ExecutionEngine* engine);
	    ~ThrusterTask();

        bool configureHook();
        bool startHook();
        void updateHook();

//        void errorHook();
//        void stopHook();
//        void cleanupHook();

        void setGazeboModel(WorldPtr, ModelPtr);

    private:
        ModelPtr model;
        gazebo::transport::NodePtr node;
        gazebo::transport::PublisherPtr thrusterPublisher;
    };
}

#endif
