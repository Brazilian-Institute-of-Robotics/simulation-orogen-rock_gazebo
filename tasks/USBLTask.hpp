/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROCK_GAZEBO_USBLTASK_TASK_HPP
#define ROCK_GAZEBO_USBLTASK_TASK_HPP

#include "rock_gazebo/USBLTaskBase.hpp"
#include <gazebo/physics/physics.hh>

namespace rock_gazebo {

    class USBLTask : public USBLTaskBase
    {
    private:
        typedef gazebo::physics::ModelPtr ModelPtr;
        typedef gazebo::physics::WorldPtr WorldPtr;

        ModelPtr model;
        WorldPtr world;

        std::string modemName;
        void calcRelPosition();

	friend class USBLTaskBase;
    protected:

    public:
        void setGazeboModel(WorldPtr const& _world, ModelPtr const& _model);

        USBLTask(std::string const& name = "rock_gazebo::USBLTask");
        USBLTask(std::string const& name, RTT::ExecutionEngine* engine);
	    ~USBLTask();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
