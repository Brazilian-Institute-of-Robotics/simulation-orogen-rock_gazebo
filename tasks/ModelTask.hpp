/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */
//======================================================================================
// Brazilian Institute of Robotics
// Authors: Thomio Watanabe
// Date: December 2014
//======================================================================================
#ifndef ROCK_GAZEBO_MODELTASK_TASK_HPP
#define ROCK_GAZEBO_MODELTASK_TASK_HPP

#include "rock_gazebo/ModelTaskBase.hpp"
#include <gazebo/physics/physics.hh>

namespace rock_gazebo {
    class ModelTask : public ModelTaskBase
    {
        public:
            typedef gazebo::physics::Joint_V Joint_V;
            typedef gazebo::physics::Link_V Link_V;
            typedef gazebo::physics::ModelPtr ModelPtr;
            typedef gazebo::physics::JointPtr JointPtr;
            typedef gazebo::physics::LinkPtr LinkPtr;

        friend class ModelTaskBase;
        private:
            ModelPtr model;
            sdf::ElementPtr sdf;

            Joint_V gazebo_joints;

            base::samples::Joints joints_in;
            void setupJoints();
            void updateJoints(base::Time const& time);

            typedef base::samples::RigidBodyState RigidBodyState;
            typedef RTT::OutputPort<RigidBodyState> RBSOutPort;

            struct ExportedLink : public LinkExport
            {
                LinkPtr source_link_ptr;
                LinkPtr target_link_ptr;
                RBSOutPort* port;
                base::Time last_update;

                ExportedLink()
                    : port(NULL) { }
                ExportedLink(LinkExport const& src)
                    : LinkExport(src)
                    , port(NULL) { }
            };
            typedef std::map<std::string, ExportedLink> ExportedLinks;
            ExportedLinks exported_links;

            void setupLinks();
            void warpModel(base::samples::RigidBodyState const& modelPose);
            Link_V gazebo_links;
            base::samples::Wrenches wrench_in;
            void updateLinks(base::Time const& time);
            void updateModelPose(base::Time const& time);
            void updateModelAccel(base::Time const& time);

            std::string checkExportedLinkElements(std::string, std::string, std::string);

            void releaseLinks();

        protected:

        public:
            void setGazeboModel(WorldPtr, ModelPtr);

            bool startHook();
            void updateHook();
            bool configureHook();
            void cleanupHook();

            /** TaskContext constructor for ModelTask
             * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
             * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
             */
            ModelTask(std::string const& name = "gazebo::ModelTask");

            /** TaskContext constructor for ModelTask
             * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
             * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
             * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
             */
            ModelTask(std::string const& name, RTT::ExecutionEngine* engine);

            /** Default deconstructor of ModelTask
             */
            ~ModelTask();
    };
}

#endif

