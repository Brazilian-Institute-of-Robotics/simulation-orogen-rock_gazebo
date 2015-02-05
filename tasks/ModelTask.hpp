/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */
//======================================================================================
// Brazilian Institute of Robotics 
// Authors: Thomio Watanabe
// Date: December 2014
//====================================================================================== 
#ifndef GAZEBO_MODELTASK_TASK_HPP
#define GAZEBO_MODELTASK_TASK_HPP

#include "gazebo/ModelTaskBase.hpp"	
#include <gazebo/physics/physics.hh>

namespace gazebo {

//	class ModelActivity : public RTT::extras::SlaveActivity
//	{
//		public:
//			ModelActivity(RTT::base::RunnableInterface *run=0){}; 
//			virtual ~ModelActivity(); 
////			virtual bool execute();
////			virtual void step();

////			RTT::internal::Signal<void(void)> update_signal;
////			RTT::Handle update_handle;
//	};

    class ModelTask : public ModelTaskBase
    {
		friend class ModelTaskBase;
		private:
			// environment == 0  => ground plane
			// environment == 1  => underwater
			int environment;
		
			physics::ModelPtr model;
			physics::WorldPtr world;
			sdf::ElementPtr sdf;

			void setJoints();
			void updateJoints();
			
			void setLinks();
			void updateLinks();
			
			RTT::InputPort<double>* joint_port;
			typedef std::vector<std::pair<RTT::InputPort<double>*,gazebo::physics::JointPtr> > JointPort_V;
			JointPort_V joint_port_list;
			
			RTT::InputPort<base::Vector3d>* link_port; 
			typedef std::vector<std::pair<RTT::InputPort<base::Vector3d>*,gazebo::physics::LinkPtr> > LinkPort_V;
			LinkPort_V link_port_list; 
		
		protected:
		
		public:
			void setGazeboModel(physics::WorldPtr, physics::ModelPtr, int);	
			void updateModel();

		    /** TaskContext constructor for ModelTask
		     * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
		     * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
		     */
		    ModelTask(std::string const& name = "gazebo::ModelTask", TaskCore::TaskState initial_state = Stopped);

		    /** TaskContext constructor for ModelTask 
		     * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
		     * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
		     * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
		     */
		    ModelTask(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

		    /** Default deconstructor of ModelTask
		     */
			~ModelTask();
    };
}

#endif

