/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
//======================================================================================
// Brazilian Institute of Robotics 
// Authors: Thomio Watanabe
// Date: December 2014
//====================================================================================== 

#include "ModelTask.hpp"

using namespace gazebo;
using namespace rock_gazebo;


ModelTask::ModelTask(std::string const& name)
	: ModelTaskBase(name)
{
}

ModelTask::ModelTask(std::string const& name, RTT::ExecutionEngine* engine)
	: ModelTaskBase(name, engine)
{
}

ModelTask::~ModelTask()
{
    for(LinkOutput::iterator it = link_list.begin(); it != link_list.end(); ++it)
        delete (*it).first;
}

void ModelTask::setGazeboModel(WorldPtr _world,  ModelPtr _model)
{
    std::string name = "gazebo:" + _world->GetName() + ":" + _model->GetName();
    provides()->setName(name);
    _name.set(name);

    world = _world;
    model = _model;
} 

void ModelTask::setupJoints()
{
    // Get all joints from a model and set Rock Input/Output Ports
    gazebo_joints = model->GetJoints();
    for(Joint_V::iterator joint = gazebo_joints.begin(); joint != gazebo_joints.end(); ++joint)
    {
        gzmsg << "ModelTask: found joint: " << world->GetName() + "/" + model->GetName() +
                "/" + (*joint)->GetName() << std::endl;

        joints_in.names.push_back( (*joint)->GetName() );
        joints_in.elements.push_back( base::JointState::Effort(0.0) );
    }
}

void ModelTask::setupLinks(NameVector link_names)
{
    // Create Rock output ports for the links defined in the configuration file.
	gazebo_links = model->GetLinks();
	for(Link_V::iterator link = gazebo_links.begin(); link != gazebo_links.end(); ++link)
	{
        for(NameVector::iterator it = link_names.begin(); it != link_names.end(); ++it)
        {
            if( (*it) == (*link)->GetName()  )
            {
                gzmsg << "ModelTask: found link: " << world->GetName() + "/" + model->GetName() +
                    "/" + (*link)->GetName() << std::endl;

                // Create the ports dynamicaly
                RBSOutPort* link_out_port = new RBSOutPort( (*link)->GetName() );
                ports()->addPort(*link_out_port);
                link_list.push_back( std::make_pair(link_out_port,*link) );
            }
        }
	}
}

void ModelTask::updateHook()
{
    updateJoints();
    updateLinks();
}

void ModelTask::updateJoints()
{
    _joints_cmd.readNewest( joints_in );

    std::vector<std::string> names;
    std::vector<double> positions;

    for(Joint_V::iterator it = gazebo_joints.begin(); it != gazebo_joints.end(); ++it )
    {
        // Apply effort to joint
        if( joints_in.getElementByName( (*it)->GetName() ).hasEffort() )
            (*it)->SetForce(0, joints_in.getElementByName( (*it)->GetName() ).effort );

        // Read joint angle from gazebo link
        names.push_back( (*it)->GetName() );
        positions.push_back( (*it)->GetAngle(0).Radian() );
    }
    _joints_samples.write( base::samples::Joints::Positions(positions,names) );
}

void ModelTask::updateLinks()
{
    for(LinkOutput::iterator it = link_list.begin(); it != link_list.end(); ++it)
    {
        RigidBodyState rbs;
        rbs.sourceFrame = std::string( (*it).second->GetName() );
        rbs.targetFrame = std::string( world->GetName() );

        math::Pose link_pose = (*it).second->GetWorldPose();
        rbs.position = base::Vector3d(
            link_pose.pos.x,link_pose.pos.y,link_pose.pos.z);
        rbs.orientation = base::Quaterniond(
            link_pose.rot.w,link_pose.rot.x,link_pose.rot.y,link_pose.rot.z );

        (*it).first->write( rbs );
    }
}

bool ModelTask::configureHook()
{
    if( ! ModelTaskBase::configureHook() )
        return false;

    // Test if setGazeboModel() has been called -> if world/model has been found
    if( (!world) && (!model) )
        return false;

    // The robot configuration YAML file must define the link that will be exported.
    // Than, these links will be loaded in link_names;
    NameVector link_names = _exported_links.get();
    setupLinks(link_names);
    setupJoints();

    return true;
}

