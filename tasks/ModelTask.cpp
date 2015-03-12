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
        gzmsg << "RockBridge: found joint: " << world->GetName() + "/" + model->GetName() +
                "/" + (*joint)->GetName() << std::endl;

        joints_in.names.push_back( (*joint)->GetName() );
        joints_in.elements.push_back( base::JointState::Effort(0.0) );
        joints_out.names.push_back( (*joint)->GetName() );
        joints_out.elements.push_back( base::JointState::Position(0.0) );
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
            if( (*it).compare( (*link)->GetName() ) == 0 )
            {
                gzmsg << "RockBridge: found link: " << world->GetName() + "/" + model->GetName() +
                    "/" + (*link)->GetName() << std::endl;

                // Create the ports dynamicaly
                RBSOutPort* link_out_port = new RBSOutPort( "link_"+ (*link)->GetName() +"_samples" );
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
    for(Joint_V::iterator it = gazebo_joints.begin(); it != gazebo_joints.end(); ++it )
    {
        // Apply effort to joint
        double effort = joints_in.getElementByName( (*it)->GetName() ).effort;
        if( ! isnan(effort) )
            (*it)->SetForce(0,effort);

        // Read joint angle from gazebo link
        double angle = (*it)->GetAngle(0).Radian();
        joints_out.getElementByName( (*it)->GetName() ).Position(angle);
    }
    _joints_samples.write( joints_out );
}

void ModelTask::updateLinks()
{
    for(LinkOutput::iterator it = link_list.begin(); it != link_list.end(); ++it)
    {
        RigidBodyState rock_rbs = createRBS( (*it).second );
        (*it).first->write( rock_rbs );
    }
}

bool ModelTask::configureHook()
{
    if( ! ModelTaskBase::configureHook() )
        return false;

    // Test if setGazeboModel() has been called -> if a model has been found
    if( ! model )
        return false;

    // The robot configuration YAML file must define the link that will be exported.
    // Than, these links will be loaded in link_names;
    NameVector link_names = _exported_links.get();
    setupLinks(link_names);
    setupJoints();

    return true;
}

base::samples::RigidBodyState ModelTask::createRBS(LinkPtr link)
{
    RigidBodyState rbs;

    rbs.sourceFrame = std::string( model->GetName() );
    rbs.targetFrame = std::string( link->GetName() );

    math::Pose link_pose = link->GetWorldPose();
    rbs.position = base::Vector3d(link_pose.pos.x,link_pose.pos.y,link_pose.pos.z);
    rbs.orientation = base::Quaterniond(
        link_pose.rot.w,link_pose.rot.x,link_pose.rot.y,link_pose.rot.z );

    return rbs;
}
