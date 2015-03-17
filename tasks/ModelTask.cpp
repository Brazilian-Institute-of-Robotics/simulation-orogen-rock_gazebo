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
    for(std::vector<RBSOutPort*>::iterator it = exported_links.link_out_port.begin();
            it != exported_links.link_out_port.end(); ++it)
        delete (*it);
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

void ModelTask::setupLinks()
{
    NameVector source_links_names = _source_links.get();
    NameVector target_links_names = _target_links.get();

    // Create Rock output ports for the links defined in the configuration file.
    for(NameVector::iterator link_name = source_links_names.begin();
            link_name != source_links_names.end(); ++link_name)
    {
        gzmsg << "ModelTask: found link: " << world->GetName() + "/" + model->GetName() +
            "/" + *link_name << std::endl;

        exported_links.source_links.push_back( model->GetLink( *link_name ) );

        // Create the ports dynamicaly
        RBSOutPort* link_out_port = new RBSOutPort( *link_name );
        ports()->addPort(*link_out_port);
        exported_links.link_out_port.push_back( link_out_port );
    }

    for(NameVector::iterator link_name = target_links_names.begin();
            link_name != target_links_names.end(); ++link_name)
        exported_links.target_links.push_back( model->GetLink( *link_name ) );
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
    for(size_t i = 0; i < exported_links.source_links.size(); ++i )
    {
        exported_links.source_frame[i] = exported_links.source_links[i]->GetWorldPose();
        if( exported_links.target_links[i]->GetName() == world->GetName() ){
            exported_links.target_frame[i] = math::Pose::Zero;
        } else{
            exported_links.target_frame[i] = exported_links.target_links[i]->GetWorldPose();
        }

        gazebo::math::Pose relative_pose( math::Pose(
                exported_links.source_frame[i] - exported_links.target_frame[i] ) );

        RigidBodyState rbs;
        rbs.position = base::Vector3d(
            relative_pose.pos.x,relative_pose.pos.y,relative_pose.pos.z);
        rbs.orientation = base::Quaterniond(
            relative_pose.rot.w,relative_pose.rot.x,relative_pose.rot.y,relative_pose.rot.z );

        exported_links.link_out_port[i]->write( rbs );
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

    setupLinks();
    setupJoints();

    return true;
}

