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
    for(LinkPort::iterator it = link_port.begin(); it != link_port.end(); ++it)
        delete (*it).second;
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
    // The robot configuration YAML file must define the exported links.
    exported_links_list = _exported_links.get();
    for(std::vector<LinkExport>::iterator it = exported_links_list.begin();
            it != exported_links_list.end(); ++it)
    {
        (*it).source_link = checkExportedLinkElements("source_link", (*it).source_link, "world");
        (*it).target_link = checkExportedLinkElements("target_link", (*it).target_link, "world");
        (*it).source_frame = checkExportedLinkElements("source_frame", (*it).source_frame, (*it).source_link);
        (*it).target_frame = checkExportedLinkElements("target_frame", (*it).target_frame, (*it).target_link);

        LinkPtr source_link = model->GetLink( (*it).source_link );
        if ( !source_link )
        {
            gzmsg << "ModelTask: source_link doesn't match a link from "<< model->GetName() <<" model " << std::endl;
            gzmsg << "Exit simulation." << std::endl;
        }else {
            // Create the ports dynamicaly
            gzmsg << "ModelTask: exporting link to rock: " << world->GetName() + "/" + model->GetName() +
                "/" + source_link->GetName() << std::endl;

            RBSOutPort* link_out_port = new RBSOutPort( (*it).source_frame + "2" + (*it).target_frame );
            ports()->addPort( *link_out_port);
            link_port.push_back(std::make_pair(*it,link_out_port) );
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
    for(LinkPort::iterator it = link_port.begin(); it != link_port.end(); ++it)
    {
        math::Pose source_pose = model->GetLink( (*it).first.source_link )->GetWorldPose();
        math::Pose target_pose;
        if( (*it).first.target_link == "world" )
        {
            target_pose = math::Pose::Zero;
        } else{
            target_pose = model->GetLink( (*it).first.target_link )->GetWorldPose();
        }

        math::Pose relative_pose( math::Pose(source_pose - target_pose) );

        RigidBodyState rbs;
        rbs.sourceFrame = (*it).first.source_frame;
        rbs.targetFrame = (*it).first.target_frame;
        rbs.position = base::Vector3d(
            relative_pose.pos.x,relative_pose.pos.y,relative_pose.pos.z);
        rbs.orientation = base::Quaterniond(
            relative_pose.rot.w,relative_pose.rot.x,relative_pose.rot.y,relative_pose.rot.z );

        (*it).second->write( rbs );
    }
}

bool ModelTask::configureHook()
{
    if( ! ModelTaskBase::configureHook() )
        return false;

    // Test if setGazeboModel() has been called -> if world/model are NULL
    if( (!world) && (!model) )
        return false;

    setupLinks();
    setupJoints();

    return true;
}

std::string ModelTask::checkExportedLinkElements(std::string element_name, std::string test, std::string option)
{
    // when not defined, source_link and target_link will recieve "world".
    // when not defined, source_frame and target_frame will receive source_link and target_link content
    if( test.empty() )
    {
        gzmsg << "ModelTask: " << model->GetName() << " " << element_name << " not defined, using "<< option << std::endl;
        return option;
    }else {
        gzmsg << "ModelTask: " << model->GetName() << " " << element_name << ": " << test << std::endl;
        return test;
    }
}

