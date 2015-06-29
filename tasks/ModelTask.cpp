/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
//======================================================================================
// Brazilian Institute of Robotics 
// Authors: Thomio Watanabe
// Date: December 2014
//====================================================================================== 

#include "ModelTask.hpp"
#include <gazebo/common/Exception.hh>

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

ModelTask::ModelTask(string const& name)
	: ModelTaskBase(name)
{
}

ModelTask::ModelTask(string const& name, RTT::ExecutionEngine* engine)
	: ModelTaskBase(name, engine)
{
}

ModelTask::~ModelTask()
{
    for(ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it)
        delete it->port;
}

void ModelTask::setGazeboModel(WorldPtr _world,  ModelPtr _model)
{
    string name = "gazebo:" + _world->GetName() + ":" + _model->GetName();
    provides()->setName(name);
    _name.set(name);

    BaseTask::setGazeboWorld(_world);
    model = _model;

    if (_model_frame.get().empty())
        _model_frame.set(_model->GetName());
    if (_world_frame.get().empty())
        _world_frame.set(_world->GetName());
} 

void ModelTask::setupJoints()
{
    // Get all joints from a model and set Rock Input/Output Ports
    for(Joint_V::iterator joint = gazebo_joints.begin(); joint != gazebo_joints.end(); ++joint)
    {
        gzmsg << "ModelTask: found joint: " << world->GetName() + "/" + model->GetName() +
                "/" + (*joint)->GetName() << endl;

        joints_in.names.push_back( (*joint)->GetName() );
        joints_in.elements.push_back( base::JointState::Effort(0.0) );
    }
}

void ModelTask::setupLinks()
{
    // The robot configuration YAML file must define the exported links.
    vector<LinkExport> export_conf = _exported_links.get();
    set<string> port_names;
    for(vector<LinkExport>::iterator it = export_conf.begin();
            it != export_conf.end(); ++it)
    {
        ExportedLink exported_link;

        exported_link.source_link =
            checkExportedLinkElements("source_link", it->source_link, "world");
        exported_link.target_link =
            checkExportedLinkElements("target_link", it->target_link, "world");
        exported_link.source_frame =
            checkExportedLinkElements("source_frame", it->source_frame, it->source_link);
        exported_link.target_frame =
            checkExportedLinkElements("target_frame", it->target_frame, it->target_link);
        exported_link.source_link_ptr = model->GetLink( it->source_link );
        exported_link.target_link_ptr = model->GetLink( it->target_link );
        exported_link.port_name = it->port_name;

        if (it->source_link != "world" && !exported_link.source_link_ptr)
        { gzthrow("ModelTask: cannot find exported source link " << it->source_link << " in model"); }
        else if (it->target_link != "world" && !exported_link.target_link_ptr)
        { gzthrow("ModelTask: cannot find exported target link " << it->target_link << " in model"); }
        else if (it->port_name.empty())
        { gzthrow("ModelTask: no port name given in link export"); }
        else if (ports()->getPort(it->port_name))
        { gzthrow("ModelTask: provided port name " << it->port_name << " already used on the task interface"); }
        else if (port_names.count(it->port_name) != 0)
        { gzthrow("ModelTask: provided port name " << it->port_name << " already used by another exported link"); }

        port_names.insert(it->port_name);
        exported_links.push_back(exported_link);
    }


    for (ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it)
    {
        // Create the ports dynamicaly
        gzmsg << "ModelTask: exporting link "
            << world->GetName() + "/" + model->GetName() + "/" + it->source_link << "2" << it->target_link
            << " through port " << it->port_name
            << endl;

        it->port = new RBSOutPort( it->port_name );
        ports()->addPort(*it->port);
    }
}

void ModelTask::updateHook()
{
    base::Time time = getCurrentTime();

    updateModelPose(time);
    updateJoints(time);
    updateLinks(time);
}

void ModelTask::updateModelPose(base::Time const& time)
{
    math::Pose model2world = model->GetWorldPose();
    math::Vector3 model2world_angular_vel = model->GetWorldAngularVel();
    math::Vector3 model2world_vel = model->GetWorldLinearVel();

    RigidBodyState rbs;
    rbs.invalidate();
    rbs.time = time;
    rbs.sourceFrame = _model_frame.get();
    rbs.targetFrame = _world_frame.get();
    rbs.position = base::Vector3d(
        model2world.pos.x,model2world.pos.y,model2world.pos.z);
    rbs.orientation = base::Quaterniond(
        model2world.rot.w,model2world.rot.x,model2world.rot.y,model2world.rot.z );
    rbs.velocity = base::Vector3d(
        model2world_vel.x, model2world_vel.y, model2world_vel.z);
    rbs.angular_velocity = base::Vector3d(
        model2world_angular_vel.x, model2world_angular_vel.y, model2world_angular_vel.z);
    _pose_samples.write(rbs);
}

void ModelTask::updateJoints(base::Time const& time)
{
    // Read positions from Gazebo link
    vector<string> names;
    vector<double> positions;
    for(Joint_V::iterator it = gazebo_joints.begin(); it != gazebo_joints.end(); ++it )
    {
        // Read joint angle from gazebo link
        names.push_back( (*it)->GetScopedName() );
        positions.push_back( (*it)->GetAngle(0).Radian() );
    }
    base::samples::Joints joints = base::samples::Joints::Positions(positions,names);
    joints.time = time;
    _joints_samples.write( joints );

    // If we have commands, pass them on to gazebo
    if (_joints_cmd.readNewest( joints_in ) == RTT::NewData)
    {
        for(Joint_V::iterator it = gazebo_joints.begin(); it != gazebo_joints.end(); ++it )
        {
            base::JointState j_cmd(joints_in[(*it)->GetScopedName()]);

            // Apply effort to joint
            if( j_cmd.isEffort() )
                (*it)->SetForce(0, j_cmd.effort );
            else if( j_cmd.isPosition() )
                (*it)->SetPosition(0, j_cmd.position );
            else if( j_cmd.isSpeed() )
                (*it)->SetVelocity(0, j_cmd.speed );
        }
    }
}

void ModelTask::updateLinks(base::Time const& time)
{
    for(ExportedLinks::const_iterator it = exported_links.begin(); it != exported_links.end(); ++it)
    {
        math::Pose source_pose = math::Pose::Zero;
        if (it->source_link_ptr)
            source_pose = it->source_link_ptr->GetWorldPose();
        math::Pose target_pose = math::Pose::Zero;
        if (it->target_link_ptr)
            target_pose = it->target_link_ptr->GetWorldPose();
        math::Pose relative_pose( math::Pose(source_pose - target_pose) );

        RigidBodyState rbs;
        rbs.sourceFrame = it->source_frame;
        rbs.targetFrame = it->target_frame;
        rbs.position = base::Vector3d(
            relative_pose.pos.x,relative_pose.pos.y,relative_pose.pos.z);
        rbs.orientation = base::Quaterniond(
            relative_pose.rot.w,relative_pose.rot.x,relative_pose.rot.y,relative_pose.rot.z );
        rbs.time = time;

        it->port->write(rbs);
    }
}

bool ModelTask::configureHook()
{
    if( ! ModelTaskBase::configureHook() )
        return false;

    // Test if setGazeboModel() has been called -> if world/model are NULL
    if( (!world) && (!model) )
        return false;

    gazebo_joints = model->GetJoints();
    setupLinks();
    setupJoints();

    return true;
}

void ModelTask::cleanupHook()
{
    ModelTaskBase::cleanupHook();

    for(ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it)
        delete it->port;
    exported_links.clear();
}

string ModelTask::checkExportedLinkElements(string element_name, string test, string option)
{
    // when not defined, source_link and target_link will recieve "world".
    // when not defined, source_frame and target_frame will receive source_link and target_link content
    if( test.empty() )
    {
        gzmsg << "ModelTask: " << model->GetName() << " " << element_name << " not defined, using "<< option << endl;
        return option;
    }else {
        gzmsg << "ModelTask: " << model->GetName() << " " << element_name << ": " << test << endl;
        return test;
    }
}

