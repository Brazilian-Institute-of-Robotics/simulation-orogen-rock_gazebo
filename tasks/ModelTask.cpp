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
    _cov_position.set(base::Matrix3d::Ones() * base::unset<double>());
    _cov_orientation.set(base::Matrix3d::Ones() * base::unset<double>());
    _cov_velocity.set(base::Matrix3d::Ones() * base::unset<double>());
}
ModelTask::ModelTask(string const& name, RTT::ExecutionEngine* engine)
	: ModelTaskBase(name, engine)
{
}

ModelTask::~ModelTask()
{
    releaseLinks();
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
#if GAZEBO_MAJOR_VERSION >= 6
        if((*joint)->HasType(physics::Base::FIXED_JOINT))
        {
            gzmsg << "ModelTask: ignore fixed joint: " << world->GetName() + "/" + model->GetName() +
                "/" + (*joint)->GetName() << endl;
            continue;
        }
#endif
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

    for(vector<LinkExport>::iterator it = export_conf.begin();
            it != export_conf.end(); ++it)
    {
        ExportedLink exported_link;

        exported_link.source_link =
            checkExportedLinkElements("source_link", it->source_link, _world_frame.get());
        exported_link.target_link =
            checkExportedLinkElements("target_link", it->target_link, _world_frame.get());
        exported_link.source_frame =
            checkExportedLinkElements("source_frame", it->source_frame, exported_link.source_link);
        exported_link.target_frame =
            checkExportedLinkElements("target_frame", it->target_frame, exported_link.target_link);

        if (it->source_link != _world_frame.get())
            exported_link.source_link_ptr = model->GetLink( it->source_link );
        if (it->target_link != _world_frame.get())
            exported_link.target_link_ptr = model->GetLink( it->target_link );
        exported_link.port_name = it->port_name;

        if (exported_link.source_link != _world_frame.get() && !exported_link.source_link_ptr)
        { gzthrow("ModelTask: cannot find exported source link " << it->source_link << " in model"); }
        else if (exported_link.target_link != _world_frame.get() && !exported_link.target_link_ptr)
        { gzthrow("ModelTask: cannot find exported target link " << it->target_link << " in model"); }
        else if (it->port_name.empty())
        { gzthrow("ModelTask: no port name given in link export"); }
        else if (ports()->getPort(it->port_name))
        { gzthrow("ModelTask: provided port name " << it->port_name << " already used on the task interface"); }
        else if (exported_links.find(it->port_name) != exported_links.end())
        { gzthrow("ModelTask: provided port name " << it->port_name << " already used by another exported link"); }

        exported_links.insert(make_pair(it->port_name, exported_link));
    }

    for (ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it)
    {
        // Create the ports dynamicaly
        gzmsg << "ModelTask: exporting link "
            << world->GetName() + "/" + model->GetName() + "/" + it->second.source_link << "2" << it->second.target_link
            << " through port " << it->first
            << endl;

        it->second.port = new RBSOutPort( it->first );
        ports()->addPort(*it->second.port);
    }
}

void ModelTask::updateHook()
{
    base::Time time = getCurrentTime();

    base::samples::RigidBodyState modelPose;
    if (_model_pose.read(modelPose) == RTT::NewData)
        warpModel(modelPose);

    updateModelPose(time);
    updateJoints(time);
    updateLinks(time);
}

void ModelTask::warpModel(base::samples::RigidBodyState const& modelPose)
{
    Eigen::Vector3d v(modelPose.position);
    math::Vector3 model2world_v(v.x(), v.y(), v.z());
    Eigen::Quaterniond q(modelPose.orientation);
    math::Quaternion model2world_q(q.w(), q.x(), q.y(), q.z());
    math::Pose model2world;
    model2world.Set(model2world_v, model2world_q);
    model->SetWorldPose(model2world);
}

void ModelTask::updateModelPose(base::Time const& time)
{
    math::Pose model2world = model->GetWorldPose();
    math::Vector3 model2world_angular_vel = model->GetRelativeAngularVel();
    math::Vector3 model2world_vel = model->GetWorldLinearVel();

    RigidBodyState rbs;
    rbs.invalidate();
    rbs.time = time;
    rbs.sourceFrame = _model_frame.get();
    rbs.targetFrame = _world_frame.get();
    rbs.position = base::Vector3d(
        model2world.pos.x,model2world.pos.y,model2world.pos.z);
    rbs.cov_position = _cov_position.get();
    rbs.orientation = base::Quaterniond(
        model2world.rot.w,model2world.rot.x,model2world.rot.y,model2world.rot.z );
    rbs.cov_orientation = _cov_orientation.get();
    rbs.velocity = base::Vector3d(
        model2world_vel.x, model2world_vel.y, model2world_vel.z);
    rbs.cov_velocity = _cov_velocity.get();
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
#if GAZEBO_MAJOR_VERSION >= 6
        // Do not export fixed joints
        if((*it)->HasType(physics::Base::FIXED_JOINT))
            continue;
#endif

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
	const std::vector<std::string> &names= joints_in.names;
	std::vector<std::string>::const_iterator result;
        for(Joint_V::iterator it = gazebo_joints.begin(); it != gazebo_joints.end(); ++it )
        {
#if GAZEBO_MAJOR_VERSION >= 6
            // Do not set fixed joints
            if((*it)->HasType(physics::Base::FIXED_JOINT))
                continue;
#endif

            // Do not set joints which are not part of the command
            result = std::find(names.begin(),names.end(),(*it)->GetScopedName());
	    if(result == names.end())
                continue;

            // Apply effort to joint
            base::JointState j_cmd(joints_in[result-names.begin()]);
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
        math::Pose source2world = math::Pose::Zero;
        if (it->second.source_link_ptr)
            source2world = it->second.source_link_ptr->GetWorldPose();
        math::Pose target2world = math::Pose::Zero;
        if (it->second.target_link_ptr)
            target2world = it->second.target_link_ptr->GetWorldPose();
        math::Pose source2target( math::Pose(source2world - target2world) );

        RigidBodyState rbs;
        rbs.sourceFrame = it->second.source_frame;
        rbs.targetFrame = it->second.target_frame;
        rbs.position = base::Vector3d(
            source2target.pos.x,source2target.pos.y,source2target.pos.z);
        rbs.orientation = base::Quaterniond(
            source2target.rot.w,source2target.rot.x,source2target.rot.y,source2target.rot.z );
        rbs.time = time;

        rbs.time = base::Time::now();
        it->second.port->write(rbs);
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
    releaseLinks();
}

void ModelTask::releaseLinks()
{
    for(ExportedLinks::iterator it = exported_links.begin(); it != exported_links.end(); ++it) {
        if (it->second.port != NULL) {
            ports()->removePort(it->second.port->getName());
            delete it->second.port;
            it->second.port = NULL;
        }
    }
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

