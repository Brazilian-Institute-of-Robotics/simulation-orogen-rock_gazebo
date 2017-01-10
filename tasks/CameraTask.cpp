/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CameraTask.hpp"

using namespace std;
using namespace gazebo;
using namespace rock_gazebo;

CameraTask::CameraTask(std::string const& name)
    : CameraTaskBase(name),
    output_frame(new base::samples::frame::Frame())
{
}

CameraTask::CameraTask(std::string const& name, RTT::ExecutionEngine* engine)
    : CameraTaskBase(name, engine),
    output_frame(new base::samples::frame::Frame())
{
}

CameraTask::~CameraTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CameraTask.hpp for more detailed
// documentation about them.

bool CameraTask::configureHook()
{
    if (! CameraTaskBase::configureHook())
        return false;

    topicSubscribe(&CameraTask::readInput, baseTopicName + "/image");
    return true;
}
bool CameraTask::startHook()
{
    if (! CameraTaskBase::startHook())
        return false;
    bnew_data = false;
    return true;
}
void CameraTask::updateHook()
{
    CameraTaskBase::updateHook();
    if(bnew_data)
    {
        _frame.write(output_frame);
        bnew_data = false;
    }
}
void CameraTask::errorHook()
{
    CameraTaskBase::errorHook();
}
void CameraTask::stopHook()
{
    CameraTaskBase::stopHook();
}
void CameraTask::cleanupHook()
{
    CameraTaskBase::cleanupHook();
}

void CameraTask::readInput( ConstImageStampedPtr & imageMsg)
{
    const msgs::Image &image = imageMsg->image();
    if(!image.has_width())
        throw std::runtime_error("rock_gazebo::CameraTask requires has_width");
    if(!image.has_height())
        throw std::runtime_error("rock_gazebo::CameraTask requires has_height");
    if(!image.has_pixel_format())
        throw std::runtime_error("rock_gazebo::CameraTask requires has_pixel_format");
    if(!image.has_step())
        throw std::runtime_error("rock_gazebo::CameraTask requires has_step");
    if(!image.has_data())
        throw std::runtime_error("rock_gazebo::CameraTask requires has_data");

    // Convert the image data to RGB and copy to frame struct
    common::Image img;
    img.SetFromData((unsigned char *)(image.data().c_str()),image.width(),image.height(),
                    (common::Image::PixelFormat)(image.pixel_format()));

    unsigned char *data = NULL;
    unsigned int size = 0;
    img.GetRGBData(&data,size);

    base::samples::frame::Frame *pframe = output_frame.write_access();
    pframe->init(image.width(),image.height(),8,base::samples::frame::MODE_RGB);
    if(size != pframe->image.size())
        throw std::runtime_error("rock_gazebo::CameraTask image size mismatch");
    memcpy((void*)&(pframe->image.front()),(void*)data,size);
    pframe->time = getCurrentTime();
    pframe->frame_status = base::samples::frame::STATUS_VALID;
    output_frame.reset(pframe);
    pframe = NULL;
    bnew_data = true;
    delete [] data;
}
