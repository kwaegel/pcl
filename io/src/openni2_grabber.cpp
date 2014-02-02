/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2010-2011, Willow Garage, Inc.
*  Copyright (c) 2012-, Open Perception, Inc.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder(s) nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <pcl/pcl_config.h>
#ifdef HAVE_OPENNI2

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2_camera/openni2_device_manager.h>
#include <pcl/io/openni2_camera/openni2_frame_listener.h>
#include <pcl/io/openni2_camera/openni_image.h>
#include <pcl/io/openni2_camera/openni_image_depth.h>
#include <pcl/io/openni2_camera/openni_image_yuv_422.h>
#include <pcl/io/openni2_camera/openni_image_rgb24.h>
#include <pcl/io/openni2_camera/openni_image_ir.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <pcl/exceptions.h>
#include <iostream>

using namespace openni2_wrapper;
using openni_wrapper::Image;
using openni_wrapper::DepthImage;


namespace pcl
{
  typedef union
  {
    struct
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    uint32_t long_value;
  } RGBValue;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::OpenNIGrabber::OpenNIGrabber (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode)
  : rgb_sync_ ()
  , ir_sync_ ()
  , device_ ()
  , rgb_frame_id_ ()
  , depth_frame_id_ ()
  , image_width_ ()
  , image_height_ ()
  , depth_width_ ()
  , depth_height_ ()
  , image_required_ (false)
  , depth_required_ (false)
  , ir_required_ (false)
  , sync_required_ (false)
  , image_signal_ (), depth_image_signal_ (), ir_image_signal_ (), image_depth_image_signal_ ()
  , ir_depth_image_signal_ (), point_cloud_signal_ (), point_cloud_i_signal_ ()
  , point_cloud_rgb_signal_ (), point_cloud_rgba_signal_ ()
  , config2oni_map_ ()
  , running_ (false)
  , rgb_focal_length_x_ (std::numeric_limits<double>::quiet_NaN ())
  , rgb_focal_length_y_ (std::numeric_limits<double>::quiet_NaN ())
  , rgb_principal_point_x_ (std::numeric_limits<double>::quiet_NaN ())
  , rgb_principal_point_y_ (std::numeric_limits<double>::quiet_NaN ())
  , depth_focal_length_x_ (std::numeric_limits<double>::quiet_NaN ())
  , depth_focal_length_y_ (std::numeric_limits<double>::quiet_NaN ())
  , depth_principal_point_x_ (std::numeric_limits<double>::quiet_NaN ())
  , depth_principal_point_y_ (std::numeric_limits<double>::quiet_NaN ())
{
  // initialize driver
  onInit (device_id, depth_mode, image_mode);

  if (!device_->hasSensor(openni::SENSOR_DEPTH))
    PCL_THROW_EXCEPTION (pcl::IOException, "Device does not provide 3D information.");

  depth_image_signal_    = createSignal<sig_cb_openni_depth_image> ();
  ir_image_signal_       = createSignal<sig_cb_openni_ir_image> ();
  point_cloud_signal_    = createSignal<sig_cb_openni_point_cloud> ();
  point_cloud_i_signal_  = createSignal<sig_cb_openni_point_cloud_i> ();
  ir_depth_image_signal_ = createSignal<sig_cb_openni_ir_depth_image> ();
  ir_sync_.addCallback (boost::bind (&OpenNIGrabber::irDepthImageCallback, this, _1, _2));

  // Register callbacks from the sensor to the grabber
  if (device_->hasSensor(openni::SENSOR_COLOR))
  {
    color_video_stream_->create(*device_, openni::SENSOR_COLOR);
    color_frame_listener_ = boost::make_shared<openni2_wrapper::OpenNI2FrameListener>();
    color_frame_listener_->setCallback(boost::bind(&OpenNIGrabber::processColorFrame, this, _1));
    color_video_stream_->addNewFrameListener(color_frame_listener_.get());
  }

  if (device_->hasSensor(openni::SENSOR_DEPTH))
  {
    depth_video_stream_->create(*device_, openni::SENSOR_DEPTH);
    depth_frame_listener_ = boost::make_shared<openni2_wrapper::OpenNI2FrameListener>();
    depth_frame_listener_->setCallback(boost::bind(&OpenNIGrabber::processDepthFrame, this, _1));
    depth_video_stream_->addNewFrameListener(depth_frame_listener_.get());
  }

  if (device_->hasSensor(openni::SENSOR_IR))
  {
    ir_video_stream_->create(*device_, openni::SENSOR_IR);
    ir_frame_listener_ = boost::make_shared<openni2_wrapper::OpenNI2FrameListener>();
    ir_frame_listener_->setCallback(boost::bind(&OpenNIGrabber::processIRFrame, this, _1));
    ir_video_stream_->addNewFrameListener(ir_frame_listener_.get());
  }


  // Register signals for grabber clients.
  // Assume we always have a depth image, so check for color sensor.
  if (device_->hasSensor(openni::SENSOR_COLOR))
  {
    // create callback signals
    image_signal_             = createSignal<sig_cb_openni_image> ();
    image_depth_image_signal_ = createSignal<sig_cb_openni_image_depth_image> ();
    point_cloud_rgb_signal_   = createSignal<sig_cb_openni_point_cloud_rgb> ();
    point_cloud_rgba_signal_  = createSignal<sig_cb_openni_point_cloud_rgba> ();
    rgb_sync_.addCallback (boost::bind (&OpenNIGrabber::imageDepthImageCallback, this, _1, _2));
    //openni_wrapper::DeviceKinect* kinect = dynamic_cast<openni_wrapper::DeviceKinect*> (device_.get ());
    //if (kinect)
    //  kinect->setDebayeringMethod (openni_wrapper::ImageBayerGRBG::EdgeAware);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::OpenNIGrabber::~OpenNIGrabber () throw ()
{
  try
  {
    stop ();

    // unregister callbacks
    if (device_->hasSensor(openni::SENSOR_DEPTH) )
      depth_video_stream_->removeNewFrameListener(depth_frame_listener_.get());
    if (device_->hasSensor(openni::SENSOR_COLOR) )
      color_video_stream_->removeNewFrameListener(color_frame_listener_.get());
    if (device_->hasSensor(openni::SENSOR_IR) )
      ir_video_stream_->removeNewFrameListener(ir_frame_listener_.get());

    // release the pointer to the device object
    device_.reset ();

    // disconnect all listeners
    disconnect_all_slots<sig_cb_openni_image> ();
    disconnect_all_slots<sig_cb_openni_depth_image> ();
    disconnect_all_slots<sig_cb_openni_ir_image> ();
    disconnect_all_slots<sig_cb_openni_image_depth_image> ();
    disconnect_all_slots<sig_cb_openni_point_cloud> ();
    disconnect_all_slots<sig_cb_openni_point_cloud_rgb> ();
    disconnect_all_slots<sig_cb_openni_point_cloud_rgba> ();
    disconnect_all_slots<sig_cb_openni_point_cloud_i> ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::checkImageAndDepthSynchronizationRequired ()
{
  // do we have anyone listening to images or color point clouds?
  if (num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0)
    sync_required_ = true;
  else
    sync_required_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::checkImageStreamRequired ()
{
  // do we have anyone listening to images or color point clouds?
  if (num_slots<sig_cb_openni_image>             () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgb>   () > 0)
    image_required_ = true;
  else
    image_required_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::checkDepthStreamRequired ()
{
  // do we have anyone listening to depth images or (color) point clouds?
  if (num_slots<sig_cb_openni_depth_image>       () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0 ||
    num_slots<sig_cb_openni_ir_depth_image>    () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_point_cloud>       () > 0 ||
    num_slots<sig_cb_openni_point_cloud_i>     () > 0 )
    depth_required_ = true;
  else
    depth_required_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::checkIRStreamRequired ()
{
  if (num_slots<sig_cb_openni_ir_image>       () > 0 ||
    num_slots<sig_cb_openni_point_cloud_i>  () > 0 ||
    num_slots<sig_cb_openni_ir_depth_image> () > 0)
    ir_required_ = true;
  else
    ir_required_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::start ()
{
  try
  {
    // check if we need to start/stop any stream
    if (image_required_)
    {
      color_video_stream_->start();
      block_signals ();
      startSynchronization ();
    }

    if (depth_required_)
    {
      block_signals ();

      if (device_->hasSensor(openni::SENSOR_COLOR)
          && device_->isImageRegistrationModeSupported (openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) )
      {
        device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      }
      depth_video_stream_->start();
      startSynchronization ();
    }

    if (ir_required_ )
    {
      block_signals ();
      ir_video_stream_->start();
    }
    running_ = true;
  }
  catch (openni_wrapper::OpenNIException& ex)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not start streams. Reason: " << ex.what ());
  }

  // workaround, since the first frame is corrupted
  // TODO: Is this still true with OpenNI 2?
  boost::this_thread::sleep (boost::posix_time::seconds (1));
  unblock_signals ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::stop ()
{
  try
  {
    if (depth_video_stream_)
      depth_video_stream_->stop();
    if (color_video_stream_)
      color_video_stream_->stop();
    if (ir_video_stream_)
      ir_video_stream_->stop();

    running_ = false;
  }
  catch (openni_wrapper::OpenNIException& ex)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not stop streams. Reason: " << ex.what ());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
  pcl::OpenNIGrabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::onInit (const std::string& device_id, const Mode& depth_mode, const Mode& image_mode)
{
  updateModeMaps (); // registering mapping from config modes to VideoModes and vice versa
  setupDevice (device_id, depth_mode, image_mode);

  rgb_frame_id_ = "/openni_rgb_optical_frame";

  depth_frame_id_ = "/openni_depth_optical_frame";
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::signalsChanged ()
{
  // reevaluate which streams are required
  checkImageStreamRequired ();
  checkDepthStreamRequired ();
  checkIRStreamRequired ();
  if (ir_required_ && image_required_)
    PCL_THROW_EXCEPTION (pcl::IOException, "Can not provide IR stream and RGB stream at the same time.");

  checkImageAndDepthSynchronizationRequired ();
  if (running_)
    start ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
  pcl::OpenNIGrabber::getName () const
{
  return std::string ("OpenNI2Grabber");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::setupDevice (const std::string& device_id, const Mode& depth_mode_enum, const Mode& image_mode_enum)
{
  // Initialize the openni device
  boost::shared_ptr<openni2_wrapper::OpenNI2DeviceManager> deviceManager 
    = openni2_wrapper::OpenNI2DeviceManager::getInstance();

  try
  {
    if (boost::filesystem::exists (device_id))
    {
      device_ = deviceManager->getDevice(device_id);	// Treat as file path
    }
    else if (deviceManager->getNumOfConnectedDevices() == 0)
    {
      PCL_THROW_EXCEPTION (pcl::IOException, "No devices connected.");
    }
    else if (device_id[0] == '#')
    {
      unsigned index = atoi (device_id.c_str () + 1);
      //printf("[%s] searching for device with index = %d\n", getName().c_str(), index);
      device_ = deviceManager->getDeviceByIndex (index - 1);
    }
    else
    {
      device_ = deviceManager->getAnyDevice();
    }
  }
  catch (const openni_wrapper::OpenNIException& exception)
  {
    if (!device_)
      PCL_THROW_EXCEPTION (pcl::IOException, "No matching device found. " << exception.what ())
    else
      PCL_THROW_EXCEPTION (pcl::IOException, "could not retrieve device. Reason " << exception.what ())
  }
  catch (const pcl::IOException&)
  {
    throw;
  }
  catch (...)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "unknown error occured");
  }

  typedef openni2_wrapper::OpenNI2VideoMode VideoMode;

  VideoMode depth_md;
  // Find compatable color mode
  if (depth_mode_enum != OpenNI_Default_Mode)
  {
    VideoMode actual_depth_md;
    if (!mapMode2XnMode (depth_mode_enum, depth_md) || !device_->findCompatibleDepthMode (depth_md, actual_depth_md))
      PCL_THROW_EXCEPTION (pcl::IOException, "could not find compatible depth stream mode " << static_cast<int> (depth_mode_enum) );

    VideoMode current_depth_md =  device_->getDepthVideoMode ();
    if (current_depth_md.x_resolution_ != actual_depth_md.x_resolution_ || current_depth_md.y_resolution_ != actual_depth_md.y_resolution_)
      device_->setDepthVideoMode (actual_depth_md);
  }
  else
  {
    // Use default video mode
    depth_md = device_->getDefaultDepthMode ();
  }

  depth_width_ = depth_md.x_resolution_;
  depth_height_ = depth_md.y_resolution_;

  // Find compatable color mode
  if (device_->hasSensor(openni::SENSOR_COLOR))
  {
    VideoMode image_md;
    if (image_mode_enum != OpenNI_Default_Mode)
    {
      VideoMode actual_image_md;
      if (!mapMode2XnMode (image_mode_enum, image_md) || !device_->findCompatibleColorMode (image_md, actual_image_md))
        PCL_THROW_EXCEPTION (pcl::IOException, "could not find compatible image stream mode " << static_cast<int> (image_mode_enum) );

      VideoMode current_image_md =  device_->getColorVideoMode ();
      if (current_image_md.x_resolution_ != actual_image_md.x_resolution_ || current_image_md.y_resolution_ != actual_image_md.y_resolution_)
        device_->setColorVideoMode (actual_image_md);
    }
    else
    {
      image_md = device_->getDefaultColorMode ();
    }

    image_width_  = image_md.x_resolution_;
    image_height_ = image_md.y_resolution_;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::startSynchronization ()
{
  openni::Status status = device_->setDepthColorSyncEnabled(true);
  if (status != openni::STATUS_OK)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not start synchronization.");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::stopSynchronization ()
{
  openni::Status status = device_->setDepthColorSyncEnabled(false);
  if (status != openni::STATUS_OK)
  {
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not stop synchronization.");
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Convert VideoFrameRef into pcl::Image and forward to registered callbacks
void pcl::OpenNIGrabber::processColorFrame(openni::VideoStream& stream)
{
  openni::VideoFrameRef frame;
  stream.readFrame(&frame);

  boost::chrono::high_resolution_clock::time_point t_readFrameTimestamp = boost::chrono::high_resolution_clock::now();

  openni::PixelFormat format = frame.getVideoMode().getPixelFormat();
  boost::shared_ptr<openni_wrapper::Image> image;

  // Convert frame to PCL image type, based on pixel format
  if (format == openni::PIXEL_FORMAT_YUV422)
    image = boost::make_shared<openni_wrapper::ImageYUV422> (frame, t_readFrameTimestamp);
  else //if (format == PixelFormat::PIXEL_FORMAT_RGB888)
    image = boost::make_shared<openni_wrapper::ImageRGB24> (frame, t_readFrameTimestamp);

  // Notify listeners of new frame
  imageCallback(image, this);
}


void pcl::OpenNIGrabber::processDepthFrame(openni::VideoStream& stream)
{
  openni::VideoFrameRef frame;
  stream.readFrame(&frame);
  boost::posix_time::ptime t_readFrameTimestamp = boost::posix_time::microsec_clock::local_time();

  float focalLength = getStreamFocalLength(depth_video_stream_);

  //status = depth_video_stream_->getProperty("LDDIS", baseline);
  double baseline = 7.62;	// HACK. Not sure how to read from device
  
  //status = depth_video_stream_->getProperty("LDDIS", baseline);
  uint64_t shadow_value_ = 0; // HACK. Not sure how to read from device
  
  //status = depth_video_stream_->getProperty("NoSampleValue", no_sample_value_);
  uint64_t no_sample_value_ = depth_video_stream_->getMinPixelValue();

  // Need: data, baseline_, getDepthFocalLength (), shadow_value_, no_sample_value_
  boost::shared_ptr<openni_wrapper::DepthImage> image = 
    boost::make_shared<openni_wrapper::DepthImage> (frame, baseline, focalLength, shadow_value_, no_sample_value_);

  // Notify listeners of new frame
  depthCallback(image, this);
}


void pcl::OpenNIGrabber::processIRFrame(openni::VideoStream& stream)
{
  openni::VideoFrameRef frame;
  stream.readFrame(&frame);
  boost::posix_time::ptime t_readFrameTimestamp = boost::posix_time::microsec_clock::local_time();

  boost::shared_ptr<openni_wrapper::IRImage> image = boost::make_shared<openni_wrapper::IRImage> (frame);

  // Notify listeners of new frame
  irCallback(image, this);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::imageCallback (boost::shared_ptr<openni_wrapper::Image> image, void*)
{
  if (num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0)
    rgb_sync_.add0 (image, image->getTimeStamp ());

  int numImageSlots = image_signal_->num_slots();
  if (numImageSlots > 0)
    image_signal_->operator()(image);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::depthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void*)
{
  if (num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
    num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
    num_slots<sig_cb_openni_image_depth_image> () > 0)
    rgb_sync_.add1 (depth_image, depth_image->getTimeStamp ());

  if (num_slots<sig_cb_openni_point_cloud_i>  () > 0 ||
    num_slots<sig_cb_openni_ir_depth_image> () > 0)
    ir_sync_.add1 (depth_image, depth_image->getTimeStamp ());

  if (depth_image_signal_->num_slots() > 0)
    depth_image_signal_->operator()(depth_image);

  if (point_cloud_signal_->num_slots() > 0)
    point_cloud_signal_->operator()(convertToXYZPointCloud(depth_image));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::irCallback (boost::shared_ptr<openni_wrapper::IRImage> ir_image, void*)
{
  if (num_slots<sig_cb_openni_point_cloud_i>  () > 0 ||
    num_slots<sig_cb_openni_ir_depth_image> () > 0)
    ir_sync_.add0(ir_image, ir_image->getTimeStamp ());

  if (ir_image_signal_->num_slots () > 0)
    ir_image_signal_->operator()(ir_image);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::imageDepthImageCallback (const boost::shared_ptr<openni_wrapper::Image> &image,
  const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image)
{
  // check if we have color point cloud slots
  if (point_cloud_rgb_signal_->num_slots () > 0)
  {
    PCL_WARN ("PointXYZRGB callbacks deprecated. Use PointXYZRGBA instead.\n");
    point_cloud_rgb_signal_->operator()(convertToXYZRGBPointCloud<pcl::PointXYZRGB> (image, depth_image));
  }

  if (point_cloud_rgba_signal_->num_slots () > 0)
    point_cloud_rgba_signal_->operator()(convertToXYZRGBPointCloud<pcl::PointXYZRGBA> (image, depth_image));

  if (image_depth_image_signal_->num_slots () > 0)
  {
    float reciprocalFocalLength = 1.0f / getStreamFocalLength(depth_video_stream_);
    image_depth_image_signal_->operator()(image, depth_image, reciprocalFocalLength);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
  pcl::OpenNIGrabber::irDepthImageCallback (const boost::shared_ptr<openni_wrapper::IRImage> &ir_image,
  const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image)
{
  // check if we have color point cloud slots
  if (point_cloud_i_signal_->num_slots () > 0)
    point_cloud_i_signal_->operator()(convertToXYZIPointCloud (ir_image, depth_image));

  if (ir_depth_image_signal_->num_slots () > 0)
  {
    float reciprocalFocalLength = 1.0f / getStreamFocalLength(depth_video_stream_);
    ir_depth_image_signal_->operator()(ir_image, depth_image, reciprocalFocalLength);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr
  pcl::OpenNIGrabber::convertToXYZPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);

  cloud->height = depth_height_;
  cloud->width = depth_width_;
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);

  register float constant_x = 1.0f / getStreamFocalLength(depth_video_stream_);
  register float constant_y = 1.0f / getStreamFocalLength(depth_video_stream_);
  register float centerX = ((float)cloud->width - 1.f) / 2.f;
  register float centerY = ((float)cloud->height - 1.f) / 2.f;

  if (pcl_isfinite (depth_focal_length_x_))
    constant_x =  1.0f / static_cast<float> (depth_focal_length_x_);

  if (pcl_isfinite (depth_focal_length_y_))
    constant_y =  1.0f / static_cast<float> (depth_focal_length_y_);

  if (pcl_isfinite (depth_principal_point_x_))
    centerX =  static_cast<float> (depth_principal_point_x_);

  if (pcl_isfinite (depth_principal_point_y_))
    centerY =  static_cast<float> (depth_principal_point_y_);

  //if (device_->isDepthRegistered ())
  if (true)
    cloud->header.frame_id = rgb_frame_id_;
  else
    cloud->header.frame_id = depth_frame_id_;


  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  // we have to use Data, since operator[] uses assert -> Debug-mode very slow!
  register const unsigned short* depth_map = (const unsigned short*) depth_image->getDepthMetaData ().getData ();
  if (depth_image->getWidth() != depth_width_ || depth_image->getHeight () != depth_height_)
  {
    static unsigned buffer_size = 0;
    static boost::shared_array<unsigned short> depth_buffer ((unsigned short*)(NULL));

    if (buffer_size < depth_width_ * depth_height_)
    {
      buffer_size = depth_width_ * depth_height_;
      depth_buffer.reset (new unsigned short [buffer_size]);
    }
    depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_buffer.get ());
    depth_map = depth_buffer.get ();
  }

  register int depth_idx = 0;
  for (int v = 0; v < depth_height_; ++v)
  {
    for (register int u = 0; u < depth_width_; ++u, ++depth_idx)
    {
      pcl::PointXYZ& pt = cloud->points[depth_idx];
      // Check for invalid measurements
      if (depth_map[depth_idx] == 0 ||
        depth_map[depth_idx] == depth_image->getNoSampleValue () ||
        depth_map[depth_idx] == depth_image->getShadowValue ())
      {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }
      pt.z = depth_map[depth_idx] * 0.001f;
      pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
      pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
    }
  }
  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.w () = 1.0f;
  cloud->sensor_orientation_.x () = 0.0f;
  cloud->sensor_orientation_.y () = 0.0f;
  cloud->sensor_orientation_.z () = 0.0f;  
  return (cloud);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
  pcl::OpenNIGrabber::convertToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image,
  const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const
{
  static unsigned rgb_array_size = 0;
  static boost::shared_array<unsigned char> rgb_array ((unsigned char*)(NULL));
  static unsigned char* rgb_buffer = 0;

  boost::shared_ptr<pcl::PointCloud<PointT> > cloud (new pcl::PointCloud<PointT>);

  cloud->header.frame_id = rgb_frame_id_;
  cloud->height = std::max (image_height_, depth_height_);
  cloud->width = std::max (image_width_, depth_width_);
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);

  // Generate default camera parameters
  float fx = device_->getColorFocalLength (depth_width_); // Horizontal focal length
  float fy = device_->getColorFocalLength (depth_width_); // Vertcal focal length
  register float cx = ((float)cloud->width - 1.f) / 2.f;  // Center x
  register float cy = ((float)cloud->height - 1.f) / 2.f; // Center y

  // Load pre-calibrated camera parameters if they exist
  if (pcl_isfinite (rgb_focal_length_x_))
    fx =  static_cast<float> (rgb_focal_length_x_);

  if (pcl_isfinite (rgb_focal_length_y_))
    fy =  static_cast<float> (rgb_focal_length_y_);

  if (pcl_isfinite (rgb_principal_point_x_))
    cx =  static_cast<float> (rgb_principal_point_x_);

  if (pcl_isfinite (rgb_principal_point_y_))
    cy =  static_cast<float> (rgb_principal_point_y_);

  // Get inverse focal length for calculations below
  register float fx_inv = 1.0f / fx;
  register float fy_inv = 1.0f / fy;

  register const OniDepthPixel* depth_map = (OniDepthPixel*) depth_image->getDepthMetaData().getData();
  if (depth_image->getWidth () != depth_width_ || depth_image->getHeight() != depth_height_)
  {
    static unsigned buffer_size = 0;
    static boost::shared_array<unsigned short> depth_buffer ((unsigned short*)(NULL));

    if (buffer_size < depth_width_ * depth_height_)
    {
      buffer_size = depth_width_ * depth_height_;
      depth_buffer.reset (new unsigned short [buffer_size]);
    }

    depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_buffer.get ());
    depth_map = depth_buffer.get();
  }

  // here we need exact the size of the point cloud for a one-one correspondence!
  if (rgb_array_size < image_width_ * image_height_ * 3)
  {
    rgb_array_size = image_width_ * image_height_ * 3;
    rgb_array.reset (new unsigned char [rgb_array_size]);
    rgb_buffer = rgb_array.get ();
  }
  image->fillRGB (image_width_, image_height_, rgb_buffer, image_width_ * 3);
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  // set xyz to Nan and rgb to 0 (black)  
  if (image_width_ != depth_width_)
  {
    PointT pt;
    pt.x = pt.y = pt.z = bad_point;
    pt.b = pt.g = pt.r = 0;
    pt.a = 255; // point has no color info -> alpha = max => transparent 
    cloud->points.assign (cloud->points.size (), pt);
  }

  // fill in XYZ values
  unsigned step = cloud->width / depth_width_;
  unsigned skip = cloud->width * step - cloud->width;

  int value_idx = 0;
  int point_idx = 0;
  for (int v = 0; v < depth_height_; ++v, point_idx += skip)
  {
    for (register int u = 0; u < depth_width_; ++u, ++value_idx, point_idx += step)
    {
      PointT& pt = cloud->points[point_idx];
      /// @todo Different values for these cases
      // Check for invalid measurements

      OniDepthPixel pixel = depth_map[value_idx];
      if (pixel != 0 &&
        pixel != depth_image->getNoSampleValue() &&
        pixel != depth_image->getShadowValue() )
      {
        pt.z = depth_map[value_idx] * 0.001f;  // millimeters to meters
        pt.x = (static_cast<float> (u) - cx) * pt.z * fx_inv;
        pt.y = (static_cast<float> (v) - cy) * pt.z * fy_inv;
      }
      else
      {
        pt.x = pt.y = pt.z = bad_point;
      }
    }
  }

  // fill in the RGB values
  step = cloud->width / image_width_;
  skip = cloud->width * step - cloud->width;

  value_idx = 0;
  point_idx = 0;
  RGBValue color;
  color.Alpha = 0;

  for (unsigned yIdx = 0; yIdx < image_height_; ++yIdx, point_idx += skip)
  {
    for (unsigned xIdx = 0; xIdx < image_width_; ++xIdx, point_idx += step, value_idx += 3)
    {
      PointT& pt = cloud->points[point_idx];

      color.Red   = rgb_buffer[value_idx];
      color.Green = rgb_buffer[value_idx + 1];
      color.Blue  = rgb_buffer[value_idx + 2];

      pt.rgba = color.long_value;
    }
  }
  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.setIdentity();
  return (cloud);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZI>::Ptr
  pcl::OpenNIGrabber::convertToXYZIPointCloud (const boost::shared_ptr<openni_wrapper::IRImage> &ir_image,
  const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const
{
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > cloud (new pcl::PointCloud<pcl::PointXYZI > ());

  cloud->header.frame_id = rgb_frame_id_;
  cloud->height = depth_height_;
  cloud->width = depth_width_;
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);


  float fx = getStreamFocalLength(color_video_stream_); // Horizontal focal length
  float fy = getStreamFocalLength(color_video_stream_);; // Vertcal focal length
  register float cx = ((float)cloud->width - 1.f) / 2.f;  // Center x
  register float cy = ((float)cloud->height - 1.f) / 2.f; // Center y

  if (pcl_isfinite (rgb_focal_length_x_))
    fx =  1.0f / static_cast<float> (rgb_focal_length_x_);

  if (pcl_isfinite (rgb_focal_length_y_))
    fy =  1.0f / static_cast<float> (rgb_focal_length_y_);

  if (pcl_isfinite (rgb_principal_point_x_))
    cx = static_cast<float>(rgb_principal_point_x_);

  if (pcl_isfinite (rgb_principal_point_y_))
    cy = static_cast<float>(rgb_principal_point_y_);

  register float fx_inv = 1.0f / fx;
  register float fy_inv = 1.0f / fy;



  register const OniDepthPixel* depth_map = (const OniDepthPixel*) depth_image->getDepthMetaData ().getData ();
  register const OniGrayscale16Pixel* ir_map = (OniGrayscale16Pixel*) ir_image->getMetaData ().getData ();

  if (depth_image->getWidth () != depth_width_ || depth_image->getHeight () != depth_height_)
  {
    static unsigned buffer_size = 0;
    static boost::shared_array<unsigned short> depth_buffer ((unsigned short*)(NULL));
    static boost::shared_array<unsigned short> ir_buffer ((unsigned short*)(NULL));

    if (buffer_size < depth_width_ * depth_height_)
    {
      buffer_size = depth_width_ * depth_height_;
      depth_buffer.reset (new unsigned short [buffer_size]);
      ir_buffer.reset (new unsigned short [buffer_size]);
    }

    depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_buffer.get ());
    depth_map = depth_buffer.get ();

    ir_image->fillRaw (depth_width_, depth_height_, ir_buffer.get ());
    ir_map = ir_buffer.get ();
  }

  register int depth_idx = 0;
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  for (int v = 0; v < depth_height_; ++v)
  {
    for (register int u = 0; u < depth_width_; ++u, ++depth_idx)
    {
      pcl::PointXYZI& pt = cloud->points[depth_idx];
      /// @todo Different values for these cases
      // Check for invalid measurements
      if (depth_map[depth_idx] == 0 ||
        depth_map[depth_idx] == depth_image->getNoSampleValue () ||
        depth_map[depth_idx] == depth_image->getShadowValue ())
      {
        pt.x = pt.y = pt.z = bad_point;
      }
      else
      {
        pt.z = depth_map[depth_idx] * 0.001f; // millimeters to meters
        pt.x = (static_cast<float> (u) - cx) * pt.z * fx;
        pt.y = (static_cast<float> (v) - cy) * pt.z * fy;
      }

      pt.data_c[0] = pt.data_c[1] = pt.data_c[2] = pt.data_c[3] = 0;
      pt.intensity = static_cast<float> (ir_map[depth_idx]);
    }
  }
  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.setIdentity();
  return (cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO: delete me?
void
pcl::OpenNIGrabber::updateModeMaps ()
{
  typedef openni2_wrapper::OpenNI2VideoMode VideoMode;

  config2oni_map_[OpenNI_SXGA_15Hz] = VideoMode(XN_SXGA_X_RES, XN_SXGA_Y_RES, 15);

  config2oni_map_[OpenNI_VGA_25Hz] = VideoMode(XN_VGA_X_RES, XN_VGA_Y_RES, 25);
  config2oni_map_[OpenNI_VGA_30Hz] = VideoMode(XN_VGA_X_RES, XN_VGA_Y_RES, 30);

  config2oni_map_[OpenNI_QVGA_25Hz] = VideoMode(XN_QVGA_X_RES, XN_QVGA_Y_RES, 25);
  config2oni_map_[OpenNI_QVGA_30Hz] = VideoMode(XN_QVGA_X_RES, XN_QVGA_Y_RES, 30);
  config2oni_map_[OpenNI_QVGA_60Hz] = VideoMode(XN_QVGA_X_RES, XN_QVGA_Y_RES, 60);

  config2oni_map_[OpenNI_QQVGA_25Hz] = VideoMode(XN_QQVGA_X_RES, XN_QQVGA_Y_RES, 25);
  config2oni_map_[OpenNI_QQVGA_30Hz] = VideoMode(XN_QQVGA_X_RES, XN_QQVGA_Y_RES, 30);
  config2oni_map_[OpenNI_QQVGA_60Hz] = VideoMode(XN_QQVGA_X_RES, XN_QQVGA_Y_RES, 60);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::OpenNIGrabber::mapMode2XnMode (Mode mode, openni2_wrapper::OpenNI2VideoMode &xnmode) const
{
  std::map<Mode, openni2_wrapper::OpenNI2VideoMode>::const_iterator it = config2oni_map_.find (mode);
  if (it != config2oni_map_.end ())
  {
    xnmode = it->second;
    return (true);
  }
  else
  {
    return (false);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::pair<int, openni2_wrapper::OpenNI2VideoMode> >
  pcl::OpenNIGrabber::getAvailableDepthModes () const
{
  openni2_wrapper::OpenNI2VideoMode dummy;
  std::vector<std::pair<int, openni2_wrapper::OpenNI2VideoMode> > result;
  for (auto it = config2oni_map_.begin (); it != config2oni_map_.end (); ++it)
  {
    if (device_->findCompatibleDepthMode (it->second, dummy))
      result.push_back (*it);
  }

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::pair<int, openni2_wrapper::OpenNI2VideoMode> >
  pcl::OpenNIGrabber::getAvailableImageModes () const
{
  openni2_wrapper::OpenNI2VideoMode dummy;
  std::vector<std::pair<int, openni2_wrapper::OpenNI2VideoMode> > result;
  for (auto itr = config2oni_map_.begin (); itr != config2oni_map_.end (); ++itr)
  {
    if (device_->findCompatibleColorMode (itr->second, dummy))
      result.push_back (*itr);
  }

  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::OpenNIGrabber::getFramesPerSecond () const
{
  depth_video_stream_->getVideoMode().getFps();
  return ( (float) depth_video_stream_->getVideoMode().getFps() );
}

float pcl::OpenNIGrabber::getStreamFocalLength(boost::shared_ptr<openni::VideoStream> stream) const
{
  int frameWidth = stream->getVideoMode().getResolutionX();
  float hFov = stream->getHorizontalFieldOfView();
  float calculatedFocalLengthX = frameWidth / (2.0f * tan(hFov / 2.0f));
  return calculatedFocalLengthX;
}

#endif
