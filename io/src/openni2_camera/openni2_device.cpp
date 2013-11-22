/*
* Copyright (c) 2013, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the Willow Garage, Inc. nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*      Author: Julius Kammerl (jkammerl@willowgarage.com)
*/

#include "OpenNI.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/chrono.hpp>

#include "pcl/io/openni2_camera/openni2_device.h"
#include "pcl/io/openni2_camera/openni2_exception.h"
#include "pcl/io/openni2_camera/openni2_convert.h"
#include "pcl/io/openni2_camera/openni2_frame_listener.h"

#include "pcl/io/openni2_camera/openni_image_yuv_422.h"

#include <string>

namespace openni2_wrapper
{
  using openni::VideoMode;
  using std::vector;

  typedef boost::chrono::high_resolution_clock hr_clock;

  OpenNI2Device::OpenNI2Device(const std::string& device_URI) throw () :
    openni_device_(),
    ir_video_started_(false),
    color_video_started_(false),
    depth_video_started_(false),
    image_registration_activated_(false),
    use_device_time_(false),
    rgb_focal_length_SXGA_ (1050),  // Magic default value from prior calibration
    depth_focal_length_SXGA_()
  {
    openni::Status status = openni::OpenNI::initialize();
    if (status != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());

    openni_device_ = boost::make_shared<openni::Device>();

    if (device_URI.length() > 0)
    {
      status = openni_device_->open(device_URI.c_str());
    }
    else
    {
      status = openni_device_->open(openni::ANY_DEVICE);
    }

    if (status != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());

    // Get depth calculation parameters
    // Not sure what property codes to use, as OpenNI 1.x used strings
    double baseline = 7.62;	// HACK
    //status = depth_video_stream_->getProperty("LDDIS", baseline);
    if (status != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the baseline failed. Reason: %s", openni::OpenNI::getExtendedError());
    // baseline from cm -> meters
    baseline_ = static_cast<float> (baseline * 0.01);

    //status = depth_video_stream_->getProperty("ShadowValue", shadow_value_);
    if (status != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the value for pixels in shadow regions failed. Reason: %s", openni::OpenNI::getExtendedError());
    shadow_value_ = 0; // HACK

    //status = depth_video_stream_->getProperty("NoSampleValue", no_sample_value_);
    if (status != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION ("reading the value for pixels with no depth estimation failed. Reason: %s", openni::OpenNI::getExtendedError());
    no_sample_value_ = getDepthVideoStream()->getMinPixelValue();


    // Set default resolution
    setColorVideoMode(getDefaultColorMode());
    setDepthVideoMode(getDefaultDepthMode());
    setIRVideoMode(getDefaultIRMode());


    device_info_ = boost::make_shared<openni::DeviceInfo>();
    *device_info_ = openni_device_->getDeviceInfo();

    color_frame_listener = boost::make_shared<OpenNI2FrameListener>();
    color_frame_listener->setCallback(boost::bind(&OpenNI2Device::processColorFrame, this, _1));

    depth_frame_listener = boost::make_shared<OpenNI2FrameListener>();
    depth_frame_listener->setCallback(boost::bind(&OpenNI2Device::processDepthFrame, this, _1));

    ir_frame_listener = boost::make_shared<OpenNI2FrameListener>();
    ir_frame_listener->setCallback(boost::bind(&OpenNI2Device::processIRFrame, this, _1));
  }

  OpenNI2Device::~OpenNI2Device()
  {
    stopAllStreams();

    shutdown();

    openni_device_->close();
  }

  const std::string OpenNI2Device::getUri() const
  {
    return std::string(device_info_->getUri());
  }

  const std::string OpenNI2Device::getVendor() const
  {
    return std::string(device_info_->getVendor());
  }

  const std::string OpenNI2Device::getName() const
  {
    return std::string(device_info_->getName());
  }

  uint16_t OpenNI2Device::getUsbVendorId() const
  {
    return device_info_->getUsbVendorId();
  }

  uint16_t OpenNI2Device::getUsbProductId() const
  {
    return device_info_->getUsbProductId();
  }

  const std::string OpenNI2Device::getStringID() const
  {
    std::string ID_str = getName() + "_" + getVendor();

    boost::replace_all(ID_str, "/", "");
    boost::replace_all(ID_str, ".", "");
    boost::replace_all(ID_str, "@", "");

    return ID_str;
  }

  bool OpenNI2Device::isValid() const
  {
    return (openni_device_.get() != 0) && openni_device_->isValid();
  }

  float OpenNI2Device::getIRFocalLength(int output_x_resolution) const
  {
    boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

    int frameWidth = stream->getVideoMode().getResolutionX();
    float hFov = stream->getHorizontalFieldOfView();
    float calculatedFocalLengthX = frameWidth / (2.0f * tan(hFov / 2.0f));
    return calculatedFocalLengthX;

    /*
    if (output_x_resolution == 0)
      output_x_resolution = stream->getVideoMode().getResolutionX();

    float scale = static_cast<float> (output_x_resolution) / static_cast<float> (XN_SXGA_X_RES);

    if (isDepthRegistered())
      return (rgb_focal_length_SXGA_ * scale);
    else
      return (depth_focal_length_SXGA_ * scale);
    */
  }

  float OpenNI2Device::getColorFocalLength(int output_x_resolution) const
  {
    boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

    int frameWidth = stream->getVideoMode().getResolutionX();
    float hFov = stream->getHorizontalFieldOfView();
    float calculatedFocalLengthX = frameWidth / (2.0f * tan(hFov / 2.0f));
    return calculatedFocalLengthX;

    // Same result for vertical, with rounding errors.
    //int frameHeight = stream->getVideoMode().getResolutionY();
    //float vFov = stream->getVerticalFieldOfView();
    //float calculatedFocalLengthY = frameHeight / (2.0f * tan(vFov / 2.0f));

    /*
    if (output_x_resolution == 0)
      output_x_resolution = stream->getVideoMode().getResolutionX();

    float scale = static_cast<float> (output_x_resolution) / static_cast<float> (XN_SXGA_X_RES);
    float scaledFocalLength = rgb_focal_length_SXGA_ * scale;
    return scaledFocalLength;
    */
  }

  float OpenNI2Device::getDepthFocalLength(int output_x_resolution) const
  {
    boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

    int frameWidth = stream->getVideoMode().getResolutionX();
    float hFov = stream->getHorizontalFieldOfView();
    float calculatedFocalLengthX = frameWidth / (2.0f * tan(hFov / 2.0f));
    return calculatedFocalLengthX;
    
    /*
    if (output_x_resolution == 0)
      output_x_resolution = stream->getVideoMode().getResolutionX();

    float scale = static_cast<float> (output_x_resolution) / static_cast<float> (XN_SXGA_X_RES);

    if (isDepthRegistered())
      return (rgb_focal_length_SXGA_ * scale);
    else
      return (depth_focal_length_SXGA_ * scale);
      */
  }

  bool OpenNI2Device::isIRVideoModeSupported(const OpenNI2VideoMode& video_mode) const
  {
    getSupportedIRVideoModes();

    bool supported = false;

    std::vector<OpenNI2VideoMode>::const_iterator it = ir_video_modes_.begin();
    std::vector<OpenNI2VideoMode>::const_iterator it_end = ir_video_modes_.end();

    while (it != it_end && !supported)
    {
      supported = (*it == video_mode);
      ++it;
    }

    return supported;
  }

  bool OpenNI2Device::isColorVideoModeSupported(const OpenNI2VideoMode& video_mode) const
  {
    getSupportedColorVideoModes();

    bool supported = false;

    std::vector<OpenNI2VideoMode>::const_iterator it = color_video_modes_.begin();
    std::vector<OpenNI2VideoMode>::const_iterator it_end = color_video_modes_.end();

    while (it != it_end && !supported)
    {
      supported = (*it == video_mode);
      ++it;
    }

    return supported;
  }

  bool OpenNI2Device::isDepthVideoModeSupported(const OpenNI2VideoMode& video_mode) const
  {
    getSupportedDepthVideoModes();

    bool supported = false;

    std::vector<OpenNI2VideoMode>::const_iterator it = depth_video_modes_.begin();
    std::vector<OpenNI2VideoMode>::const_iterator it_end = depth_video_modes_.end();

    while (it != it_end && !supported)
    {
      supported = (*it == video_mode);
      ++it;
    }

    return supported;

  }

  bool OpenNI2Device::hasIRSensor() const
  {
    return openni_device_->hasSensor(openni::SENSOR_IR);
  }

  bool OpenNI2Device::hasColorSensor() const
  {
    return openni_device_->hasSensor(openni::SENSOR_COLOR);
  }

  bool OpenNI2Device::hasDepthSensor() const
  {
    return openni_device_->hasSensor(openni::SENSOR_DEPTH);
  }

  void OpenNI2Device::startIRStream()
  {
    boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

    if (stream)
    {
      stream->setMirroringEnabled(false);
      stream->start();
      stream->addNewFrameListener(ir_frame_listener.get());
      ir_video_started_ = true;
    }

  }

  void OpenNI2Device::startColorStream()
  {
    boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

    if (stream)
    {
      stream->setMirroringEnabled(false);
      stream->start();
      stream->addNewFrameListener(color_frame_listener.get());
      color_video_started_ = true;
    }
  }
  void OpenNI2Device::startDepthStream()
  {
    boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

    if (stream)
    {
      stream->setMirroringEnabled(false);
      stream->start();
      stream->addNewFrameListener(depth_frame_listener.get());
      depth_video_started_ = true;
    }
  }

  void OpenNI2Device::stopAllStreams()
  {
    stopIRStream();
    stopColorStream();
    stopDepthStream();
  }

  void OpenNI2Device::stopIRStream()
  {
    if (ir_video_stream_.get() != 0)
    {
      ir_video_started_ = false;

      ir_video_stream_->removeNewFrameListener(ir_frame_listener.get());

      ir_video_stream_->stop();
    }
  }
  void OpenNI2Device::stopColorStream()
  {
    if (color_video_stream_.get() != 0)
    {
      color_video_started_ = false;

      color_video_stream_->removeNewFrameListener(color_frame_listener.get());

      color_video_stream_->stop();
    }
  }
  void OpenNI2Device::stopDepthStream()
  {
    if (depth_video_stream_.get() != 0)
    {
      depth_video_started_ = false;

      depth_video_stream_->removeNewFrameListener(depth_frame_listener.get());

      depth_video_stream_->stop();
    }
  }

  void OpenNI2Device::shutdown()
  {
    if (ir_video_stream_.get() != 0)
      ir_video_stream_->destroy();

    if (color_video_stream_.get() != 0)
      color_video_stream_->destroy();

    if (depth_video_stream_.get() != 0)
      depth_video_stream_->destroy();

  }

  bool OpenNI2Device::isIRStreamStarted()
  {
    return ir_video_started_;
  }
  bool OpenNI2Device::isColorStreamStarted()
  {
    return color_video_started_;
  }
  bool OpenNI2Device::isDepthStreamStarted()
  {
    return depth_video_started_;
  }

  bool OpenNI2Device::isImageRegistrationModeSupported() const
  {
    return openni_device_->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  }

  void OpenNI2Device::setImageRegistrationMode(bool setEnable) throw ()
  {
    bool registrationSupported = isImageRegistrationModeSupported();
    if (registrationSupported)
    {
      if (setEnable)
      {
        openni::Status rc = openni_device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        if (rc != openni::STATUS_OK)
          THROW_OPENNI_EXCEPTION("Enabling image registration mode failed: \n%s\n", openni::OpenNI::getExtendedError());
      }
      else
      {
        openni::Status rc = openni_device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
        if (rc != openni::STATUS_OK)
          THROW_OPENNI_EXCEPTION("Enabling image registration mode failed: \n%s\n", openni::OpenNI::getExtendedError());
      }
      image_registration_activated_ = setEnable;
    }
  }

  bool OpenNI2Device::isDepthRegistered() const throw ()
  {
    return openni_device_->getImageRegistrationMode() == openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
  }

  void OpenNI2Device::setDepthColorSync(bool enabled) throw ()
  {
    openni::Status rc = openni_device_->setDepthColorSyncEnabled(enabled);
    if (rc != openni::STATUS_OK)
      THROW_OPENNI_EXCEPTION("Enabling depth color synchronization failed: \n%s\n", openni::OpenNI::getExtendedError());
  }

  const OpenNI2VideoMode OpenNI2Device::getIRVideoMode() throw ()
  {
    OpenNI2VideoMode ret;

    boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

    if (stream)
    {
      openni::VideoMode video_mode = stream->getVideoMode();

      ret = openniModeToGrabberMode(video_mode);
    }
    else
      THROW_OPENNI_EXCEPTION("Could not create video stream.");

    return ret;
  }
  const OpenNI2VideoMode OpenNI2Device::getColorVideoMode() throw ()
  {
    OpenNI2VideoMode ret;

    boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

    if (stream)
    {
      openni::VideoMode video_mode = stream->getVideoMode();

      ret = openniModeToGrabberMode(video_mode);
    }
    else
      THROW_OPENNI_EXCEPTION("Could not create video stream.");

    return ret;
  }
  const OpenNI2VideoMode OpenNI2Device::getDepthVideoMode() throw ()
  {
    OpenNI2VideoMode ret;

    boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

    if (stream)
    {
      openni::VideoMode video_mode = stream->getVideoMode();

      ret = openniModeToGrabberMode(video_mode);
    }
    else
      THROW_OPENNI_EXCEPTION("Could not create video stream.");

    return ret;
  }

  void OpenNI2Device::setIRVideoMode(const OpenNI2VideoMode& video_mode) throw ()
  {
    boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();

    if (stream)
    {
      const openni::VideoMode videoMode = grabberModeToOpenniMode(video_mode);
      const openni::Status rc = stream->setVideoMode(videoMode);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't set IR video mode: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
  void OpenNI2Device::setColorVideoMode(const OpenNI2VideoMode& video_mode) throw ()
  {
    boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

    if (stream)
    {
      openni::VideoMode videoMode = grabberModeToOpenniMode(video_mode);
      const openni::Status rc = stream->setVideoMode(videoMode);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't set color video mode: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }
  void OpenNI2Device::setDepthVideoMode(const OpenNI2VideoMode& video_mode) throw ()
  {
    boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

    if (stream)
    {
      const openni::VideoMode videoMode = grabberModeToOpenniMode(video_mode);
      const openni::Status rc = stream->setVideoMode(videoMode);
      if (rc != openni::STATUS_OK)
        THROW_OPENNI_EXCEPTION("Couldn't set depth video mode: \n%s\n", openni::OpenNI::getExtendedError());
    }
  }

  OpenNI2VideoMode OpenNI2Device::getDefaultIRMode() const
  {
    // Search for and return VGA@30 Hz mode
    vector<OpenNI2VideoMode> modeList = getSupportedIRVideoModes();
    for (vector<OpenNI2VideoMode>::iterator modeItr = modeList.begin(); modeItr != modeList.end(); modeItr++)
    {
      OpenNI2VideoMode mode = *modeItr;
      if ( (mode.x_resolution_ == 640) && (mode.y_resolution_ == 480) && (mode.frame_rate_ = 30.0) )
        return mode;
    }
    return modeList.at(0); // Return first mode if we can't find VGA
  }
  OpenNI2VideoMode OpenNI2Device::getDefaultColorMode() const
  {
    // Search for and return VGA@30 Hz mode
    vector<OpenNI2VideoMode> modeList = getSupportedColorVideoModes();
    for (vector<OpenNI2VideoMode>::iterator modeItr = modeList.begin(); modeItr != modeList.end(); modeItr++)
    {
      OpenNI2VideoMode mode = *modeItr;
      if ( (mode.x_resolution_ == 640) && (mode.y_resolution_ == 480) && (mode.frame_rate_ = 30.0) )
        return mode;
    }
    return modeList.at(0); // Return first mode if we can't find VGA
  }
  OpenNI2VideoMode OpenNI2Device::getDefaultDepthMode() const
  {
    // Search for and return VGA@30 Hz mode
    vector<OpenNI2VideoMode> modeList = getSupportedDepthVideoModes();
    for (vector<OpenNI2VideoMode>::iterator modeItr = modeList.begin(); modeItr != modeList.end(); modeItr++)
    {
      OpenNI2VideoMode mode = *modeItr;
      if ( (mode.x_resolution_ == 640) && (mode.y_resolution_ == 480) && (mode.frame_rate_ = 30.0) )
        return mode;
    }
    return modeList.at(0); // Return first mode if we can't find VGA
  }

  const std::vector<OpenNI2VideoMode>& OpenNI2Device::getSupportedIRVideoModes() const
  {
    boost::shared_ptr<openni::VideoStream> stream = getIRVideoStream();
    ir_video_modes_.clear();

    if (stream)
    {
      const openni::SensorInfo& sensor_info = stream->getSensorInfo();

      ir_video_modes_ = openniModeToGrabberMode(sensor_info.getSupportedVideoModes());
    }

    return ir_video_modes_;
  }
  const std::vector<OpenNI2VideoMode>& OpenNI2Device::getSupportedColorVideoModes() const
  {
    boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

    color_video_modes_.clear();

    if (stream)
    {
      const openni::SensorInfo& sensor_info = stream->getSensorInfo();

      color_video_modes_ = openniModeToGrabberMode(sensor_info.getSupportedVideoModes());
    }

    return color_video_modes_;
  }
  const std::vector<OpenNI2VideoMode>& OpenNI2Device::getSupportedDepthVideoModes() const
  {
    boost::shared_ptr<openni::VideoStream> stream = getDepthVideoStream();

    depth_video_modes_.clear();

    if (stream)
    {
      const openni::SensorInfo& sensor_info = stream->getSensorInfo();

      depth_video_modes_ = openniModeToGrabberMode(sensor_info.getSupportedVideoModes());
    }

    return depth_video_modes_;
  }

  bool OpenNI2Device::findCompatibleIRMode (const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const throw ()
  {
    if ( isIRVideoModeSupported(requested_mode) )
    {
      actual_mode = requested_mode;
      return (true);
    }
    else
    {
      // Find a resize-compatable mode
      std::vector<OpenNI2VideoMode> supportedModes = getSupportedIRVideoModes();
      bool found = findCompatibleVideoMode(supportedModes, requested_mode, actual_mode);
      return (found);
    }
  }
  bool OpenNI2Device::findCompatibleColorMode (const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const throw ()
  {
    if ( isColorVideoModeSupported(requested_mode) )
    {
      actual_mode = requested_mode;
      return (true);
    }
    else
    {
      // Find a resize-compatable mode
      std::vector<OpenNI2VideoMode> supportedModes = getSupportedColorVideoModes();
      bool found = findCompatibleVideoMode(supportedModes, requested_mode, actual_mode);
      return (found);
    }
  }
  bool OpenNI2Device::findCompatibleDepthMode (const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const throw ()
  {
    if ( isDepthVideoModeSupported(requested_mode) )
    {
      actual_mode = requested_mode;
      return (true);
    }
    else
    {
      // Find a resize-compatable mode
      std::vector<OpenNI2VideoMode> supportedModes = getSupportedDepthVideoModes();
      bool found = findCompatibleVideoMode(supportedModes, requested_mode, actual_mode);
      return (found);
    }
  }

  // Generic support method for the above findCompatable...Mode calls above
  bool OpenNI2Device::findCompatibleVideoMode (const std::vector<OpenNI2VideoMode> supportedModes, const OpenNI2VideoMode& requested_mode, OpenNI2VideoMode& actual_mode) const throw ()
  {
    bool found = false;
    for (std::vector<OpenNI2VideoMode>::const_iterator modeIt = supportedModes.begin(); modeIt != supportedModes.end(); ++modeIt)
    {
      if (modeIt->frame_rate_ == requested_mode.frame_rate_ 
        && resizingSupported(modeIt->x_resolution_, modeIt->y_resolution_, requested_mode.x_resolution_, requested_mode.y_resolution_))
      {
        if (found)
        { // check wheter the new mode is better -> smaller than the current one.
          if (actual_mode.x_resolution_ * actual_mode.x_resolution_ > modeIt->x_resolution_ * modeIt->y_resolution_ )
            actual_mode = *modeIt;
        }
        else
        {
          actual_mode = *modeIt;
          found = true;
        }
      }
    }
    return (found);
  }

  bool OpenNI2Device::resizingSupported (size_t input_width, size_t input_height, size_t output_width, size_t output_height) const
  {
    return (output_width <= input_width && output_height <= input_height && input_width % output_width == 0 && input_height % output_height == 0 );
  }

  void OpenNI2Device::setAutoExposure(bool enable) throw ()
  {
    boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

    if (stream)
    {
      openni::CameraSettings* camera_seeting = stream->getCameraSettings();
      if (camera_seeting)
      {
        const openni::Status rc = camera_seeting->setAutoExposureEnabled(enable);
        if (rc != openni::STATUS_OK)
          THROW_OPENNI_EXCEPTION("Couldn't set auto exposure: \n%s\n", openni::OpenNI::getExtendedError());
      }

    }
  }
  void OpenNI2Device::setAutoWhiteBalance(bool enable) throw ()
  {
    boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

    if (stream)
    {
      openni::CameraSettings* camera_seeting = stream->getCameraSettings();
      if (camera_seeting)
      {
        const openni::Status rc = camera_seeting->setAutoWhiteBalanceEnabled(enable);
        if (rc != openni::STATUS_OK)
          THROW_OPENNI_EXCEPTION("Couldn't set auto white balance: \n%s\n", openni::OpenNI::getExtendedError());
      }

    }
  }

  bool OpenNI2Device::getAutoExposure() const
  {
    bool ret = false;

    boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

    if (stream)
    {
      openni::CameraSettings* camera_seeting = stream->getCameraSettings();
      if (camera_seeting)
        ret = camera_seeting->getAutoExposureEnabled();
    }

    return ret;
  }
  bool OpenNI2Device::getAutoWhiteBalance() const
  {
    bool ret = false;

    boost::shared_ptr<openni::VideoStream> stream = getColorVideoStream();

    if (stream)
    {
      openni::CameraSettings* camera_setting = stream->getCameraSettings();
      if (camera_setting)
        ret = camera_setting->getAutoWhiteBalanceEnabled();
    }

    return ret;
  }


  boost::shared_ptr<openni::VideoStream> OpenNI2Device::getIRVideoStream() const throw ()
  {
    if (ir_video_stream_.get() == 0)
    {
      if (hasIRSensor())
      {
        ir_video_stream_ = boost::make_shared<openni::VideoStream>();

        const openni::Status rc = ir_video_stream_->create(*openni_device_, openni::SENSOR_IR);
        if (rc != openni::STATUS_OK)
          THROW_OPENNI_EXCEPTION("Couldn't create IR video stream: \n%s\n", openni::OpenNI::getExtendedError());
      }
    }
    return ir_video_stream_;
  }

  boost::shared_ptr<openni::VideoStream> OpenNI2Device::getColorVideoStream() const throw ()
  {
    if (color_video_stream_.get() == 0)
    {
      if (hasColorSensor())
      {
        color_video_stream_ = boost::make_shared<openni::VideoStream>();

        const openni::Status rc = color_video_stream_->create(*openni_device_, openni::SENSOR_COLOR);
        if (rc != openni::STATUS_OK)
          THROW_OPENNI_EXCEPTION("Couldn't create color video stream: \n%s\n", openni::OpenNI::getExtendedError());
      }
    }
    return color_video_stream_;
  }

  boost::shared_ptr<openni::VideoStream> OpenNI2Device::getDepthVideoStream() const throw ()
  {
    if (depth_video_stream_.get() == 0)
    {
      if (hasDepthSensor())
      {
        depth_video_stream_ = boost::make_shared<openni::VideoStream>();

        const openni::Status rc = depth_video_stream_->create(*openni_device_, openni::SENSOR_DEPTH);
        if (rc != openni::STATUS_OK)
          THROW_OPENNI_EXCEPTION("Couldn't create depth video stream: \n%s\n", openni::OpenNI::getExtendedError());
      }
    }
    return depth_video_stream_;
  }

  std::ostream& operator <<(std::ostream& stream, const OpenNI2Device& device)
  {

    stream << "Device info (" << device.getUri() << ")" << std::endl;
    stream << "   Vendor: " << device.getVendor() << std::endl;
    stream << "   Name: " << device.getName() << std::endl;
    stream << "   USB Vendor ID: " << device.getUsbVendorId() << std::endl;
    stream << "   USB Product ID: " << device.getUsbVendorId() << std::endl << std::endl;

    if (device.hasIRSensor())
    {
      stream << "IR sensor video modes:" << std::endl;
      const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedIRVideoModes();

      std::vector<OpenNI2VideoMode>::const_iterator it = video_modes.begin();
      std::vector<OpenNI2VideoMode>::const_iterator it_end = video_modes.end();
      for (; it != it_end; ++it)
        stream << "   - " << *it << std::endl;
    }
    else
    {
      stream << "No IR sensor available" << std::endl;
    }

    if (device.hasColorSensor())
    {
      stream << "Color sensor video modes:" << std::endl;
      const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedColorVideoModes();

      std::vector<OpenNI2VideoMode>::const_iterator it = video_modes.begin();
      std::vector<OpenNI2VideoMode>::const_iterator it_end = video_modes.end();
      for (; it != it_end; ++it)
        stream << "   - " << *it << std::endl;
    }
    else
    {
      stream << "No Color sensor available" << std::endl;
    }

    if (device.hasDepthSensor())
    {
      stream << "Depth sensor video modes:" << std::endl;
      const std::vector<OpenNI2VideoMode>& video_modes = device.getSupportedDepthVideoModes();

      std::vector<OpenNI2VideoMode>::const_iterator it = video_modes.begin();
      std::vector<OpenNI2VideoMode>::const_iterator it_end = video_modes.end();
      for (; it != it_end; ++it)
        stream << "   - " << *it << std::endl;
    }
    else
    {
      stream << "No Depth sensor available" << std::endl;
    }

    return stream;

  }


  // Convert VideoFrameRef into pcl::Image and forward to registered callbacks
  void OpenNI2Device::processColorFrame(openni::VideoStream& stream)
  {
    hr_clock::time_point t_callback = hr_clock::now();

    openni::VideoFrameRef frame;
    stream.readFrame(&frame);

    openni::PixelFormat format = frame.getVideoMode().getPixelFormat();
    boost::shared_ptr<openni_wrapper::Image> image;

    // Convert frame to PCL image type, based on pixel format
    if (format == openni::PIXEL_FORMAT_YUV422)
      image = boost::make_shared<openni_wrapper::ImageYUV422> (frame, t_callback);
    else //if (format == PixelFormat::PIXEL_FORMAT_RGB888)
      image = boost::make_shared<openni_wrapper::ImageRGB24> (frame, t_callback);

    // Notify listeners of new frame
    for (std::map< OpenNIDevice::CallbackHandle, ActualImageCallbackFunction >::iterator callbackIt = image_callback_.begin (); callbackIt != image_callback_.end (); ++callbackIt)
    {
      callbackIt->second.operator()(image);
    }
  }


  void OpenNI2Device::processDepthFrame(openni::VideoStream& stream)
  {
    openni::VideoFrameRef frame;
    stream.readFrame(&frame);

    boost::posix_time::ptime t_readFrameTimestamp = boost::posix_time::microsec_clock::local_time();
    int frameWidth = frame.getWidth();
    float focalLength = getDepthFocalLength (frameWidth);

    // Need: data, baseline_, getDepthFocalLength (), shadow_value_, no_sample_value_
    boost::shared_ptr<DepthImage> image = 
      boost::make_shared<DepthImage> (frame, baseline_, focalLength, shadow_value_, no_sample_value_);

    // Notify listeners of new frame
    for (std::map< OpenNIDevice::CallbackHandle, ActualDepthImageCallbackFunction >::iterator callbackIt = depth_callback_.begin (); callbackIt != depth_callback_.end (); ++callbackIt)
    {
      callbackIt->second.operator()(image);
    }
  }


  void OpenNI2Device::processIRFrame(openni::VideoStream& stream)
  {
    openni::VideoFrameRef frame;
    stream.readFrame(&frame);
    boost::posix_time::ptime t_readFrameTimestamp = boost::posix_time::microsec_clock::local_time();

    boost::shared_ptr<IRImage> image = boost::make_shared<openni_wrapper::IRImage> (frame);

    // Notify listeners of new frame
    for (std::map< OpenNIDevice::CallbackHandle, ActualIRImageCallbackFunction >::iterator callbackIt = ir_callback_.begin (); callbackIt != ir_callback_.end (); ++callbackIt)
    {
      callbackIt->second.operator()(image);
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  OpenNI2Device::CallbackHandle 
    OpenNI2Device::registerImageCallback (const ImageCallbackFunction& callback, void* custom_data) throw ()
  {
    image_callback_[image_callback_handle_counter_] = boost::bind (callback, _1, custom_data);
    return (image_callback_handle_counter_++);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool 
    OpenNI2Device::unregisterImageCallback (const OpenNI2Device::CallbackHandle& callbackHandle) throw ()
  {
    return (image_callback_.erase (callbackHandle) != 0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  OpenNI2Device::CallbackHandle 
    OpenNI2Device::registerDepthCallback (const DepthImageCallbackFunction& callback, void* custom_data) throw ()
  {
    depth_callback_[depth_callback_handle_counter_] = boost::bind (callback, _1, custom_data);
    return (depth_callback_handle_counter_++);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool 
    openni2_wrapper::OpenNI2Device::unregisterDepthCallback (const OpenNI2Device::CallbackHandle& callbackHandle) throw ()
  {
    return (depth_callback_.erase (callbackHandle) != 0);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  OpenNI2Device::CallbackHandle 
    OpenNI2Device::registerIRCallback (const IRImageCallbackFunction& callback, void* custom_data) throw ()
  {
    ir_callback_[ir_callback_handle_counter_] = boost::bind (callback, _1, custom_data);
    return (ir_callback_handle_counter_++);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool 
    OpenNI2Device::unregisterIRCallback (const OpenNI2Device::CallbackHandle& callbackHandle) throw ()
  {
    return (ir_callback_.erase (callbackHandle) != 0);
  }



}
