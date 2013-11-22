/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <pcl/io/openni2_camera/openni_image_ir.h>
#include <sstream>
#include <limits>
#include <iostream>

using namespace std;

using openni_wrapper::IRImage;

  IRImage::IRImage (openni::VideoFrameRef ir_meta_data) throw ()
  : ir_md_ (ir_meta_data)
  {
    timestamp_ = boost::chrono::high_resolution_clock::now();
  }

  IRImage::IRImage (openni::VideoFrameRef ir_meta_data, IRImage::Timestamp timestamp) throw ()
  : ir_md_ (ir_meta_data),
  timestamp_(timestamp)
  { }

  IRImage::~IRImage () throw ()
  {
  }

  unsigned IRImage::getWidth () const throw ()
  {
	  return ir_md_.getWidth();
  }

  unsigned IRImage::getHeight () const throw ()
  {
	  return ir_md_.getHeight();
  }

  unsigned IRImage::getFrameID () const throw ()
  {
	  return ir_md_.getFrameIndex();
  }

  unsigned long IRImage::getTimeStamp () const throw ()
  {
	  return static_cast<unsigned long> (ir_md_.getTimestamp ());
  }

  IRImage::Timestamp IRImage::getSystemTimestamp () const throw ()
  {
	  return timestamp_;
  }

  const openni::VideoFrameRef& IRImage::getMetaData () const throw ()
  {
	  return ir_md_;
  }

  void IRImage::fillRaw (unsigned width, unsigned height, unsigned short* ir_buffer, unsigned line_step) const
  {
    if (width > ir_md_.getWidth () || height > ir_md_.getHeight ())
      THROW_OPENNI_EXCEPTION ("upsampling not supported: %d x %d -> %d x %d", ir_md_.getWidth (), ir_md_.getHeight (), width, height);

    if (ir_md_.getWidth () % width != 0 || ir_md_.getHeight () % height != 0)
      THROW_OPENNI_EXCEPTION ("downsampling only supported for integer scale: %d x %d -> %d x %d", ir_md_.getWidth (), ir_md_.getHeight (), width, height);

    if (line_step == 0)
      line_step = width * static_cast<unsigned> (sizeof (unsigned short));

    // special case no sclaing, no padding => memcopy!
    if (width == ir_md_.getWidth () && height == ir_md_.getHeight () && (line_step == width * sizeof (unsigned short)))
    {
      memcpy (ir_buffer, ir_md_.getData(), ir_md_.getDataSize());
      return;
    }

    // padding skip for destination image
    unsigned bufferSkip = line_step - width * static_cast<unsigned> (sizeof (unsigned short));

    // step and padding skip for source image
    unsigned xStep = ir_md_.getWidth () / width;
    unsigned ySkip = (ir_md_.getHeight () / height - 1) * ir_md_.getWidth ();

    unsigned irIdx = 0;

    for (unsigned yIdx = 0; yIdx < height; ++yIdx, irIdx += ySkip)
    {
      for (unsigned xIdx = 0; xIdx < width; ++xIdx, irIdx += xStep, ++ir_buffer)
        *ir_buffer = static_cast<unsigned short> (  ((unsigned short*)ir_md_.getData())[irIdx]  );

      // if we have padding
      if (bufferSkip > 0)
      {
        char* cBuffer = reinterpret_cast<char*> (ir_buffer);
        ir_buffer = reinterpret_cast<unsigned short*> (cBuffer + bufferSkip);
      }
    }
  }

#endif //HAVE_OPENNI

