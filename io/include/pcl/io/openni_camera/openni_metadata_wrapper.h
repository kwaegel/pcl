#pragma once
#ifndef __OPENNI_METADATA_WRAPPER__
#define __OPENNI_METADATA_WRAPPER__

#include <pcl/pcl_config.h>

#if defined(HAVE_OPENNI)

#include <pcl/io/image_metadata_wrapper.h>
#include <pcl/io/openni_camera/openni.h>


namespace openni_wrapper
{
  class OpenniFrameWrapper : public pcl::io::FrameWrapper
  {
    public:
      OpenniFrameWrapper(boost::shared_ptr<xn::ImageMetaData> metadata)
        : metadata_(metadata)
      {}
      
      virtual inline const void* getData() const   { return metadata_->Data();       }
      virtual inline unsigned getDataSize() const  { return metadata_->DataSize ();  }
      virtual inline unsigned getWidth() const     { return metadata_->XRes ();      }
      virtual inline unsigned getHeight() const    { return metadata_->YRes ();      }
      virtual inline unsigned getFrameID() const   { return metadata_->FrameID();    }
      virtual inline uint64_t getTimestamp() const { return metadata_->Timestamp (); }
      
      const inline boost::shared_ptr<xn::ImageMetaData> getMetaData() const { return metadata_; }

    private:
      boost::shared_ptr<xn::ImageMetaData> metadata_;
  };

} // namespace
#endif // HAVE_OPENNI
  
#endif // __OPENNI_METADATA_WRAPPER__