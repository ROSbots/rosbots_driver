#ifndef _ROS_alfred_driver_MotionBundle_h
#define _ROS_alfred_driver_MotionBundle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "sensor_msgs/Image.h"

namespace alfred_driver
{

  class MotionBundle : public ros::Msg
  {
    public:
      uint32_t motion_images_length;
      typedef sensor_msgs::Image _motion_images_type;
      _motion_images_type st_motion_images;
      _motion_images_type * motion_images;

    MotionBundle():
      motion_images_length(0), motion_images(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->motion_images_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motion_images_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motion_images_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motion_images_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motion_images_length);
      for( uint32_t i = 0; i < motion_images_length; i++){
      offset += this->motion_images[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t motion_images_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motion_images_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motion_images_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motion_images_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motion_images_length);
      if(motion_images_lengthT > motion_images_length)
        this->motion_images = (sensor_msgs::Image*)realloc(this->motion_images, motion_images_lengthT * sizeof(sensor_msgs::Image));
      motion_images_length = motion_images_lengthT;
      for( uint32_t i = 0; i < motion_images_length; i++){
      offset += this->st_motion_images.deserialize(inbuffer + offset);
        memcpy( &(this->motion_images[i]), &(this->st_motion_images), sizeof(sensor_msgs::Image));
      }
     return offset;
    }

    const char * getType(){ return PSTR( "alfred_driver/MotionBundle" ); };
    const char * getMD5(){ return PSTR( "997503979c7d63881d262f2d6c183129" ); };

  };

}
#endif