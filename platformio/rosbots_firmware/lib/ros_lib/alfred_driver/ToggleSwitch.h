#ifndef _ROS_SERVICE_ToggleSwitch_h
#define _ROS_SERVICE_ToggleSwitch_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h"
#include "ros/time.h"

namespace alfred_driver
{

static const char TOGGLESWITCH[] PROGMEM = "alfred_driver/ToggleSwitch";

  class ToggleSwitchRequest : public ros::Msg
  {
    public:
      typedef uint32_t _remote_id_type;
      _remote_id_type remote_id;
      typedef uint32_t _button_id_type;
      _button_id_type button_id;
      typedef uint8_t _turn_on_type;
      _turn_on_type turn_on;

    ToggleSwitchRequest():
      remote_id(0),
      button_id(0),
      turn_on(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->remote_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->remote_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->remote_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->remote_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->remote_id);
      *(outbuffer + offset + 0) = (this->button_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->button_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->button_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->button_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->button_id);
      *(outbuffer + offset + 0) = (this->turn_on >> (8 * 0)) & 0xFF;
      offset += sizeof(this->turn_on);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->remote_id =  ((uint32_t) (*(inbuffer + offset)));
      this->remote_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->remote_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->remote_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->remote_id);
      this->button_id =  ((uint32_t) (*(inbuffer + offset)));
      this->button_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->button_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->button_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->button_id);
      this->turn_on =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->turn_on);
     return offset;
    }

    const char * getType(){ return TOGGLESWITCH; };
    const char * getMD5(){ return PSTR( "2274f1d370e76210765d4fb9aaae19e0" ); };

  };

  class ToggleSwitchResponse : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;

    ToggleSwitchResponse():
      stamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
     return offset;
    }

    const char * getType(){ return TOGGLESWITCH; };
    const char * getMD5(){ return PSTR( "84d365d08d5fc49dde870daba1c7992c" ); };

  };

  class ToggleSwitch {
    public:
    typedef ToggleSwitchRequest Request;
    typedef ToggleSwitchResponse Response;
  };

}
#endif
