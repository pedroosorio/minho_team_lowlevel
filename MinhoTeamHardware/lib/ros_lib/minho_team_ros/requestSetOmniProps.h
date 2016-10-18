#ifndef _ROS_SERVICE_requestSetOmniProps_h
#define _ROS_SERVICE_requestSetOmniProps_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "minho_team_ros/omniConfig.h"

namespace minho_team_ros
{

static const char REQUESTSETOMNIPROPS[] = "minho_team_ros/requestSetOmniProps";

  class requestSetOmniPropsRequest : public ros::Msg
  {
    public:
      minho_team_ros::omniConfig omniConf;

    requestSetOmniPropsRequest():
      omniConf()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->omniConf.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->omniConf.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return REQUESTSETOMNIPROPS; };
    const char * getMD5(){ return "294d2a8d409cb2af8b414e22d13dfaf6"; };

  };

  class requestSetOmniPropsResponse : public ros::Msg
  {
    public:
      bool success;

    requestSetOmniPropsResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return REQUESTSETOMNIPROPS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class requestSetOmniProps {
    public:
    typedef requestSetOmniPropsRequest Request;
    typedef requestSetOmniPropsResponse Response;
  };

}
#endif
