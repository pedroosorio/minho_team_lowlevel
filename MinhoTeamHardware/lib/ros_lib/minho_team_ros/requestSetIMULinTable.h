#ifndef _ROS_SERVICE_requestSetIMULinTable_h
#define _ROS_SERVICE_requestSetIMULinTable_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "minho_team_ros/imuConfig.h"

namespace minho_team_ros
{

static const char REQUESTSETIMULINTABLE[] = "minho_team_ros/requestSetIMULinTable";

  class requestSetIMULinTableRequest : public ros::Msg
  {
    public:
      minho_team_ros::imuConfig imuConf;

    requestSetIMULinTableRequest():
      imuConf()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->imuConf.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->imuConf.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return REQUESTSETIMULINTABLE; };
    const char * getMD5(){ return "9e6482de230fd27484d1771670a9c37a"; };

  };

  class requestSetIMULinTableResponse : public ros::Msg
  {
    public:
      bool success;

    requestSetIMULinTableResponse():
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

    const char * getType(){ return REQUESTSETIMULINTABLE; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class requestSetIMULinTable {
    public:
    typedef requestSetIMULinTableRequest Request;
    typedef requestSetIMULinTableResponse Response;
  };

}
#endif
