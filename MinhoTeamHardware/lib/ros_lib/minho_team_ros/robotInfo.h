#ifndef _ROS_minho_team_ros_robotInfo_h
#define _ROS_minho_team_ros_robotInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "minho_team_ros/pose.h"
#include "minho_team_ros/position.h"

namespace minho_team_ros
{

  class robotInfo : public ros::Msg
  {
    public:
      minho_team_ros::pose robot_pose;
      minho_team_ros::position ball_position;
      int16_t imu_value;
      uint8_t has_ball;
      bool sees_ball;
      uint32_t obstacles_length;
      minho_team_ros::position st_obstacles;
      minho_team_ros::position * obstacles;

    robotInfo():
      robot_pose(),
      ball_position(),
      imu_value(0),
      has_ball(0),
      sees_ball(0),
      obstacles_length(0), obstacles(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->robot_pose.serialize(outbuffer + offset);
      offset += this->ball_position.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_imu_value;
      u_imu_value.real = this->imu_value;
      *(outbuffer + offset + 0) = (u_imu_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_imu_value.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->imu_value);
      *(outbuffer + offset + 0) = (this->has_ball >> (8 * 0)) & 0xFF;
      offset += sizeof(this->has_ball);
      union {
        bool real;
        uint8_t base;
      } u_sees_ball;
      u_sees_ball.real = this->sees_ball;
      *(outbuffer + offset + 0) = (u_sees_ball.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sees_ball);
      *(outbuffer + offset + 0) = (this->obstacles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->obstacles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->obstacles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->obstacles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obstacles_length);
      for( uint32_t i = 0; i < obstacles_length; i++){
      offset += this->obstacles[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->robot_pose.deserialize(inbuffer + offset);
      offset += this->ball_position.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_imu_value;
      u_imu_value.base = 0;
      u_imu_value.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_imu_value.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->imu_value = u_imu_value.real;
      offset += sizeof(this->imu_value);
      this->has_ball =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->has_ball);
      union {
        bool real;
        uint8_t base;
      } u_sees_ball;
      u_sees_ball.base = 0;
      u_sees_ball.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sees_ball = u_sees_ball.real;
      offset += sizeof(this->sees_ball);
      uint32_t obstacles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->obstacles_length);
      if(obstacles_lengthT > obstacles_length)
        this->obstacles = (minho_team_ros::position*)realloc(this->obstacles, obstacles_lengthT * sizeof(minho_team_ros::position));
      obstacles_length = obstacles_lengthT;
      for( uint32_t i = 0; i < obstacles_length; i++){
      offset += this->st_obstacles.deserialize(inbuffer + offset);
        memcpy( &(this->obstacles[i]), &(this->st_obstacles), sizeof(minho_team_ros::position));
      }
     return offset;
    }

    const char * getType(){ return "minho_team_ros/robotInfo"; };
    const char * getMD5(){ return "196ef574d5d16f545df3ccb40fbe1ec4"; };

  };

}
#endif