#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoData_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperSlipServoData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "pr2_gripper_sensor_msgs/PR2GripperSensorRTState.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperSlipServoData : public ros::Msg
  {
    public:
      ros::Time stamp;
      float deformation;
      float left_fingertip_pad_force;
      float right_fingertip_pad_force;
      float joint_effort;
      bool slip_detected;
      bool deformation_limit_reached;
      bool fingertip_force_limit_reached;
      bool gripper_empty;
      pr2_gripper_sensor_msgs::PR2GripperSensorRTState rtstate;

    PR2GripperSlipServoData():
      stamp(),
      deformation(0),
      left_fingertip_pad_force(0),
      right_fingertip_pad_force(0),
      joint_effort(0),
      slip_detected(0),
      deformation_limit_reached(0),
      fingertip_force_limit_reached(0),
      gripper_empty(0),
      rtstate()
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
      offset += serializeAvrFloat64(outbuffer + offset, this->deformation);
      offset += serializeAvrFloat64(outbuffer + offset, this->left_fingertip_pad_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->right_fingertip_pad_force);
      offset += serializeAvrFloat64(outbuffer + offset, this->joint_effort);
      union {
        bool real;
        uint8_t base;
      } u_slip_detected;
      u_slip_detected.real = this->slip_detected;
      *(outbuffer + offset + 0) = (u_slip_detected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->slip_detected);
      union {
        bool real;
        uint8_t base;
      } u_deformation_limit_reached;
      u_deformation_limit_reached.real = this->deformation_limit_reached;
      *(outbuffer + offset + 0) = (u_deformation_limit_reached.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->deformation_limit_reached);
      union {
        bool real;
        uint8_t base;
      } u_fingertip_force_limit_reached;
      u_fingertip_force_limit_reached.real = this->fingertip_force_limit_reached;
      *(outbuffer + offset + 0) = (u_fingertip_force_limit_reached.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fingertip_force_limit_reached);
      union {
        bool real;
        uint8_t base;
      } u_gripper_empty;
      u_gripper_empty.real = this->gripper_empty;
      *(outbuffer + offset + 0) = (u_gripper_empty.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_empty);
      offset += this->rtstate.serialize(outbuffer + offset);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->deformation));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->left_fingertip_pad_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->right_fingertip_pad_force));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->joint_effort));
      union {
        bool real;
        uint8_t base;
      } u_slip_detected;
      u_slip_detected.base = 0;
      u_slip_detected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->slip_detected = u_slip_detected.real;
      offset += sizeof(this->slip_detected);
      union {
        bool real;
        uint8_t base;
      } u_deformation_limit_reached;
      u_deformation_limit_reached.base = 0;
      u_deformation_limit_reached.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->deformation_limit_reached = u_deformation_limit_reached.real;
      offset += sizeof(this->deformation_limit_reached);
      union {
        bool real;
        uint8_t base;
      } u_fingertip_force_limit_reached;
      u_fingertip_force_limit_reached.base = 0;
      u_fingertip_force_limit_reached.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fingertip_force_limit_reached = u_fingertip_force_limit_reached.real;
      offset += sizeof(this->fingertip_force_limit_reached);
      union {
        bool real;
        uint8_t base;
      } u_gripper_empty;
      u_gripper_empty.base = 0;
      u_gripper_empty.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_empty = u_gripper_empty.real;
      offset += sizeof(this->gripper_empty);
      offset += this->rtstate.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "pr2_gripper_sensor_msgs/PR2GripperSlipServoData"; };
    const char * getMD5(){ return "a49728a2e0c40706b3c9b74046f006aa"; };

  };

}
#endif