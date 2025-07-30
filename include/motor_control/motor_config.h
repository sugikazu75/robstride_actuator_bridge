//
// Created by Mu Shibo on 04/2024
//

#ifndef _MOTOR_CONFIG_H_
#define _MOTOR_CONFIG_H_

#include <vector>
#include <memory>
#include "ros/ros.h"
#include "stdint.h"
#include "can_msgs/Frame.h"
#include "motor_control/MotorFeedback.h"
#include "motor_control/robstride.h"

typedef struct
{
  uint16_t state;
  float pos;
  float vel;
  float tor;
  float Kp;
  float Kd;
  float Tmos;
  float Tcoil;
} motor_state_t;

typedef struct
{
  float pos_d;
  float vel_d;
  float tor_d;
  float Kp;
  float Kd;
} motor_cmd_t;

class MotorControlSet
{
public:
  MotorControlSet(ros::NodeHandle* node_handle, const std::string& can_rx, const std::string& can_tx,
                  const std::vector<uint8_t>& motor_ids, const std::vector<uint8_t> motor_types);
  ~MotorControlSet();

  void shutdownCallback();

  // create subscriber
  ros::Subscriber can_receive;
  ros::Subscriber command_bridge;
  // creat publisher
  ros::Publisher can_send;
  ros::Publisher robstride_state_pub;

  void can1_rx_Callback(can_msgs::Frame msg);
  void command_Callback(can_msgs::Frame msg);

  std::vector<motor_state_t> motor_state_;
  std::vector<motor_cmd_t> motor_cmd_;

  motor_control::MotorFeedback joint_feedback_;

  RobStrite_Motor& getMotor(uint8_t ID)
  {
    return motors_.at(ID);
  }

private:
  std::vector<uint8_t> motor_ids_;
  std::vector<RobStrite_Motor> motors_;
};

#endif
