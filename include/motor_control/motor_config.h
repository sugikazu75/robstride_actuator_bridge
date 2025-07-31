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
#include "motor_control/MotorCommand.h"
#include "motor_control/MotorFeedback.h"
#include "motor_control/robstride.h"

class MotorControlSet
{
public:
  MotorControlSet(ros::NodeHandle* node_handle, const std::string& can_rx, const std::string& can_tx,
                  const std::vector<uint8_t>& motor_ids, const std::vector<uint8_t> motor_types);
  ~MotorControlSet();

  void shutdownCallback();

  // create subscriber
  ros::Subscriber can_receive;
  ros::Subscriber command_sub_;

  // creat publisher
  ros::Publisher robstride_state_pub;

  void can1_rx_Callback(can_msgs::Frame msg);
  void commandCallback(motor_control::MotorCommand msg);

  motor_control::MotorFeedback joint_feedback_;

  void update(uint8_t index);
  RobStrite_Motor& getMotor(uint8_t index)
  {
    return motors_.at(index);
  }

private:
  std::vector<uint8_t> motor_ids_;
  std::vector<RobStrite_Motor> motors_;
  std::vector<motor_control::MotorCommand> commands_;
};

#endif
