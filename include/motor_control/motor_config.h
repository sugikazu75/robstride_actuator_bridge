#ifndef _MOTOR_CONFIG_H_
#define _MOTOR_CONFIG_H_

#include <vector>
#include <memory>
#include "ros/ros.h"
#include "stdint.h"
#include "can_msgs/Frame.h"
#include "motor_control/MotorCommand.h"
#include "motor_control/MotorFeedback.h"
#include "motor_control/MotorTorqueCommand.h"
#include "motor_control/robstride.h"
#include <sensor_msgs/JointState.h>

class MotorControlSet
{
public:
  MotorControlSet(ros::NodeHandle* node_handle, const std::string& can_rx, const std::string& can_tx,
                  const std::vector<uint8_t>& motor_ids, const std::vector<uint8_t> motor_types,
                  const std::vector<std::string> motor_names, const std::vector<double> reductions);
  ~MotorControlSet();

  void shutdownCallback();

  // create subscriber
  ros::Subscriber can_receive;
  ros::Subscriber command_sub_;
  ros::Subscriber torque_enable_sub_;

  // creat publisher
  ros::Publisher robstride_state_pub;
  ros::Publisher joint_state_pub_;

  motor_control::MotorFeedback robstride_state_;
  sensor_msgs::JointState joint_state_;

  void update(uint8_t index);
  RobStrite_Motor& getMotor(uint8_t index)
  {
    return motors_.at(index);
  }

private:
  int motor_num_;
  double joint_state_pub_interval_ = 0.01;
  double joint_state_last_pub_time_ = 0;
  std::vector<uint8_t> motor_ids_;
  std::vector<RobStrite_Motor> motors_;
  std::vector<motor_control::MotorCommand> commands_;
  std::vector<double> reductions_;
  std::vector<int> torque_enable_;

  void can1_rx_Callback(can_msgs::Frame msg);
  void commandCallback(motor_control::MotorCommand msg);
  void torqueEnableCallback(motor_control::MotorTorqueCommand msg);
};

#endif
