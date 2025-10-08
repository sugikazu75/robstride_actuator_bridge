#ifndef _MOTOR_CONFIG_H_
#define _MOTOR_CONFIG_H_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <robstride_actuator_bridge/MotorCommand.h>
#include <robstride_actuator_bridge/MotorFeedback.h>
#include <robstride_actuator_bridge/MotorTorqueCommand.h>
#include <robstride_actuator_bridge/robstride.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8MultiArray.h>

#include <memory>
#include <vector>
#include <stdint.h>

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
  ros::Subscriber joint_command_sub_;
  ros::Subscriber motor_command_sub_;
  ros::Subscriber torque_enable_sub_;
  ros::Subscriber servo_on_sub_;
  ros::Subscriber servo_off_sub_;
  ros::Subscriber motor_calib_sub_;

  // create publisher
  ros::Publisher robstride_state_pub;
  ros::Publisher joint_state_pub_;
  ros::Publisher torque_state_pub_;

  robstride_actuator_bridge::MotorFeedback robstride_state_;
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
  double torque_state_pub_interval_ = 1.0;
  double torque_state_last_pub_time_ = 0;
  std::vector<uint8_t> motor_ids_;
  std::vector<RobStrite_Motor> motors_;
  std::vector<robstride_actuator_bridge::MotorCommand> commands_;
  std::vector<double> reductions_;
  std::vector<int> torque_enable_;

  void servoOn(int index);
  void servoOff(int index);
  void servoCalib(int index);

  void can1_rx_Callback(can_msgs::Frame msg);
  void motorCommandCallback(robstride_actuator_bridge::MotorCommand msg);
  void jointCommandCallback(robstride_actuator_bridge::MotorCommand msg);
  void torqueEnableCallback(robstride_actuator_bridge::MotorTorqueCommand msg);
  void servoOnCallback(std_msgs::Empty msg);
  void servoOffCallback(std_msgs::Empty msg);
  void motorCalibCallback(std_msgs::UInt8MultiArray msg);
};

#endif
