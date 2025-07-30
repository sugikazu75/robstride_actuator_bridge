//
// Created by Mu Shibo on 04/2024
//

#include "motor_control/motor_config.h"

MotorControlSet::MotorControlSet(ros::NodeHandle* node_handle, const std::string& can_rx, const std::string& can_tx,
                                 const std::vector<uint8_t>& motor_ids, const std::vector<uint8_t> motor_types)
{
  can_receive = node_handle->subscribe(can_rx, 100, &MotorControlSet::can1_rx_Callback, this);
  can_send = node_handle->advertise<can_msgs::Frame>(can_tx, 100);
  command_bridge = node_handle->subscribe("robstride_cmd", 100, &MotorControlSet::command_Callback, this);
  robstride_state_pub = node_handle->advertise<motor_control::MotorFeedback>("/robstride_state", 100);

  motor_ids_.clear();
  motors_.clear();
  for (int i = 0; i < motor_ids.size(); i++)
  {
    motor_ids_.push_back(motor_ids.at(i));
    motors_.push_back(RobStrite_Motor(static_cast<int>(motor_ids.at(i)), static_cast<MotorType>(motor_types.at(i)),
                                      node_handle, "robstride_cmd"));
  }

  joint_feedback_.pos.resize(motor_ids_.size());
  joint_feedback_.vel.resize(motor_ids_.size());
  joint_feedback_.tor.resize(motor_ids_.size());
  joint_feedback_.temp.resize(motor_ids_.size());
  joint_feedback_.error_code.resize(motor_ids_.size());
  joint_feedback_.pattern.resize(motor_ids_.size());
}

void MotorControlSet::shutdownCallback()
{
  for (int i = 0; i < motors_.size(); i++)
    motors_.at(i).Disenable_Motor(0);
}

MotorControlSet::~MotorControlSet()
{
  shutdownCallback();
}

void MotorControlSet::can1_rx_Callback(can_msgs::Frame msg)
{
  uint8_t id = (msg.id & 0xFF00) >> 8;
  for (int i = 0; i < motor_ids_.size(); i++)
  {
    if (motor_ids_.at(i) == id)
    {
      motors_.at(i).RobStrite_Motor_Analysis(msg.data.data(), msg.id);
      joint_feedback_.pos[i] = motors_.at(i).Pos_Info.Angle;
      joint_feedback_.vel[i] = motors_.at(i).Pos_Info.Speed;
      joint_feedback_.tor[i] = motors_.at(i).Pos_Info.Torque;
      joint_feedback_.temp[i] = motors_.at(i).Pos_Info.Temp;
      joint_feedback_.error_code[i] = motors_.at(i).error_code;
      joint_feedback_.pattern[i] = motors_.at(i).Pos_Info.pattern;
      break;
    }
  }
  robstride_state_pub.publish(joint_feedback_);
}

void MotorControlSet::command_Callback(can_msgs::Frame msg)
{
  can_send.publish(msg);
}
