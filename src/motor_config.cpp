//
// Created by Mu Shibo on 04/2024
//

#include "motor_control/motor_config.h"

MotorControlSet::MotorControlSet(ros::NodeHandle* node_handle, const std::string& can_rx, const std::string& can_tx,
                                 const std::vector<uint8_t>& motor_ids, const std::vector<uint8_t> motor_types)
{
  robstride_state_pub = node_handle->advertise<motor_control::MotorFeedback>("robstride_state", 100);

  can_receive = node_handle->subscribe(can_rx, 100, &MotorControlSet::can1_rx_Callback, this);
  command_sub_ = node_handle->subscribe("robstride_command", 100, &MotorControlSet::commandCallback, this);

  motor_ids_.clear();
  motors_.clear();
  commands_.clear();
  for (int i = 0; i < motor_ids.size(); i++)
  {
    motor_ids_.push_back(motor_ids.at(i));
    motors_.push_back(RobStrite_Motor(static_cast<int>(motor_ids.at(i)), static_cast<MotorType>(motor_types.at(i)),
                                      node_handle, can_tx));
    commands_.push_back(motor_control::MotorCommand());
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

void MotorControlSet::update(uint8_t index)
{
  getMotor(index).RobStrite_Motor_move_control(commands_.at(index).torque, commands_.at(index).angle,
                                               commands_.at(index).velocity, commands_.at(index).kp,
                                               commands_.at(index).kd);
}

void MotorControlSet::commandCallback(motor_control::MotorCommand msg)
{
  uint8_t index = msg.index;
  commands_.at(index).torque = msg.torque;
  commands_.at(index).angle = msg.angle;
  commands_.at(index).angle = msg.angle;
  commands_.at(index).velocity = msg.velocity;
  commands_.at(index).kp = msg.kp;
  commands_.at(index).kd = msg.kd;
}
