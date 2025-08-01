#include "motor_control/motor_config.h"

MotorControlSet::MotorControlSet(ros::NodeHandle* node_handle, const std::string& can_rx, const std::string& can_tx,
                                 const std::vector<uint8_t>& motor_ids, const std::vector<uint8_t> motor_types,
                                 const std::vector<std::string> motor_names, const std::vector<double> reductions)
{
  robstride_state_pub = node_handle->advertise<motor_control::MotorFeedback>("robstride_state", 100);
  joint_state_pub_ = node_handle->advertise<sensor_msgs::JointState>("joint_states", 100);

  can_receive = node_handle->subscribe(can_rx, 100, &MotorControlSet::can1_rx_Callback, this);
  command_sub_ = node_handle->subscribe("robstride_command", 100, &MotorControlSet::commandCallback, this);

  motor_ids_.clear();
  motors_.clear();
  commands_.clear();
  reductions_.clear();
  joint_state_.name.clear();
  joint_state_.position.clear();
  joint_state_.velocity.clear();
  joint_state_.effort.clear();
  for (int i = 0; i < motor_ids.size(); i++)
  {
    motor_ids_.push_back(motor_ids.at(i));
    motors_.push_back(RobStrite_Motor(static_cast<int>(motor_ids.at(i)), static_cast<MotorType>(motor_types.at(i)),
                                      node_handle, can_tx));
    commands_.push_back(motor_control::MotorCommand());
    reductions_.push_back(reductions.at(i));

    joint_state_.name.push_back(motor_names.at(i));
    joint_state_.position.push_back(0);
    joint_state_.velocity.push_back(0);
    joint_state_.effort.push_back(0);
  }

  robstride_state_.pos.resize(motor_ids_.size());
  robstride_state_.vel.resize(motor_ids_.size());
  robstride_state_.tor.resize(motor_ids_.size());
  robstride_state_.temp.resize(motor_ids_.size());
  robstride_state_.error_code.resize(motor_ids_.size());
  robstride_state_.pattern.resize(motor_ids_.size());
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
      robstride_state_.pos[i] = motors_.at(i).Pos_Info.Angle;
      robstride_state_.vel[i] = motors_.at(i).Pos_Info.Speed;
      robstride_state_.tor[i] = motors_.at(i).Pos_Info.Torque;
      robstride_state_.temp[i] = motors_.at(i).Pos_Info.Temp;
      robstride_state_.error_code[i] = motors_.at(i).error_code;
      robstride_state_.pattern[i] = motors_.at(i).Pos_Info.pattern;

      joint_state_.position[i] = robstride_state_.pos[i] / reductions_.at(i);
      joint_state_.velocity[i] = robstride_state_.vel[i] / reductions_.at(i);
      joint_state_.effort[i] = robstride_state_.tor[i] * reductions_.at(i);

      break;
    }
  }
  robstride_state_pub.publish(robstride_state_);
}

void MotorControlSet::update(uint8_t index)
{
  getMotor(index).RobStrite_Motor_move_control(commands_.at(index).torque, commands_.at(index).angle,
                                               commands_.at(index).velocity, commands_.at(index).kp,
                                               commands_.at(index).kd);

  if (ros::Time::now().toSec() - joint_state_last_pub_time_ > joint_state_pub_interval_)
  {
    joint_state_pub_.publish(joint_state_);
    joint_state_last_pub_time_ = ros::Time::now().toSec();
  }
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
