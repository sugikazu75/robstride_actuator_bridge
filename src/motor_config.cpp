#include <robstride_actuator_bridge/motor_config.h>

MotorControlSet::MotorControlSet(ros::NodeHandle* node_handle, const std::string& can_rx, const std::string& can_tx,
                                 const std::vector<uint8_t>& motor_ids, const std::vector<uint8_t> motor_types,
                                 const std::vector<std::string> motor_names, const std::vector<double> reductions)
{
  robstride_state_pub = node_handle->advertise<robstride_actuator_bridge::MotorFeedback>("robstride_state", 1);
  joint_state_pub_ = node_handle->advertise<sensor_msgs::JointState>("joint_states", 1);
  torque_state_pub_ =
      node_handle->advertise<robstride_actuator_bridge::MotorTorqueCommand>("robstride_torque_state", 1);

  can_receive = node_handle->subscribe(can_rx, 100, &MotorControlSet::can1_rx_Callback, this);
  motor_command_sub_ =
      node_handle->subscribe("robstride_motor_command", 100, &MotorControlSet::motorCommandCallback, this);
  joint_command_sub_ =
      node_handle->subscribe("robstride_joint_command", 100, &MotorControlSet::jointCommandCallback, this);
  torque_enable_sub_ =
      node_handle->subscribe("robstride_torque_enable", 100, &MotorControlSet::torqueEnableCallback, this);
  motor_calib_sub_ = node_handle->subscribe("robstride_calib", 100, &MotorControlSet::motorCalibCallback, this);

  motor_num_ = motor_ids.size();
  for (int i = 0; i < motor_num_; i++)
  {
    motor_ids_.push_back(motor_ids.at(i));
    motors_.push_back(RobStrite_Motor(static_cast<int>(motor_ids.at(i)), static_cast<MotorType>(motor_types.at(i)),
                                      node_handle, can_tx));
    commands_.push_back(robstride_actuator_bridge::MotorCommand());
    reductions_.push_back(reductions.at(i));
    torque_enable_.push_back(1);

    joint_state_.name.push_back(motor_names.at(i));
    joint_state_.position.push_back(0);
    joint_state_.velocity.push_back(0);
    joint_state_.effort.push_back(0);
  }

  robstride_state_.pos.resize(motor_num_);
  robstride_state_.vel.resize(motor_num_);
  robstride_state_.tor.resize(motor_num_);
  robstride_state_.temp.resize(motor_num_);
  robstride_state_.error_code.resize(motor_num_);
  robstride_state_.pattern.resize(motor_num_);
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
  if (!torque_enable_.at(index))
  {
    getMotor(index).Disenable_Motor(0);
  }
  else
  {
    getMotor(index).RobStrite_Motor_move_control(commands_.at(index).torque, commands_.at(index).angle,
                                                 commands_.at(index).velocity, commands_.at(index).kp,
                                                 commands_.at(index).kd);
  }

  if (ros::Time::now().toSec() - joint_state_last_pub_time_ > joint_state_pub_interval_)
  {
    joint_state_.header.stamp = ros::Time::now();
    joint_state_pub_.publish(joint_state_);
    joint_state_last_pub_time_ = ros::Time::now().toSec();
  }

  if (ros::Time::now().toSec() - torque_state_last_pub_time_ > torque_state_pub_interval_)
  {
    robstride_actuator_bridge::MotorTorqueCommand torque_state;
    for (int i = 0; i < motor_num_; i++)
    {
      torque_state.index.push_back(i);
      torque_state.torque_enable.push_back(torque_enable_.at(i));
    }
    torque_state_pub_.publish(torque_state);
    torque_state_last_pub_time_ = ros::Time::now().toSec();
  }
}

void MotorControlSet::motorCommandCallback(robstride_actuator_bridge::MotorCommand msg)
{
  uint8_t index = msg.index;
  commands_.at(index).torque = msg.torque;
  commands_.at(index).angle = msg.angle;
  commands_.at(index).velocity = msg.velocity;
  commands_.at(index).kp = msg.kp;
  commands_.at(index).kd = msg.kd;
}

void MotorControlSet::jointCommandCallback(robstride_actuator_bridge::MotorCommand msg)
{
  uint8_t index = msg.index;
  commands_.at(index).torque = msg.torque / reductions_.at(index);
  commands_.at(index).angle = msg.angle * reductions_.at(index);
  commands_.at(index).velocity = msg.velocity * reductions_.at(index);
  commands_.at(index).kp = msg.kp;
  commands_.at(index).kd = msg.kd;
}

void MotorControlSet::servoOn(int index)
{
  if (index >= motor_num_)
  {
    std::cout << "exceed motor num" << std::endl;
  }
  else
  {
    if (!torque_enable_.at(index))
    {
      getMotor(index).Enable_Motor();
      commands_.at(index).torque = 0.0;
      commands_.at(index).angle = robstride_state_.pos[index];
      commands_.at(index).velocity = 0.0;
    }
    torque_enable_.at(index) = 1;
  }
}

void MotorControlSet::servoOff(int index)
{
  torque_enable_.at(index) = 0;
}

void MotorControlSet::servoCalib(int index)
{
  commands_.at(index).angle = 0;
  getMotor(index).Set_ZeroPos();
}

void MotorControlSet::torqueEnableCallback(robstride_actuator_bridge::MotorTorqueCommand msg)
{
  if (msg.index.size() != msg.torque_enable.size())
  {
    std::cout << "size of index and torque is not identical" << std::endl;
    return;
  }

  for (int i = 0; i < msg.index.size(); i++)
  {
    int index = msg.index.at(i);
    int torque_enable = msg.torque_enable.at(i);
    if (torque_enable)
      servoOn(index);
    else
      servoOff(index);
  }
}

void MotorControlSet::motorCalibCallback(std_msgs::UInt8MultiArray msg)
{
  for (int i = 0; i < msg.data.size(); i++)
  {
    int index = msg.data.at(i);
    if (index >= motor_num_)
    {
      std::cout << "index for calub exceed motor num" << std::endl;
      continue;
    }
    else
    {
      servoCalib(index);
    }
  }
}
