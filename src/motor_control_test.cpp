//
// Created by Mu Shibo on 12/2024
//

#include <sstream>
#include <memory>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "can_msgs/Frame.h"
#include "motor_control/motor_config.h"
#include "motor_control/MotorFeedback.h"

#include "stdint.h"
#include "math.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_control_test");
  ros::NodeHandle nm;
  ros::Rate loop_rate(300);

  std::vector<uint8_t> motor_ids = { 122, 123, 124, 125, 126, 127 };
  std::vector<uint8_t> motor_types = { MotorType::Robstride03, MotorType::Robstride03, MotorType::Robstride00,
                                       MotorType::Robstride00, MotorType::Robstride00, MotorType::Robstride00 };

  std::shared_ptr<MotorControlSet> motor_controller =
      std::make_shared<MotorControlSet>(&nm, "can1_rx", "can1_tx", motor_ids, motor_types);

  ros::Duration(0.5).sleep();
  // enable the motor
  for (int i = 0; i < motor_ids.size(); i++)
  {
    motor_controller->getMotor(i).Enable_Motor();
    std::cout << "Joint motor " << static_cast<int>(motor_ids.at(i)) << " with type "
              << static_cast<int>(motor_types.at(i)) << " is enabled." << std::endl;
  }
  ros::Duration(0.1).sleep();

  int id = 0;
  int cnt = 0;
  while (ros::ok())
  {
    motor_controller->getMotor(id).RobStrite_Motor_move_control(0, 0, 1.0, 0, 10);

    id = (id + 1) % motor_ids.size();
    cnt = (cnt + 1) % 1000;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
