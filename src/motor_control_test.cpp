#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <motor_control/motor_config.h>
#include <motor_control/MotorFeedback.h>

#include <math.h>
#include <memory>
#include <sstream>
#include <stdint.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_control_test");
  ros::NodeHandle nh;
  ros::Rate loop_rate(600);

  std::vector<uint8_t> motor_ids(0);
  std::vector<uint8_t> motor_types(0);
  std::vector<std::string> motor_names(0);
  std::vector<double> reductions(0);

  XmlRpc::XmlRpcValue all_servo_params;
  nh.getParam("robstride", all_servo_params);
  for (auto robstride_param : all_servo_params)
  {
    if (robstride_param.first.find("controller") != std::string::npos)
    {
      int id = robstride_param.second["id"];
      std::string type = robstride_param.second["type"];
      std::string name = robstride_param.second["name"];
      double reduction = robstride_param.second["reduction"];
      ROS_INFO_STREAM("id: " << id << ", type: " << type << ", name: " << name);

      motor_ids.push_back(id);
      motor_types.push_back(XiaomiMotor::motorName2int(type));
      motor_names.push_back(name);
      reductions.push_back(reduction);
    }
  }

  std::shared_ptr<MotorControlSet> motor_controller =
      std::make_shared<MotorControlSet>(&nh, "can1_rx", "can1_tx", motor_ids, motor_types, motor_names, reductions);

  ros::Duration(0.5).sleep();
  // enable the motor
  for (int i = 0; i < motor_ids.size(); i++)
  {
    motor_controller->getMotor(i).Enable_Motor();
    std::cout << "Joint motor " << static_cast<int>(motor_ids.at(i)) << " with type "
              << static_cast<int>(motor_types.at(i)) << " is enabled." << std::endl;
  }
  ros::Duration(0.1).sleep();

  int index = 0;
  while (ros::ok())
  {
    motor_controller->update(index);

    index = (index + 1) % motor_ids.size();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
