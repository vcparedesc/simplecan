#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ambpros_can_driver/can_base.hpp"
#include "ambpros_can_driver/control_configuration.h"
#include "ambpros_can_driver/ControlService.h"
#include <sstream>

#define ANKLE_ID 127
#define KNEE_ID 126

int Feedback_enabled = 0;
int configuration_flag = 0;
int setting_flag = 0;

int control_type_flag = -1;
double *reference_container;
double *feedback_container;
int number_drivers;
int *current_driver_ID;

double absolute_position;

can_base CanDriver(1000); // 500 Kbps

void configuration_handler(const ambpros_can_driver::control_configurationConstPtr& msg)
{
  if(msg->control_type == "position"){
     std::cout<<"Position Selected!"<<std::endl;
    control_type_flag = 1;
  }else if(msg->control_type == "velocity"){
     std::cout<<"Velocity Selected!"<<std::endl;
    control_type_flag = 2;
  }else if(msg->control_type == "torque"){
    std::cout<<"Torque Selected!"<<std::endl;
    control_type_flag = 3;
  }

  number_drivers = msg->number_actuators;

  reference_container = new double[number_drivers];
  feedback_container = new double[3*number_drivers];
  current_driver_ID = new int[number_drivers];

  for(int i = 0; i < number_drivers; i++){
    current_driver_ID[i] = 127 - i;
  }

  std::cout<<"Configuring!!"<<std::endl;
  CanDriver.can_information(number_drivers, current_driver_ID);
  CanDriver.configure_mode(control_type_flag);
  configuration_flag = 0;
}

bool close_loop(ambpros_can_driver::ControlService::Request &req,
              ambpros_can_driver::ControlService::Response &res)
{
  if(req.action_type == 0){
    Feedback_enabled = 0;

    reference_container[0] = req.control_signal_ankle;
    reference_container[1] = req.control_signal_knee;

    setting_flag = 1;
  }else if(req.action_type == 1){
    Feedback_enabled = 1;

    reference_container[0] = req.control_signal_ankle;
    reference_container[1] = req.control_signal_knee;

    setting_flag = 1;
  }else if(req.action_type == 2){
    Feedback_enabled = 1;
    setting_flag = 0;
  }

  if(setting_flag == 1){
    CanDriver.set_references(reference_container,control_type_flag);
    setting_flag = 0;
  }

  if(Feedback_enabled == 1){

    CanDriver.get_feedback(feedback_container,control_type_flag);
    /* Ankle */
    res.position_feedback_ankle = feedback_container[0];
    res.velocity_feedback_ankle = feedback_container[1];
    res.torque_feedback_ankle = feedback_container[2];

    /* Knee */
    res.position_feedback_knee = feedback_container[3];
    res.velocity_feedback_knee = feedback_container[4];
    res.torque_feedback_knee = feedback_container[5];

    Feedback_enabled = 0;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"can_node");

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("looping",close_loop);

  ros::Subscriber configuration_signal =
      n.subscribe("can_configuration",10, configuration_handler);

  CanDriver.open_can();

  ros::spin();

  CanDriver.stop_devices();
  CanDriver.close_can();
  return 0;
}
