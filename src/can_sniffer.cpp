#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ambpros_can_driver/control_configuration.h"
#include "ambpros_can_driver/ControlService.h"

#include <iostream>
#include <sstream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_sniffer");

  ros::NodeHandle n;
  char selector, selector2;
  int reference;

  ambpros_can_driver::control_configuration msg_configuration;
  
  ambpros_can_driver::ControlService srv;

  ros::ServiceClient client = n.serviceClient<ambpros_can_driver::ControlService>("looping");

  ros::Publisher conf_command =
      n.advertise<ambpros_can_driver::control_configuration>("can_configuration",1000);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    cout<<"[s]et target OR [c]onfigure?"<<endl;
    cin>>selector;

    if(selector == 's'){ /* set target */
      cout<<"Set Referece :"<<endl;
      cin>>reference;
      cout<<"Referece is: "<<reference<<endl;
      /* TESTING MESSAGES */
      srv.request.action_type = 1;
      srv.request.control_signal_ankle = reference;
      srv.request.control_signal_knee = reference;

      if(client.call(srv)){
        std::cout<<"Feedback-position-ankle: "<<srv.response.position_feedback_ankle<<std::endl;
        std::cout<<"Feedback-velocity-ankle: "<<srv.response.velocity_feedback_ankle<<std::endl;
        std::cout<<"Feedback-torque-ankle: "<<srv.response.torque_feedback_ankle<<std::endl;
        std::cout<<"Feedback-position-knee: "<<srv.response.position_feedback_knee<<std::endl;
        std::cout<<"Feedback-velocity-knee: "<<srv.response.velocity_feedback_knee<<std::endl;
        std::cout<<"Feedback-torque-knee: "<<srv.response.torque_feedback_knee<<std::endl;
      }

    }else if(selector == 'c'){ /* configure */
      cout<<"[p]osition, [v]elocity or [t]orque?"<<endl;
      cin>>selector2;

      if(selector2 == 'p'){
        msg_configuration.control_type = "position";
      }else if(selector2 == 'v'){
        msg_configuration.control_type = "velocity";
      }else if(selector2 == 't'){
        msg_configuration.control_type = "torque";
      }

      msg_configuration.number_actuators = 2; /* Test purposes */
      conf_command.publish(msg_configuration);
    }
    ros::spinOnce();
  }


  return 0;
}
