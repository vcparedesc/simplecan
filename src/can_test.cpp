#include "ambpros_can_driver/can_base.hpp"
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
double feedback_obj[6];

static int mapping = 0;
byte data_le[] = {32,32};
byte data_le2[] = {16};
byte lengths[][2] = {{32,32},{16,0}};

int pdo_number[] = {3,4};
int n_obje[] = {2,1};
int d_ids[] = {127,126};


can_base CanDriver(1000); // 500 Kbps

void configuration_handler(char *msg )
{
  if(msg == "position"){
     std::cout<<"Position Selected!"<<std::endl;
    control_type_flag = 1;
  }else if(msg == "velocity"){
     std::cout<<"Velocity Selected!"<<std::endl;
    control_type_flag = 2;
  }else if(msg == "torque"){
    std::cout<<"Torque Selected!"<<std::endl;
    control_type_flag = 3;
  }

  number_drivers = 2;

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

bool test_loop( )
{

    return true;
}

int main(int argc, char **argv)
{
  CanDriver.open_can();
  configuration_handler("torque");
  CanDriver.map_ambpro();
  int flag;
  sleep(4);

  while(true){
      sleep(1);
      CanDriver.get_feedback_pdo(feedback_obj);
/*
      for(int i = 0; i < 6; i++){
          std::cout<<"feedbacks--"<<i<<"   :"<<feedback_obj[i]<<std::endl;
      }
      */

  }


  CanDriver.stop_devices();
  CanDriver.close_can();
  return 0;
}
