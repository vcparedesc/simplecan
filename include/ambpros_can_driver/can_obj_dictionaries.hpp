#ifndef CAN_OBJ_DICTIONARY_HPP
#define CAN_OBJ_DICTIONARY_HPP

namespace CAN_FAMILY
{
  /* Object Dictionary for CANopen DS 301 */
  /* http://www.elmomc.com/support/manuals/MAN-CAN301IG.pdf */
  namespace DS301
  {
    enum OBJECT
    {
      CAN_controller_type = 0x1000,
      Error_register = 0x1001,
      Manufacturer_status_register = 0x1002,
      Pre_defined_error_field = 0x1003,
      COB_ID_SYNC = 0x1005,
      Communication_cycle_period = 0x1006,
      Manufacturers_device_name = 0x1008,
      Hardware_version = 0x1009,
      Software_version = 0x100A,
      Node_ID = 0x100B,
      Store_Parameters = 0x1010,
      Restore_Parameters = 0x1011,
      COB_ID_Time_Stamp_msg = 0x1012,
      High_Resolution_Time_Stamp = 0x1013,
      COB_ID_emergency_msg = 0x1014,
      Consumer_heartbeat_time = 0x1016,
      Producer_heartbeat_time = 0x1017,
      LSS_address = 0x1018,
      OS_Interpreter = 0x1023,
      OS_Command_mode = 0x1024,
      Error_behavior = 0x1029,
      SDO1_server_link = 0x1200,
      PDO1_Rx_comm_link = 0x1400,
      PDO2_Rx_comm = 0x1401,
      PDO3_Rx_comm = 0x1402,
      PDO4_Rx_comm = 0x1403,
      PDO1_Rx_map_link = 0x1600,
      PDO2_Rx_map = 0x1601,
      PDO3_Rx_map = 0x1602,
      PDO4_Rx_map_link = 0x1603,
      PDO1_Rx_comm = 0x1800,
      PDO2_Tx_comm = 0x1801,
      PDO3_Tx_comm = 0x1802,
      PDO4_Tx_comm = 0x1803,
      PDO1_Tx_map = 0x1A00,
      PDO2_Tx_map = 0x1A01,
      PDO3_Tx_map = 0x1A02,
      PDO4_Tx_map = 0x1A03,
      PVT_data = 0x2001,
      PT_data = 0x2002,
      Fast_position = 0x2003,
      ECAM_data = 0x2004,
      Binary_interpreter_input = 0x2012,
      Binary_interpreter_output = 0x2013,
      Recorded_data_output = 0x2030,
      Group_ID = 0x2040,
      Amplifier_free_running_timer = 0x2041,
      CAN_controller_status = 0x2082,
      Begin_on_time = 0x208A,
      Firmware_download = 0x2090,
      Auxiliary_position_actual_value = 0x20A0,
      Position_error = 0x20A1,
      Digital_input = 0x2200,
      Digital_inputs_low_byte = 0x2201,
      User_integer = 0x2F00,
      User_Float_Array = 0x2F01,
      ET_Array = 0x2F02,
      PVT_buffer_head_ptr = 0x2F11,
      PVT_buffer_tail_ptr = 0x2F12,
      Buffered_PTP_remainded_point = 0x2F15,
      Asynchronous_PDO_event = 0x2F20,
      Emergency_event = 0x2F21,
      Bus_off_timeout = 0x2F22,
      Digital_input_TPDO_event_param = 0x2F23,
      Last_time_stamp_correction = 0x2F30,
      Internal_usec_counter_last_SYNC = 0x2F31,
      Configuration_object = 0x2F40
    }; // OBJECT
  } // DS301

  /* Object Dictionary for CANopen DS 402 */
  /* http://www.elmomc.com/support/manuals/MAN-CAN402IG.pdf */
  namespace DS402
  {
    enum OBJECT
    {
      Abort_connection_opt_code = 0x6007,
      Error_code = 0x603F,
      Controlword = 0x6040,
      Statusword = 0x6041,
      Quick_stop_option_code = 0x605A,
      Shut_down_option_code = 0x605B,
      Disable_operation_opt_code = 0x605C,
      Halt_option_code = 0x605D,
      Fault_reaction_opt_code = 0x605E,
      Modes_operation = 0x6060,
      Modes_operation_display = 0x6061,
      Position_demand_value = 0x6062,
      Actual_position_internal_unit = 0x6063,
      Position_actual_value = 0x6064,
      Position_following_error_window = 0x6065,
      Position_following_error_window_time = 0x6066,
      Position_window = 0x6067,
      Position_window_time = 0x6068,
      Velocity_sensor_actual_value = 0x6069,
      Velocity_sensor_selection_code = 0x606A,
      Velocity_demand_value = 0x606B,
      Velocity_actual_sensor = 0x606C,
      Velocity_window = 0x606D,
      Velocity_window_time = 0x606E,
      Velocity_threshold = 0x606F,
      Velocity_threshold_time = 0x6070,
      Target_torque = 0x6071,
      Max_torque = 0x6072,
      Max_current = 0x6073,
      Torque_demand_value = 0x6074,
      Motor_rated_current = 0x6075,
      Motor_rated_torque = 0x6076,
      Toque_actual_value = 0x6077,
      Current_actual_value = 0x6078,
      Profiled_target_position = 0x607A,
      Position_range_limit = 0x607B,
      Homing_offset = 0x607C,
      Software_position_limit = 0x607D,
      Polarity = 0x607E,
      Max_profile_velocity = 0x607F,
      Profile_velocity = 0x6081,
      Profile_acceleration = 0x6083,
      Profile_deceleration = 0x6084,
      Quick_stop_deceleration = 0x6085,
      Motion_profile_type = 0x6086,
      Torque_slope = 0x6087,
      Torque_profile_type = 0x6088,
      Position_notation_index = 0x6089,
      Position_dimension_index = 0x608A,
      Velocity_notation_index = 0x608B,
      Velocity_dimension_index = 0x608C,
      Acceleration_notation_index = 0x608D,
      Acceleration_dimension_index = 0x608E,
      Position_encoder_resolution = 0x608F,
      Velocity_encoder_resolution = 0x6090,
      Position_factor = 0x6093,
      Velocity_encoder_factor = 0x6090,
      Position_factor = 0x6093,
      Velocity_encoder_factor = 0x6094,
      Velocity_factor_1 = 0x6095,
      Velocityt_factor_2 = 0x6096,
      Acceleration_factor = 0x6097,
      Homing_method = 0x6098,
      Homing_speed = 0x6099,
      Homing_acceleration = 0x609A,
      Interpolated_position_sub_mode = 0x60C0,
      Interpolated_data_record = 0x60C1,
      Interpolated_position_time_period = 0x60C2,
      Interpolation_data_configuration = 0x60C4,
      Position_demand_value = 0x60FC,
      Digital_input = 0x60FD,
      Target_velocity = 0x60FF,
      Motor_type = 0x6402,
      Motor_catalog_number = 0x6403,
      Motor_manufacturer = 0x6404,
      HTTP_motor_catalog_address = 0x6405,
      Motor_calibration_date = 0x6406,
      Motor_service_period = 0x6407,
      Driver_modes = 0x6502,
      Driver_manufacturer = 0x6504,
      Driver_manufacturer_website = 0x6505
    };
  }
}

#endif