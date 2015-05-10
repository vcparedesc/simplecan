#ifndef CAN_BASE_HPP
#define CAN_BASE_HPP

#include "std_msgs/String.h"
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

typedef unsigned char byte;

struct HexaByte{
  byte lowByte;
  byte HighByte;
};

namespace SDO
{
enum DataObjectType
{
  InitiateDownload = 0,
  ImplementDownload = 1,
  InitiateUpload = 2,
  ImplementUpload = 3
};

namespace FrameAux
{
  enum NumberOfBytesNoData{
    NO_DATA_0BYTE = 0,
    NO_DATA_1BYTE = 1,
    NO_DATA_2BYTE = 2,
    NO_DATA_3BYTE = 3,
    NO_DATA_4BYTE = 3
  };

  enum TransferType{
    NORMAL_TRANSFER = 0,
    EXPEDITED_TRANSFER = 1
  };

  enum SizeIndicator{
    NOT_INDICATED = 0,
    INDICATED = 1
  };

  enum ToggleBit{
    TOGGLE_0 = 0,
    TOGGLE_1 = 1
  };

  enum MoreSegments{
    MORE_SEGMENTS = 0,
    NO_MORE_SEGMENTS = 1
  };
} // namespace FrameAux

struct HeaderAux
{
  FrameAux::NumberOfBytesNoData n;
  FrameAux::TransferType e;
  FrameAux::SizeIndicator s;
  FrameAux::ToggleBit t;
  FrameAux::MoreSegments c;
};

} // namespace SDO

enum ProcessStatus
{
  InProcess = 0,
  ProcessFinished = 1,
  Error = 2
};

union CAN_Frame
{
  struct{
    uint32_t canID;
    byte frame_byte[8];
  };

  struct
  {
    uint32_t canID;
    byte Header;
    byte Multiplexor[3];
    byte Data[4];
  }SDO_InitiateService;

  struct
  {
    uint32_t canID;
    byte Header;
    byte Seg_data[7];
  }SDO_ImplementService;

};

namespace PDO{
    namespace COB_ID{
        namespace TPDO{
          int TPDO1_base = 385;
          int TPDO2_base = 641;
          int TPDO3_base = 897;
          int TPDO4_base = 1153;
        }

        namespace RPDO{
          int RPDO1_base = 513;
          int RPDO2_base = 769;
          int RPDO3_base = 1025;
          int RPDO4_base = 1281;
        }
    }

    namespace Comm{
        namespace Rx{
          int Rxc1_base = 5120;
          int Rxc2_base = 5121;
          int Rxc3_base = 5122;
          int Rxc4_base = 5123;
        }

        namespace Tx{
          int Txc1_base = 6144;
          int Txc2_base = 6145;
          int Txc3_base = 6146;
          int Txc4_base = 6147;
        }
    }

    namespace Map{
        namespace Rx{
          int Rxm1_base = 5632;
          int Rxm2_base = 5633;
          int Rxm3_base = 5634;
          int Rxm4_base = 5635;
        }

        namespace Tx{
          int Txm1_base = 6656;
          int Txm2_base = 6657;
          int Txm3_base = 6658;
          int Txm4_base = 6659;
        }
    }
}

namespace Interpreter{
  union PDO_8bytes{
    byte _byte[8];
    struct{
      byte fistCommand;
      byte secondCommand;
      byte index[2];
      byte data[4];
    }inner;
  };

  union PDO_4bytes{
    byte _byte[8];
    struct{
      byte fistCommand;
      byte secondCommand;
      byte index[2];
      byte data[4];
    }inner;
  };
} //namespace Interpreter

namespace DeviceControl{
  enum ControlWordCommand{
    SHUTDOWN = 0,
    SWITCH_ON = 1,
    SWITCH_ON_S = 2,
    DISABLE_VOLTAGE = 3,
    QUICK_STOP = 4,
    DISABLE_OPERATION = 5,
    ENABLE_OPERATION = 6,
    FAULT_RESET = 7
  };

  enum StatusWordCommand{

  };
}

struct ControlWordBase{
  DeviceControl::ControlWordCommand Command;
  byte control_byte[2];

  int switchON;
  int enableVoltage;
  int quickStop;
  int EnableOperation;
  int OperationModeSpecific;
  int FaultReset;
  int Halt;
  /* Specific */
  int BIT4;
  int BIT5;
  int BIT6;
  int BIT8;
};

namespace NMT
{
  enum Service{
    START_REMOTE_NODE = 1,
    STOP_REMOTE_NODE = 2,
    ENTER_PRE_OPERATIONAL_STATE = 128,
    RESET_NODE = 129,
    RESET_COMMUNICATION = 130
  };
}

namespace MODE_OPERATION
{
  enum MODE{
    NO_MODE = -1,
    PROFILE_POSITION_MODE = 1,
    PROFILED_VELOCITY_MODE = 3,
    TORQUE_PROFILED_MODE = 4,
    HOMMING_MODE = 6,
    INTERPOLATED_POSITION_MODE = 7
  };
}

class can_base
{
private:
  int can_port;
  int bitrate;
  bool loopback;

  int nDrives;
  int *DriveIDs;
  bool *driveStat;

  ProcessStatus Status;
  byte ErrorInfo[4];

  int s; /* socket */
  int nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;

  double EncoderResolution;
  double MotorReduction;
  double MotorRatedCurrent;

  int CheckAvailaility(int canID);
  CAN_Frame build_ImplementDownload_SDO(SDO::HeaderAux, byte*, byte*,int);
  CAN_Frame build_InitiateDownload_SDO(SDO::HeaderAux, byte*, byte*, int);
  CAN_Frame build_ImplementUpload_SDO(SDO::HeaderAux, byte*, byte*,int);
  CAN_Frame build_InitiateUpload_SDO(SDO::HeaderAux, byte*, byte*,int);
  CAN_Frame buildFrame(SDO::DataObjectType, SDO::HeaderAux, byte*, byte*, int);
  int check_sdo_frame(SDO::DataObjectType, CAN_Frame, CAN_Frame);
  int check_InitiateDownload(CAN_Frame,CAN_Frame);
  int check_ImplementDownload(CAN_Frame,CAN_Frame);
  int check_InitiateUpload(CAN_Frame,CAN_Frame);
  int check_ImplementUpload(CAN_Frame,CAN_Frame);
  void abort_sdo_protocol(CAN_Frame);
  CAN_Frame check_answer_frame(int drive_ID, int nbytes, int v1, int v2, int i1, int i2);

  /* Based on default map PDO1, PDO2 */
  int SetMode(int drive_ID, int mode);
  int MotorON(int drive_ID);
  int MotorOFF(int drive_ID);
  int BeginMotion(int drive_ID);
  int SetSpeed(int drive_ID, int Speed);

public:
  can_base(int);

  /* Basic Informatin Setting */
  void can_information(int _nDrives, int *_drive_ID);

  /* Low level funcions */
  void send_frame(CAN_Frame, int);
  void read_frame(CAN_Frame &, int &);
  void open_can();
  void close_can();

  /* For now, we are going to use Elmo's default PDO mapping */
  void controlWord(int drive_ID,
                   int Bit4,
                   int Bit5,
                   int Bit6,
                   int Bit8,
                   int Bit13);

  CAN_Frame statusWord(int drive_ID);

  CAN_Frame send_SDO(SDO::DataObjectType, SDO::HeaderAux, byte*, byte*, int);
  int recv_SDO(SDO::DataObjectType, CAN_Frame &, CAN_Frame);

  void NetworkManagement(NMT::Service, byte);

  void PDO_Mapping(int drive_ID, int pdo_id, int pdo_number, int *ObjAddress, byte *dataLength, int nObjects);
  void send_PDO(int PDO_address, int drive_ID, byte *raw_data, int nData);
  CAN_Frame recv_PDO(int pdo_number, int drive_ID, int nbytes);
  int read_pdo(int &answer_state, CAN_Frame &_ret_frame);
  int recv_PDO_mapped(int pdo_number[], int *drive_ID, int n_drives, int n_pdos, byte lengths[][2], int *nObj, int *container);
  void receive_PDO_Mapped_sync(int drive_ID, int *pdo_number, double *container, byte lengths[][2], int *nObj, int n_pdo);
  CAN_Frame read_stack_pdo(CAN_Frame *buffer_frame, int pdo_number, int drive_ID, int n_drives, int n_pdos);
  void send_sync(int drive_ID);

  int SetMotorJoggingSpeed(int drive_ID, int Speed);
  int GetMotorJoggingSpeed(int drive_ID, int &Speed);

  int SetModeOperation(MODE_OPERATION::MODE mode, int drive_ID);
  int DisplayModeOperation(int &mode, int drive_ID);

  int PCF_PositionDemandValue(int &Position, int drive_ID);
  int PCF_PositionActualValue(int &Position, int drive_ID);
  int PCF_PositionActualValue_userDefined(int &Position, int drive_ID);
  void get_position_socket_pdo(int &feedback, int socket,int drive_ID);

  int PositionFactor(int drive_ID, int num, int den);
  int VelocityEncoderFactor(int drive_ID, int num, int den);
  int PositionEncoderResolution(int drive_ID, int num, int den);
  int VelocityEncoderResolution(int drive_ID, int num, int den);

  int TargetPosition(int drive_ID, int target_position);
  int PositionRangeLimit(int drive_ID, int MinLimit, int MaxLimit);
  int SoftwarePositionLimit(int drive_ID, int MinLimit, int MaxLimit);
  int MaxProfileVelocity(int drive_ID, int MaxVel);
  int ProfiledVelocity(int drive_ID, int Speed);
  int EndVelocity(int drive_ID, int Speed);
  int ProfileAcceleration(int drive_ID, int Acc);
  int ProfileDeceleration(int drive_ID, int Dec);
  int QuickStopDeceleration(int drive_ID, int QuickDec);
  int PositionConfigure(int drive_ID, int MinSoftRange, int MaxSoftRange, int MaxVel, int ProfVel, int Acc, int Dec, int QuickDec);
  CAN_Frame PositionSetPoint(int drive_ID, int Position, int SetNewPoint, int ChangeImmediately, int abs_rel, int Halt, int newBuffPoint);

  int OperationEnable(int drive_ID);
  int Transition_Shutdown(int drive_ID);
  int Transition_SwitchON(int drive_ID);
  int Transition_EnableOperation(int drive_ID);

  int TargetTorque(int drive_ID, int target_torque);
  int ActualTorque(int drive_ID, int &actual_torque);
  int CurrentActualValue(int drive_ID, int &actual_current);
  int TorqueSlope(int drive_ID, int rate);

  int VelocitySensorActualValue(int drive_ID, int &actual_velocity);
  int SensorSelectionCode(int drive_ID, int pos_vel_encoder);
  int VelocityDemandValue(int drive_ID, int &demand_value);
  int VelocityWindow(int drive_ID, int &velocity_window);
  int TargetVelocity(int drive_ID, int target_velocity);

  /* Motion-oriented Functionalities */

  /* Test Functions */
  void MoveAbsolute();
  void TestComm();

  /* Meta Functions */
  void configure_mode(int flag_type);
  void configure_position_mode();
  void configure_velocity_mode();
  void configure_torque_mode();

  void map_ambpro();

  void set_references(double *references_raw, int flag_type);

  void get_feedback(double *feedback,int flag_type);
  void get_feedback_pdo(double *feedback);

  void stop_devices();

  void convert_to_integer_data(double *raw_data, int *int_data, int flag_type);
  void convert_to_double_data(double *raw_data, int *int_data, int flag_type);

  class utilities{
  public:
      int hex2number(char *word);
  };
};

#endif
