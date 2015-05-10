#include "ambpros_can_driver/can_base.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <sys/socket.h>
#include <sys/uio.h>

using namespace std;

int can_base::utilities::hex2number(char* word)
{
    std::string myhex(word);
    int x = strtoul(myhex.substr(0, 4).c_str(), NULL, 16);

    return x;
}

void bytes_to_decimal(byte *frame, int nbytes, int &decimal)
{
  int signed_var = 0;

  decimal = 0;

  if(frame[nbytes - 1] >> 7 == 1){ /* Is it a negative number? */
    signed_var = 1;
    for(int i = 0; i < nbytes; i++){
      frame[i] = 255 - frame[i];
    }

    frame[0] = frame[0] + 1;

  }

  for(int i = 0; i < nbytes; i++){
    decimal += frame[i]*pow(256,i);
  }

  if(signed_var == 1){
    decimal = -decimal;
  }
}

void decimal_to_bytes(byte *frame, int &nbytes, int decimal)
{
  int signed_var = 0;

  if(decimal < 0)
  {
    decimal = -decimal;
    signed_var = 1;
  }

  nbytes = floor((1.0/8.0 * log2(decimal))) + 1;
  int t_dec = decimal;

  for(int i = 0; i < nbytes; i++){
    frame[nbytes- 1 -i] = (t_dec - (t_dec % (int)(pow(256,nbytes- 1 -i)))) >> (int)(8 * (nbytes - 1 -i));
    t_dec = decimal % (int)(pow(256,nbytes- 1 -i));
  }

  if(signed_var == 1){
    for(int i = 0; i < 4; i++){
    frame[i] = 255 - frame[i];
    }

    frame[0] = frame[0]+1;
  }
}

HexaByte int_to_address(int addr)
{
  HexaByte address;

  address.lowByte = (addr % 256);
  address.HighByte = (addr - address.lowByte) / 256;

  return address;
}

HexaByte int_to_hex( int i )
{
  HexaByte Hexa;
  char lo;
  char hi;

  if(i <= 255 && i >= 0){
    lo = i - ((i >> 4) << 4);
    hi = i >> 4;
  }else{
    /* Not a byte */
    /* ROS ERROR */
  }

  if(lo >= 0 && lo < 10){
    Hexa.lowByte = lo + 48;
  }else if(lo >= 10 && lo <= 15){
    Hexa.lowByte = lo + 55;
  }

  if(hi >= 0 && hi < 10){
    Hexa.HighByte = hi + 48;
  }else if(hi >= 10 && hi <= 15){
    Hexa.HighByte = hi + 55;
  }

  return Hexa;
}


void debug_print_frame(CAN_Frame Frame, int nbytes=8)
{
  HexaByte temp;
  cout<<Frame.canID<<"#";
  for(int i = 0; i < nbytes; i++){
    temp = int_to_hex(Frame.frame_byte[i]);
    cout<<temp.HighByte<<temp.lowByte<<" ";
  }
  cout<<endl;
}

int bitAt(byte b, int index)
{
  return ((b >> (index)) - ((b >> index+1)<<1));
}

void setBitAt(byte &b, int index, int value)
{
  b = (b + (value<<index));
}

/**
  Constructor for can_base class, initialize bitrates and some specific details
  @arg int Baudrate: Read the bitrate in kbps.
**/
can_base::can_base(int Baudrate)
{
  int PermissibleBauds[] = {10,20,50,125,250,500,1000};
  bool isPermissible = false;

  for(int i = 0; i < 7; i++){
    if(Baudrate == PermissibleBauds[i] || Baudrate == 0){
      isPermissible = true;
      break;
    }
  }

  if(isPermissible == false){
    cout<<"WARNING: Non-Standard baudrate for CANopen implementation"<<endl;
  }

  this->bitrate = Baudrate;

  if(this->bitrate == 0){
    this->loopback = 1;
    cout<<"CAN: CAN assigned to work on loopback"<<endl;
  }else{
    this->loopback = 0;
    cout<<"CAN: CAN assigned to work with "<<this->bitrate<<" kbps"<<endl;
  }
}

void can_base::can_information(int _nDrives, int *_drive_ID)
{
  this->nDrives = _nDrives;
  this->DriveIDs = new int[this->nDrives];
  for(int i = 0; i < nDrives; i++){
    this->DriveIDs[i] = _drive_ID[i];
  }
  this->EncoderResolution = 4096*4;
  this->MotorRatedCurrent = 8.7; // from datasheet Max Current 37A
  this->MotorReduction = 80;
}


CAN_Frame can_base::check_answer_frame(int drive_ID, int nbytes, int v1, int v2, int i1, int i2)
{
  CAN_Frame Frame;
  int loopbreaker = 0;
  int stop_condition = 0;
  int nbytesr = 0;

  while(nbytesr != nbytes || loopbreaker == 0){
    read_frame(Frame,nbytes);

    if(Frame.frame_byte[i1] == v1 && Frame.frame_byte[i2] == v2){
      loopbreaker = 1;
      return Frame;
    }else{
      loopbreaker = 0;
      if((stop_condition++) == 1000){
        Frame.canID = -1;
        return Frame;
      }
    }
  }
}

/**
  Talk with system to open can0 device and start to work with it
**/
void can_base::open_can()
{ 
  cout<<"Opening CAN driver ..."<<endl;

//  system("${DRIVER_ELMO_PATH}/scripts/open_can.sh");

  this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  strcpy(this->ifr.ifr_name, "can0");
  ioctl(this->s, SIOCGIFINDEX, &this->ifr);

  this->addr.can_family = AF_CAN;
  this->addr.can_ifindex = this->ifr.ifr_ifindex;

  bind(this->s, (struct sockaddr *) &this->addr, sizeof(this->addr));
  sleep(1);
}

/**
  Talk with shell system and close CAN driver to specific device
**/
void can_base::close_can()
{
  cout<<"Closing CANopen driver ..."<<endl;

  close(this->s);
}

void can_base::read_frame(CAN_Frame &UpFrame, int &dlc)
{
  struct can_frame frame;
  volatile int nbytes = 0;
  struct timespec tim, tim2;
  tim.tv_sec = 0;
  tim.tv_nsec = 1000;

  bind(s, (struct sockaddr *) &addr, sizeof(addr));

  nanosleep(&tim , &tim2); /* Workaround */

  while(nbytes == 0){
    nbytes = read(s, &frame, sizeof(struct can_frame));
  }

  if(nbytes < 0){
    perror("can raw socket read");
    return;
  }

  if(nbytes < sizeof(struct can_frame)){
    fprintf(stderr, "read: incomplete CAN frame \n");
  }

  dlc = frame.can_dlc;

  UpFrame.canID = frame.can_id;

  for(int i = 0; i < dlc; i++){
    UpFrame.frame_byte[i] = frame.data[i];
  }

  //close(s);
}

void can_base::send_frame(CAN_Frame UpcomingFrame, int dlc)
{
  int nbytes;
  struct can_frame frame;

  frame.can_id = UpcomingFrame.canID;
  frame.can_dlc = dlc;

  for(int i = 0; i <= 8; i++){
    frame.data[i] = UpcomingFrame.frame_byte[i];
  }

  /* send frame */
  if ((nbytes = write(s, &frame, sizeof(frame))) != sizeof(frame)) {
    perror("write");
    return;
  }

  //close(s);

}

/**
  Sends a Frame over the CAN bus.
**/
CAN_Frame can_base::send_SDO(SDO::DataObjectType Type, SDO::HeaderAux h, byte *m, byte *d, int id)
{
  HexaByte HexaSupport;
  CAN_Frame Frame;
  CAN_Frame AnswerFrame;

  Frame = buildFrame(Type,h,m,d,id);

  /* DEBUG */
  for(int i = 0; i < 8; i++){
    HexaSupport = int_to_hex(Frame.frame_byte[i]);
  }
  //debug_print_frame(Frame,8);

  send_frame(Frame,8);
  if(recv_SDO(Type,AnswerFrame,Frame)){
    return AnswerFrame;
  }else{
    this->Status = Error;
  }

}

CAN_Frame can_base::build_InitiateDownload_SDO(SDO::HeaderAux h,
                                               byte* m, byte* d, int driveID)
{
  CAN_Frame Frame;
  int css = 1;

  Frame.canID = 1536 + driveID;

  Frame.SDO_InitiateService.Header =
      css * pow(2,5) + h.n * pow(2,2) + h.e * pow(2,1) + h.s * pow(2,0);

  for(int i = 0; i < 2; i++){
    Frame.SDO_InitiateService.Multiplexor[1-i] = m[i];
  }
  Frame.SDO_InitiateService.Multiplexor[2] = m[2];

  if(h.n != 0 && !((h.e == 1) && (h.s == 1))){
    /* ROS ERROR: Bad Frame construction */
  }

  if(h.e == 0 && h.s == 0){
    /* d is reserved */

    for(int i = 0; i < 4; i++){
      Frame.SDO_InitiateService.Data[i] = 0;
    }

  }else if(h.e == 0 && h.s == 1){
    /* Number of bytes to be downloaded */
    for(int i = 0; i < 4; i++){
      Frame.SDO_InitiateService.Data[i] = d[i];
    }
  }else if(h.e == 1 && h.s == 1){
    /* Data of length 4-n to be downloaded */

    for(int i = 0; i < 4; i++){
      Frame.SDO_InitiateService.Data[i] = d[i];
    }

  }else if(h.e == 1 && h.s == 0){
    /* Unspecified number of bytes to be downloaded */

    for(int i = 0; i < 4; i++){
      Frame.SDO_InitiateService.Data[i] = d[i];
    }

  }

  return Frame;
}

CAN_Frame can_base::build_ImplementDownload_SDO(SDO::HeaderAux h,
                                               byte* m, byte* d, int driveID)
{
  CAN_Frame Frame;

  Frame.canID = 1536 + driveID;

  int css = 0;
  Frame.SDO_InitiateService.Header =
      css * pow(2,5) + h.t * pow(2,4) + h.n * pow(2,1) + h.c * pow(2,0);

  for (int i = 0; i < 7; i++){
    Frame.SDO_ImplementService.Seg_data[i] = d[i];
  }

  return Frame;
}

CAN_Frame can_base::build_InitiateUpload_SDO(SDO::HeaderAux h,
                                               byte* m, byte* d,int driveID)
{
  CAN_Frame Frame;

  Frame.canID = 1536 + driveID;

  int css = 2;
  Frame.SDO_InitiateService.Header =
      css * pow(2,5) + 0 * pow(2,0);

  for(int i = 0; i < 2; i++){
    Frame.SDO_InitiateService.Multiplexor[1-i] = m[i];
  }
  Frame.SDO_InitiateService.Multiplexor[2] = m[2];

  for(int i = 0; i < 4; i++){
    Frame.SDO_InitiateService.Data[i] = 0;
  }

  return Frame;
}

CAN_Frame can_base::build_ImplementUpload_SDO(SDO::HeaderAux h,
                                               byte* m, byte* d, int driveID)
{
  CAN_Frame Frame;

  Frame.canID = 1536 + driveID;

  int css = 3;
  Frame.SDO_ImplementService.Header =
      css * pow(2,5) + h.t * pow(2,4) + 0 * pow(2,0);

  for (int i = 0; i < 7; i++){
    Frame.SDO_ImplementService.Seg_data[i] = 0;
  }

  return Frame;
}

CAN_Frame can_base::buildFrame(SDO::DataObjectType Type, SDO::HeaderAux h, byte *m, byte *d,int id)
{
  CAN_Frame Frame;

  switch(Type)
  {
    case 0:
      Frame = build_InitiateDownload_SDO(h,m,d,id);
      break;
    case 1:
      Frame = build_ImplementDownload_SDO(h,m,d,id);
      break;
    case 2:
      Frame = build_InitiateUpload_SDO(h,m,d,id);
      break;
    case 3:
      Frame = build_ImplementUpload_SDO(h,m,d,id);
      break;
    default:
      /* ROS MESSAGE ERROR */
      break;
  }

  return Frame;
}

void can_base::MoveAbsolute()
{
  CAN_Frame Frame;
  Frame.canID = 601;
  Frame.frame_byte[0] = 35;
  Frame.frame_byte[1] = 131;
  Frame.frame_byte[2] = 60;
  Frame.frame_byte[3] = 0;
  Frame.frame_byte[4] = 64;
  Frame.frame_byte[5] = 66;
  Frame.frame_byte[6] = 15;
  Frame.frame_byte[7] = 0;

  send_frame(Frame,8);

  /* After Response */
  Frame.canID = 601;
  Frame.frame_byte[0] = 35;
  Frame.frame_byte[1] = 132;
  Frame.frame_byte[2] = 60;
  Frame.frame_byte[3] = 0;
  Frame.frame_byte[4] = 64;
  Frame.frame_byte[5] = 66;
  Frame.frame_byte[6] = 15;
  Frame.frame_byte[7] = 0;

  send_frame(Frame,8);

  /* After Response */
  Frame.canID = 601;
  Frame.frame_byte[0] = 35;
  Frame.frame_byte[1] = 129;
  Frame.frame_byte[2] = 60;
  Frame.frame_byte[3] = 0;
  Frame.frame_byte[4] = 64;
  Frame.frame_byte[5] = 66;
  Frame.frame_byte[6] = 15;
  Frame.frame_byte[7] = 0;

  send_frame(Frame,8);
}

void can_base::TestComm()
{
  CAN_Frame Frame;
  int nbytes;
  Frame.canID = 0;
  Frame.frame_byte[0] = 130;
  Frame.frame_byte[1] = 00;
  send_frame(Frame,2);
  read_frame(Frame,nbytes);

  if(Frame.canID = 767){
    cout<<"Test OK"<<endl;

    cout<<"--Debug"<<endl;
    HexaByte dummyHexa;
    cout<<(int)Frame.canID<<endl;
    cout<<(int)nbytes<<endl;

    for(int i = 0 ; i < nbytes; i++){
      dummyHexa = int_to_hex(Frame.frame_byte[i]);
      cout<<dummyHexa.HighByte<<dummyHexa.lowByte<<" ";
    }
    cout<<endl;
  }
}

int can_base::check_InitiateDownload(CAN_Frame NewFrame, CAN_Frame OldFrame)
{
  int check_m = 1;
  int check_id = 1;

  if( (NewFrame.canID - 1408) != (OldFrame.canID - 1536) ){
    check_id = 0;
    return 0;
  }

  if( (NewFrame.SDO_InitiateService.Header >> 5) == 3){

    for(int i = 0; i < 3; i++){

      if(NewFrame.SDO_InitiateService.Multiplexor[i] != OldFrame.SDO_InitiateService.Multiplexor[i]){
        check_m = 0;
        return 0;
      }

    }

  }else{ /* Error Frame */

    check_m = 0;
    return 0;
  }

  return check_m;
}

int can_base::check_ImplementDownload(CAN_Frame NewFrame, CAN_Frame OldFrame)
{
  int check_t = 1;
  int check_id = 1;

  if( (NewFrame.canID - 1408) != (OldFrame.canID - 1536) ){
    check_id = 0;
    return 0;
  }

  if( (NewFrame.SDO_ImplementService.Header >> 5) == 1 ){

    if(bitAt(NewFrame.SDO_ImplementService.Header,4) != bitAt(OldFrame.SDO_ImplementService.Header,4)){
      check_t = 0;
    }

  }else{ /* Error Frame */
      check_t = 0;
  }

  return check_t;
}

int can_base::check_InitiateUpload(CAN_Frame NewFrame, CAN_Frame OldFrame)
{
  int check_m = 1;
  int check_id = 1;

  if( (NewFrame.canID - 1408) != (OldFrame.canID - 1536) ){
    check_id = 0;
    return 0;
  }

  if( (NewFrame.SDO_InitiateService.Header >> 5) == 2 ){

    for(int i = 0; i < 3; i++){
      if(NewFrame.SDO_InitiateService.Multiplexor[i] != OldFrame.SDO_InitiateService.Multiplexor[i]){
        check_m = 0;
      }
    }

  }else{ /* Error Frame */
    check_m = 0;
  }

  return check_m;
}

int can_base::check_ImplementUpload(CAN_Frame NewFrame, CAN_Frame OldFrame)
{
  int check_t = 1;
  int check_id = 1;

  if( (NewFrame.canID - 1408) != (OldFrame.canID - 1536) ){
    check_id = 0;
    return 0;
  }

  if( (NewFrame.SDO_ImplementService.Header >> 5) == 0){
    if(bitAt(NewFrame.SDO_ImplementService.Header,4) != bitAt(OldFrame.SDO_ImplementService.Header,4)){
      check_t = 0;
    }
  }else{ /* Error Frame */
    check_t = 0;
  }

  return check_t;
}

int can_base::check_sdo_frame(SDO::DataObjectType Type, CAN_Frame NewFrame,
                              CAN_Frame OldFrame)
{
  switch(Type)
  {
    case 0:
      return check_InitiateDownload(NewFrame, OldFrame);
      break;
    case 1:
      return check_ImplementDownload(NewFrame, OldFrame);
      break;
    case 2:
      return check_InitiateUpload(NewFrame, OldFrame);
      break;
    case 3:
      return check_ImplementUpload(NewFrame, OldFrame);
      break;
    default:
      /* It should never happen, but if it does, return -1 */
      return -1;
      break;
  }
}

void can_base::abort_sdo_protocol(CAN_Frame Frame)
{
  if((Frame.SDO_InitiateService.Header >> 5) == 4){
    this->Status = Error;

    for(int i = 0; i < 4 ; i++){
      this->ErrorInfo[3 - i] = Frame.SDO_InitiateService.Data[i];
    }

  }else{
    return; /* Unexpected Behavior */
  }
}

int can_base::recv_SDO(SDO::DataObjectType Type, CAN_Frame & Frame,
                       CAN_Frame OldFrame)
{
  CAN_Frame newFrame;
  int nbytes = 8;

  //read_frame(newFrame,nbytes);
  newFrame = check_answer_frame(OldFrame.canID - 1536,nbytes,OldFrame.frame_byte[1],OldFrame.frame_byte[2],1,2);

  if(check_sdo_frame(Type,newFrame, OldFrame) == 0){
    /* Check Error Frames - Abort SDO Protocol */
    abort_sdo_protocol(newFrame);
    std::cout<<"Here?"<<std::endl;
    return 0;
  }

  Frame = newFrame;
  return 1;
}

int can_base::CheckAvailaility(int canID)
{
  for(int i = 0; i < this->nDrives; i++){
    if(canID == this->DriveIDs[i]){
      return 1; /* If we get the same ID twice or more? */
    }
  }
}

void can_base::NetworkManagement(NMT::Service Service, byte driveID)
{
  /* We should kill Motor applications before proceeding */
  CAN_Frame NMT_FRAME;
  CAN_Frame answer_frame;
  int nbytes;
  struct timespec tim, tim2;

  NMT_FRAME.canID = 0;
  NMT_FRAME.frame_byte[0] = Service;
  NMT_FRAME.frame_byte[1] = driveID;

  send_frame(NMT_FRAME, 2);

  tim.tv_sec = 0;
  tim.tv_nsec = 1000000000;

  //bind(s, (struct sockaddr *) &addr, sizeof(addr));

  //nanosleep(&tim , &tim2); /* Workaround */
  sleep(1);
}

void can_base::PDO_Mapping(int drive_ID, int pdo_id, int pdo_number, int *ObjAddress, byte *dataLength, int nObjects)
{
  SDO::HeaderAux h;
  byte multiplexerPDO[3];
  byte multiplexerOBJ[3];
  byte data[4];
  HexaByte Address_PDO;
  HexaByte Address_Obj;
  int sub_index = 0;
  int PDO_CommParam;
  int PDO_address;

  if(pdo_id == 0){ /* TPDO */
      PDO_CommParam = PDO::Comm::Tx::Txc1_base + pdo_number - 1;
      PDO_address = PDO::Map::Tx::Txm1_base + pdo_number - 1;
  }else if(pdo_id == 1){ /* RPDO */
      PDO_CommParam = PDO::Comm::Rx::Rxc1_base + pdo_number - 1;
      PDO_address = PDO::Map::Rx::Rxm1_base + pdo_number - 1;
  }


  /* Zero Step, Disable PDO InitiateDownload*/
  h.n = SDO::FrameAux::NO_DATA_0BYTE;
  h.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  h.s = SDO::FrameAux::INDICATED;

  Address_Obj = int_to_address(PDO_CommParam);


  multiplexerOBJ[0] = Address_Obj.HighByte;
  multiplexerOBJ[1] = Address_Obj.lowByte;
  multiplexerOBJ[2] = 1; /* Object Sub-Index */

  if(pdo_id == 0){
    Address_Obj = int_to_address(PDO::COB_ID::TPDO::TPDO1_base + 256*(pdo_number - 1) + drive_ID - 1);
  }else if(pdo_id == 1){
    Address_Obj = int_to_address(PDO::COB_ID::RPDO::RPDO1_base + 256*(pdo_number - 1) + drive_ID - 1);
  }

  /* First Step, STOP Transmission of (T/R)PDOx */
  h.n = SDO::FrameAux::NO_DATA_3BYTE;
  h.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  h.s = SDO::FrameAux::INDICATED;

  Address_PDO = int_to_address(PDO_address);

  multiplexerPDO[0] = Address_PDO.HighByte;
  multiplexerPDO[1] = Address_PDO.lowByte;
  multiplexerPDO[2] = sub_index; // This time is 0

  data[0] = 0; // 0 Mapped Objects (STOP)
  data[1] = data[2] = data[3] = 0;

  send_SDO(SDO::InitiateDownload,h,multiplexerPDO,data,drive_ID);

  /* Second Step, mapping objects */

  for(int i = 0 ; i < nObjects; i++){
    h.n = SDO::FrameAux::NO_DATA_0BYTE;
    h.e = SDO::FrameAux::EXPEDITED_TRANSFER;
    h.s = SDO::FrameAux::INDICATED;

    Address_Obj = int_to_address(ObjAddress[i]);

    multiplexerPDO[2] = (i+1);

    multiplexerOBJ[0] = Address_Obj.HighByte;
    multiplexerOBJ[1] = Address_Obj.lowByte;
    multiplexerOBJ[2] = 0;

    /* DATA Format */
    data[0] = dataLength[i];
    data[1] = multiplexerOBJ[2];
    data[2] = multiplexerOBJ[1];
    data[3] = multiplexerOBJ[0];

    send_SDO(SDO::InitiateDownload,h,multiplexerPDO,data,drive_ID);
  }


  // Asynchronous type is implemented, TODO: make this more flexible.
  /* Third step, Set (T/R)PDOx type */
  Address_Obj = int_to_address(PDO_CommParam);

  multiplexerOBJ[0] = Address_Obj.HighByte;
  multiplexerOBJ[1] = Address_Obj.lowByte;
  multiplexerOBJ[2] = 2; /* Object Sub-Index */

  data[0] = 1; data[1] = data[2] = data[3] = 0;
  send_SDO(SDO::InitiateDownload,h,multiplexerOBJ,data,drive_ID);

  /* Fourth Step, Activate PDO mapping */
  multiplexerPDO[2] = 0;
  data[0] = nObjects;
  send_SDO(SDO::InitiateDownload,h,multiplexerPDO,data,drive_ID);

  /* Final Step, Disable PDO */
  h.n = SDO::FrameAux::NO_DATA_3BYTE;
  h.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  h.s = SDO::FrameAux::INDICATED;

  Address_Obj = int_to_address(PDO_CommParam);

  multiplexerOBJ[0] = Address_Obj.HighByte;
  multiplexerOBJ[1] = Address_Obj.lowByte;
  multiplexerOBJ[2] = 1; /* Object Sub-Index */

  if(pdo_id == 0){
    Address_Obj = int_to_address(PDO::COB_ID::TPDO::TPDO1_base + 256*(pdo_number - 1) + drive_ID - 1);
  }else if(pdo_id == 1){
    Address_Obj = int_to_address(PDO::COB_ID::RPDO::RPDO1_base + 256*(pdo_number - 1) + drive_ID - 1);
  }

  Address_Obj = int_to_address(PDO::COB_ID::TPDO::TPDO1_base + 256*(pdo_number - 1) + drive_ID - 1);
  data[0] = Address_Obj.lowByte; data[1] = Address_Obj.HighByte; data[2] = 0x00; data[3] = 0x00;
  send_SDO(SDO::InitiateDownload,h,multiplexerOBJ,data,drive_ID);

}

void can_base::send_sync(int drive_ID)
{
    CAN_Frame frame;
    frame.canID = 128;
    send_frame(frame,0);
}

void can_base::receive_PDO_Mapped_sync(int drive_ID, int *pdo_number, double *container, byte lengths[][2], int *nObj, int n_pdo)
{
    CAN_Frame ans_frame;
    int obj_counter = 0;
    byte frame[8] = {0};
    int decimal;

    /* For now it's recognizing just one pdo mapping, we need to do a better implementation using data structures to manage several PDO mappings */
    send_sync(0);

    for(int k = 0; k < n_pdo; k++){
        ans_frame = recv_PDO(pdo_number[k],drive_ID,8);

        for(int i = 0; i < nObj[k]; i++){
            container[obj_counter] = 0;
            int start = (i == 0) ? 0 : (lengths[k][i-1]/8);
            int top = (lengths[k][i]/8);
            for (int j = start; j < start + top; j++){
                frame[j-start] = ans_frame.frame_byte[j];
                //container[obj_counter]  += ans_frame.frame_byte[j] * pow(256,j-start);
            }
            bytes_to_decimal(frame,lengths[k][i], decimal);
            container[obj_counter] = decimal;
            std::cout<<"valor: "<<container[obj_counter]* 360 / 8192 <<std::endl;
            obj_counter++;
        }
    }

}

/**
  Send PDO data, since it depends on configuration, it's a user responsability
  to send the correct raw_data
  **/
void can_base::send_PDO(int PDO_address, int drive_ID, byte *raw_data, int nData)
{
  CAN_Frame Frame;
  Frame.canID = PDO_address + drive_ID;

  for(int i = 0; i < nData; i++){
    Frame.frame_byte[i] = raw_data[i];
  }

  send_frame(Frame,nData);
  /* Is possible than more than 1 frame is answered-back, we need to implement
    a way to wait for them before proceeding */
}

CAN_Frame can_base::recv_PDO(int pdo_number, int drive_ID, int nbytes)
{
    int limiter = 0;
    int _nbytes;
    CAN_Frame Frame;

    read_frame(Frame, _nbytes);

    while(Frame.canID - (385 + 256*(pdo_number - 1)) != drive_ID - 1 || limiter > 5){
        read_frame(Frame, _nbytes);
        limiter ++;
    }
    if(limiter == 6){
        /* TODO: error handling */
    }

    return Frame;
}

CAN_Frame can_base::read_stack_pdo(CAN_Frame *buffer_frame, int pdo_number, int drive_ID, int n_drives, int n_pdos)
{
    CAN_Frame Frame;

    for(int i = 0; i < n_drives * n_pdos; i++)
    {
        if(buffer_frame[i].canID - (385 + 256*(pdo_number - 1)) == drive_ID - 1){
            Frame = buffer_frame[i];
            return Frame;
        }
    }

    return Frame;
}

int can_base::read_pdo(int &answer_state, CAN_Frame &_ret_frame){
    CAN_Frame PDO_frame;
    int nbytes = 0;
    volatile int err;

    read_frame(PDO_frame, nbytes);
    err = PDO_frame.canID;
    while(err < 1000){
        read_frame(PDO_frame, nbytes);
        err = PDO_frame.canID;
    }

    err = PDO_frame.canID;

    _ret_frame = PDO_frame;
    //debug_print_frame(_ret_frame);

    if(_ret_frame.canID > 0){
        answer_state = 1;
        return answer_state;
    }else{
        answer_state = 0;
        return answer_state;
    }

}

int can_base::recv_PDO_mapped(int pdo_number[], int *drive_ID, int n_drives, int n_pdos, byte lengths[][2], int *nObj, int *container)
{
    CAN_Frame Frame_buffer[n_drives * n_pdos];
    CAN_Frame Frame[n_drives * n_pdos];
    CAN_Frame ans_frame;
    int obj_counter = 0;
    byte frame[8] = {0};
    int decimal = 0;
    int state = 0;
    int id_check = 0;

    /* reading and stacking all the mapped frames */
    //std::cout<< "Number :"<<n_drives * n_pdos<<std::endl;
    for(int i = 0; i < n_drives * n_pdos; i++){
        this->read_pdo(state,ans_frame);
        while(state != 1){
            //std::cout<<"here"<<std::endl;
            this->read_pdo(state,ans_frame);
        }
        Frame_buffer[i] = ans_frame;
        state = 0;
        //debug_print_frame(Frame_buffer[i]);
    }

    /* sorting ex [id127 pdo3, id127 pdo4, id126 pdo3, id126 pdo4] */
    //cout<<"pdo"<<pdo_number[0]<<endl;
    for(int k = 0; k < n_drives; k++){
        for(int m = 0; m < n_pdos; m++){
            Frame[n_pdos * k + m] = read_stack_pdo(Frame_buffer,pdo_number[m],drive_ID[k],n_drives,n_pdos);
        }
    }

    for(int t = 0; t < n_drives; t++){
        for(int k = 0; k < n_pdos; k++){
            ans_frame = Frame[t * n_pdos + k];

               for(int i = 0; i < nObj[k]; i++){
                container[obj_counter] = 0;
                int start = (i == 0) ? 0 : (lengths[k][i-1]/8);
                int top = (lengths[k][i]/8);
                for (int j = start; j < start + top; j++){
                    frame[j-start] = ans_frame.frame_byte[j];
                }

                bytes_to_decimal(frame,lengths[k][i]/8, decimal);
                container[obj_counter] = decimal;
                //std::cout<<"container :"<<container[obj_counter]<<std::endl;
                obj_counter++;
            }
        }
    }

    state = 1;
    return state;
}

int can_base::SetMode(int drive_ID, int mode)
{
  byte raw_data[8] = {85, 77, 0, 0, mode, 0, 0, 0};
  CAN_Frame Frame;
  int nbytes;
  int loopbreaker = 0;
  int stop_condition = 0;

  send_PDO(768,drive_ID, raw_data, 8);

  while(nbytes != 8 || loopbreaker == 0){
    read_frame(Frame,nbytes);
    debug_print_frame(Frame);

    if(Frame.frame_byte[0] == 85 && Frame.frame_byte[1] == 77){
      loopbreaker = 1;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      return 1;
    }else{
      loopbreaker = 0;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      if((stop_condition++) == 1000){
        return 0;
      }
    }

  }
}

int can_base::MotorON(int drive_ID)
{
  byte raw_data[8] = {109, 111, 0, 0, 1, 0, 0, 0};
  CAN_Frame Frame;
  int nbytes;
  int loopbreaker = 0;
  int stop_condition = 0;

  send_PDO(768,drive_ID, raw_data, 8);

  while(nbytes != 8 || loopbreaker == 0){
    read_frame(Frame,nbytes);
    cout<<"Waiting new frame! ";
    debug_print_frame(Frame);
    cout<<Frame.frame_byte[0]<<" "<<Frame.frame_byte[1]<<endl;
    if((Frame.frame_byte[0] == 109) && (Frame.frame_byte[1] == 111)){
      loopbreaker = 1;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      return 1;
    }else{
      loopbreaker = 0;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      if((stop_condition++) == 1000){
        return 0;
      }
    }

  }

  return 0;
}

int can_base::MotorOFF(int drive_ID)
{
  byte raw_data[8] = {77, 79, 0, 0, 0, 0, 0, 0};
  CAN_Frame Frame;
  int nbytes;
  int loopbreaker = 0;
  int stop_condition = 0;

  send_PDO(768,drive_ID,raw_data, 8);

  while(nbytes != 8 || loopbreaker == 0){
    read_frame(Frame,nbytes);
    cout<<"Waiting new frame! ";
    debug_print_frame(Frame);

    if(Frame.frame_byte[0] == 77 && Frame.frame_byte[1] == 79){
      loopbreaker = 1;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      return 1;
    }else{
      loopbreaker = 0;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      if((stop_condition++) == 1000){
        return 0;
      }
    }

  }

}

int can_base::BeginMotion(int drive_ID)
{
  byte raw_data[4] = {98, 103, 0, 0};
  CAN_Frame Frame;
  int nbytes;
  int loopbreaker = 0;
  int stop_condition = 0;

  send_PDO(768,drive_ID,raw_data, 4);

  while(nbytes != 8 || loopbreaker == 0){
    read_frame(Frame,nbytes);
    cout<<"Waiting new frame! ";
    debug_print_frame(Frame);

    if(Frame.frame_byte[0] == 98 && Frame.frame_byte[1] == 103){
      loopbreaker = 1;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      return 1;
    }else{
      loopbreaker = 0;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      if((stop_condition++) == 1000){
        return 0;
      }
    }

  }

}

int can_base::SetSpeed(int drive_ID, int Speed)
{
  byte raw_data[8] = {106, 118, 0, 0, 0, 0, 0, 0};
  byte raw_speed[4];
  int nbytes = 8;
  CAN_Frame Frame;
  int loopbreaker = 0;
  int stop_condition = 0;

  decimal_to_bytes(raw_speed,nbytes,Speed);

  if(nbytes > 4){
    this->Status = Error;
    return 0;
  }

  for(int i = 4; i < 4 + nbytes; i++){
    raw_data[i] = raw_speed[i-4];
  }

  send_PDO(768,drive_ID,raw_data,8);

  while(nbytes != 8 || loopbreaker == 0){
    read_frame(Frame,nbytes);
    cout<<"Waiting new frame! ";
    debug_print_frame(Frame);

    if(Frame.frame_byte[0] == 106 && Frame.frame_byte[1] == 118){
      loopbreaker = 1;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      return 1;
    }else{
      loopbreaker = 0;
      cout<<"loopbreaker :"<<loopbreaker<<endl;
      if((stop_condition++) == 1000){
        return 0;
      }
    }

  }
}

int can_base::SetMotorJoggingSpeed(int drive_ID, int Speed)
{
  int stop_handler = 1;

  if(Speed > 0){
    //CanDriver.NetworkManagement(NMT::START_REMOTE_NODE,0);
    //stop_handler = MotorOFF(127);
    if(stop_handler == 1)
     stop_handler = SetMode(127,2);
    if(stop_handler == 1)
      stop_handler = MotorON(127);
    if(stop_handler == 1)
      stop_handler = SetSpeed(127,Speed);
    if(stop_handler == 1)
      stop_handler = BeginMotion(127);
  }else if(Speed == 0){
    MotorOFF(127);
  }else if(Speed == -1){
    NetworkManagement(NMT::STOP_REMOTE_NODE,0);
    NetworkManagement(NMT::RESET_COMMUNICATION,0);
    NetworkManagement(NMT::START_REMOTE_NODE,0);
  }
}

int can_base::SetModeOperation(MODE_OPERATION::MODE mode, int drive_ID)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_3BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[1] = 96;
  Multiplexor[0] = 96;
  Multiplexor[2] = 0;

  Data[0] = mode;

  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::DisplayModeOperation(int &mode, int drive_ID)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_3BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[1] = 96;
  Multiplexor[0] = 97;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateUpload, Aux, Multiplexor, Data, drive_ID);
  mode = AnswerFrame.SDO_InitiateService.Data[0];

  if(mode = -1){
    return 0;
  }else{
    return 1;
  }
}

int can_base::PCF_PositionDemandValue(int &Position, int drive_ID)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 98;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateUpload, Aux, Multiplexor, Data, drive_ID);

  bytes_to_decimal(AnswerFrame.SDO_InitiateService.Data,4,Position);

  return 1;
}

void can_base::get_position_socket_pdo(int &feedback, int socket,int drive_ID)
{
  Interpreter::PDO_4bytes pdo;
  CAN_Frame AnswerFrame;
  CAN_Frame SenderFrame;
  utilities uti;
  int nbytes = 8;

  pdo.inner.fistCommand = 'F';
  pdo.inner.secondCommand = 'P';
  pdo.inner.index[0] = socket;
  pdo.inner.index[1] = uti.hex2number("40");

  SenderFrame.canID = 768 + drive_ID;

  for(int i = 0; i < 4; i++){
      SenderFrame.frame_byte[i] = pdo._byte[i];
  }

  send_frame(SenderFrame,4);
  //std::cout<<"here"<<std::endl;
  AnswerFrame = recv_PDO(2,drive_ID,nbytes);
  bytes_to_decimal(AnswerFrame.SDO_InitiateService.Data,4,feedback);
  /* DEBUG FRAMES */
  //debug_print_frame(SenderFrame,4);
}

int can_base::PCF_PositionActualValue(int &Position, int drive_ID)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 99;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateUpload, Aux, Multiplexor, Data, drive_ID);

  bytes_to_decimal(AnswerFrame.SDO_InitiateService.Data,4,Position);

}

int can_base::PCF_PositionActualValue_userDefined(int &Position, int drive_ID)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 100;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateUpload, Aux, Multiplexor, Data, drive_ID);
  Position = AnswerFrame.SDO_InitiateService.Data[3] * pow(2,24) +
             AnswerFrame.SDO_InitiateService.Data[2] * pow(2,16) +
             AnswerFrame.SDO_InitiateService.Data[1] * pow(2,8)  +
             AnswerFrame.SDO_InitiateService.Data[0];
}

void can_base::controlWord(int drive_ID,
                           int Bit4,
                           int Bit5,
                           int Bit6,
                           int Bit8,
                           int Bit13
                           )
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_2BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 64;
  Multiplexor[2] = 0;

  /* Giving format to data */
  Data[0] = (1 << 0) + (1 << 1) + (1 << 2) + (1 << 3); /* switch on */
  setBitAt(Data[0], 4, Bit4);
  setBitAt(Data[0], 5, Bit5);
  setBitAt(Data[0], 6, Bit6);

  Data[1] = 0;
  setBitAt(Data[1], 0, Bit8);
  setBitAt(Data[1], 5, Bit13);

  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);
}

CAN_Frame can_base::statusWord(int drive_ID)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_2BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 65;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateUpload, Aux, Multiplexor, Data, drive_ID);
  return AnswerFrame;
}

int can_base::TargetPosition(int drive_ID, int target_position)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 122;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, target_position);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::PositionRangeLimit(int drive_ID, int MinLimit, int MaxLimit)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 123;
  Multiplexor[2] = 1;

  decimal_to_bytes(Data, nbytes, MinLimit);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  Multiplexor[0] = 96;
  Multiplexor[1] = 123;
  Multiplexor[2] = 2;

  for(int i = 0; i < 4; i++){
    Data[i] = 0;
  }

  decimal_to_bytes(Data, nbytes, MaxLimit);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);
}

int can_base::SoftwarePositionLimit(int drive_ID, int MinLimit, int MaxLimit)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 125;
  Multiplexor[2] = 1;

  decimal_to_bytes(Data, nbytes, MinLimit);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  Multiplexor[0] = 96;
  Multiplexor[1] = 125;
  Multiplexor[2] = 2;

  decimal_to_bytes(Data, nbytes, MaxLimit);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::MaxProfileVelocity(int drive_ID, int MaxVel)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 127;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, MaxVel);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::ProfiledVelocity(int drive_ID, int Speed)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 129;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, Speed);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::EndVelocity(int drive_ID, int Speed)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 130;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, Speed);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::ProfileAcceleration(int drive_ID, int Acc)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 131;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, Acc);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::ProfileDeceleration(int drive_ID, int Dec)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 132;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, Dec);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::QuickStopDeceleration(int drive_ID, int QuickDec)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 133;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, QuickDec);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

CAN_Frame can_base::PositionSetPoint(int drive_ID, int Position, int SetNewPoint,
                               int ChangeImmediately, int abs_rel, int Halt,
                               int newBuffPoint)
{
  int status;
  CAN_Frame Frame;

  status = TargetPosition(drive_ID,Position);
  controlWord(drive_ID,SetNewPoint, ChangeImmediately, abs_rel, Halt, newBuffPoint);
  Frame = statusWord(drive_ID);
  if(bitAt(Frame.SDO_InitiateService.Data[0],4)){
    controlWord(drive_ID,0, ChangeImmediately, abs_rel, Halt, newBuffPoint);
    Frame = statusWord(drive_ID);
  }

  return Frame;
}

int can_base::PositionConfigure(int drive_ID,int MinSoftRange,
                                int MaxSoftRange, int MaxVel,
                                int ProfVel, int Acc, int Dec, int QuickDec)
{
  SoftwarePositionLimit(drive_ID, MinSoftRange,MaxSoftRange);
  MaxProfileVelocity(drive_ID, MaxVel);
  ProfiledVelocity(drive_ID,ProfVel);
  ProfileAcceleration(drive_ID,Acc);
  ProfileDeceleration(drive_ID,Dec);
  QuickStopDeceleration(drive_ID, QuickDec);

  return 1;
}

int can_base::OperationEnable(int drive_ID)
{
  /* Using default PDO */
  byte raw_data[2] = {6, 0};
  int loopbreaker = 0;
  int stop_condition = 0;
  CAN_Frame Frame;

  send_PDO(512, drive_ID, raw_data ,2);
  check_answer_frame(drive_ID,2,49,2,0,1);

  raw_data[0] = 7; raw_data[1] = 0;
  send_PDO(512, drive_ID, raw_data ,2);
  check_answer_frame(drive_ID,2,51,2,0,1);

  raw_data[0] = 15; raw_data[1] = 0;
  send_PDO(512, drive_ID, raw_data ,2);
  check_answer_frame(drive_ID,2,55,2,0,1);

  cout<<"Operation Enabled!"<<endl;

}

int can_base::TargetTorque(int drive_ID, int target_torque)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_2BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 113;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, target_torque);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::ActualTorque(int drive_ID, int &actual_torque)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_2BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 119;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateUpload, Aux, Multiplexor, Data, drive_ID);
  bytes_to_decimal(AnswerFrame.SDO_InitiateService.Data, 2, actual_torque);

  return 1;
}

int can_base::CurrentActualValue(int drive_ID, int &actual_current)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::NORMAL_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 120;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);
  bytes_to_decimal(AnswerFrame.SDO_InitiateService.Data,4,actual_current);

  return 1;
}

int can_base::TorqueSlope(int drive_ID, int rate)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 135;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, rate);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::VelocitySensorActualValue(int drive_ID, int &actual_velocity)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 105;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateUpload, Aux, Multiplexor, Data, drive_ID);

  bytes_to_decimal(AnswerFrame.SDO_InitiateService.Data,4,actual_velocity);

  return 1;
}

int can_base::SensorSelectionCode(int drive_ID, int pos_vel_encoder)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_2BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 106;
  Multiplexor[2] = 0;

  Data[0] = pos_vel_encoder;
  Data[1] = 0;

  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::VelocityDemandValue(int drive_ID, int &demand_value)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 108;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateUpload, Aux, Multiplexor, Data, drive_ID);
  bytes_to_decimal(AnswerFrame.SDO_InitiateService.Data,4,demand_value);

  return 1;
}

int can_base::VelocityWindow(int drive_ID, int &velocity_window)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 109;
  Multiplexor[2] = 0;

  AnswerFrame = send_SDO(SDO::InitiateUpload, Aux, Multiplexor, Data, drive_ID);
  bytes_to_decimal(AnswerFrame.SDO_InitiateService.Data,4,velocity_window);

  return 1;
}

int can_base::TargetVelocity(int drive_ID, int target_velocity)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 255;
  Multiplexor[2] = 0;

  decimal_to_bytes(Data, nbytes, target_velocity);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  controlWord(drive_ID,0,0,0,0,0);
  AnswerFrame = statusWord(drive_ID);

  return 1;
}

int can_base::Transition_Shutdown(int drive_ID)
{
  byte raw_data[2] = {6, 0};
  send_PDO(512, drive_ID, raw_data ,2);
  check_answer_frame(drive_ID,2,49,2,0,1);
  return 1;
}

int can_base::Transition_SwitchON(int drive_ID)
{
  byte raw_data[2];
  raw_data[0] = 7; raw_data[1] = 0;
  send_PDO(512, drive_ID, raw_data ,2);
  check_answer_frame(drive_ID,2,51,2,0,1);
  return 1;
}

int can_base::Transition_EnableOperation(int drive_ID)
{
  byte raw_data[2];
  raw_data[0] = 15; raw_data[1] = 0;
  send_PDO(512, drive_ID, raw_data ,2);
  check_answer_frame(drive_ID,2,55,2,0,1);
  return 1;
}

int can_base::PositionFactor(int drive_ID, int num, int den)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 147;
  Multiplexor[2] = 1;

  decimal_to_bytes(Data, nbytes, num);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  Multiplexor[0] = 96;
  Multiplexor[1] = 147;
  Multiplexor[2] = 2;

  for(int i = 0; i < 4; i++){
    Data[i] = 0;
  }

  decimal_to_bytes(Data, nbytes, den);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

int can_base::PositionEncoderResolution(int drive_ID, int num, int den)
{
  SDO::HeaderAux Aux;
  byte Multiplexor[3];
  byte Data[4] = {0};
  CAN_Frame AnswerFrame;
  int nbytes;

  /* Mode of Operation */
  Aux.n = SDO::FrameAux::NO_DATA_0BYTE;
  Aux.e = SDO::FrameAux::EXPEDITED_TRANSFER;
  Aux.s = SDO::FrameAux::INDICATED;

  Multiplexor[0] = 96;
  Multiplexor[1] = 143;
  Multiplexor[2] = 1;

  decimal_to_bytes(Data, nbytes, num);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  Multiplexor[0] = 96;
  Multiplexor[1] = 143;
  Multiplexor[2] = 2;

  for(int i = 0; i < 4; i++){
    Data[i] = 0;
  }

  decimal_to_bytes(Data, nbytes, den);
  AnswerFrame = send_SDO(SDO::InitiateDownload, Aux, Multiplexor, Data, drive_ID);

  return 1;
}

void can_base::configure_position_mode()
{
  this->NetworkManagement(NMT::RESET_NODE,0);
    sleep(4);
  this->NetworkManagement(NMT::RESET_COMMUNICATION,0);
  this->NetworkManagement(NMT::START_REMOTE_NODE,0);

  for(int i = 0; i < this->nDrives; i++){
    std::cout << "start to configure Drive_" << i << "with ID :" << DriveIDs[i] << endl;
    this->Transition_Shutdown(this->DriveIDs[i]);
    std::cout << "motor is shut down" << endl;
    this->Transition_SwitchON(this->DriveIDs[i]);
    std::cout << "motor is switched on" << endl;
    this->PositionRangeLimit(this->DriveIDs[i],-600000,600000);
    this->Transition_EnableOperation(this->DriveIDs[i]);
    std::cout << "motor is enabled for operation" << endl;

    this->SetModeOperation(MODE_OPERATION::PROFILE_POSITION_MODE,this->DriveIDs[i]);
    this->PositionConfigure(this->DriveIDs[i],-600000,600000,600000,600000,1000000,1000000,5000000);
  }
}

void can_base::configure_velocity_mode()
{
  this->NetworkManagement(NMT::RESET_NODE,0);
    sleep(4);
  this->NetworkManagement(NMT::RESET_COMMUNICATION,0);
  this->NetworkManagement(NMT::START_REMOTE_NODE,0);

  for(int i = 0; i < this->nDrives; i++){
    std::cout << "start to configure Drive_" << i << "with ID :" << DriveIDs[i] << endl;
    this->Transition_Shutdown(this->DriveIDs[i]);
    std::cout << "motor is shut down" << endl;
    this->Transition_SwitchON(this->DriveIDs[i]);
    std::cout << "motor is switched on" << endl;
    this->Transition_EnableOperation(this->DriveIDs[i]);
    std::cout << "motor is enabled for operation" << endl;

    this->SetModeOperation(MODE_OPERATION::PROFILED_VELOCITY_MODE,this->DriveIDs[i]);
    this->PositionConfigure(this->DriveIDs[i],-600000,600000,600000,600000,60000,60000,300000);
    this->SensorSelectionCode(this->DriveIDs[i],0);
  }
}

void can_base::configure_torque_mode()
{
  this->NetworkManagement(NMT::RESET_NODE,0);
    sleep(4);
  this->NetworkManagement(NMT::RESET_COMMUNICATION,0);
  this->NetworkManagement(NMT::START_REMOTE_NODE,0);

  std::cout<<"Torque starts to configure!"<<std::endl;
  for(int i = 0; i < this->nDrives; i++){
    std::cout << "start to configure Drive_" << i << "with ID :" << DriveIDs[i] << endl;
    this->Transition_Shutdown(this->DriveIDs[i]);
    std::cout << "motor is shut down" << endl;
    this->Transition_SwitchON(this->DriveIDs[i]);
    std::cout << "motor is switched on" << endl;
    this->Transition_EnableOperation(this->DriveIDs[i]);
    std::cout << "motor is enabled for operation" << endl;

    this->SetModeOperation(MODE_OPERATION::TORQUE_PROFILED_MODE,this->DriveIDs[i]);
    this->TorqueSlope(this->DriveIDs[i],10000000);  //used to be 1000000000000  // 100000000
  }
}

void can_base::configure_mode(int flag_type)
{
  if(flag_type == 1){
    this->configure_position_mode();
    std::cout<<"Position done!"<<std::endl;
  }else if(flag_type == 2){
    this->configure_velocity_mode();
    std::cout<<"Velocity done!"<<std::endl;
  }else if(flag_type == 3){
    this->configure_torque_mode();
    std::cout<<"Torque done!"<<std::endl;
  }
}

void can_base::set_references(double *references_raw, int flag_type)
{
  int references[this->nDrives];

  this->convert_to_integer_data(references_raw, references, flag_type);

  if(flag_type == 1){
    for(int i = 0; i < this->nDrives; i++){
      this->PositionSetPoint(this->DriveIDs[i],references[i],1,1,0,0,1);
    }

  }else if(flag_type == 2){
    for(int i = 0; i < this->nDrives; i++){
      this->TargetVelocity(this->DriveIDs[i],references[i]);
    }

  }else if(flag_type == 3){
    for(int i = 0; i < this->nDrives; i++){
      this->TargetTorque(this->DriveIDs[i],references[i]);
    }

  }
}

void can_base::get_feedback(double *feedback, int flag_type)
{
  int feedback_internal[this->nDrives * 3];
  int absolute_feedback;
  double abs_fed;

  if(flag_type == 1){
    for(int i = 0; i < this->nDrives; i++){
      this->PCF_PositionActualValue(feedback_internal[3*i],this->DriveIDs[i]);
      this->VelocitySensorActualValue(this->DriveIDs[i],feedback_internal[3*i+1]);
      this->ActualTorque(this->DriveIDs[i],feedback_internal[3*i+2]);
//      this->CurrentActualValue(this->DriveIDs[i],feedback_internal[3*i+2]);
    }

  }else if(flag_type == 2){
    for(int i = 0; i < this->nDrives; i++){
      this->PCF_PositionActualValue(feedback_internal[3*i],this->DriveIDs[i]);
      this->VelocitySensorActualValue(this->DriveIDs[i],feedback_internal[3*i+1]);
      this->ActualTorque(this->DriveIDs[i],feedback_internal[3*i+2]);
//      this->CurrentActualValue(this->DriveIDs[i],feedback_internal[3*i+2]);
    }

  }else if(flag_type == 3){
    for(int i = 0; i < this->nDrives; i++){
      this->PCF_PositionActualValue(feedback_internal[3*i],this->DriveIDs[i]);
      this->VelocitySensorActualValue(this->DriveIDs[i],feedback_internal[3*i+1]);
      this->ActualTorque(this->DriveIDs[i],feedback_internal[3*i+2]);
//      this->CurrentActualValue(this->DriveIDs[i],feedback_internal[3*i+2]);
    }
  }
  convert_to_double_data(feedback,feedback_internal,flag_type);
}

void can_base::map_ambpro()
{
    std::cout<<"Init Mapping"<<std::endl;
    int obj_address[] = {24675, 24681};
    int obj_address2[] = {24695};
    int n_obj = 2;
    int n_obj2 = 1;
    byte data_le[] = {32,32};
    byte data_le2[] = {16};

    PDO_Mapping(127,0,3,obj_address,data_le, n_obj);
    PDO_Mapping(127,0,4,obj_address2,data_le2,n_obj2);

    PDO_Mapping(126,0,3,obj_address,data_le, n_obj);
    PDO_Mapping(126,0,4,obj_address2,data_le2,n_obj2);

    std::cout<<"Finished Mapping"<<std::endl;
}

void can_base::get_feedback_pdo(double *feedback)
{
    static int pdo_number[] = {3,4};
    static int n_obje[] = {2,1};
    static int d_ids[] = {127,126};
    static byte lengths[][2] = {{32,32},{16,0}};
    int feedback_dec[6] = {0};
    int stat = 0;

    send_sync(0);

    while(stat == 0){
     stat = this->recv_PDO_mapped(pdo_number,d_ids,2,2,lengths,n_obje,feedback_dec);
    }

//    if(stat != 1){
//        flag = stat;
//        //std::cout<<flag<<std::endl;
//    }

    for(int i = 0; i < 2; i++){
        feedback[3*i] = (double)feedback_dec[3*i] * (360.0/this->EncoderResolution) / this->MotorReduction;
        feedback[3*i + 1] = (double)feedback_dec[3*i+ 1] * (360.0/this->EncoderResolution) / this->MotorReduction;
        feedback[3*i + 2] = (double)feedback_dec[3*i + 2] * this->MotorRatedCurrent / 1000.0;
    }


}

void can_base::convert_to_integer_data(double *raw_data, int *int_data, int flag_type)
{  
  if(flag_type == 1){ /* Position Mode */ /* output-deg to counts */
    for(int i = 0; i < this->nDrives; i++){
      int_data[i] = (int)(raw_data[i] * this->MotorReduction * this->EncoderResolution/360.0);
    }
  }else if(flag_type == 2){ /* Velocity Mode */ /* output-deg/s to counts/s */
    for(int i = 0; i < this->nDrives; i++){
      int_data[i] = (int)(raw_data[i] * this->MotorReduction * this->EncoderResolution/360.0);
    }
  }else if(flag_type == 3){ /* Torque Mode */ /* Amp to % Rated Motor Current */
    for(int i = 0; i < this->nDrives; i++){
      int_data[i] = (int)(raw_data[i] / this->MotorRatedCurrent * 1000.0);
    }
  }
}


void can_base::convert_to_double_data(double *raw_data, int *int_data, int flag_type)
{
  for(int i = 0; i < this->nDrives; i++){
      raw_data[3*i]     = (double)int_data[3*i] * (360.0/this->EncoderResolution) / this->MotorReduction;
      raw_data[3*i + 1] = (double)int_data[3*i+1] * (360.0/this->EncoderResolution) / this->MotorReduction;
      raw_data[3*i + 2] = (double)int_data[3*i+2] * this->MotorRatedCurrent / 1000.0;
  }
}

void can_base::stop_devices()
{
  byte raw_data[2] = {7, 0};
  for(int i = 0; i < this->nDrives; i++){
    send_PDO(512, this->DriveIDs[i], raw_data ,2);
    //check_answer_frame(drive_ID,2,49,2,0,1);
  }

  /* Making Sure it's stopped */
  this->NetworkManagement(NMT::RESET_NODE,0);
  this->NetworkManagement(NMT::RESET_COMMUNICATION,0);
}
