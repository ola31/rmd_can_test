#include "rmd_can_test/rmd_can_driver.h"

RMD::RMD(){
  can_handle = LINUX_CAN_Open("/dev/pcan32", O_RDWR);
  CAN_Init(can_handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
}

RMD::RMD(char* can_port){
  can_handle = LINUX_CAN_Open(can_port, O_RDWR);
  CAN_Init(can_handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
}
RMD::~RMD(){

}

void RMD::close_CAN(){
  double result = CAN_Close(this->can_handle);
  printf("can close result = %d \n",result);
}

void RMD::Motor_OFF(int motor_id){

  /*
   * MOTOR_OFF_COMMAND
   *
   * Turn off the motor,
   * and clear the running state of the motor
   * and the previously received control commands
   * at the same time.
   */

  TPCANMsg can_msg[1];
  can_msg[0].LEN = 8;
  can_msg[0].ID = 0x142; //
  can_msg[0].DATA[0] = MOTOR_OFF_COMMAND; //0x80
  can_msg[0].DATA[1] = 0x00;
  can_msg[0].DATA[2] = 0x00;
  can_msg[0].DATA[3] = 0x00;
  can_msg[0].DATA[4] = 0x00;
  can_msg[0].DATA[5] = 0x00;
  can_msg[0].DATA[6] = 0x00;
  can_msg[0].DATA[7] = 0x00;

  LINUX_CAN_Write_Timeout(this->can_handle, &can_msg[0],0);
}


void RMD::Motor_STOP(int motor_id){

  /*
   * MOTOR_STOP_COMMAND
   *
   * Stop the motor,
   * but do not clear the running state of the motor
   * and the previously received controlcommands.
   */

  TPCANMsg can_msg[1];
  can_msg[0].LEN = 8;
  can_msg[0].ID = 0x142; //
  can_msg[0].DATA[0] = MOTOR_STOP_COMMAND; //0x81
  can_msg[0].DATA[1] = 0x00;
  can_msg[0].DATA[2] = 0x00;
  can_msg[0].DATA[3] = 0x00;
  can_msg[0].DATA[4] = 0x00;
  can_msg[0].DATA[5] = 0x00;
  can_msg[0].DATA[6] = 0x00;
  can_msg[0].DATA[7] = 0x00;

  LINUX_CAN_Write_Timeout(this->can_handle, &can_msg[0],0);
}

void RMD::Motor_RUN(int motor_id){

  /*
   * MOTOR_RUNNING_COMMAND
   *
   * Resume motor operation from the motor stop command (recover the control mode before the stop).
   * (recover the control mode before the stop).
   *
   */

  TPCANMsg can_msg[1];
  can_msg[0].LEN = 8;
  can_msg[0].ID = 0x142; //
  can_msg[0].DATA[0] = MOTOR_RUNNING_COMMAND; //0x88
  can_msg[0].DATA[1] = 0x00;
  can_msg[0].DATA[2] = 0x00;
  can_msg[0].DATA[3] = 0x00;
  can_msg[0].DATA[4] = 0x00;
  can_msg[0].DATA[5] = 0x00;
  can_msg[0].DATA[6] = 0x00;
  can_msg[0].DATA[7] = 0x00;

  LINUX_CAN_Write_Timeout(this->can_handle, &can_msg[0],0);
}

void RMD::Position_Control_1(int motor_id, double position_degree){

  int32_t position = DEG2LSP*position_degree;

  TPCANMsg can_msg[1];
  can_msg[0].LEN = 8;
  can_msg[0].ID = motor_id;//0x142;
  can_msg[0].DATA[0] = POSITION_CLOSED_LOOP_COMMAND_1; //0xA3
  can_msg[0].DATA[1] = 0x00;
  can_msg[0].DATA[2] = 0x00;
  can_msg[0].DATA[3] = 0x00;
  can_msg[0].DATA[4] = *(uint8_t*)(&position);
  can_msg[0].DATA[5] = *((uint8_t*)(&position)+1);
  can_msg[0].DATA[6] = *((uint8_t*)(&position)+2);
  can_msg[0].DATA[7] = *((uint8_t*)(&position)+3);

  LINUX_CAN_Write_Timeout(this->can_handle, &can_msg[0],0);

}


void RMD::RPM_control(int moter_id, int32_t rpm){
  int32_t speed = rpm * RPM2DSP * DEG2LSP; // 0.01 deg/sec = 1;

  TPCANMsg can_msg[1];
  can_msg[0].LEN = 8;
  can_msg[0].ID = moter_id;
  can_msg[0].DATA[0] = SPEED_CLOESED_LOOP_COMMAND;  //0xA2;
  can_msg[0].DATA[1] = 0x00;
  can_msg[0].DATA[2] = 0x00;
  can_msg[0].DATA[3] = 0x00;
  can_msg[0].DATA[4] = *(uint8_t*)(&speed);
  can_msg[0].DATA[5] = *((uint8_t*)(&speed)+1);
  can_msg[0].DATA[6] = *((uint8_t*)(&speed)+2);
  can_msg[0].DATA[7] = *((uint8_t*)(&speed)+3);

  double result = LINUX_CAN_Write_Timeout(this->can_handle, &can_msg[0],0);
  ///printf("can_write[%lf]\n",result);
}

void RMD::Read_RMD_Data(){

  TPCANRdMsg can_recv_msg[1];
  LINUX_CAN_Read_Timeout(this->can_handle, &can_recv_msg[0],0); //10 usec


  //printf("canstat:%d\n", CAN_Status(can_handle));
  //printf("Frame(Read) = %08lx %02x %02x %02x %02x %02x %02x %02x %02x," "Time Stamp = %u ms\n",
    //      (unsigned long) can_recv_msg[0].Msg.ID,
    //      can_recv_msg[0].Msg.DATA[0],
    //      can_recv_msg[0].Msg.DATA[1],
    //      can_recv_msg[0].Msg.DATA[2],
    //      can_recv_msg[0].Msg.DATA[3],
    //      can_recv_msg[0].Msg.DATA[4],
    //      can_recv_msg[0].Msg.DATA[5],
    //      can_recv_msg[0].Msg.DATA[6],
    //      can_recv_msg[0].Msg.DATA[7],
    //      can_recv_msg[0].dwTime
    //      );
    /// printf("[%d]\n",((uint16_t)can_recv_msg[0].Msg.DATA[6]<<8)+(uint16_t)can_recv_msg[0].Msg.DATA[7]);
   Encoder_Data = (uint16_t)can_recv_msg[0].Msg.DATA[6]+((uint16_t)can_recv_msg[0].Msg.DATA[7]<<8);
}

//RMD::Position_Control(int moter_id, )

