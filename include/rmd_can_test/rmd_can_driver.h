#ifndef RMD_CAN_DRIVER_H
#define RMD_CAN_DRIVER_H

#include <iostream>
#include <stdio.h>

#include <fcntl.h>      // for pcan     O_RDWR
#include <libpcan.h>    // for pcan library.
#include <stdint.h>


#define DPS2RPM       0.166666
#define RPM2DSP       6.0000024

#define DEG2LSP       100

#define ENCODER_RANGE 65535      // 16bit encoder 0~65535
#define DEG2ENC       182.0444   // 65536 / 360(deg)
#define ENC2DEG       0.0005493  // 360 / 65536
#define DEG2RAD       0.017453   // 180 / PI
#define RAD2DEG       57.29578   // PI / 180

#define MOTOR_OFF_COMMAND                                   0X80
#define MOTOR_STOP_COMMAND                                  0X81
#define MOTOR_RUNNING_COMMAND                               0X88
#define SPEED_CLOESED_LOOP_COMMAND                          0XA2
#define POSITION_CLOSED_LOOP_COMMAND_1                      0XA3
#define POSITION_CLOSED_LOOP_COMMAND_2                      0xA4
#define POSITION_CLOSED_LOOP_COMMAND_3                      0xA5
#define POSITION_CLOSED_LOOP_COMMAND_4                      0xA6
#define MULTITURN_INTRIMENTAL_POSITION_CONTROL_COMMAND_1    0XA7
#define MULTITURN_INTRIMENTAL_POSITION_CONTROL_COMMAND_2    0XA8

#define MOT_1_ID 0x142


class RMD
{

public :

  RMD();
  RMD(char* can_port);
  ~RMD();
  void close_CAN();
  void Motor_OFF(int motor_id);
  void Motor_STOP(int motor_id);
  void Motor_RUN(int motor_id);
  void RPM_control(int moter_id, int32_t rpm);
  void Read_RMD_Data();
  void Position_Control_1(int motor_id, int position_degree);

private:

  HANDLE can_handle = nullptr;

};


#endif // RMD_CAN_DRIVER_H
