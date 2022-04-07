#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <fcntl.h>
//#include <pcan.h>
#include "can_test/can.h"

#include <unistd.h>
#include <time.h>

//#include <fcntl.h>      // for pcan     O_RDWR
//#include <libpcan.h>    // for pcan library.

#include <signal.h>
#include <iostream>
using namespace std;


/************xenomai**************/

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>

#include "rmd_can_test/rmd_can_test_node.h"




#define NSEC_PER_SEC 1000000000
unsigned int cycle_ns = 1000000; //Control Cycle 1[ms]

//Xenomai time variables
RT_TASK RT_task1;
RTIME now1, previous1; //Tread 1 cycle time
double del_time1;
double thread_time1 = 0.0; //Thread 1 cycle time
double max_time = 0.0;


int can_run = 1;

RT_TASK RT_task3;

//CAN
HANDLE can_handle = nullptr;
HANDLE can_handle1 = nullptr;


#define dps2rpm 0.166666
#define rpm2dsp 6.0000024

//loop flag(thread)
bool CAN_run = true;
//void CAN_task(void* arg);

void print_CAN(struct can_frame frame);
void catch_signal(int sig);


void can_task(void* arg);

RMD rmd;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rmd_can_test_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  signal(SIGINT, catch_signal);
  signal(SIGTERM,catch_signal);


 // rt_task_create(&RT_task1,"CAN_tesk",0,90,0);
 // printf("thread_created..\n");
 // rt_task_start(&RT_task1,&CAN_task,NULL);
 // printf("thread_started..\n");


  // CAN Setting
  printf(" Init CAN ...\n");
  /*
  can_handle = LINUX_CAN_Open("/dev/pcan32", O_RDWR);
  CAN_Init(can_handle, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
  */


  ////can_handle1 = LINUX_CAN_Open("/dev/pcan33", O_RDWR);
  ////CAN_Init(can_handle1, CAN_BAUD_1M, CAN_INIT_TYPE_ST);


  //xenomai core setting?
  cpu_set_t cpu_can;

  CPU_ZERO(&cpu_can);
  CPU_SET(3 ,&cpu_can);



 // rt_task_create(&RT_task1, "Motion_task", 0, 99, 0);
 // rt_task_create(&RT_task2, "Print_task", 0, 80, 0);
  rt_task_create(&RT_task3, "CAN_task", 0, 99, 0);
  rt_task_set_affinity(&RT_task3, &cpu_can);

 // rt_task_start(&RT_task1, &motion_task, NULL);
 // rt_task_start(&RT_task2, &print_task, NULL);
  rt_task_start(&RT_task3, &can_task, NULL);




  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }

  //CAN_run = false;
  return 0;
}


void print_CAN(struct can_frame frame){

  printf("\n");
  printf("can_id     : %#X\n",frame.can_id&=~CAN_EFF_FLAG);
  printf("can_length : %d\n",frame.can_dlc);
  printf("can_data   :\n");
  printf("word : [%c %c %c %c %c %c %c %c]\n",
         frame.data[0],
         frame.data[1],
         frame.data[2],
         frame.data[3],
         frame.data[4],
         frame.data[5],
         frame.data[6],
         frame.data[7] );
  printf("Hexa : [%X %X %X %X %X %X %X %x]\n",
         frame.data[0],
         frame.data[1],
         frame.data[2],
         frame.data[3],
         frame.data[4],
         frame.data[5],
         frame.data[6],
         frame.data[7] );
  printf("\n");
}
/*
void CAN_task(void* arg){

  unsigned int count=0;
  rt_task_set_periodic(NULL, TM_NOW, cycle_ns*1);  //1ms (mininum : 3ms)
  ROS_INFO("test");



  CAN can1("can0",_Virtual);
  CAN can2("can1",_Virtual);

  can1.CAN_initialize(_1M);
  can2.CAN_initialize(_1M);

  struct can_frame send_frame1;
  struct can_frame recv_frame;

  can1.set_can_frame(send_frame1,0x11111111,8,true);

  BYTE can_array1[8]={'H','e','l','l','o','I','D','1'};


  while (CAN_run){

    rt_task_wait_period(NULL);

    if(can2.CAN_read(recv_frame)){
      print_CAN(recv_frame);
    }

    can1.CAN_write(send_frame1, can_array1);

  }


}
*/

void can_task(void* arg) {
    TPCANMsg can_QP_msg[1]; //msgType for Can-Send
    TPCANRdMsg can_recv_msg[1]; //msgType for Can-Read

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    //CAN_Write(can_handle, &can_QP_msg[0]);
    usleep(10);

    static struct timespec pre_time;
    static struct timespec now_time;

    static struct timespec diff_time;
    int32_t micro_timediff = 0;

    int overrun_count = 0;
    int overrun_worst = 0;
    int lat_worst = 0;

    rmd.RPM_control(MOT_1_ID, 400);

    while (can_run) {

        rt_task_wait_period(NULL);
        //clock_gettime(CLOCK_MONOTONIC, &pre_time);
        // for time check //
        previous1 = rt_timer_read();

        //CAN_Write(can_handle, &can_QP_msg[0]);

        //LINUX_CAN_Read_Timeout(can_handle, &can_recv_msg[0],0);
       // usleep(10);
        /*printf("canstat:%d\n", CAN_Status(can_handle));
        printf("Frame(Read) = %08lx %02x %02x %02x %02x %02x %02x %02x %02x," "Time Stamp = %u ms\n",
                (unsigned long) can_recv_msg[0].Msg.ID,
                can_recv_msg[0].Msg.DATA[0],
                can_recv_msg[0].Msg.DATA[1],
                can_recv_msg[0].Msg.DATA[2],
                can_recv_msg[0].Msg.DATA[3],
                can_recv_msg[0].Msg.DATA[4],
                can_recv_msg[0].Msg.DATA[5],
                can_recv_msg[0].Msg.DATA[6],
                can_recv_msg[0].Msg.DATA[7],
                can_recv_msg[0].dwTime
                );

                */
        rmd.Read_RMD_Data();
        int32_t RPM = 200;
        rmd.RPM_control(MOT_1_ID, 400);
        //int32_t speed = RPM*rpm2dsp*100;

/*
        printf("[%d]\n",((uint16_t)can_recv_msg[0].Msg.DATA[6]<<8)+(uint16_t)can_recv_msg[7].Msg.DATA[3]);
        LINUX_CAN_Write_Timeout(can_handle, &can_QP_msg[0],0);
        can_QP_msg[0].MSGTYPE = MSGTYPE_STANDARD;
        can_QP_msg[0].LEN = 8; //QP;
        can_QP_msg[0].ID = 0x142; //
        can_QP_msg[0].DATA[0] = 0xA2;//0x33;
        can_QP_msg[0].DATA[1] = 0x00;//0x33;
        can_QP_msg[0].DATA[2] = 0x00;//0x22;
        can_QP_msg[0].DATA[3] = 0x00;//0x22;
        can_QP_msg[0].DATA[4] = *(uint8_t*)(&speed);//0x00;//0x11;
        can_QP_msg[0].DATA[5] = *((uint8_t*)(&speed)+1);//0x00;//0x11;
        can_QP_msg[0].DATA[6] = *((uint8_t*)(&speed)+2);//0x00;//0x00;
        can_QP_msg[0].DATA[7] = *((uint8_t*)(&speed)+3); //0x00;//0x00;
        */
        //CAN_Write(can_handle, &can_QP_msg[0]);
 /*       printf("Frame(Write) = %08lx %02x %02x %02x %02x %02x %02x %02x %02x \n",
                (unsigned long) can_recv_msg[0].Msg.ID,
                can_QP_msg[0].DATA[0],
                can_QP_msg[0].DATA[1],
                can_QP_msg[0].DATA[2],
                can_QP_msg[0].DATA[3],
                can_QP_msg[0].DATA[4],
                can_QP_msg[0].DATA[5],
                can_QP_msg[0].DATA[6],
                can_QP_msg[0].DATA[7]
                );*/
        printf("---------------------------------------\n");
       // std::cout << "________________________" << std::endl;

        //get time diff
        /*
        clock_gettime(CLOCK_MONOTONIC, &now_time);
        if ((now_time.tv_nsec - pre_time.tv_nsec) < 0) {
            diff_time.tv_sec = now_time.tv_sec - pre_time.tv_sec - 1;
            diff_time.tv_nsec = now_time.tv_nsec - pre_time.tv_nsec + 1000000000;
        }
        else {
            diff_time.tv_sec = now_time.tv_sec - pre_time.tv_sec;
            diff_time.tv_nsec = now_time.tv_nsec - pre_time.tv_nsec;
        }

        micro_timediff = (int)((diff_time.tv_sec*1000000000 + diff_time.tv_nsec)/1000);
        if(micro_timediff > cycle_ns/1000){
          overrun_count++;
        }
        if(micro_timediff > lat_worst){
          lat_worst = micro_timediff;
        }
        */
        // for time check //
        now1 = rt_timer_read();
        del_time1 = (long) (now1 - previous1); /// 1000000;
        if(del_time1 > max_time){
          max_time = del_time1;
        }
        micro_timediff = del_time1/1000;
        if(micro_timediff > cycle_ns/1000){
          overrun_count++;
        }
        lat_worst = max_time/1000;

        printf("## Start_time - End_time = [%d]micro_sec  \n## Overrun_count[%d], lat_worst[%d] \n", micro_timediff, overrun_count, lat_worst);

        //rt_task_wait_period(NULL);

    }
}

void catch_signal(int sig) {
 //signal(sig, SIG_IGN);
 printf("Program END...\n");
 cout<<"program end"<<endl;
 //char c;
 //cin>>c;
 //printf("%c",c);

 //FileSave();
 //CAN_Close(rmd.can_handle);
 rmd.close_CAN();

// rt_task_delete(&RT_task1);
//rt_task_delete(&RT_task2);
 rt_task_delete(&RT_task3);

 printf("Program END...\n");
 ros::shutdown();
 exit(0);
}
