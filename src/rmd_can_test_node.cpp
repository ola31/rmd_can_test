#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

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
RTIME now1, previous1, previous_before_sleep; //Tread 1 cycle time

double del_time1,del_time2;
double thread_time1 = 0.0; //Thread 1 cycle time
double max_time = 0.0;

int32_t micro_timediff = 0;
int32_t thread_timediff = 0;

int overrun_count = 0;
int overrun_worst = 0;
int lat_worst = 0;


int can_run = 1;
int print_run = 1;

RT_TASK RT_task3;  //task for CAN
RT_TASK RT_task4;  //task for Print

//CAN
HANDLE can_handle = nullptr;
HANDLE can_handle1 = nullptr;

#define PI      3.141592
#define dps2rpm 0.166666
#define rpm2dsp 6.0000024


void print_CAN(struct can_frame frame);
void catch_signal(int sig);


void can_task(void* arg);
void print_task(void* arg);

RMD rmd;


double dt = 1;   //1 ms
double T = 5000; //5 sec
double t = 0;

double command_posi1_g;
double present_posi1_g;

double command_posi2_g;
double present_posi2_g;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rmd_can_test_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher present_degree_pub = nh.advertise<std_msgs::Float32>("/present_degree", 1000);

  signal(SIGINT, catch_signal);
  signal(SIGTERM,catch_signal);




  // CAN Setting
  printf(" Init CAN ...\n");


  ////can_handle1 = LINUX_CAN_Open("/dev/pcan33", O_RDWR);
  ////CAN_Init(can_handle1, CAN_BAUD_1M, CAN_INIT_TYPE_ST);


  //xenomai core setting?
  cpu_set_t cpu_can;
  cpu_set_t print_can;

  CPU_ZERO(&cpu_can);
  CPU_ZERO(&print_can);

  CPU_SET(6 ,&print_can);
  CPU_SET(7 ,&cpu_can);

/*
  CPU_CLR (0, &cpu_can);
  CPU_CLR (1, &cpu_can);
  CPU_CLR (2, &cpu_can);
  CPU_CLR (3, &cpu_can);
  CPU_CLR (4, &cpu_can);
  CPU_CLR (5, &cpu_can);
*/


  rt_task_create(&RT_task3, "CAN_task", 0, 99, 0);
  rt_task_set_affinity(&RT_task3, &cpu_can);

  rt_task_create(&RT_task4, "PRINT_task", 0, 90, 0);
  rt_task_set_affinity(&RT_task4, &print_can);

  rt_task_start(&RT_task3, &can_task, NULL);
  rt_task_start(&RT_task4, &print_task, NULL);


  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    std_msgs::String msg;
    std_msgs::Float32 degree_msg;
    msg.data = "hello world";
    degree_msg.data = present_posi1_g*RAD2DEG;
    //degree_msg.data = present_posi2_g*RAD2DEG;

    chatter_pub.publish(msg);
    present_degree_pub.publish(degree_msg);

    ros::spinOnce();

    loop_rate.sleep();

  }

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


void can_task(void* arg) {
    TPCANMsg can_QP_msg[1]; //msgType for Can-Send
    TPCANRdMsg can_recv_msg[1]; //msgType for Can-Read

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

    usleep(10);

    //rmd.RPM_control(MOT_1_ID, 400);

    for(int i=0;i<30;i++){
      rmd.RPM_control(MOT_1_ID, 0);
      rmd.RPM_control(MOT_2_ID, 0);
     // rmd.RPM_control(MOT_2_ID, 0);
      usleep(500);
      rmd.Read_RMD_Data();
      rmd.Read_RMD_Data();
    //  rmd.Read_RMD_Data();
      usleep(500);
    }
    std::cout<<"just waiting"<<endl;
    std::cout<<"just waiting"<<endl;
    usleep(5000000);

    /*
    double start_posi;
    double goal_posi;
    double goal_posi_1 = 0.0  *DEG2RAD;
    double goal_posi_2 = 6*360.0 *DEG2RAD;
    double present_posi = 0;  //radian
    double command_posi;
    */


    rmd.mot1.goal_posi_1 = 0.0  *DEG2RAD;
    rmd.mot1.goal_posi_2 = 6*360.0 *DEG2RAD;

    rmd.mot2.goal_posi_1 = 0.0  *DEG2RAD;
    rmd.mot2.goal_posi_2 = 10*360.0 *DEG2RAD;


    rmd.mot1.goal_posi = rmd.mot1.goal_posi_1;
    rmd.mot2.goal_posi = rmd.mot2.goal_posi_1;

    rmd.mot1.present_posi = rmd.mot1.Encoder_Data* ENC2DEG*DEG2RAD;
    rmd.mot2.present_posi = rmd.mot2.Encoder_Data* ENC2DEG*DEG2RAD;

    rmd.mot1.start_posi = rmd.mot1.present_posi;
    rmd.mot2.start_posi = rmd.mot2.present_posi;

    rmd.RPM_control(MOT_1_ID, 0);
    rmd.RPM_control(MOT_2_ID, 0);
    usleep(2000);


    while (can_run) {
        /*time check before sleep*/
        previous_before_sleep = rt_timer_read();

        rt_task_wait_period(NULL);

        /*time check after sleep*/
        previous1 = rt_timer_read();

       ///  curtask=rt_task_self();
       /// rt_task_inquire(curtask,&curtaskinfo);

       /// rt_printf("Task name : %s \n", curtaskinfo.name);
       /// rt_printf("Task stat : %s \n", curtaskinfo.stat);
       /// rt_printf("Task pid : %d \n", curtaskinfo.pid);
       /// rt_printf("Task prio : %d \n", curtaskinfo.prio);


        rmd.Read_RMD_Data();
        rmd.Read_RMD_Data();

        ///int32_t RPM = 200;
        ///rmd.RPM_control(MOT_1_ID, 400);

        if(t<T){

          rmd.mot1.present_posi = rmd.mot1.Encoder_Data * ENC2DEG * DEG2RAD;
          rmd.mot2.present_posi = rmd.mot2.Encoder_Data * ENC2DEG * DEG2RAD;
          //command_posi = present_posi;

          rmd.mot1.command_posi = rmd.mot1.start_posi+(rmd.mot1.goal_posi - rmd.mot1.start_posi)*0.5*(1.0-cos(PI*(t/T)));
          rmd.mot2.command_posi = rmd.mot2.start_posi+(rmd.mot2.goal_posi - rmd.mot2.start_posi)*0.5*(1.0-cos(PI*(t/T)));

          rmd.Position_Control_1(MOT_1_ID,rmd.mot1.command_posi*RAD2DEG);
          rmd.Position_Control_1(MOT_2_ID,rmd.mot2.command_posi*RAD2DEG);

          ///
          //rmd.RPM_control(MOT_1_ID, 200);
          //rmd.RPM_control(MOT_2_ID, 200);
          /// rmd.Position_Control_1(MOT_1_ID,0.0);

          command_posi1_g = rmd.mot1.command_posi;  //global variable
          present_posi1_g = rmd.mot1.present_posi;  //global variable

          command_posi2_g = rmd.mot2.command_posi;  //global variable
          present_posi2_g = rmd.mot2.present_posi;  //global variable

          t +=dt;


        }
        else{  //switching goal position and t = 0
          if(abs(rmd.mot1.goal_posi - rmd.mot1.goal_posi_1)<0.0001){
            rmd.mot1.goal_posi = rmd.mot1.goal_posi_2;
            rmd.mot1.start_posi = rmd.mot1.goal_posi_1;

          }
          else{
            rmd.mot1.goal_posi = rmd.mot1.goal_posi_1;
            rmd.mot1.start_posi = rmd.mot1.goal_posi_2;

          }
          if(abs(rmd.mot2.goal_posi - rmd.mot2.goal_posi_1)<0.0001){
            rmd.mot2.goal_posi = rmd.mot2.goal_posi_2;
            rmd.mot2.start_posi = rmd.mot2.goal_posi_1;

          }
          else{
            rmd.mot2.goal_posi = rmd.mot2.goal_posi_1;
            rmd.mot2.start_posi = rmd.mot2.goal_posi_2;

          }
          //start_posi = present_posi;
          t = 0.0;


        }


        /************************/

        /* time check */
        now1 = rt_timer_read();

        del_time1 = (long) (now1 - previous1);
        del_time2 = (long) (now1-previous_before_sleep);
        if(del_time1 > max_time){
          max_time = del_time1;
        }
        micro_timediff = del_time1/1000;
        thread_timediff = del_time2/1000;
        if(micro_timediff > cycle_ns/1000){
          overrun_count++;
        }
        lat_worst = max_time/1000;
        /************************/

    }
}



void print_task(void* arg) {

    rt_task_set_periodic(NULL, TM_NOW, 100*cycle_ns);  //100us period
    usleep(10);

    while (print_run){
        rt_task_wait_period(NULL);

        printf("## Start_time - End_time = [%d]micro_sec  \n## Overrun_count[%d], lat_worst[%d] \n", micro_timediff, overrun_count, lat_worst);
        printf("moter1_posi[%f]\n",rmd.mot1.Encoder_Data*ENC2DEG);
        printf("moter2_posi[%f]\n",rmd.mot2.Encoder_Data*ENC2DEG);
        printf("Thread_DelT[%d]\n",thread_timediff);
        printf("command_posi_1 [%lf]\n",command_posi1_g*RAD2DEG);
        printf("command_posi_2 [%lf]\n",command_posi2_g*RAD2DEG);
    }
}





void catch_signal(int sig) {
 //signal(sig, SIG_IGN);
 printf("Program END...\n");
 cout<<"program end"<<endl;

 can_run = false;
 rt_task_delete(&RT_task3);
 rt_task_delete(&RT_task4);

 rmd.Motor_STOP(MOT_1_ID);
 usleep(1000);
 rmd.Motor_STOP(MOT_2_ID);
 usleep(1000);

 rmd.Motor_OFF(MOT_1_ID);
 usleep(1000);
 rmd.Motor_OFF(MOT_2_ID);
 usleep(1000);

 rmd.close_CAN();
 usleep(1000);


 printf("Program END...\n");
 ros::shutdown();
 exit(0);
}
