
#ifndef DefineList_H
#define DefineList_H

// essential header for ROS
#include <ros/ros.h>

// for using serial communication
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>

// for topic message
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
std_msgs::Int32 TSW_Status;

// setup the initial name
using namespace ros;
using namespace std;

//#define DEBUG_MODE
//#define DEBUG_MODE_Agent_List
#define ROS_FREQ  5.0    // while loop frequency [Hz]

#define PORT1 		"/dev/ttyACM0"
#define BAUDRATE 	115200

#define D2R	        3.1415926535/180.0
#define eps             0.000001


int   FdPort1;
int   count_ros = 0;
float t_cur     = 0.0;

int   status_MAG = -1;
int   status_TSW = -1;


// ROS topic define
ros::Publisher ROS_RX_TSW_Data_pub;
ros::Subscriber ROS_TX_CMD_Data_sub;

float satmax(float, float);
float satmin(float, float);
void  CalChecksumAB(void *, int32_t, uint8_t *, uint8_t *);

int   OpenSerial(char* device_name);
void  SerialSend(int fd);
void  SerialReceive(int FdPort1);
void* receive_p_thread(void *fd);

void  UpdateCommand(void);
void  TSW_State_Publish(void);
void  ROS_CMD_MAG_Data_Callback(const std_msgs::Int32& msg_input);

#pragma pack(1)
struct struct_ROS_TX_CMD_Data
{
    // -------------
    // Packet Header
    // -------------
    uint8_t  Header[2];           // 0x12, 0x34 (ROS to STM32Duino
    uint8_t  IDs[2];              // 0x02 (Mission 2), 0x03 (Mission 3), 
          	                  // 0x01 (UAV), 0x02 (UGV)
	
    uint8_t  FlagA;               /* Magnet;
                                    [0] Magnet Off
                                    [1] Magnet On */
};
#pragma pack()

#pragma pack(1)
struct struct_ROS_RX_MAG_Data
{
    // -------------
    // Packet Header
    // -------------
    uint8_t  Header[2];      	  // 0x43, 0x21 (STM32Duino to ROS)
    uint8_t  Status_MAG;          /* Magnet;
                                    [0] Magnet Off
                                    [1] Magnet On */
    uint8_t  Status_TSW;          /* Tactile Sensor;
                                    [0] Touched
                                    [1] Untouched */
};
#pragma pack()


struct struct_ROS_TX_CMD_Data  StrROS_TX_CMD_Data;
struct struct_ROS_RX_MAG_Data  StrROS_RX_MAG_Data;


float satmax(float data, float max)
{
    float res;

    if(fabs(data) > max)
        res = (data + eps)/fabs(data + eps)*max;
    else
        res = data;

    return res;
}


float satmin(float data, float min)
{
    float res;

    if(fabs(data) < min)
        res = (data + eps)/fabs(data + eps)*min;
    else
        res = data;

    return res;
}

void CalChecksumAB(void * PtrStartAddr, int32_t Size, uint8_t * PtrChecksumA, uint8_t * PtrChecksumB)
{
	int32_t Ind;
	uint8_t * Ptr = (uint8_t *)PtrStartAddr;

	uint8_t ChecksumA = 0;
	uint8_t ChecksumB = 0;

	for(Ind = 0; Ind<Size; Ind++)
	{
		ChecksumA += Ptr[Ind];
		ChecksumB += ChecksumA;
	}

	*PtrChecksumA = ChecksumA;
	*PtrChecksumB = ChecksumB;
}



#endif
