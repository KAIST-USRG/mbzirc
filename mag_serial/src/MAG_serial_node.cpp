
#include "DefineList.h"
#include "CommModule.h"

// my first repo^^
// my second repo***
// my third repo*^^*

/*
	git add .
	git commit -m "my first commit test"
	git push
	<ID>, <PWD>
*/

// node main loop, for ROS
int main(int argc, char** argv)
{
	init(argc, argv, "MAG_serial_node");      // node name initialization
	NodeHandle nh;                           // assign node handler

	printf("Initiate: MAG_serial_node\n");    // for debugging

// Publish Topic
	ROS_RX_MAG_Data_pub = nh.advertise<std_msgs::Bool>("/switch_on", 1);
	printf("Initiate: publish rostopic </MAG_Status>\n");   // for debugging

// Subscribe Topic
	ROS_TX_CMD_Data_sub = nh.subscribe("/magnet_on", 1, ROS_CMD_MAG_Data_Callback);
	printf("Initiate: Subscribe rostopic </ROS_CMD_MAG>\n");   // for debugging

	FdPort1 = OpenSerial(PORT1);             // Open Serial
	SerialReceive(FdPort1);                  // Serial Receive (pthread)
	Rate loop_rate(ROS_FREQ);                // setup the loop speed, [Hz]

// node loop, for ROS, check ros status, ros::ok()
	while( ok() )
	{
		UpdateCommand();

	// Serial TX part
		SerialSend(FdPort1);

		TSW_State_Publish();

		count_ros++;
		t_cur = count_ros/ROS_FREQ;

#ifdef DEBUG_MODE
		system("clear");
// status
		printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
		printf("\t[USRG] Serial data from STM32Duino\n");
		printf("[Header(0)] \t 0x%X \t[Header(1)] \t 0x%X\n", StrROS_RX_MAG_Data.Header[0], StrROS_RX_MAG_Data.Header[1]);
		printf("[Status_MAG] \t %d \t[Status_TSW] \t %d\n", StrROS_RX_MAG_Data.Status_MAG, StrROS_RX_MAG_Data.Status_TSW);
		printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
		printf("\n");
// command
		printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
		printf("\t[USRG] Serial data to STM32Duino\n");
		printf("[Header(0)] \t 0x%X \t[Header(1)] \t 0x%X\n", StrROS_TX_CMD_Data.Header[0], StrROS_TX_CMD_Data.Header[1]);
		printf("[ID(0)] \t 0x%X \t[ID(1)] \t 0x%X\n", StrROS_TX_CMD_Data.IDs[0], StrROS_TX_CMD_Data.IDs[1]);
		printf("[FlagA] \t %d\n", StrROS_TX_CMD_Data.FlagA);
		printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
		printf("\n");
#endif

// loop rate [Hz]
		loop_rate.sleep();

// loop sampling, ros
		spinOnce();
	}

// for debugging
	printf("Terminate: MAG_Serial_node\n");

	return 0;
}


void UpdateCommand(void)
{

}

void TSW_State_Publish(void)
{
	if (status_TSW == 0)
	{
		TSW_Status.data = false;
	}
	else
	{
		TSW_Status.data = true;
	}
	
    ROS_RX_MAG_Data_pub.publish(TSW_Status);
}

void ROS_CMD_MAG_Data_Callback(const std_msgs::Int32& msg_input)
{
	ROS_INFO("Get the rostopic from /magnet_on");
	StrROS_TX_CMD_Data.FlagA = (uint8_t)msg_input.data;
}

