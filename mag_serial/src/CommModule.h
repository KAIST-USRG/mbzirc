
#include "DefineList.h"

using namespace ros;

int OpenSerial(char *device_name)
{
    int fd;
    struct termios newtio;

    fd = open(device_name, O_RDWR | O_NOCTTY);

    if(fd < 0)
    {
        printf("Serial Port Open Fail.\n");
        return -1;
    }

    memset(&newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_cflag = CS8|CLOCAL|CREAD;

    switch(BAUDRATE)
    {
        case 921600 : newtio.c_cflag |= B921600;
        break;
        case 115200 : newtio.c_cflag |= B115200;
        break;
        case 57600  : newtio.c_cflag |= B57600;
        break;
    }

    newtio.c_lflag 	= 0;
    newtio.c_cc[VTIME] 	= 0;
    newtio.c_cc[VMIN] 	= (sizeof(struct_ROS_RX_MAG_Data));

    tcflush(fd,TCIFLUSH);
    tcsetattr(fd,TCSANOW, &newtio);

    return fd;
}



void SerialSend(int fd)
{
    //===== initial header =====//
    StrROS_TX_CMD_Data.Header[0] = 0x12;
    StrROS_TX_CMD_Data.Header[1] = 0x34;

    StrROS_TX_CMD_Data.IDs[0] = 0x02;
    StrROS_TX_CMD_Data.IDs[1] = 0x01;

    write(fd,&StrROS_TX_CMD_Data,sizeof(struct_ROS_TX_CMD_Data));
}



void SerialReceive(int FdPort1)
{
    // pthread create
    pthread_t p_thread[2];
    int thread_rx;

    thread_rx = pthread_create(&p_thread[0], NULL, receive_p_thread, (void *)FdPort1);

    if(thread_rx < 0)
    {
        perror("thread create error : ");
        exit(0);
    }
}



void* receive_p_thread(void *fdt)
{
    int fd = *((int*)&fdt);
    unsigned char RXRawData[sizeof(struct_ROS_RX_MAG_Data)];

    printf("pthread RX process start!\n\n");
    while( ok() )
    {
        int ParsingMode   = 1;
        int ContinueWhile = 1;
        while(ContinueWhile)
        {
            switch(ParsingMode)
            {
            case 1:
                if(read((int)fd, &RXRawData[0], 1) == 1)
                {
                    if(RXRawData[0] == 0x43)
                    {
			            StrROS_RX_MAG_Data.Header[0] = 0x43;
                        ParsingMode = 2;
                    }
                    else
                        ParsingMode = 1;
                }
                break;

            case 2:
                if(read((int)fd, &RXRawData[1], 1) == 1)
                {
                    if(RXRawData[1] == 0x21)
                    {
			            StrROS_RX_MAG_Data.Header[1] = 0x21;
                        ParsingMode = 3;
                    }
                    else
                        ParsingMode = 1;
                }
                break;

            case 3:
                if(read((int)fd, &RXRawData[2], 1) == 1)
                {
                    status_MAG = RXRawData[2];
		            StrROS_RX_MAG_Data.Status_MAG = status_MAG;
                    ParsingMode = 4;
                }
                else
                    ParsingMode = 1;

            case 4:
                if(read((int)fd, &RXRawData[3], 1) == 1)
                {
                    status_TSW = RXRawData[3];
		            StrROS_RX_MAG_Data.Status_TSW = status_TSW;
                }
                ParsingMode = 1;

                break;

            default:
                ParsingMode = 1;
                break;
            }
        }

    }

}


