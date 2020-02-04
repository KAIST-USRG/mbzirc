
//#define DEBUG_MODE
//#define DEBUG_MODE_ROS_RX
//#define DEBUG_MODE_ROS_TX

// --------------------
// Variable Declaration
// --------------------


// Command Message for Speed Control
#pragma pack(1)
struct struct_t_TX_Command // Always Make 16 Bytes (+4 Bytes for Header, ID)
{
	    uint8_t     Header[2];         // 0x12, 0x34 (ROS to STM32Duino
	    uint8_t     IDs[2];            // 0x02 (Mission 2), 0x03 (Mission 3), 
                  	                 // 0x01 (UAV), 0x02 (UGV)
	
      uint8_t     FlagA;             /* Magnet;
                                        [0] Magnet Off
                                        [1] Magnet On */
};
#pragma pack()

//
// RX_UHF Message for Monitoring
#pragma pack(1)
struct struct_t_RX_Status
{
     uint8_t     Header[2];          // 0x43, 0x21 (STM32Duino to ROS)

     uint8_t     status_MAG;         /* Magnet;
                                        [0] Magnet Off
                                        [1] Magnet On */
     uint8_t     status_TSW;         /* Tactile Sensor;
                                        [0] Touched
                                        [1] Untouched */
};
#pragma pack()
