
#include "header.h"


// -------------------
// Variable Declaration
// -------------------

//
// structs
struct struct_t_TX_Command     StrTXCommand;
struct struct_t_RX_Status      StrRXStatus;

//
// Variable
uint32_t cur_time = 0;
uint32_t prev_time = 0;
int count_loop = 0;

int flag_ROS_TX_Status = -1;
int status_TSW = -1;
int status_TSW_1 = -1;
int status_TSW_2 = -1;
int status_TSW_3 = -1;
int status_TSW_4 = -1;
int status_MAG = -1;
int status_MAG_LEFT = -1; // [-1] Neutral, [0] OFF, [1] ON
int status_MAG_RIGHT = -1; // [-1] Neutral, [0] OFF, [1] ON
int time_MAG = 0;

int cur_FlagA = -1;
int prev_FlagA = -1;

// --------------------
// Function Declaration
// --------------------

void Toggle_OnOff_LED(int LED_Number, int Status) {
  static int LED_HEARTBEAT = 0; // [0] LOW,   [1] HIGH

  switch(LED_Number)
  {
    case 13: // heartbeat
      LED_HEARTBEAT = ~LED_HEARTBEAT;
      digitalWrite(13, LED_HEARTBEAT);
//      digitalWrite(PC13, Status);
      break;
  }
}

void ROS_TX(int TX_MODE) {
  int i;
  
  switch(TX_MODE)
  {
    case 1: // Magnet Status
#ifdef DEBUG_MODE_ROS_TX
      Serial.print("[TX_MODE]\t1\n");
#endif
      if (flag_ROS_TX_Status == 1)
      {
          StrRXStatus.Header[0] = 0x43;
          StrRXStatus.Header[1] = 0x21;
          StrRXStatus.status_MAG = status_MAG;
          StrRXStatus.status_TSW = status_TSW;
          Serial.write((byte*) &StrRXStatus, sizeof(struct_t_RX_Status));
          flag_ROS_TX_Status = 0;
//          Toggle_LED(0);
      
#ifdef DEBUG_MODE_ROS_TX
          Serial.println("\t[ROS (MAG Status) TX Data]");
    
          Serial.print("\t[Packet_Size]\t"); Serial.println(sizeof(struct_t_RX_Status));
          
          Serial.print("[Header(0)]\t0x"); Serial.print(StrRXStatus.Header[0], HEX);
          Serial.print("\t[Header(1)]\t0x"); Serial.print(StrRXStatus.Header[1], HEX);
          
          Serial.print("[status_MAG]\t\t0x"); Serial.print(StrRXStatus.status_MAG, HEX);
          Serial.print("[status_TSW]\t\t0x"); Serial.println(StrRXStatus.status_TSW, HEX);
          Serial.println(" ");
#endif
//        }
      }
      break;
    
    
    
    default:
#ifdef DEBUG_MODE_ROS_TX
      Serial.println("\t[TX_MODE]\tNaN");
#endif
      break;
  }
}





void ROS_RX(void) {
  static int ParsingMode = 1;
  uint8_t    TempData[sizeof(struct_t_TX_Command)];
  uint8_t    HeaderA = 0;
  uint8_t    HeaderB = 0;
#ifdef DEBUG_MODE_ROS_RX
  int        i;
#endif
  switch(ParsingMode)
  {
    
    case 1:
      if(Serial.available() >= 1)
      {
        Serial.readBytes(&TempData[0], 1);
        if(TempData[0] == 0x12)
        {
          HeaderA = TempData[0];
          ParsingMode = 2;
        }
        else
        {
          ParsingMode = 1;
        }
      }
      break;
    case 2:
      if(Serial.available() >= 1)
      {
        Serial.readBytes(&TempData[1], 1);
        if(TempData[1] == 0x34)
        {
        HeaderB = TempData[1];
          ParsingMode = 3;
        }
        else
        {
          ParsingMode = 1;
        }
      }
      break;
    case 3:
      if(Serial.available() >= 2)
      {
        Serial.readBytes(&TempData[2], 2);
        if((TempData[2] == 0x02) && (TempData[3] == 0x01))
        {
          StrTXCommand.Header[0] = HeaderA;
          StrTXCommand.Header[1] = HeaderB;
          StrTXCommand.IDs[0] = TempData[2];
          StrTXCommand.IDs[1] = TempData[3];
          ParsingMode = 4;
        }
        else
        {
          ParsingMode = 1;
        }
      }
      break;
    case 4:
      if(Serial.available() >= (sizeof(struct_t_TX_Command)-4))
      {
        Serial.readBytes(&TempData[4], (sizeof(struct_t_TX_Command)-4));
        memcpy((void *)(&StrTXCommand.FlagA), (void *)(&TempData[4]), (sizeof(struct_t_TX_Command)-4));
        cur_FlagA = StrTXCommand.FlagA;
//        Toggle_LED(1);
        ParsingMode = 1;
    
#ifdef DEBUG_MODE_ROS_RX
        Serial.println("\t[ROS (Command Message) RX Data]");
    
        Serial.print("[ParsingMode]\t"); Serial.print(ParsingMode);
        Serial.print("\t[Packet_Size]\t"); Serial.println(sizeof(struct_t_TX_Command));
         
        Serial.print("[Header(0)]\t0x"); Serial.print(StrTXCommand.Header[0], HEX);
        Serial.print("\t[Header(1)]\t0x"); Serial.print(StrTXCommand.Header[1], HEX);
        Serial.print("\t[IDs(0)]\t0x"); Serial.print(StrTXCommand.IDs[0], HEX);
        Serial.print("\t[IDs(1)]\t0x"); Serial.println(StrTXCommand.IDs[1], HEX);
          
        Serial.print("[FlagA]\t\t0x"); Serial.println(StrTXCommand.FlagA, HEX);
        Serial.println(" ");
#endif
      }
      break;
  }
}
