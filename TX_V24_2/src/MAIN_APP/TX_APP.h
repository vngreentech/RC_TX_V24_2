
#ifndef __TX_APP__
#define __TX_APP__

/*======================================================
              Include Files
========================================================*/
#include <Arduino.h>
#include <EEPROM.h>
#include <PCF8574.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "randomHelpers.h"
#include <SimpleKalmanFilter.h>
/*======================================================*/

/*======================================================
              Defines & Macros
========================================================*/

/* NRF24L01 - SPI PIN */
#define MOSI (PA7)
#define MISO (PA6)
#define SCK  (PA5)
#define CE   (PB10)
#define CSN  (PB11)

/* PCF8574 - I2C PIN */
#define SDA (PB7)
#define SCL (PB6)

/* 4CH RC */
#define CH1_AIL (A1)
#define CH2_ELE (A0)
#define CH3_THR (A2)
#define CH4_RUD (A3)

/* Switch & Pontentionmeter */
#define CH5_SW      (PB5)
#define CH6_PON1    (PA4)
#define CH7_PON2    (PB0)
#define CH8_SW3POS  (PB1)

#define SW1 (PA11)
#define SW2 (PA12)
#define SW3 (PB12)
#define SW4 (PB13)
#define SW5 (PB14)
#define SW6 (PB15)

#define COI           (PB3)
#define BUTTON_BIND   (PB9)
#define LED           (PC13)

/* Print Data */
// #define PRINT_BUTTON_TRIM
// #define PRINT_SW_CONFIG
// #define PRINT_CHANNEL
// #define PRINT_DATA_SEND

/* Control */
#define LED_ON  ( digitalWrite(LED,HIGH) )
#define LED_OFF ( digitalWrite(LED,LOW) )
#define COI_ON  ( digitalWrite(COI,LOW) )
#define COI_OFF ( digitalWrite(COI,HIGH) )

/* Trip Setup */
#define LONG_MIN (0UL)
#define LONG_MID (127UL)
#define LONG_MAX (255UL)
#define SHORT_MIN (50UL)
#define SHORT_MID (127UL)
#define SHORT_MAX (205UL)

/*======================================================*/

/*======================================================
              ENUMS
========================================================*/
/*======================================================*/

/*======================================================
              TYPEDEFS & STRUCTURES
========================================================*/

typedef struct ButtonTrim
{
  uint8_t Button1; //CH1-
  uint8_t Button2; //Ch1+
  uint8_t Button3; //CH2-
  uint8_t Button4; //CH2+
  uint8_t Button5; //CH3-
  uint8_t Button6; //CH3+
  uint8_t Button7; //CH4-
  uint8_t Button8; //CH4+
}ButtonTrim_typedef;

typedef struct SwitchConfig
{
  uint8_t Switch1;
  uint8_t Switch2;
  uint8_t Switch3;
  uint8_t Switch4;
  uint8_t Switch5;
  uint8_t Switch6;
  uint8_t ButtonBind;
}SwitchConfig_typedef;

typedef struct ChannelRead
{
  uint16_t CH1_Ali;
  uint16_t CH2_Ele;
  uint16_t CH3_Thr;
  uint16_t CH4_Rud;
  uint8_t  CH5_Switch1;
  uint16_t CH6_Pontentionmeter1;
  uint16_t CH7_Pontentionmeter2;
  uint8_t  CH8_Switch3Pos;
}ChannelRead_typedef;

typedef struct
{
  uint16_t CH1_MIN;
  uint16_t CH1_Mid;
  uint16_t CH1_MAX;

  uint16_t CH2_MIN;
  uint16_t CH2_Mid;
  uint16_t CH2_MAX;

  uint16_t CH3_MIN;
  uint16_t CH3_Mid;
  uint16_t CH3_MAX;

  uint16_t CH4_MIN;
  uint16_t CH4_Mid;
  uint16_t CH4_MAX;

  uint16_t CH8_MIN;
  uint16_t CH8_Mid;
  uint16_t CH8_MAX;
}ChannelLimit_typedef;

typedef struct DataSend
{
  uint8_t CH1; /* Ali */
  uint8_t CH2; /* Ele */
  uint8_t CH3; /* Thro */
  uint8_t CH4; /* Rud */
  uint8_t CH5; /* Switch on/off */
  uint8_t CH6; /* Pontentionmeter 1 */
  uint8_t CH7; /* Pontentionmeter 2 */
  uint8_t CH8; /* Switch 3 pos */
}DataSend_typedef;

typedef struct 
{
  uint16_t V_Trim_CH1;
  uint16_t V_Trim_CH2;
  uint16_t V_Trim_CH3;
  uint16_t V_Trim_CH4;  
}ValueTrim_typedef;

typedef struct 
{
  uint16_t ADD_Write;
  uint16_t ADD_Read;
  uint8_t CHANNEL;
  uint8_t dummy[3];  
}Address_typedef;

typedef struct 
{
  uint8_t Check_Save;
  Address_typedef ADDRESS;
  ChannelLimit_typedef CH_Limit;
  ValueTrim_typedef Value_Trim;
  uint16_t CRC_CheckSum;
}ConfigMachine_typedef;

typedef struct 
{
  uint8_t ID1;
  uint16_t ID2;
  uint16_t ID3;
}UUID_typedef;
#define UID ((UUID_typedef*)0x1FFFF7E8UL)

/*======================================================*/

/*======================================================
              GLOBAL VARIABLES DECLARATIONS
========================================================*/
extern DataSend_typedef DATA_SEND;
/*======================================================*/

/*======================================================
              GLOBAL FUNCTION DECLARATIONS
========================================================*/
void F_Init(void);
void F_Main(void);
/*======================================================*/

#endif /* __TX_APP__ */


