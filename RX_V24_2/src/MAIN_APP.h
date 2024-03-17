
#ifndef __MAIN_APP__
#define __MAIN_APP__

/*======================================================
              Include Files
========================================================*/
#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
/*======================================================*/

/*======================================================
              Defines & Macros
========================================================*/
#define SCK   13
#define MOSI  11
#define MISO  12
#define CE    9
#define CSN   10
#define PINCONNECT  3
#define PinCH1 (A0)
#define PinCH2 (A1)
#define PinCH3 (A2)
#define PinCH4 (A3)
#define PinCH5 (A4)
#define PinCH6 (A5)
#define PinCH7 4
#define PinCH8 5
#define PinCH9 6
#define PinCH10 7
#define LED 2
#define VOLSENSOR (A6)

/* CONTROL */
#define LED_ON (digitalWrite(LED, HIGH))
#define LED_OFF (digitalWrite(LED, LOW))
#define READ_BIND (digitalRead(PINCONNECT))

/* Config RX Address */
#define RX_ADDRESS (39563) /* 0-65536 */
#define RF_CHANNEL (111)    /* 0-124 */

/*======================================================*/

/*======================================================
              ENUMS
========================================================*/
/*======================================================*/

/*======================================================
              TYPEDEFS & STRUCTURES
========================================================*/
typedef struct ChannelData
{
  uint8_t CH1; 
  uint8_t CH2; 
  uint8_t CH3; 
  uint8_t CH4; 
  uint8_t CH5; 
  uint8_t CH6;
  uint8_t CH7; 
  uint8_t CH8;
  uint8_t CH9;
  uint8_t CH10; 
}ChannelData_Typedef;

typedef struct 
{
  uint16_t ADD_Write;
  uint16_t ADD_Read;
  uint8_t CHANNEL;
  uint8_t dummy[3];
}Address_typedef;
/*======================================================*/

/*======================================================
              GLOBAL VARIABLES DECLARATIONS
========================================================*/
/*======================================================*/

/*======================================================
              GLOBAL FUNCTION DECLARATIONS
========================================================*/
void APP_Init(void);
void APP_Main(void);
/*======================================================*/

#endif /* __MAIN_APP__ */


