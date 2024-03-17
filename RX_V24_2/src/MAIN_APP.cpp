
#include "MAIN_APP.h"

RF24 radio(CE, CSN);
static Address_typedef ADDRESS;
ChannelData_Typedef DataRead;
Servo OUTCH1;
Servo OUTCH2;
Servo OUTCH3;
Servo OUTCH4;
Servo OUTCH5;
Servo OUTCH6;
Servo OUTCH7;
Servo OUTCH8;
Servo OUTCH9;
Servo OUTCH10;
static uint8_t BIND=false;

static void LED_POWERON(void)
{
  for(int i=0; i<=2; i++)
  {
    LED_ON;
    delay(200);
    LED_OFF;
    delay(200);
  }
}

static void F_PrintDataRead(void)
{
  char DataPrint[100];
  sprintf(DataPrint,"CH1: %d - CH2: %d - CH3: %d - CH4: %d - CH5: %d - CH6: %d - CH7: %d - CH8: %d - CH9: %d - CH10: %d \n", 
          DataRead.CH1, DataRead.CH2, DataRead.CH3, DataRead.CH4, DataRead.CH5, \
          DataRead.CH6, DataRead.CH7, DataRead.CH8, DataRead.CH9, DataRead.CH10);
  Serial.print(DataPrint);
}

static void F_SaveData(Address_typedef *AddressSave)
{
  EEPROM.put(0, *AddressSave);
  delay(1);
}

static void F_ReadData(Address_typedef *AddressRead)
{
  EEPROM.get(0, *AddressRead);
  delay(1);

  Serial.print("C: "); Serial.print(AddressRead->CHANNEL);
  Serial.print(" - R: "); Serial.print(AddressRead->ADD_Read);
  Serial.print(" - W: "); Serial.println(AddressRead->ADD_Write);
}

static void F_ServoInit(void)
{
  OUTCH1.attach(PinCH1,500,2500);
  OUTCH2.attach(PinCH2,500,2500);

  OUTCH3.attach(PinCH3,1000,2000);

  OUTCH4.attach(PinCH4,500,2500);
  OUTCH5.attach(PinCH5,500,2500);
  OUTCH6.attach(PinCH6,500,2500);
  OUTCH7.attach(PinCH7,500,2500);
  OUTCH8.attach(PinCH8,500,2500);
}

static void F_ControlServo(void)
{
  OUTCH1.write( (int)map(DataRead.CH1,0,255,0,180) );
  OUTCH2.write( (int)map(DataRead.CH2,0,255,0,180) );
  OUTCH3.write( (int)map(DataRead.CH3,0,255,0,180) );
  OUTCH4.write( (int)map(DataRead.CH4,0,255,0,180) );
  OUTCH5.write( (int)map(DataRead.CH5,0,255,0,180) );
  OUTCH6.write( (int)map(DataRead.CH6,0,255,0,180) );
  OUTCH7.write( (int)map(DataRead.CH7,0,255,0,180) );

  if( DataRead.CH8<50 ) /* 180' */
  {
    OUTCH8.write(180);
  }
  else if( DataRead.CH8>=180 && DataRead.CH8<=230 ) /* 90' */
  {
    OUTCH8.write(90);
  }
  else /* 0' */
  {
    OUTCH8.write(0);
  }
  
}

static void F_RF_READ(void)
{
  // static uint8_t StepSend=0;
  // static uint8_t StepRead=0;
  // static uint8_t Send=0;
  // static uint8_t Read=0;
  // static uint32_t LastTick=0;
  // if( (uint32_t)(micros()-LastTick)>=50000 ) /* 10ms- SendData */
  // {
  //   Send=1;
  //   LastTick=micros();
  // }

  // if(Send==1)
  // {
  //   radio.stopListening();
  //   radio.write(&ADDRESS, sizeof(ADDRESS));
  //   Send=0;
  // }

  // radio.startListening();
  if (radio.available()) 
  {
    radio.read(&DataRead, sizeof(DataRead));
    // F_PrintDataRead();
  }

  F_ControlServo();

}

void APP_Init(void)
{
  Serial.begin(115200);
  pinMode(VOLSENSOR,INPUT);
  pinMode(PINCONNECT, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  F_ServoInit();

  F_ReadData(&ADDRESS);

  while (!radio.begin()) {LED_ON;} 
  LED_OFF;

  if( READ_BIND==0 ) /* Bind mode */
  {
    radio.setAutoAck(false);
    radio.openWritingPipe((uint64_t)11111);
    radio.openReadingPipe(1,(uint64_t)33444);
    radio.setChannel(111);   
    radio.setPALevel(RF24_PA_MIN); 
    radio.setDataRate(RF24_250KBPS); 
    radio.startListening();  

    BIND=true;
  }
  else 
  {
    // radio.setAutoAck(true);
    // radio.openWritingPipe((uint64_t)ADDRESS.ADD_Read);
    radio.openReadingPipe(1, (uint64_t)ADDRESS.ADD_Write);
    radio.setChannel(ADDRESS.CHANNEL);
    radio.setPALevel(RF24_PA_MAX);                   
    radio.setDataRate(RF24_250KBPS); 
    radio.startListening();
  }

  while (BIND==true)
  {
    static bool LEDSTATE=0;
    static uint8_t STEP=0;
    static uint32_t lasttick=0;
    static Address_typedef INFO_READ;

    if(STEP==0)
    {
      memset(&INFO_READ,0,sizeof(INFO_READ));
      STEP=1;
    }
    else if(STEP==1)
    {
      if( (uint32_t)(millis()-lasttick)>=200 )
      {
        LEDSTATE=!LEDSTATE;
        lasttick=millis();
      }
      if(LEDSTATE==0) LED_ON;
      else LED_OFF;

      if(radio.available()>0)
      {
        radio.read(&INFO_READ, sizeof(INFO_READ));
        Serial.print("NC: "); Serial.print(INFO_READ.CHANNEL);
        Serial.print(" - NR: "); Serial.print(INFO_READ.ADD_Read);
        Serial.print(" - NW: "); Serial.println(INFO_READ.ADD_Write);        

        if( (INFO_READ.CHANNEL>0&&INFO_READ.CHANNEL<=120) && \
            (INFO_READ.ADD_Read>0&&INFO_READ.ADD_Read<=65535) && \
            (INFO_READ.ADD_Write>0&&INFO_READ.ADD_Write<=65535))
        {
          ADDRESS.CHANNEL=INFO_READ.CHANNEL;
          ADDRESS.ADD_Read=INFO_READ.ADD_Read;
          ADDRESS.ADD_Write=INFO_READ.ADD_Write;

          F_SaveData(&ADDRESS);                    

          Serial.print("NC: "); Serial.print(ADDRESS.CHANNEL);
          Serial.print(" - NR: "); Serial.print(ADDRESS.ADD_Read);
          Serial.print(" - NW: "); Serial.println(ADDRESS.ADD_Write);

          STEP=2;
        }
      }
    }
    else
    {
      LED_ON;
      radio.stopListening();
      radio.write(&ADDRESS, sizeof(ADDRESS)); 
      // BIND=false;

      delay(5);
    }

  }

  LED_POWERON();

}

void APP_Main(void)
{
  F_RF_READ();

  // Serial.println(analogRead(VOLSENSOR));
  // delay(500);
}

