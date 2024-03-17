/*======================================================
              Include Files
========================================================*/
#include "TX_APP.h"
/*======================================================*/

/*======================================================
        STATIC VARIABLES & CLASS DECLARATION
========================================================*/
RF24 radio(CE, CSN);
PCF8574 PCF(0x20, &Wire);
HardwareTimer Timer_Read_Switch(TIM2);
HardwareTimer Timer_Channel(TIM3);
HardwareTimer Timer_SendData(TIM4);

static volatile uint8_t Flag_Read_Switch=false;
static volatile uint8_t Flag_Read_Channel=false;
static volatile uint8_t Flag_Send_Data=false;
static volatile ButtonTrim_typedef BUTTON_TRIM;
static volatile SwitchConfig_typedef Switch_Config;
static volatile ChannelRead_typedef Read_Channel;
static volatile ChannelLimit_typedef Limit_Channel;
static ConfigMachine_typedef Config_Machine;
static volatile ValueTrim_typedef Last_Value_Trim;
static volatile ValueTrim_typedef New_Value_Trim;

static uint8_t Config_MIN_MAX=false;
static volatile uint8_t READ_CH1, READ_CH2;
static uint8_t RF_BIND=false;

static bool TRIP_SELECT=false;
static uint8_t TRIP_MIN;
static uint8_t TRIP_MAX;

static SimpleKalmanFilter KALMAN_CH1(4, 2, 0.01);
static SimpleKalmanFilter KALMAN_CH2(4, 2, 0.01);
static SimpleKalmanFilter KALMAN_CH3(4, 2, 0.01);
static SimpleKalmanFilter KALMAN_CH4(4, 2, 0.01);

/*======================================================*/

/*======================================================
        GLOBAL VARIABLES & CLASS DECLARATION
========================================================*/
DataSend_typedef DATA_SEND;
/*======================================================*/

/*======================================================
            FUNCTIONS PROTOTYPE
========================================================*/
void ISR_Read_Switch(void);
void ISR_Read_Channel(void);
static void F_ReadDataChannel(void);
static void F_ReadSwitchConfig(void);
static void F_Save_Config_Machine(ConfigMachine_typedef *ConfigSave);
/*======================================================*/

/*======================================================
             BUZZER FUNCTIONS
========================================================*/
static void COI_LED_POWER_ON(void)
{
  LED_ON;
  COI_ON;
  delay(100);
  LED_OFF;
  COI_OFF;
  delay(100);
  
  LED_ON;
  COI_ON;
  delay(100);
  LED_OFF;
  COI_OFF;
  delay(100);
}

static void COI_SAVE_OK(void)
{
  LED_ON;
  COI_ON;
  delay(100);
  LED_OFF;
  COI_OFF;
  delay(100);
  LED_ON;
  COI_ON;
  delay(100);
  LED_OFF;
  COI_OFF;
  delay(100);  

  LED_ON;
  COI_ON;
  delay(1000);
  LED_OFF;
  COI_OFF; 
}

static void COI_SAVE_ERR(void)
{
  for(int i=0; i<=5; i++)
  {
    LED_ON;
    COI_ON;
    delay(100);
    LED_OFF;
    COI_OFF;
    delay(100);
  }
}

static void COI_BIND_OK(void)
{
  for(int i=0; i<=3; i++)
  {
    LED_ON;
    COI_ON;
    delay(100);
    LED_OFF;
    COI_OFF;  
    delay(100);    
  }

  LED_ON;
  COI_ON;
  delay(1000);
  LED_OFF;
  COI_OFF; 
}

/*======================================================*/

/*======================================================
              LORA FUNCTIONS
========================================================*/
static void RF_INIT(ConfigMachine_typedef *MachineConfig)
{
  while (!radio.begin()) 
  {
    LED_ON;
    COI_ON;
  } 
  LED_OFF;
  COI_OFF;                               

  if( digitalRead(BUTTON_BIND)==0 ) /* Bind mode */
  {
    Timer_SendData.pause();

    radio.setAutoAck(false);
    radio.openWritingPipe(33444);
    radio.openReadingPipe(1, 11111);
    radio.setChannel(111);    
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_250KBPS);       
    radio.stopListening();                                   

    RF_BIND=true;
  }
  else 
  {
    radio.setAutoAck(true);
    radio.openWritingPipe((uint64_t)MachineConfig->ADDRESS.ADD_Write);
    // radio.openReadingPipe(1, (uint64_t)MachineConfig->ADDRESS.ADD_Read);
    radio.setChannel(MachineConfig->ADDRESS.CHANNEL);                  
    radio.setDataRate(RF24_250KBPS); 
    radio.setPALevel(RF24_PA_MAX); 
    // radio.setPALevel(RF24_PA_MIN); 
    radio.stopListening();
  }
  
  while (RF_BIND==true)
  {
    static bool LEDSTATE=0;
    static uint8_t STEP=0;
    static Address_typedef NEW_ADDRESS; /* Data send */
    static Address_typedef INFO_READ;
    static uint32_t lasttick=0;

    if(STEP==0)
    {
      NEW_ADDRESS.ADD_Write = MachineConfig->ADDRESS.ADD_Write;
      NEW_ADDRESS.ADD_Read = MachineConfig->ADDRESS.ADD_Read;
      NEW_ADDRESS.CHANNEL = MachineConfig->ADDRESS.CHANNEL;

      // Serial.println(NEW_ADDRESS.ADD_Read);
      // Serial.println(NEW_ADDRESS.ADD_Write);
      // Serial.println(NEW_ADDRESS.CHANNEL);      

      STEP=1;
    }
    else if(STEP==1)
    {
      if( (uint32_t)(millis()-lasttick)>=200 )
      {
        LEDSTATE=!LEDSTATE;
        lasttick=millis();
      }
      if(LEDSTATE==0)
      {
        COI_ON;
        LED_ON;
      }
      else
      {
        COI_OFF;
        LED_OFF;
      }

      radio.stopListening(); //Ngưng nhận
      radio.write(&NEW_ADDRESS, sizeof(NEW_ADDRESS));

      radio.startListening(); //Bắt đầu nhận
      if(radio.available())
      {
        radio.read(&INFO_READ, sizeof(INFO_READ));
          Serial.print("NC: "); Serial.print(INFO_READ.CHANNEL);
          Serial.print(" - NR: "); Serial.print(INFO_READ.ADD_Read);
          Serial.print(" - NW: "); Serial.println(INFO_READ.ADD_Write);             

        if( INFO_READ.CHANNEL==NEW_ADDRESS.CHANNEL && \
            INFO_READ.ADD_Read==NEW_ADDRESS.ADD_Read && \
            INFO_READ.ADD_Write==NEW_ADDRESS.ADD_Write)
        {
          STEP=2;
        }
      }
      delay(3);
    }
    else 
    {
      radio.setAutoAck(false);
      radio.openWritingPipe((uint64_t)MachineConfig->ADDRESS.ADD_Write);
      radio.openReadingPipe(1, (uint64_t)MachineConfig->ADDRESS.ADD_Read);
      radio.setChannel(MachineConfig->ADDRESS.CHANNEL);                  
      radio.setDataRate(RF24_250KBPS); 
      radio.setPALevel(RF24_PA_MAX,1); 
      radio.stopListening(); 

      Timer_SendData.resume();

      // Serial.print("NC: "); Serial.print(MachineConfig->ADDRESS.CHANNEL);
      // Serial.print(" - NR: "); Serial.print(MachineConfig->ADDRESS.ADD_Read);
      // Serial.print(" - NW: "); Serial.println(MachineConfig->ADDRESS.ADD_Write);

      COI_BIND_OK();

      RF_BIND=false;
      break;
    }

  }
  
}

static void RF_SEND(void)
{
  radio.write(&DATA_SEND, sizeof(DATA_SEND));

  if( radio.write(&DATA_SEND, sizeof(DATA_SEND))==true )
  {
    LED_OFF;
  }
  else LED_ON;
  
}
/*======================================================*/

/*======================================================
              LOCAL FUNCTION
========================================================*/

static inline void Swap(volatile uint16_t *A, volatile uint16_t *B)
{
  *A = (*A)^(*B);
  *B = (*A)^(*B);
  *A = (*A)^(*B);
}

static void F_Save_Config_Machine(ConfigMachine_typedef *ConfigSave)
{
  EEPROM.put(0, *ConfigSave);
  // delay(1);
}

static void F_Read_Config_Machine(ConfigMachine_typedef *Config_Read)
{
  // memset(&Config_Machine,0,sizeof(Config_Machine)); /* Clear ALL Config */
  // F_Save_Config_Machine(&Config_Machine);

  // Config_Machine.Check_Save=1;
  // Config_Machine.CH_Limit.CH1_MIN=123;
  // Config_Machine.CH_Limit.CH1_Mid=67;
  // Config_Machine.CH_Limit.CH1_MAX=99;
  // EEPROM.put(0, Config_Machine);

  EEPROM.get(0, *Config_Read);
  delay(1);

  if( Config_Read->Value_Trim.V_Trim_CH1==0 ||\
      Config_Read->Value_Trim.V_Trim_CH2==0 ||\
      Config_Read->Value_Trim.V_Trim_CH3==0 ||\
      Config_Read->Value_Trim.V_Trim_CH4==0
    )
  {
    Last_Value_Trim.V_Trim_CH1 = New_Value_Trim.V_Trim_CH1 = Config_Read->Value_Trim.V_Trim_CH1 = 2048;
    Last_Value_Trim.V_Trim_CH2 = New_Value_Trim.V_Trim_CH2 = Config_Read->Value_Trim.V_Trim_CH2 = 2048;
    Last_Value_Trim.V_Trim_CH3 = New_Value_Trim.V_Trim_CH3 = Config_Read->Value_Trim.V_Trim_CH3 = 2048;
    Last_Value_Trim.V_Trim_CH4 = New_Value_Trim.V_Trim_CH4 = Config_Read->Value_Trim.V_Trim_CH4 = 2048;  
    F_Save_Config_Machine(Config_Read);
  }
  else 
  {
    Last_Value_Trim.V_Trim_CH1 = New_Value_Trim.V_Trim_CH1 = Config_Read->Value_Trim.V_Trim_CH1;
    Last_Value_Trim.V_Trim_CH2 = New_Value_Trim.V_Trim_CH2 = Config_Read->Value_Trim.V_Trim_CH2;
    Last_Value_Trim.V_Trim_CH3 = New_Value_Trim.V_Trim_CH3 = Config_Read->Value_Trim.V_Trim_CH3;
    Last_Value_Trim.V_Trim_CH4 = New_Value_Trim.V_Trim_CH4 = Config_Read->Value_Trim.V_Trim_CH4;
  }

  // Serial.print("C: "); Serial.print(Config_Read->ADDRESS.CHANNEL);
  // Serial.print(" - R: "); Serial.print(Config_Read->ADDRESS.ADD_Read);
  // Serial.print(" - W: "); Serial.println(Config_Read->ADDRESS.ADD_Write);

  // Serial.print("Trim Ch1: "); Serial.print(Last_Value_Trim.V_Trim_CH1);
  // Serial.print(" - Trim Ch2: "); Serial.print(Last_Value_Trim.V_Trim_CH2);
  // Serial.print(" - Trim Ch3: "); Serial.print(Last_Value_Trim.V_Trim_CH3);
  // Serial.print(" - Trim Ch4: "); Serial.println(Last_Value_Trim.V_Trim_CH4);

  // char dataprint[100];
  // sprintf(dataprint,"Save: %d - CH_Min: %d - CH_Mid: %d - CH_Max: %d\n", \
  //         Config_Read->Check_Save, \
  //         Config_Read->CH_Limit.CH1_MIN, Config_Read->CH_Limit.CH1_Mid, Config_Read->CH_Limit.CH1_MAX);
  // Serial.print(dataprint);
}

static void F_Init_Peripheral(void)
{
  analogReadResolution(12);

  pinMode(SW1,INPUT_PULLUP);
  pinMode(SW2,INPUT_PULLUP);
  pinMode(SW3,INPUT_PULLUP);
  pinMode(SW4,INPUT_PULLUP);
  pinMode(SW5,INPUT_PULLUP);
  pinMode(SW6,INPUT_PULLUP);

  pinMode(CH1_AIL,INPUT_ANALOG);
  pinMode(CH2_ELE,INPUT_ANALOG);
  pinMode(CH3_THR,INPUT_ANALOG);
  pinMode(CH4_RUD,INPUT_ANALOG);
  pinMode(CH5_SW,INPUT_PULLUP);
  pinMode(CH6_PON1,INPUT_ANALOG);
  pinMode(CH7_PON2,INPUT_ANALOG);
  pinMode(CH8_SW3POS,INPUT_ANALOG);

  pinMode(BUTTON_BIND,INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(COI, OUTPUT);

  while( !PCF.begin() )
  {
    LED_ON;
    COI_ON;
  } 

  COI_LED_POWER_ON();
  LED_OFF;
  COI_OFF;   

  Timer_Channel.pause();
  Timer_Channel.setOverflow((1000*10), MICROSEC_FORMAT); /* 10ms */
  Timer_Channel.attachInterrupt(ISR_Read_Channel);
  Timer_Channel.resume();  

  Timer_Read_Switch.pause();
  Timer_Read_Switch.setOverflow((1000*30), MICROSEC_FORMAT); /* 30ms */ 
  Timer_Read_Switch.attachInterrupt(ISR_Read_Switch);
  Timer_Read_Switch.resume();

  if(TRIP_SELECT==false)
  {
    TRIP_MIN = LONG_MIN;
    TRIP_MAX = LONG_MAX;
  }

  F_Read_Config_Machine(&Config_Machine);
  if( (Config_Machine.ADDRESS.ADD_Read) != (abs(UID->ID2)) ) /* Save Add Read, write, channel */
  {
    Config_Machine.ADDRESS.ADD_Read = abs(UID->ID2);
    if( (Config_Machine.ADDRESS.ADD_Write) != (abs(UID->ID3)) )
    {
      Config_Machine.ADDRESS.ADD_Write = abs(UID->ID3);
    }

    if( abs(UID->ID1)>=123 ) Config_Machine.ADDRESS.CHANNEL = 123;
    else Config_Machine.ADDRESS.CHANNEL = abs(UID->ID1);

    // Serial.println(Config_Machine.ADDRESS.ADD_Read);
    // Serial.println(Config_Machine.ADDRESS.ADD_Write);
    // Serial.println(Config_Machine.ADDRESS.CHANNEL);

    F_Save_Config_Machine(&Config_Machine);
  }

  if( digitalRead(SW1)==1 && digitalRead(SW2)==1 && \
      digitalRead(SW3)==1 && digitalRead(SW4)==1 && \
      digitalRead(SW5)==1 && digitalRead(SW6)==1)
  {
    Config_MIN_MAX=true;
  }

  uint32_t Time_Last=0;
  uint8_t CheckCH1=false, CheckCH2=false, CheckCH3=false, CheckCH4=false, CheckCH8=false;
  uint8_t MiddleCheckCH1=0, MiddleCheckCH2=0, MiddleCheckCH3=0, MiddleCheckCH4=0, MiddleCheckCH8=0;
  while (Config_MIN_MAX==true) /* Set Min/Max 4 channel */
  {
    static uint8_t CheckSetRange=false;

    LED_ON;
    
    if( Flag_Read_Channel==true )
    {
      F_ReadDataChannel();
      F_ReadSwitchConfig();
      Flag_Read_Channel=false;
    }     

    /* CH1 SET MIN/MIDDLE/MAX */
    if( digitalRead(SW1)==1 ) /* CH1 - Save Value MIN */
    {
      if(CheckCH1==false) 
      {
        Limit_Channel.CH1_MIN = analogRead(CH1_AIL);
        MiddleCheckCH1=1;
      }
    }
    else /* CH1 - Save value MAX */
    {
      if(CheckCH1==false)
      {
        if(CheckSetRange==false) 
        {
          CheckSetRange=true;
          Time_Last=millis();
        }

        if(MiddleCheckCH1==1 && CheckSetRange==true) /* Save Middle */
        { 
          if( ((uint32_t)(millis() - Time_Last)>=3000))
          {
            Limit_Channel.CH1_Mid = analogRead(CH1_AIL);

            COI_ON;
            delay(100);
            COI_OFF;

            Time_Last=millis();
            MiddleCheckCH1=2;
          }
        }
        else if(MiddleCheckCH1==2 && CheckSetRange==true) /* Save MAX */
        {
          if( ((uint32_t)(millis() - Time_Last)>=3000) )
          {
            Limit_Channel.CH1_MAX = analogRead(CH1_AIL);

            if( Limit_Channel.CH1_MIN>=Limit_Channel.CH1_MAX )
            {
              Swap( &Limit_Channel.CH1_MIN, &Limit_Channel.CH1_MAX );
            }

            Config_Machine.CH_Limit.CH1_MAX = Limit_Channel.CH1_MAX;
            Config_Machine.CH_Limit.CH1_Mid = Limit_Channel.CH1_Mid;
            Config_Machine.CH_Limit.CH1_MIN = Limit_Channel.CH1_MIN;

            COI_SAVE_OK();
            CheckCH1=true;
            CheckSetRange=false;
            MiddleCheckCH1=0;
          }
        }
      }
    }

    /* CH2 SET MIN/MIDDLE/MAX */
    if( digitalRead(SW2)==1 ) /* CH2 - Save Value MIN */
    {
      if(CheckCH2==false) 
      {
        Limit_Channel.CH2_MIN = analogRead(CH2_ELE);
        MiddleCheckCH2=1;
      }
    }
    else /* CH2 - Save value MAX */
    {
      if(CheckCH2==false)
      {
        if(CheckSetRange==false) 
        {
          CheckSetRange=true;
          Time_Last=millis();
        }

        if(MiddleCheckCH2==1 && CheckSetRange==true) /* Save Middle */
        { 
          if( ((uint32_t)(millis() - Time_Last)>=3000))
          {
            Limit_Channel.CH2_Mid = analogRead(CH2_ELE);

            COI_ON;
            delay(100);
            COI_OFF;

            Time_Last=millis();
            MiddleCheckCH2=2;
          }
        }
        else if(MiddleCheckCH2==2 && CheckSetRange==true) /* Save MAX */
        {
          if( ((uint32_t)(millis() - Time_Last)>=3000) )
          {
            Limit_Channel.CH2_MAX = analogRead(CH2_ELE);

            if( Limit_Channel.CH2_MIN>=Limit_Channel.CH2_MAX )
            {
              Swap( &Limit_Channel.CH2_MIN, &Limit_Channel.CH2_MAX );
            }

            Config_Machine.CH_Limit.CH2_MAX = Limit_Channel.CH2_MAX;
            Config_Machine.CH_Limit.CH2_Mid = Limit_Channel.CH2_Mid;
            Config_Machine.CH_Limit.CH2_MIN = Limit_Channel.CH2_MIN;

            COI_SAVE_OK();
            CheckCH2=true;
            CheckSetRange=false;
            MiddleCheckCH2=0;
          }
        }
      }
    }

    /* CH3 SET MIN/MIDDLE/MAX */
    if( digitalRead(SW3)==1 ) /* CH3 - Save Value MIN */
    {
      if(CheckCH3==false) 
      {
        Limit_Channel.CH3_MIN = analogRead(CH3_THR);
        MiddleCheckCH3=1;
      }
    }
    else /* CH3 - Save value MAX */
    {
      if(CheckCH3==false)
      {
        if(CheckSetRange==false) 
        {
          CheckSetRange=true;
          Time_Last=millis();
        }

        if(MiddleCheckCH3==1 && CheckSetRange==true) /* Save Middle */
        { 
          if( ((uint32_t)(millis() - Time_Last)>=3000))
          {
            Limit_Channel.CH3_Mid = analogRead(CH3_THR);

            COI_ON;
            delay(100);
            COI_OFF;

            Time_Last=millis();
            MiddleCheckCH3=2;
          }
        }
        else if(MiddleCheckCH3==2 && CheckSetRange==true) /* Save MAX */
        {
          if( ((uint32_t)(millis() - Time_Last)>=3000) )
          {
            Limit_Channel.CH3_MAX = analogRead(CH3_THR);

            if( Limit_Channel.CH3_MIN>=Limit_Channel.CH3_MAX )
            {
              Swap( &Limit_Channel.CH3_MIN, &Limit_Channel.CH3_MAX );
            }

            Config_Machine.CH_Limit.CH3_MAX = Limit_Channel.CH3_MAX;
            Config_Machine.CH_Limit.CH3_Mid = Limit_Channel.CH3_Mid;
            Config_Machine.CH_Limit.CH3_MIN = Limit_Channel.CH3_MIN;

            COI_SAVE_OK();
            CheckCH3=true;
            CheckSetRange=false;
            MiddleCheckCH3=0;
          }
        }
      }
    }

    /* CH4 SET MIN/MIDDLE/MAX */
    if( digitalRead(SW4)==1 ) /* CH4 - Save Value MIN */
    {
      if(CheckCH4==false) 
      {
        Limit_Channel.CH4_MIN = analogRead(CH4_RUD);
        MiddleCheckCH4=1;
      }
    }
    else /* CH1 - Save value MAX */
    {
      if(CheckCH4==false)
      {
        if(CheckSetRange==false) 
        {
          CheckSetRange=true;
          Time_Last=millis();
        }

        if(MiddleCheckCH4==1 && CheckSetRange==true) /* Save Middle */
        { 
          if( ((uint32_t)(millis() - Time_Last)>=3000))
          {
            Limit_Channel.CH4_Mid = analogRead(CH4_RUD);

            COI_ON;
            delay(100);
            COI_OFF;

            Time_Last=millis();
            MiddleCheckCH4=2;
          }
        }
        else if(MiddleCheckCH4==2 && CheckSetRange==true) /* Save MAX */
        {
          if( ((uint32_t)(millis() - Time_Last)>=3000) )
          {
            Limit_Channel.CH4_MAX = analogRead(CH4_RUD);

            if( Limit_Channel.CH4_MIN>=Limit_Channel.CH4_MAX )
            {
              Swap( &Limit_Channel.CH4_MIN, &Limit_Channel.CH4_MAX );
            }

            Config_Machine.CH_Limit.CH4_MAX = Limit_Channel.CH4_MAX;
            Config_Machine.CH_Limit.CH4_Mid = Limit_Channel.CH4_Mid;
            Config_Machine.CH_Limit.CH4_MIN = Limit_Channel.CH4_MIN;

            COI_SAVE_OK();
            CheckCH4=true;
            CheckSetRange=false;
            MiddleCheckCH4=0;
          }
        }
      }
    }

    /* CH8 SET MIN/MIDDLE/MAX */
    if( digitalRead(SW5)==1 ) /* CH8 - Save Value MIN */
    {
      if(CheckCH8==false) 
      {
        Limit_Channel.CH8_MIN = analogRead(CH8_SW3POS);
        MiddleCheckCH8=1;
      }
    }
    else /* CH8 - Save value MAX */
    {
      if(CheckCH8==false)
      {
        if(CheckSetRange==false) 
        {
          CheckSetRange=true;
          Time_Last=millis();
        }

        if(MiddleCheckCH8==1 && CheckSetRange==true) /* Save Middle */
        { 
          if( ((uint32_t)(millis() - Time_Last)>=3000))
          {
            Limit_Channel.CH8_Mid = analogRead(CH8_SW3POS);

            COI_ON;
            delay(100);
            COI_OFF;

            Time_Last=millis();
            MiddleCheckCH8=2;
          }
        }
        else if(MiddleCheckCH8==2 && CheckSetRange==true) /* Save MAX */
        {
          if( ((uint32_t)(millis() - Time_Last)>=3000) )
          {
            Limit_Channel.CH8_MAX = analogRead(CH8_SW3POS);

            if( Limit_Channel.CH8_MIN>=Limit_Channel.CH8_MAX )
            {
              Swap( &Limit_Channel.CH8_MIN, &Limit_Channel.CH8_MAX );
            }

            Config_Machine.CH_Limit.CH8_MAX = Limit_Channel.CH8_MAX;
            Config_Machine.CH_Limit.CH8_Mid = Limit_Channel.CH8_Mid;
            Config_Machine.CH_Limit.CH8_MIN = Limit_Channel.CH8_MIN;

            COI_SAVE_OK();
            CheckCH8=true;
            CheckSetRange=false;
            MiddleCheckCH8=0;
          }
        }
      }
    }

    // Serial.print(Limit_Channel.CH8_MIN); Serial.print(" - ");
    // Serial.print(Limit_Channel.CH8_Mid); Serial.print(" - ");
    // Serial.println(Limit_Channel.CH8_MAX);

    if( digitalRead(SW1)==0 && digitalRead(SW2)==0 && \
        digitalRead(SW3)==0 && digitalRead(SW4)==0 && \
        digitalRead(SW5)==0 && digitalRead(SW6)==0)
    {
      Config_Machine.Check_Save=1;
      F_Save_Config_Machine(&Config_Machine);

      New_Value_Trim.V_Trim_CH1=Config_Machine.CH_Limit.CH1_Mid;
      New_Value_Trim.V_Trim_CH2=Config_Machine.CH_Limit.CH2_Mid;
      New_Value_Trim.V_Trim_CH3=Config_Machine.CH_Limit.CH3_Mid;
      New_Value_Trim.V_Trim_CH4=Config_Machine.CH_Limit.CH4_Mid;

      Config_MIN_MAX=false;
      break;
    }

    delay(50);
  }

  RF_INIT(&Config_Machine);

  COI_LED_POWER_ON();

}

static void F_Read_Button(volatile uint8_t *DataRead, volatile uint8_t F_ReadButton, volatile uint16_t *Min, volatile uint16_t *Max, volatile uint16_t *Value, uint8_t UpDown)
{
  *DataRead = F_ReadButton;

  if( *DataRead==0 )
  {
    if( UpDown==false ) 
    {
      if( *Value > (*Min+(((*Min)*10)/100)) ) *Value-=50;
      else *Value = (*Min+(((*Min)*10)/100));
    }
    else
    {
      if( *Value < (*Max-(((*Max)*10)/100)) ) *Value+=50;
      else *Value = (*Max-(((*Max)*10)/100));
    }
  }
}

static uint8_t F_SoftMap(volatile uint16_t *ReadChannel, volatile uint16_t Min, volatile uint16_t Middle, volatile uint16_t Max, volatile uint8_t *Convert, uint8_t Channel)
{
  uint16_t DataConvert=*ReadChannel;
  DataConvert = constrain(DataConvert, Min, Max);

  if(Channel==3)
  {
    if( DataConvert<=Middle ) DataConvert = map(DataConvert, Min, Middle, 0, 127);
    else DataConvert = map(DataConvert, Middle, Max, 128, 255); 
  }
  else 
  {
    if( DataConvert<=Middle ) DataConvert = map(DataConvert, Min, Middle, TRIP_MIN, 127); // 50-127
    else DataConvert = map(DataConvert, Middle, Max, 128, TRIP_MAX); // 128-205
  }

  if( (*Convert)==true ) DataConvert = (uint8_t)(255-(uint8_t)DataConvert);

  return (uint8_t)DataConvert;
}

static void F_ReadButtonTrim(void)
{
  F_Read_Button(&BUTTON_TRIM.Button1, PCF.readButton(0), &Config_Machine.CH_Limit.CH1_MIN, &Config_Machine.CH_Limit.CH1_MAX, &New_Value_Trim.V_Trim_CH1, false); //CH1-
  F_Read_Button(&BUTTON_TRIM.Button2, PCF.readButton(1), &Config_Machine.CH_Limit.CH1_MIN, &Config_Machine.CH_Limit.CH1_MAX, &New_Value_Trim.V_Trim_CH1, true);  //CH1+

  F_Read_Button(&BUTTON_TRIM.Button3, PCF.readButton(2), &Config_Machine.CH_Limit.CH2_MIN, &Config_Machine.CH_Limit.CH2_MAX, &New_Value_Trim.V_Trim_CH2, false); //CH2-
  F_Read_Button(&BUTTON_TRIM.Button4, PCF.readButton(3), &Config_Machine.CH_Limit.CH2_MIN, &Config_Machine.CH_Limit.CH2_MAX, &New_Value_Trim.V_Trim_CH2, true);  //CH2+

  F_Read_Button(&BUTTON_TRIM.Button5, PCF.readButton(6), &Config_Machine.CH_Limit.CH3_MIN, &Config_Machine.CH_Limit.CH3_MAX, &New_Value_Trim.V_Trim_CH3, false); //CH3-
  F_Read_Button(&BUTTON_TRIM.Button6, PCF.readButton(7), &Config_Machine.CH_Limit.CH3_MIN, &Config_Machine.CH_Limit.CH3_MAX, &New_Value_Trim.V_Trim_CH3, true);  //CH3+

  F_Read_Button(&BUTTON_TRIM.Button7, PCF.readButton(4), &Config_Machine.CH_Limit.CH4_MIN, &Config_Machine.CH_Limit.CH4_MAX, &New_Value_Trim.V_Trim_CH4, false); //CH4-
  F_Read_Button(&BUTTON_TRIM.Button8, PCF.readButton(5), &Config_Machine.CH_Limit.CH4_MIN, &Config_Machine.CH_Limit.CH4_MAX, &New_Value_Trim.V_Trim_CH4, true);  //CH4+ 

  if( (New_Value_Trim.V_Trim_CH1!=Last_Value_Trim.V_Trim_CH1) ||\
      (New_Value_Trim.V_Trim_CH2!=Last_Value_Trim.V_Trim_CH2) ||\
      (New_Value_Trim.V_Trim_CH3!=Last_Value_Trim.V_Trim_CH3) ||\
      (New_Value_Trim.V_Trim_CH4!=Last_Value_Trim.V_Trim_CH4) )
  {
    Config_Machine.Value_Trim.V_Trim_CH1 = Last_Value_Trim.V_Trim_CH1 = New_Value_Trim.V_Trim_CH1;
    Config_Machine.Value_Trim.V_Trim_CH2 = Last_Value_Trim.V_Trim_CH2 = New_Value_Trim.V_Trim_CH2;
    Config_Machine.Value_Trim.V_Trim_CH3 = Last_Value_Trim.V_Trim_CH3 = New_Value_Trim.V_Trim_CH3;
    Config_Machine.Value_Trim.V_Trim_CH4 = Last_Value_Trim.V_Trim_CH4 = New_Value_Trim.V_Trim_CH4;

    F_Save_Config_Machine(&Config_Machine);

    // Serial.print("Trim Ch1: "); Serial.print(Last_Value_Trim.V_Trim_CH1);
    // Serial.print(" - Trim Ch2: "); Serial.print(Last_Value_Trim.V_Trim_CH2);
    // Serial.print(" - Trim Ch3: "); Serial.print(Last_Value_Trim.V_Trim_CH3);
    // Serial.print(" - Trim Ch4: "); Serial.println(Last_Value_Trim.V_Trim_CH4);
  }

  #ifdef PRINT_BUTTON_TRIM
  char dataprint[100];
  sprintf(dataprint, "BT1: %d - BT2: %d - BT3: %d - BT4: %d - BT5: %d - BT6: %d - BT7: %d - BT8: %d\n", 
          BUTTON_TRIM.Button1, BUTTON_TRIM.Button2, BUTTON_TRIM.Button3, BUTTON_TRIM.Button4,
          BUTTON_TRIM.Button5, BUTTON_TRIM.Button6, BUTTON_TRIM.Button7, BUTTON_TRIM.Button8);
  Serial.print(dataprint);
  #endif

}

static void F_ReadSwitchConfig(void)
{
  Switch_Config.Switch1 = digitalRead(SW1);
  Switch_Config.Switch2 = digitalRead(SW2);
  Switch_Config.Switch3 = digitalRead(SW3);
  Switch_Config.Switch4 = digitalRead(SW4);
  Switch_Config.Switch5 = digitalRead(SW5);
  Switch_Config.Switch6 = digitalRead(SW6);
  Switch_Config.ButtonBind = digitalRead(BUTTON_BIND);

  #ifdef PRINT_SW_CONFIG
  char dataprint[100];
  sprintf(dataprint, "SW1: %d - SW2: %d - SW3: %d - SW4: %d - SW5: %d - SW6: %d - BIND: %d\n", 
          Switch_Config.Switch1, Switch_Config.Switch2, Switch_Config.Switch3, Switch_Config.Switch4,
          Switch_Config.Switch5, Switch_Config.Switch6, Switch_Config.ButtonBind);
  Serial.print(dataprint);  
  #endif

}

static void F_ReadDataChannel(void)
{
  Read_Channel.CH1_Ali = analogRead(CH1_AIL);
  Read_Channel.CH1_Ali = KALMAN_CH1.updateEstimate((float)Read_Channel.CH1_Ali);
  Read_Channel.CH2_Ele = analogRead(CH2_ELE);
  Read_Channel.CH2_Ele = KALMAN_CH2.updateEstimate((float)Read_Channel.CH2_Ele);
  Read_Channel.CH3_Thr = analogRead(CH3_THR);
  Read_Channel.CH3_Thr = KALMAN_CH3.updateEstimate((float)Read_Channel.CH3_Thr);
  Read_Channel.CH4_Rud = analogRead(CH4_RUD);
  Read_Channel.CH4_Rud = KALMAN_CH4.updateEstimate((float)Read_Channel.CH4_Rud);

  Read_Channel.CH5_Switch1 = digitalRead(CH5_SW);
  Read_Channel.CH6_Pontentionmeter1 = analogRead(CH6_PON1);
  Read_Channel.CH7_Pontentionmeter2 = analogRead(CH7_PON2);
  Read_Channel.CH8_Switch3Pos = analogRead(CH8_SW3POS);

  #ifdef PRINT_CHANNEL
  char dataprint[100];
  sprintf(dataprint, "CH1: %d - CH2: %d - CH3: %d - CH4: %d - CH5: %d - CH6: %d - CH7: %d - CH8: %d\n", 
          Read_Channel.CH1_Ali, Read_Channel.CH2_Ele, Read_Channel.CH3_Thr, Read_Channel.CH4_Rud,
          Read_Channel.CH5_Switch1, Read_Channel.CH6_Pontentionmeter1, Read_Channel.CH7_Pontentionmeter2, Read_Channel.CH8_Switch3Pos);
  Serial.print(dataprint);   
  #endif
}

static void F_ActionConvertData(void)
{
  if( (Switch_Config.Switch5==0 && Switch_Config.Switch6==0) || \
      (Switch_Config.Switch5==1 && Switch_Config.Switch6==1) )
  {
    DATA_SEND.CH1=F_SoftMap(&Read_Channel.CH1_Ali, Config_Machine.CH_Limit.CH1_MIN, New_Value_Trim.V_Trim_CH1, Config_Machine.CH_Limit.CH1_MAX, &Switch_Config.Switch1, 1);
    DATA_SEND.CH2=F_SoftMap(&Read_Channel.CH2_Ele, Config_Machine.CH_Limit.CH2_MIN, New_Value_Trim.V_Trim_CH2, Config_Machine.CH_Limit.CH2_MAX, &Switch_Config.Switch2, 2);

    // DATA_SEND.CH1 = map(Read_Channel.CH1_Ali, Config_Machine.CH_Limit.CH1_MIN, Config_Machine.CH_Limit.CH1_MAX, 0, 255);
    // DATA_SEND.CH2 = map(Read_Channel.CH2_Ele, Config_Machine.CH_Limit.CH2_MIN, Config_Machine.CH_Limit.CH2_MAX, 0, 255);
  }
  else 
  {
    // DATA_SEND.CH1 = map(Read_Channel.CH1_Ali, Config_Machine.CH_Limit.CH1_MIN, Config_Machine.CH_Limit.CH1_MAX, 0, 255);
    // DATA_SEND.CH2 = map(Read_Channel.CH2_Ele, Config_Machine.CH_Limit.CH2_MIN, Config_Machine.CH_Limit.CH2_MAX, 0, 255);

    READ_CH1 = F_SoftMap(&Read_Channel.CH1_Ali, Config_Machine.CH_Limit.CH1_MIN, New_Value_Trim.V_Trim_CH1, Config_Machine.CH_Limit.CH1_MAX, &Switch_Config.Switch1, 1);
    READ_CH2 = F_SoftMap(&Read_Channel.CH2_Ele, Config_Machine.CH_Limit.CH2_MIN, New_Value_Trim.V_Trim_CH2, Config_Machine.CH_Limit.CH2_MAX, &Switch_Config.Switch2, 2);    
    
    if( Switch_Config.Switch5==1 && Switch_Config.Switch6==0 )
    {   
      if( READ_CH2<(Config_Machine.CH_Limit.CH2_Mid) )
      {    
        if(( ((READ_CH1)-(READ_CH2))+127 )<=TRIP_MIN) DATA_SEND.CH1=TRIP_MIN;
        else if(( ((READ_CH1)-(READ_CH2))+127 )>=TRIP_MAX) DATA_SEND.CH1=TRIP_MAX;
        else DATA_SEND.CH1 = ((READ_CH1)-(READ_CH2))+127;

        if(( ((READ_CH1)+(READ_CH2))-127 )<=TRIP_MIN) DATA_SEND.CH2=TRIP_MIN;
        else if(( ((READ_CH1)+(READ_CH2))-127 )>=TRIP_MAX) DATA_SEND.CH2=TRIP_MAX;
        else DATA_SEND.CH2 = ( ((READ_CH1)+(READ_CH2))-127 );
      }

      if( READ_CH2>(Config_Machine.CH_Limit.CH2_Mid) )
      {
        READ_CH1 = map(READ_CH1,TRIP_MIN,TRIP_MAX,TRIP_MAX,TRIP_MIN);          

        if((((READ_CH1)-(READ_CH2))+127)<=TRIP_MIN) DATA_SEND.CH1=TRIP_MIN;
        else if((((READ_CH1)-(READ_CH2))+127)>=TRIP_MAX) DATA_SEND.CH1=TRIP_MAX;
        else DATA_SEND.CH1 = (((READ_CH1)-(READ_CH2))+127);

        if((((READ_CH1)+(READ_CH2))-127)<=TRIP_MIN) DATA_SEND.CH2=TRIP_MIN;
        else if((((READ_CH1)+(READ_CH2))-127)>=TRIP_MAX) DATA_SEND.CH2=TRIP_MAX; 
        else DATA_SEND.CH2 = (((READ_CH1)+(READ_CH2))-127);
      }
    }
    else if( Switch_Config.Switch5==0 && Switch_Config.Switch6==1 )
    {
      if( READ_CH1<(Config_Machine.CH_Limit.CH1_Mid) )
      {    
        if(( ((READ_CH2)-(READ_CH1))+127 )<=TRIP_MIN) DATA_SEND.CH1=TRIP_MIN;
        else if(( ((READ_CH2)-(READ_CH1))+127 )>=TRIP_MAX) DATA_SEND.CH1=TRIP_MAX;
        else DATA_SEND.CH1 = ((READ_CH2)-(READ_CH1))+127;

        if(( ((READ_CH2)+(READ_CH1))-127 )<=TRIP_MIN) DATA_SEND.CH2=TRIP_MIN;
        else if(( ((READ_CH2)+(READ_CH1))-127 )>=TRIP_MAX) DATA_SEND.CH2=TRIP_MAX;
        else DATA_SEND.CH2 = ( ((READ_CH2)+(READ_CH1))-127 );
      }

      if( READ_CH1>(Config_Machine.CH_Limit.CH1_Mid) )
      {
        READ_CH2 = map(READ_CH2,TRIP_MIN,TRIP_MAX,TRIP_MAX,TRIP_MIN);          

        if((((READ_CH2)-(READ_CH1))+127)<=TRIP_MIN) DATA_SEND.CH1=TRIP_MIN;
        else if((((READ_CH2)-(READ_CH1))+127)>=TRIP_MAX) DATA_SEND.CH1=TRIP_MAX;
        else DATA_SEND.CH1 = (((READ_CH2)-(READ_CH1))+127);

        if((((READ_CH2)+(READ_CH1))-127)<=TRIP_MIN) DATA_SEND.CH2=TRIP_MIN;
        else if((((READ_CH2)+(READ_CH1))-127)>=TRIP_MAX) DATA_SEND.CH2=TRIP_MAX; 
        else DATA_SEND.CH2 = (((READ_CH2)+(READ_CH1))-127);
      }
    }
  }

  DATA_SEND.CH3=F_SoftMap(&Read_Channel.CH3_Thr, Config_Machine.CH_Limit.CH3_MIN, New_Value_Trim.V_Trim_CH3, Config_Machine.CH_Limit.CH3_MAX, &Switch_Config.Switch3, 3);
  DATA_SEND.CH4=F_SoftMap(&Read_Channel.CH4_Rud, Config_Machine.CH_Limit.CH4_MIN, New_Value_Trim.V_Trim_CH4, Config_Machine.CH_Limit.CH4_MAX, &Switch_Config.Switch4, 4);

  DATA_SEND.CH5 = Read_Channel.CH5_Switch1==1?255:0;
  DATA_SEND.CH6 = map(Read_Channel.CH6_Pontentionmeter1,0,4095,0,255);
  DATA_SEND.CH7 = map(Read_Channel.CH7_Pontentionmeter2,0,4095,0,255);
  DATA_SEND.CH8 = map(Read_Channel.CH8_Switch3Pos,Config_Machine.CH_Limit.CH8_MIN,Config_Machine.CH_Limit.CH8_MAX,0,255);

  #ifdef PRINT_DATA_SEND
  char dataprint[100];
  sprintf(dataprint, "CH1: %d - CH2: %d - CH3: %d - CH4: %d - CH5: %d - CH6: %d - CH7: %d - CH8: %d\n", 
          DATA_SEND.CH1, DATA_SEND.CH2, DATA_SEND.CH3, DATA_SEND.CH4,
          DATA_SEND.CH5, DATA_SEND.CH6, DATA_SEND.CH7, DATA_SEND.CH8);
  Serial.print(dataprint);
  // delay(50);
  #endif

}

static void F_Action(void)
{
  if( digitalRead(SW1)==1 && digitalRead(SW2)==1 && digitalRead(SW3)==1 && \
      digitalRead(SW4)==1 && digitalRead(SW5)==1 && digitalRead(SW6)==1 )
  {
    LED_ON;
    if( PCF.readButton(0)==0 && PCF.readButton(1)==0 )
    {
      COI_ON;
      delay(500);
      
      memset(&Config_Machine,0,sizeof(Config_Machine)); /* Clear ALL Config */
      F_Save_Config_Machine(&Config_Machine);
      
      delay(500);
      NVIC_SystemReset();
    }
  }
  else 
  {
    static uint8_t Step_select_trip=0;
    static uint32_t Time_tick=0;
    if( digitalRead(BUTTON_BIND)==0 ) /* Select TRIP: LONG or SHORT */
    {
      if(Step_select_trip==0)
      {
        Time_tick=millis();
        Step_select_trip=1;
      }
      else 
      {
        if((uint32_t)millis()-Time_tick>=3000) /* Nhan giu 3s */
        {
          TRIP_SELECT = !TRIP_SELECT;

          if(TRIP_SELECT==false)
          {
            TRIP_MIN = LONG_MIN;
            TRIP_MAX = LONG_MAX;
          }
          else 
          {
            TRIP_MIN = SHORT_MIN;
            TRIP_MAX = SHORT_MAX;          
          }

          LED_ON; COI_ON;
          delay(200);
          LED_OFF; COI_OFF;
          delay(200);
          LED_ON; COI_ON;
          delay(200);
          LED_OFF; COI_OFF;
          delay(200);

          Step_select_trip=0;
        }
      }
    }
    else 
    {
      Time_tick=0;
      Step_select_trip=0;
    }

    if( Flag_Read_Channel==true ) /* 10ms */
    {
      F_ReadDataChannel();
      F_ActionConvertData();
      Flag_Read_Channel=false;
    }

    if( Flag_Read_Switch==true ) /* 30ms */
    {
      F_ReadButtonTrim();
      F_ReadSwitchConfig();
      Flag_Read_Switch=false;
    }

    RF_SEND();

  }

}

void ISR_Read_Switch(void)
{
  Flag_Read_Switch=true;
}

void ISR_Read_Channel(void)
{
  Flag_Read_Channel=true;
}

/*======================================================*/

/*======================================================
              TX APP MAIN
========================================================*/
void F_Init(void)
{
  Serial.begin(115200);

  F_Init_Peripheral();

}

void F_Main(void)
{
  F_Action();

  delay(2);
}
/*======================================================*/


