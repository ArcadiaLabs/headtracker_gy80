//-----------------------------------------------------------------------------
// Original project by Dennis Frie - 2012 - Dennis.frie@gmail.com
// Discussion: http://www.rcgroups.com/forums/showthread.php?t=1677559
//
// Other contributors to this code:
//  Mark Mansur (Mangus on rcgroups)
//  
// Version history:
// - 0.01 - 0.08 - Dennis Frie - preliminary releases
// - 1.01 - April 2013 - Mark Mansur - code clean-up and refactoring, comments
//      added. Added pause functionality, added settings retrieval commands.
//      Minor optimizations.
//-----------------------------------------------------------------------------

#include <Wire.h>
#include "config.h"
#include "functions.h"
#include "sensors.h"
#include "FaceTrack.h"
#include <EEPROM.h>
//#include <SoftwareSerial.h>

//SoftwareSerial1 Serial1(BT_RX, BT_TX); // RX | TX


/*
Channel mapping/config for PPM out:

1 = PPM CHANNEL 1
2 = PPM CHANNEL 2
3 = PPM CHANNEL 3
4 = PPM CHANNEL 4
5 = PPM CHANNEL 5
6 = PPM CHANNEL 6
7 = PPM CHANNEL 7
8 = PPM CHANNEL 8
9 = PPM CHANNEL 9
10 = PPM CHANNEL 10
11 = PPM CHANNEL 11
12 = PPM CHANNEL 12

Mapping example:
$123456789111CH
*/
//int FTEnable=0;

// Local file variables
//
int frameNumber = 0;		    // Frame count since last debug Serial1 output

char SERIAL_data[101];          // Array for Serial1-data 
unsigned char SERIAL_index = 0; // How many bytes have been received?
char string_started = 0;        // Only saves data if string starts with right byte
unsigned char channel_mapping[13];

char outputMag = 0;             // Stream magnetometer data to host
char outputAcc = 0;             // Stream accelerometer data to host
char outputMagAcc = 0;          // Stream mag and accell data (for calibration on PC)
char outputTrack = 0;	        // Stream angle data to host

// Keep track of button press
char lastButtonState = 0;           // 0 is not pressed, 1 is pressed
unsigned long buttonDownTime = 0;   // the system time of the press
char pauseToggled = 0;              // Used to make sure we toggle pause only once per hold
char ht_paused = 0;

// External variables (defined in other files)
//
extern unsigned char PpmIn_PpmOut[13];
extern char read_sensors;
extern char resetValues;   
extern char tiltInverse;
extern char rollInverse;
extern char panInverse;

// Settings (Defined in sensors.cpp)
//
extern float tiltRollBeta;
extern float panBeta;
extern float gyroWeightTiltRoll;
extern float GyroWeightPan;
extern int servoPanCenter;
extern int servoTiltCenter;
extern int servoRollCenter;
extern int panMaxPulse;
extern int panMinPulse;
extern int tiltMaxPulse;
extern int tiltMinPulse;
extern int rollMaxPulse;
extern int rollMinPulse;
extern float panFactor;
extern float tiltFactor;  
extern float rollFactor;
extern unsigned char servoReverseMask;
extern unsigned char htChannels[];
extern float gyroOff[];
extern float magOffset[];
extern int accOffset[]; 

// Original Facetrack.cpp page,
// Not working with SoftwareSerial1
#include <Wire.h>
#include "config.h"
#include "functions.h"
#include "sensors.h"
#include <EEPROM.h>

float XZero,YZero,ZZero;
typedef struct {
  int16_t  Begin  ;   // 2  Debut
  uint16_t Cpt ;      // 2  Compteur trame or Code info or error
  float    gyro[3];   // 12 [Y, P, R]    gyro
  float    acc[3];    // 12 [x, y, z]    Acc
  int16_t  End ;      // 2  Fin
} 
_hat;

uint16_t unit;

extern float panAngle;
extern float tiltAngle;
extern float rollAngle;

typedef struct  {
  int16_t Begin  ;   // 2  Debut
  uint16_t Code ;     // 2  Code info
  char     Msg[24];   // 24 Message
  int16_t End ;      // 2  Fin
} 
_msginfo;

_msginfo msginfo;
_hat hat;
float yaw;
float pitch;
float roll;
int Mag_HardIron_On=0;
int Acc_Calib_On=0;
void PrintCodeSerial1(uint16_t code,char Msg[24],bool EOL ) {
  msginfo.Code=code;
  memset(msginfo.Msg,0x00,24);
  strcpy(msginfo.Msg,Msg);
  if (EOL) msginfo.Msg[23]=0x0A;
  // Send HATIRE message to  PC
   Serial1.write((byte*)&msginfo,30);
}
//att_t att;
void FT_Setup()
{
  hat.Begin=0xAAAA;
  hat.Cpt=0;
  hat.End=0x5555;
  msginfo.Begin=0xAAAA;
  msginfo.Code=0;
  msginfo.End=0x5555;
  PrintCodeSerial1(5000,"HAT BEGIN",true); 
  yaw=panAngle;
  pitch=tiltAngle;
  roll=rollAngle;
  XZero=yaw;
  YZero=pitch;
  ZZero=roll;
}

void Serial1Event()
{
  char Commande;
   Commande = (char)Serial1.read();
  switch (Commande) 
  {
  case 'Z':
    // last_z_angle=0;
    yaw=panAngle;
    pitch=tiltAngle;
    roll=rollAngle;
    XZero=(yaw);
    YZero=(pitch);
    ZZero=(roll);
    //PrintCodeSerial1(5000,"Generated",true); 
    break; 
  }
  if (Commande == 'h')
  {
    Mag_HardIron_On=1; 
  }
  if (Commande == 'a')
  {
    Acc_Calib_On=1; 
    for (int i = 0; i < 3; i++) 
    {
      //   accel_min[i]=0;
      // accel_max[i]=0;
    }
  }
  if (Commande == 's') // _s_ynch request
  {
    // Read ID
    //        byte id[2];
    //        id[0] = readChar();
    //        id[1] = readChar();

    // Reply with synch message
    //        Serial1.print("#SYNCH");
    //        Serial1.write(id, 2);
    //        Serial1.println();
    //        Calib_On=1;
  } 
}


void FTData()
{
  if(Serial1.available() > 0)  Serial1Event();
//    roll=rollAngle;
//    yaw=panAngle;  
//    pitch=tiltAngle;
  yaw=panAngle;
  pitch=tiltAngle;
  roll=rollAngle;
  hat.gyro[0]=yaw-XZero;
  hat.gyro[1]=pitch-YZero;
  hat.gyro[2]=roll-ZZero;
/*  Serial1.print(hat.gyro[0]);
  Serial1.print(" ");
  Serial1.print(hat.gyro[1]);
  Serial1.print(" ");
  Serial1.print(hat.gyro[2]);
  Serial1.println(" ");*/
  Serial1.write((byte*)&hat,30);
  hat.Cpt++;
  if (hat.Cpt>999) {
    hat.Cpt=0;
  }
  delay(10); 


}
// Original Facetrack.cpp page END


// End settings   

//--------------------------------------------------------------------------------------
// Func: setup
// Desc: Called by Arduino framework at initialization time. This sets up pins for I/O,
//       initializes sensors, etc.
//--------------------------------------------------------------------------------------
void setup()
{
//    Serial1.begin(SERIAL_BAUD);
    
//    pinMode(BT_KEY, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) LOW to switch module to normal mode
//    digitalWrite(BT_KEY, LOW);
//    pinMode(BT_POWER, OUTPUT);  // this pin will pull the HC-05 power pin HIGH to switch module on
//    digitalWrite(BT_POWER, HIGH);
    
    Serial1.begin(115200);

    
//    pinMode(9,OUTPUT);
//    digitalWrite(2,HIGH);
//    digitalWrite(3,HIGH);  
  
    // Set all other pins to input, for safety.
//    pinMode(0,INPUT);
//    pinMode(1,INPUT);
//    pinMode(2,INPUT);
//    pinMode(3,INPUT);
//    pinMode(6,INPUT);
//    pinMode(7,INPUT);  
//    pinMode(8,INPUT);    

    // Set button pin to input:
    pinMode(BUTTON_INPUT,INPUT);
  
    // Set internal pull-up resistor. 
    digitalWrite(BUTTON_INPUT,HIGH);
  
//    digitalWrite(0,LOW); // pull-down resistor
//    digitalWrite(1,LOW);
//    digitalWrite(2,HIGH);
//    digitalWrite(3,HIGH);  
  
    pinMode(ARDUINO_LED,OUTPUT);    // Arduino LED
    digitalWrite(ARDUINO_LED, HIGH);
    
#if FATSHARK_HT_MODULE
    pinMode(BUZZER,OUTPUT);         // Buzzer
    digitalWrite(BUZZER, HIGH);
#endif

    // Give it time to be noticed, then turn it off
    delay(200); // Note: only use delay here. This won't work when Timer0 is repurposed later.
    digitalWrite(ARDUINO_LED, LOW);

#if FATSHARK_HT_MODULE
    digitalWrite(BUZZER, LOW);
#endif

    InitPWMInterrupt();         // Start PWM interrupt  
    Wire.begin();               // Start I2C

    // If the device have just been programmed, write initial config/values to EEPROM:
    if (EEPROM.read(0) != 8)
    {
//#if (DEBUG)
        Serial1.println("New board - saving default values!");
//#endif    
    
        InitSensors();
#if (ALWAYS_CAL_GYRO)    
        SetGyroOffset();
#endif     

        SaveSettings();
        SaveMagData();
        SaveAccelData();
    }
 
    GetSettings();                 // Get settings saved in EEPROM
    InitSensors();                // Initialize I2C sensors
    CalibrateMag();
    ResetCenter();
    InitTimerInterrupt();        // Start timer interrupt (for sensors)  
    #if FTEnable==1
        FT_Setup();
        Serial1.println(" B");
    #endif
   // Serial1.println(" A");
}

//--------------------------------------------------------------------------------------
// Func: loop
// Desc: Called by the Arduino framework once per frame. Represents main program loop.
//--------------------------------------------------------------------------------------
void loop()
{  
      
      #if FTEnable==1
          FTData();
      #endif
    // Check input button for reset/pause request
#if FTEnable==0
    char buttonPressed = (digitalRead(BUTTON_INPUT) == 0);

    if ( buttonPressed && lastButtonState == 0)
    {
        resetValues = 1; 
        buttonDownTime = 0;
        lastButtonState = 1;
    }
    
    if ( buttonPressed )
    {
        if ( !pauseToggled && (buttonDownTime > BUTTON_HOLD_PAUSE_THRESH) )
        {
            // Pause/unpause
            ht_paused = !ht_paused;
            resetValues = 1;
            pauseToggled = 1;
        }
    }
    else
    {
        pauseToggled = 0;
        lastButtonState = 0;
    }
    
    // All this is used for communication with GUI 
    //

    if (Serial1.available())
    {
        if (string_started == 1)
        {
            // Read incoming byte
            SERIAL_data[SERIAL_index++] = Serial1.read();
           
            // If string ends with "CH" it's channel configuration, that have been received.
            // String must always be 12 chars/bytes and ending with CH to be valid. 
            if (SERIAL_index == 14 &&
                SERIAL_data[SERIAL_index-2] == 'C' &&
                SERIAL_data[SERIAL_index-1] == 'H' )
            {
                // To keep it simple, we will not let the channels be 0-initialized, but
                // start from 1 to match actual channels. 
                for (unsigned char i = 0; i < 13; i++)
                {
                    channel_mapping[i + 1] = SERIAL_data[i] - 48;
                  
                    // Update the dedicated PPM-in -> PPM-out array for faster performance.
                    if ((SERIAL_data[i] - 48) < 14)
                    {
                        PpmIn_PpmOut[SERIAL_data[i]-48] = i + 1;
                    }
                }
               
                Serial1.println("Channel mapping received");
               
               // Reset SERIAL_index and SERIAL_started
               SERIAL_index = 0;
               string_started = 0;
            }
            
            // Configure headtracker
            else if (SERIAL_data[SERIAL_index-2] == 'H' &&
                     SERIAL_data[SERIAL_index-1] == 'E')
            {
                // HT parameters are passed in from the PC in this order:
                //
                // 0 tiltRollBeta      
                // 1 panBeta       
                // 2 gyroWeightTiltRoll    
                // 3 GyroWeightPan 
                // 4 tiltFactor        
                // 5 panFactor          
                // 6 rollFactor
                // 7 servoReverseMask
                // 8 servoPanCenter
                // 9 panMinPulse 
                // 10 panMaxPulse
                // 11 servoTiltCenter
                // 12 tiltMinPulse
                // 13 tiltMaxPulse
                // 14 servoRollCenter
                // 15 rollMinPulse
                // 16 rollMaxPulse
                // 17 htChannels[0]  // pan            
                // 18 htChannels[1]  // tilt 
                // 19 htChannels[2]  // roll         
             
                // Parameters from the PC client need to be scaled to match our local
                // expectations

                Serial1.println("HT config received:");
           
                int valuesReceived[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                int comma_index = 0;

                for (unsigned char k = 0; k < SERIAL_index - 2; k++)
                {
                    // Looking for comma
                    if (SERIAL_data[k] == 44)
                    {
                        comma_index++;
                    }
                    else
                    {
                        valuesReceived[comma_index] = valuesReceived[comma_index] * 10 + (SERIAL_data[k] - 48);
                    }
             
#if (DEBUG)
                    Serial1.print(SERIAL_data[k]);
#endif
                }

#if (DEBUG)
                Serial1.println();
                for (unsigned char k = 0; k < comma_index+1; k++)
                {
                    Serial1.print(valuesReceived[k]); 
                    Serial1.print(",");           
                }
                Serial1.println();
#endif

                tiltRollBeta  = (float)valuesReceived[0] / 100;  
                panBeta       = (float)valuesReceived[1] / 100;
                gyroWeightTiltRoll = (float)valuesReceived[2] / 100;
                GyroWeightPan = (float)valuesReceived[3] / 100;
                tiltFactor    = (float)valuesReceived[4] / 10;         
                panFactor     = (float)valuesReceived[5] / 10;          
                rollFactor    = (float)valuesReceived[6] / 10;   

                servoReverseMask = (unsigned char)valuesReceived[7];

                tiltInverse = 1;
                rollInverse = 1;
                panInverse = 1;           
                
                if ((servoReverseMask & HT_PAN_REVERSE_BIT) != 0)
                {
                    panInverse = -1;
                }
                if ((servoReverseMask & HT_ROLL_REVERSE_BIT) != 0)
                {
                    rollInverse = -1; 
                }
                if ((servoReverseMask & HT_TILT_REVERSE_BIT) != 0)
                {
                    tiltInverse = -1;
                }

                servoPanCenter = valuesReceived[8];
                panMinPulse = valuesReceived[9];
                panMaxPulse = valuesReceived[10];         
         
                servoTiltCenter = valuesReceived[11];
                tiltMinPulse = valuesReceived[12];
                tiltMaxPulse = valuesReceived[13];         

                servoRollCenter = valuesReceived[14];
                rollMinPulse = valuesReceived[15];
                rollMaxPulse = valuesReceived[16];              
     
                htChannels[0] = valuesReceived[17];                   
                htChannels[1] = valuesReceived[18];              
                htChannels[2] = valuesReceived[19];                       

                Serial1.println(htChannels[0]);
                Serial1.println(htChannels[1]);
                Serial1.println(htChannels[2]);                
        
                SaveSettings();

                SERIAL_index = 0;
                string_started = 0;
            } // end configure headtracker
          
            // Debug info
            else if (SERIAL_data[SERIAL_index-5] == 'D' &&
                     SERIAL_data[SERIAL_index-4] == 'E' &&
                     SERIAL_data[SERIAL_index-3] == 'B' &&
                     SERIAL_data[SERIAL_index-2] == 'U' &&
                     SERIAL_data[SERIAL_index-1] == 'G')
            {  
                DebugOutput();
                SERIAL_index = 0;
                string_started = 0; 
            }

            // Firmware version requested
            else if (SERIAL_data[SERIAL_index-4] == 'V' &&
                     SERIAL_data[SERIAL_index-3] == 'E' &&
                     SERIAL_data[SERIAL_index-2] == 'R' &&
                     SERIAL_data[SERIAL_index-1] == 'S')
            {
                Serial1.print("FW: ");
                Serial1.println(FIRMWARE_VERSION_FLOAT, 2);
                SERIAL_index = 0;
                string_started = 0; 
            }
          
            // Start mag and accel data stream
            else if (SERIAL_data[SERIAL_index-4] == 'C' &&
                     SERIAL_data[SERIAL_index-3] == 'M' &&
                     SERIAL_data[SERIAL_index-2] == 'A' &&
                     SERIAL_data[SERIAL_index-1] == 'S')
            {  
                outputMagAcc = 1;
                outputMag = 0;
                outputAcc = 0;
                outputTrack = 0;
                SERIAL_index = 0;
                string_started = 0;
            }        

            // Stop mag and accel data stream
            else if (SERIAL_data[SERIAL_index-4] == 'C' &&
                     SERIAL_data[SERIAL_index-3] == 'M' &&
                     SERIAL_data[SERIAL_index-2] == 'A' &&
                     SERIAL_data[SERIAL_index-1] == 'E')
            {  
                outputMagAcc = 0;
                outputMag = 0;
                outputTrack = 0;
                outputAcc = 0;
                SERIAL_index = 0;
                string_started = 0;
            }        

            // Start magnetometer data stream
            else if (SERIAL_data[SERIAL_index-4] == 'C' &&
                     SERIAL_data[SERIAL_index-3] == 'A' &&
                     SERIAL_data[SERIAL_index-2] == 'S' &&
                     SERIAL_data[SERIAL_index-1] == 'T')
            {  
                outputMag = 1;
                outputMagAcc = 0;
                outputAcc = 0;
                outputTrack = 0;
                SERIAL_index = 0;
                string_started = 0; 

            }        
          
            // Stop magnetometer data stream
            else if (SERIAL_data[SERIAL_index-4] == 'C' &&
                     SERIAL_data[SERIAL_index-3] == 'A' &&
                     SERIAL_data[SERIAL_index-2] == 'E' &&
                     SERIAL_data[SERIAL_index-1] == 'N')
            {  
                outputMag = 0;
                outputMagAcc = 0;
                outputAcc = 0;
                outputTrack = 0;
                SERIAL_index = 0;
                string_started = 0; 
            }

            // Start accelerometer data stream
            else if (SERIAL_data[SERIAL_index-4] == 'G' &&
                     SERIAL_data[SERIAL_index-3] == 'R' &&
                     SERIAL_data[SERIAL_index-2] == 'A' &&
                     SERIAL_data[SERIAL_index-1] == 'V')
            {  
                outputAcc = 1;     
                outputMagAcc = 0;
                outputMag = 0;
                outputTrack = 0;
                SERIAL_index = 0;
                string_started = 0; 
            }              
          
            // Stop accelerometer data stream
            else if (SERIAL_data[SERIAL_index-4] == 'G' &&
                     SERIAL_data[SERIAL_index-3] == 'R' &&
                     SERIAL_data[SERIAL_index-2] == 'E' &&
                     SERIAL_data[SERIAL_index-1] == 'N')
            {  
                outputAcc = 0;
                outputMag = 0;
                outputMagAcc = 0;
                outputTrack = 0;
                SERIAL_index = 0;
                string_started = 0; 
            }

            // Start tracking data stream
            else if (SERIAL_data[SERIAL_index-4] == 'P' &&
                     SERIAL_data[SERIAL_index-3] == 'L' &&
                     SERIAL_data[SERIAL_index-2] == 'S' &&
                     SERIAL_data[SERIAL_index-1] == 'T')
            {  
                outputTrack = 1;
                outputMagAcc = 0;
                outputMag = 0;
                outputAcc = 0;
                SERIAL_index = 0;
                string_started = 0; 
            }        

            // Stop tracking data stream          
            else if (SERIAL_data[SERIAL_index-4] == 'P' &&
                     SERIAL_data[SERIAL_index-3] == 'L' &&
                     SERIAL_data[SERIAL_index-2] == 'E' &&
                     SERIAL_data[SERIAL_index-1] == 'N')
            {  
                outputTrack = 0;
                outputMag = 0;
                outputAcc = 0;
                outputMagAcc = 0;
                SERIAL_index = 0;
                string_started = 0; 
            }
          
            // Save RAM settings to EEPROM
            else if (SERIAL_data[SERIAL_index-4] == 'S' &&
                     SERIAL_data[SERIAL_index-3] == 'A' &&
                     SERIAL_data[SERIAL_index-2] == 'V' &&
                     SERIAL_data[SERIAL_index-1] == 'E')
            {  
                SaveSettings();     
                SERIAL_index = 0;
                string_started = 0; 
            }          
          
            // Calibrate gyro
            else if (SERIAL_data[SERIAL_index-4] == 'C' &&
                     SERIAL_data[SERIAL_index-3] == 'A' &&
                     SERIAL_data[SERIAL_index-2] == 'L' &&
                     SERIAL_data[SERIAL_index-1] == 'G')
            { 
                SetGyroOffset();
                SaveSettings();
               
                Serial1.print("Gyro offset measured:");
                Serial1.print(gyroOff[0]);
                Serial1.print(",");   
                Serial1.print(gyroOff[1]);
                Serial1.print(",");      
                Serial1.println(gyroOff[2]);    
               
                SERIAL_index = 0;
                string_started = 0; 
            }

            // Store magnetometer offset
            else if (SERIAL_data[SERIAL_index-3] == 'M' &&
                     SERIAL_data[SERIAL_index-2] == 'A' &&
                     SERIAL_data[SERIAL_index-1] == 'G')
            {
                Serial1.println(SERIAL_data);
                int valuesReceived[5] = {0,0,0,0,0};
                int comma_index = 0; 
              
                for (unsigned char k = 0; k < SERIAL_index - 3; k++)
                {
                    // Looking for comma
                    if (SERIAL_data[k] == 44)
                    {
                        comma_index++;
                    }
                    else
                    {
                        valuesReceived[comma_index] = valuesReceived[comma_index] * 10 + (SERIAL_data[k] - 48);
                    }
                }
                
                // Y and Z are swapped on purpose.
                magOffset[0] = valuesReceived[0] - 2000;
                magOffset[1] = valuesReceived[2] - 2000;
                magOffset[2] = valuesReceived[1] - 2000;

                SERIAL_index = 0;
                string_started = 0; 

                SaveMagData();                
            }

            // Store accelerometer offset
            else if (SERIAL_data[SERIAL_index-3] == 'A' &&
                     SERIAL_data[SERIAL_index-2] == 'C' &&
                     SERIAL_data[SERIAL_index-1] == 'C')
            {
                Serial1.println(SERIAL_data);
                int valuesReceived[5] = {0,0,0,0,0};
                int comma_index = 0; 
                for (unsigned char k = 0; k < SERIAL_index - 3; k++)
                {
                    // Looking for comma
                    if (SERIAL_data[k] == 44)
                    {
                        comma_index++;
                    }
                    else
                    {
                        valuesReceived[comma_index] = valuesReceived[comma_index] * 10 + (SERIAL_data[k] - 48);
                    }              
                }

                accOffset[0] = valuesReceived[0] - 2000;
                accOffset[1] = valuesReceived[1] - 2000;
                accOffset[2] = valuesReceived[2] - 2000;
                
                SERIAL_index = 0;
                string_started = 0; 

                SaveAccelData();                
            }

            // Retrieve settings
            else if (SERIAL_data[SERIAL_index-4] == 'G' &&
                     SERIAL_data[SERIAL_index-3] == 'S' &&
                     SERIAL_data[SERIAL_index-2] == 'E' &&
                     SERIAL_data[SERIAL_index-1] == 'T' )
            {
                // Get Settings. Scale our local values to
                // real-world values usable on the PC side.
                //
                Serial1.print("$SET$"); // something recognizable in the stream

                Serial1.print(tiltRollBeta * 100);
                Serial1.print(",");   
                Serial1.print(panBeta * 100);
                Serial1.print(",");
                Serial1.print(gyroWeightTiltRoll * 100);  
                Serial1.print(",");
                Serial1.print(GyroWeightPan * 100);
                Serial1.print(",");
                Serial1.print(tiltFactor * 10);
                Serial1.print(",");
                Serial1.print(panFactor * 10);
                Serial1.print(",");
                Serial1.print(rollFactor * 10);
                Serial1.print(",");
                Serial1.print(servoReverseMask);
                Serial1.print(",");
                Serial1.print(servoPanCenter);
                Serial1.print(",");
                Serial1.print(panMinPulse);
                Serial1.print(",");
                Serial1.print(panMaxPulse);
                Serial1.print(",");
                Serial1.print(servoTiltCenter);
                Serial1.print(",");
                Serial1.print(tiltMinPulse);
                Serial1.print(",");
                Serial1.print(tiltMaxPulse);
                Serial1.print(",");
                Serial1.print(servoRollCenter);
                Serial1.print(",");
                Serial1.print(rollMinPulse);
                Serial1.print(",");
                Serial1.print(rollMaxPulse);
                Serial1.print(",");
                Serial1.print(htChannels[0]);
                Serial1.print(",");
                Serial1.print(htChannels[1]);
                Serial1.print(",");
                Serial1.println(htChannels[2]);

                Serial1.println("Settings Retrieved!");

                SERIAL_index = 0;
                string_started = 0;
            }
            else if (SERIAL_index > 100)
            {
                // If more than 100 bytes have been received, the string is not valid.
                // Reset and "try again" (wait for $ to indicate start of new string). 
                SERIAL_index = 0;
                string_started = 0;
            }
        }
        else if (Serial1.read() == '$')
        {
            string_started = 1;
        }
    } // Serial1 port input
#endif
    // if "read_sensors" flag is set high, read sensors and update
    if (read_sensors == 1 && ht_paused == 0)
    {
        UpdateSensors();
        GyroCalc();
        AccelCalc();
        MagCalc();
        FilterSensorData();    
               
        // Only output this data every X frames.
        if (frameNumber++ >= SERIAL_OUTPUT_FRAME_INTERVAL)
        {
            if (outputTrack == 1)
            {
                trackerOutput();
            }
            else if (outputMagAcc == 1)
            {
                calMagAccOutput();
            }
            else if (outputMag == 1)
            {
                calMagOutput(); 
            }
            else if (outputAcc == 1)
            {
                calAccOutput();
            }
            frameNumber = 0; 
        }

        // Will first update read_sensors when everything is done.  
        read_sensors = 0;
    }
}

//--------------------------------------------------------------------------------------
// Func: SaveSettings
// Desc: Saves device settings to EEPROM for retrieval at boot-up.
//--------------------------------------------------------------------------------------
void SaveSettings()
{  
    EEPROM.write(1, (unsigned char)(tiltRollBeta * 100));
    EEPROM.write(2, (unsigned char)(panBeta * 100));
    EEPROM.write(3, (unsigned char)(gyroWeightTiltRoll * 100));
    EEPROM.write(4, (unsigned char)(GyroWeightPan * 100));
  
    EEPROM.write(5, (unsigned char)servoReverseMask);
    
    // 6 unused
  
    EEPROM.write(7, (unsigned char)servoPanCenter);
    EEPROM.write(8, (unsigned char)(servoPanCenter >> 8));  
  
    EEPROM.write(9, (unsigned char)(tiltFactor * 10));
    EEPROM.write(10, (int)((tiltFactor * 10)) >> 8);  

    EEPROM.write(11, (unsigned char) (panFactor * 10));
    EEPROM.write(12, (int)((panFactor * 10)) >> 8);  

    EEPROM.write(13, (unsigned char) (rollFactor * 10));
    EEPROM.write(14, (int)((rollFactor * 10)) >> 8);  

    // 15 unused

    EEPROM.write(16, (unsigned char)servoTiltCenter);
    EEPROM.write(17, (unsigned char)(servoTiltCenter >> 8));  

    EEPROM.write(18, (unsigned char)servoRollCenter);
    EEPROM.write(19, (unsigned char)(servoRollCenter >> 8));  


    EEPROM.write(20, (unsigned char)panMaxPulse);
    EEPROM.write(21, (unsigned char)(panMaxPulse >> 8));  
  
    EEPROM.write(22, (unsigned char)panMinPulse);
    EEPROM.write(23, (unsigned char)(panMinPulse >> 8));    

    EEPROM.write(24, (unsigned char)tiltMaxPulse);
    EEPROM.write(25, (unsigned char)(tiltMaxPulse >> 8));    

    EEPROM.write(26, (unsigned char)tiltMinPulse);
    EEPROM.write(27, (unsigned char)(tiltMinPulse >> 8));

    EEPROM.write(28, (unsigned char)rollMaxPulse);
    EEPROM.write(29, (unsigned char)(rollMaxPulse >> 8));    

    EEPROM.write(30, (unsigned char)rollMinPulse);
    EEPROM.write(31, (unsigned char)(rollMinPulse >> 8)); 
  
    EEPROM.write(32, (unsigned char)htChannels[0]);
    EEPROM.write(33, (unsigned char)htChannels[1]);
    EEPROM.write(34, (unsigned char)htChannels[2]);
  
    // Saving gyro calibration values
    int temp = (int)(gyroOff[0] + 500.5);
    EEPROM.write(35, (unsigned char)temp);
    EEPROM.write(36, (unsigned char)(temp >> 8));   
  
    temp = (int)(gyroOff[1] + 500.5);
    EEPROM.write(37, (unsigned char)temp);
    EEPROM.write(38, (unsigned char)(temp >> 8));     

    temp = (int)(gyroOff[2] + 500.5);
    EEPROM.write(39, (unsigned char)temp);
    EEPROM.write(40, (unsigned char)(temp >> 8));    
  
    // Mark the memory to indicate that it has been
    // written. Used to determine if board is newly flashed
    // or not.
    EEPROM.write(0,8); 

    Serial1.println("Settings saved!");
}

//--------------------------------------------------------------------------------------
// Func: GetSettings
// Desc: Retrieves device settings from EEPROM.
//--------------------------------------------------------------------------------------
void GetSettings()
{  
    tiltRollBeta    = (float)EEPROM.read(1) / 100;
    panBeta         = (float)EEPROM.read(2) / 100;
    gyroWeightTiltRoll = (float)EEPROM.read(3) / 100;
    GyroWeightPan   = (float)EEPROM.read(4) / 100;  
  
    tiltInverse = 1;
    rollInverse = 1;
    panInverse = 1;

    unsigned char temp = EEPROM.read(5);
    if ( temp & HT_TILT_REVERSE_BIT )
    {
        tiltInverse = -1;
    }  
    if ( temp & HT_ROLL_REVERSE_BIT )
    {
        rollInverse = -1;
    }
    if ( temp & HT_PAN_REVERSE_BIT )
    {
        panInverse = -1;
    }

    // 6 unused

    servoPanCenter  = EEPROM.read(7) + (EEPROM.read(8) << 8);
    tiltFactor      = (float)(EEPROM.read(9) + (EEPROM.read(10) << 8)) / 10;
    panFactor       = (float)(EEPROM.read(11) + (EEPROM.read(12) << 8)) / 10;
    rollFactor       = (float)(EEPROM.read(13) + (EEPROM.read(14) << 8)) / 10;  

    // 15 unused

    servoTiltCenter = EEPROM.read(16) + (EEPROM.read(17) << 8);
    servoRollCenter = EEPROM.read(18) + (EEPROM.read(19) << 8);  
  
    panMaxPulse   = EEPROM.read(20) + (EEPROM.read(21) << 8);  
    panMinPulse   = EEPROM.read(22) + (EEPROM.read(23) << 8);    
  
    tiltMaxPulse  = EEPROM.read(24) + (EEPROM.read(25) << 8);  
    tiltMinPulse  = EEPROM.read(26) + (EEPROM.read(27) << 8);      
  
    rollMaxPulse  = EEPROM.read(28) + (EEPROM.read(29) << 8);  
    rollMinPulse  = EEPROM.read(30) + (EEPROM.read(31) << 8);        
  
    htChannels[0] = EEPROM.read(32);  
    htChannels[1] = EEPROM.read(33);  
    htChannels[2] = EEPROM.read(34);    
  
    gyroOff[0] = EEPROM.read(35) + (EEPROM.read(36) << 8) - 500; 
    gyroOff[1] = EEPROM.read(37) + (EEPROM.read(38) << 8) - 500; 
    gyroOff[2] = EEPROM.read(39) + (EEPROM.read(40) << 8) - 500;   
  
    magOffset[0] = EEPROM.read(200) + (EEPROM.read(201) << 8) - 2000;     
    magOffset[1] = EEPROM.read(202) + (EEPROM.read(203) << 8) - 2000;     
    magOffset[2] = EEPROM.read(204) + (EEPROM.read(205) << 8) - 2000;       
  
    accOffset[0] = EEPROM.read(206) + (EEPROM.read(207) << 8) - 2000;     
    accOffset[1] = EEPROM.read(208) + (EEPROM.read(209) << 8) - 2000;     
    accOffset[2] = EEPROM.read(210) + (EEPROM.read(211) << 8) - 2000;       
  
 
#if (DEBUG)
    DebugOutput();
#endif
}

//--------------------------------------------------------------------------------------
// Func: SaveMagData
// Desc: Stores magnetometer calibration info to EEPROM.
//--------------------------------------------------------------------------------------
void SaveMagData()
{
    int temp = (int)(magOffset[0] + 2000);
    EEPROM.write(200, (unsigned char)temp);
    EEPROM.write(201, (unsigned char)(temp >> 8));   
  
    temp = (int)(magOffset[1] + 2000);
    EEPROM.write(202, (unsigned char)temp);
    EEPROM.write(203, (unsigned char)(temp >> 8));   
  
    temp = (int)(magOffset[2] + 2000);
    EEPROM.write(204, (unsigned char)temp);
    EEPROM.write(205, (unsigned char)(temp >> 8));   
  
    Serial1.println("Mag offset saved!"); 
    Serial1.print(magOffset[0]);
    Serial1.print(", "); 
    Serial1.print(magOffset[1]);
    Serial1.print(", ");   
    Serial1.println(magOffset[2]); 
}

//--------------------------------------------------------------------------------------
// Func: SaveAccelData
// Desc: Stores accelerometer calibration data to EEPOM.
//--------------------------------------------------------------------------------------
void SaveAccelData()
{
    int temp = (int)(accOffset[0] + 2000);
    EEPROM.write(206, (unsigned char)temp);
    EEPROM.write(207, (unsigned char)(temp >> 8));   
  
    temp = (int)(accOffset[1] + 2000);
    EEPROM.write(208, (unsigned char)temp);
    EEPROM.write(209, (unsigned char)(temp >> 8));   
  
    temp = (int)(accOffset[2] + 2000);
    EEPROM.write(210, (unsigned char)temp);
    EEPROM.write(211, (unsigned char)(temp >> 8));   
  
    Serial1.println("Acc offset saved!"); 
    Serial1.print(accOffset[0]);
    Serial1.print(","); 
    Serial1.print(accOffset[1]);
    Serial1.print(",");   
    Serial1.println(accOffset[2]);  
}

//--------------------------------------------------------------------------------------
// Func: DebugOutput
// Desc: Outputs useful device/debug information to the Serial1 port.
//--------------------------------------------------------------------------------------
void DebugOutput()
{
    Serial1.println();  
    Serial1.println();
    Serial1.println();
    Serial1.println("------ Debug info------");

    Serial1.print("FW Version: ");
    Serial1.println(FIRMWARE_VERSION_FLOAT, 2);
    
    Serial1.print("tiltRollBeta: ");
    Serial1.println(tiltRollBeta); 

    Serial1.print("panBeta: ");
    Serial1.println(panBeta); 
 
    Serial1.print("gyroWeightTiltRoll: ");
    Serial1.println(gyroWeightTiltRoll); 

    Serial1.print("GyroWeightPan: ");
    Serial1.println(GyroWeightPan); 

    Serial1.print("servoPanCenter: ");
    Serial1.println(servoPanCenter); 
 
    Serial1.print("servoTiltCenter: ");
    Serial1.println(servoTiltCenter); 

    Serial1.print("servoRollCenter: ");
    Serial1.println(servoRollCenter); 

    Serial1.print("tiltFactor: ");
    Serial1.println(tiltFactor); 

    Serial1.print("panFactor: ");
    Serial1.println(panFactor);  
 
    Serial1.print("Gyro offset stored ");
    Serial1.print(gyroOff[0]);
    Serial1.print(",");   
    Serial1.print(gyroOff[1]);
    Serial1.print(",");      
    Serial1.println(gyroOff[2]);    
 
    Serial1.print("Mag offset stored ");
    Serial1.print(magOffset[0]);
    Serial1.print(",");   
    Serial1.print(magOffset[1]);
    Serial1.print(",");      
    Serial1.println(magOffset[2]);
 
    Serial1.print("Acc offset stored ");
    Serial1.print(accOffset[0]);
    Serial1.print(",");   
    Serial1.print(accOffset[1]);
    Serial1.print(",");      
    Serial1.println(accOffset[2]);
 
    SensorInfoPrint();    
}



