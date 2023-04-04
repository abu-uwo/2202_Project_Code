 #include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Uncomment keywords to enable debugging output
#define DEBUG_DRIVE_SPEED 1
#define DEBUG_ENCODER_COUNT 1

// Port pin constants

#define MODE_BUTTON     0   // GPIO0  pin 27 for Push Button 1

#define FORWARD_A       35      //List of probably inaccurate motor names
#define BACKWARD_A      36
#define FORWARD_B       37
#define BACKWARD_B      38

#define RISE            2
#define FALL            1

#define Ultrasonic      45    // This sensor determines if the ultrasonic sensor should steer left or right if it strays


#define ul_U_steer_ping 4 // wall
#define ul_U_steer_data 5

#define ul_U_check_ping 6 // table
#define ul_U_check_data 7

// github test

#define FRONT_RACK_LARGE_EXTEND     11  // When DIP Switch S1-1 is on, Left encoder A signal is connected to pin 8 GPIO15 (J15)
                                // When DIP Switch S1-1 is off, J15 can be used as analog AD2-4
#define FRONT_RACK_LARGE_RETRACT    12 // When DIP Switch S1-2 is on, Left encoder B signal is connected to pin 9 GPIO16 (J16)
                                // When DIP Switch S1-2 is off, J16 can be used as analog AD2-5
#define FRONT_RACK_SMALL_EXTEND     13  // When DIP Switch S1-3 is on, Left encoder Direction signal is connected to pin 10 GPIO17 (J17)
                                // When DIP Switch S1-3 is off, J17 can be used as analog AD2-6
#define FRONT_RACK_SMALL_RETRACT    14  // When DIP Switch S1-4 is on, Left encoder Speed signal is connected to pin 11 GPIO18 (J18)
                                // When DIP Switch S1-4 is off, J18 can be used as analog AD2-7


#define REAR_RACK_LARGE_EXTEND      15  // When DIP Switch S1-7 is on, Right encoder A signal is connected to pin 19 GPIO11 (J11)
                                // When DIP Switch S1-7 is off, J11 can be used as analog AD2-0
#define REAR_RACK_LARGE_RETRACT     16  // When DIP Switch S1-8 is on, Right encoder B signal is connected to pin 20 GPIO12 (J12)
                                // When DIP Switch S1-8 is off, J12 can be used as analog AD2-1
#define REAR_RACK_SMALL_EXTEND      17  // When DIP Switch S1-9 is on, Right encoder Direction signal is connected to pin 21 GPIO13 (J13)
                                // When DIP Switch S1-9 is off, J13 can be used as analog AD2-2
#define REAR_RACK_SMALL_RETRACT     18  // When DIP Switch S1-10 is on, Right encoder Speed signal is connected to pin 22 GPIO14 (J14)
                                // When DIP Switch S1-10 is off, J14 can be used as analog AD2-3

#define SMART_LED           21  // When DIP Switch S1-11 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1   // Number of SMART LEDs in use



const int ci_Display_Update = 100;                                            // Update interval for Smart LED in milliseconds

int numOfPings=0;

boolean bt_3_S_Time_Up = false;                                               // 3 second timer elapsed flag
unsigned int ui_Mode_PB_Debounce;                                             // Pushbutton debounce timer count
unsigned int ui_RunMode=0;                                                    // Index count during performance

unsigned long ul_3_Second_timer = 0;                                          // 3 second timer count in milliseconds
unsigned long ul_Display_Time;                                                // Heartbeat LED update timer
unsigned long ul_Previous_Micros;                                             // Last microsecond count
unsigned long ul_Current_Micros;                                              // Current microsecond count

unsigned long t1_curr=0;
unsigned long t1_prev=0;
unsigned long t2_curr=0;                                                      // Ultrasonic sensor timer
unsigned long t2_prev=0;                                                      // Ultrasonic sensor timer
unsigned long t3_curr=0;                                                      // Ping timer
unsigned long t3_prev=0;                                                      // Ping timer


unsigned long ul_echo_steer=0;
unsigned long ul_echo_check=0;                                                     // Echo time of sonic bursts
unsigned long ul_echo_steer_ref=0;
unsigned long ul_echo_check_ref=0;                                                 // Reference echo time of sonic bursts



Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};

unsigned int  ui_Robot_Mode_Index = 0;                                        // Robot operational state
unsigned int  ui_Mode_Indicator[7] = {                                        // Colours for different modes
  SmartLEDs.Color(255,0,0),                                                   //   Red - Stop
  SmartLEDs.Color(45,196,227),                                                //   Bright Cyan- GO
  SmartLEDs.Color(0,0,255),                                                   //   Blue - Test stepper
  SmartLEDs.Color(255,255,0),                                                 //   Yellow - Test claw servo
  SmartLEDs.Color(255,0,255),                                                 //   Magenta - Test shoulder servo
  SmartLEDs.Color(0,255,255),                                                 //   Cyan - Test IR receiver
  SmartLEDs.Color(255,165,0)                                                  //   Orange - empty case
};

Motion Bot= Motion();                                                          // Instance of motion class
void Indicator();                                                             // For mode/heartbeat on Smart LED

void setup() {
  // put your setup code here, to run once:
   Bot.driveBegin("D1", FORWARD_A, BACKWARD_A, FORWARD_B, BACKWARD_B);        // Set up motors as Drive 1

   SmartLEDs.begin();                                                         // Initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                         // Clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                         // Set pixel colors to 'off'
   SmartLEDs.show();                                                          // Send the updated pixel colors to the hardware

   ui_Mode_PB_Debounce = 0;
                                                                              // Reset debounce timer count
              
   pinMode(Ultrasonic, OUTPUT); // voltage pin for table
   digitalWrite(Ultrasonic, LOW);

   pinMode(ul_U_steer_ping, OUTPUT); // input signal for steering
   pinMode(ul_U_steer_data, INPUT); // echo for steering

   pinMode(ul_U_check_ping, OUTPUT); // input signal for table check
   pinMode(ul_U_check_data, INPUT); // echo for table check
                                                                              
   pinMode(RISE, OUTPUT); //the same pin for both linear actuators 
   pinMode(FALL, OUTPUT);

   pinMode(FRONT_RACK_LARGE_EXTEND, OUTPUT);
   pinMode(FRONT_RACK_LARGE_RETRACT, OUTPUT);
   pinMode(FRONT_RACK_SMALL_EXTEND, OUTPUT);
   pinMode(FRONT_RACK_SMALL_RETRACT, OUTPUT);

   pinMode(REAR_RACK_LARGE_EXTEND, OUTPUT);
   pinMode(REAR_RACK_LARGE_RETRACT, OUTPUT);
   pinMode(REAR_RACK_SMALL_EXTEND, OUTPUT);
   pinMode(REAR_RACK_SMALL_RETRACT, OUTPUT);

   pinMode(MODE_BUTTON, INPUT_PULLUP);                                        //set up mode button with internal pullup

}
void loop()
{
    t1_curr = millis();                                                       // Start robot timer
    t2_curr = micros();                                                       // Start sensor timer
    t3_curr = millis();                                                       // Start ping timer
    digitalWrite(Ultrasonic, HIGH);


   ul_Current_Micros = micros();                                              // Get current time in microseconds
   if ((ul_Current_Micros - ul_Previous_Micros) >= 1000)                      // Enter when 1 ms has elapsed
   {
      ul_Previous_Micros = ul_Current_Micros;                                 // Record current time in microseconds

      // Mode pushbutton debounce and toggle
      if(!digitalRead(MODE_BUTTON))                                           // If pushbutton GPIO goes LOW (nominal push)
      {
         // Start debounce
         if(ui_Mode_PB_Debounce <= 25)                                        // 25 millisecond debounce time
         {
            ui_Mode_PB_Debounce = ui_Mode_PB_Debounce + 1;                    // Increment debounce timer count
            if(ui_Mode_PB_Debounce > 25)                                      // If held for at least 25 mS
            {
               ui_Mode_PB_Debounce = 1000;                                    // Change debounce timer count to 1 second
            }
         }
         if(ui_Mode_PB_Debounce >= 1000)                                      // Maintain 1 second timer count until release
         {
            ui_Mode_PB_Debounce = 1000;
         }
      }
      else                                                                    // Pushbutton GPIO goes HIGH (nominal release)
      {
         if(ui_Mode_PB_Debounce <= 26)                                        // If release occurs within debounce interval
         {
            ui_Mode_PB_Debounce = 0;                                          // Reset debounce timer count
         }
         else
         {
            ui_Mode_PB_Debounce = ui_Mode_PB_Debounce + 1;                    // Increment debounce timer count
            if(ui_Mode_PB_Debounce >= 1025)                                   // If pushbutton was released for 25 mS
            {
               ui_Mode_PB_Debounce = 0;                                       // Reset debounce timer count
               ui_Robot_Mode_Index++;                                         // Switch to next mode
               ui_Robot_Mode_Index = ui_Robot_Mode_Index % 2;                 // Keep mode index between 0 and 1
               t1_prev=t1_curr;
               t2_prev=t2_curr;
               ui_RunMode=0;
               numOfPings=0;
            }
         }
      }
   }


      // modes
      // 0 = Default after power up/reset. Robot is stopped.
      // 1 = Press mode button once to enter. Run robot.
   switch(ui_Robot_Mode_Index)
    {
        case 0: // Robot stopped
        {
            digitalWrite(RISE, LOW); //HIGH POWER 5V
            digitalWrite(FALL, LOW); //HIGH POWER 5V

            digitalWrite(FRONT_RACK_LARGE_EXTEND, LOW);
            digitalWrite(FRONT_RACK_LARGE_RETRACT, LOW);
            digitalWrite(FRONT_RACK_SMALL_EXTEND, LOW);
            digitalWrite(FRONT_RACK_SMALL_RETRACT, LOW);

            digitalWrite(REAR_RACK_LARGE_EXTEND, LOW);
            digitalWrite(REAR_RACK_LARGE_RETRACT, LOW);
            digitalWrite(REAR_RACK_SMALL_EXTEND, LOW);
            digitalWrite(REAR_RACK_SMALL_RETRACT, LOW);


            Bot.Stop("D1");
            if (t3_curr - t3_prev >= 20)
            {
             digitalWrite(ul_U_steer_ping, HIGH);
             digitalWrite(ul_U_check_ping, HIGH);
             t3_prev = t3_curr;
             t2_prev = t2_curr;
             delayMicroseconds(10);
            }

            if(digitalRead(ul_U_steer_ping)==HIGH && digitalRead(ul_U_check_ping)==HIGH)
            {
                
                    digitalWrite(ul_U_steer_ping, LOW);
                    digitalWrite(ul_U_check_ping, LOW);
                    ul_echo_check_ref=pulseIn(ul_U_check_data, HIGH, 5000);
                    ul_echo_steer_ref=pulseIn(ul_U_steer_data, HIGH, 5000);
                    t2_prev = t2_curr;
                
            } //determine the "distance" (not exactly distance because ul_echo of time) from wall before the robot starts moving
           numOfPings=0;
           
            break;
         }

        case 1: // Run robot
        {
         switch(ui_RunMode)
          {
            case 0:
            {
                
             if ((t3_curr - t3_prev) >= 20)
              {
              digitalWrite(ul_U_steer_ping, HIGH);
              digitalWrite(ul_U_check_ping, HIGH);
              t3_prev = t3_curr;
              t2_prev = t2_curr;
              delayMicroseconds(10);
              }

             if(digitalRead(ul_U_steer_ping)==HIGH && digitalRead(ul_U_check_ping)==HIGH)
             {
              
                digitalWrite(ul_U_steer_ping, LOW);
                digitalWrite(ul_U_check_ping, LOW);
                ul_echo_check=pulseIn(ul_U_check_data, HIGH, 4000);
                ul_echo_steer=pulseIn(ul_U_steer_data, HIGH, 7000);
                t2_prev = t2_curr;
                numOfPings++;
              
             }


              if(numOfPings!=0)
              {
                if (ul_echo_steer >= (ul_echo_steer_ref+70))
                {
                    Bot.Forward("D1",170,255);
                } //if more than 2 cm deviation to the right assuming the wall is to the left
                else if (ul_echo_steer <= (ul_echo_steer_ref-70))
                {
                    Bot.Forward("D1",255,170);
                }//if more than 2 cm deviation to the left assuming the wall is to the left
                else
                {
                    Bot.Forward("D1",255,255);
                }

                if (ul_echo_check==0)
                {
                 Bot.Stop("D1");
                 t1_prev=t1_curr;
                 ui_RunMode=1;
                 numOfPings=0;
                }
              }
                break;
            }
            case 1:
            {
                if ((t1_curr - t1_prev) < 2450)
                {
                   digitalWrite(FRONT_RACK_LARGE_EXTEND, HIGH);
                   digitalWrite(FRONT_RACK_SMALL_EXTEND, HIGH);
                }
                else if ((t1_curr - t1_prev) >= 2450 && (t1_curr - t1_prev) < 4450)
                {
                  digitalWrite(FRONT_RACK_LARGE_EXTEND, LOW);
                }
                else
                {
                   digitalWrite(FRONT_RACK_LARGE_EXTEND, LOW);
                   digitalWrite(FRONT_RACK_SMALL_EXTEND, LOW);
                   t1_prev=t1_curr;
                   ui_RunMode=2;
                }
                break;
            }
            case 2:
            {
                 //Rise for 12 seconds and then stop
                 if ((t1_curr - t1_prev) < 19000)
                 {
                  digitalWrite(RISE, HIGH);
                 }
                 else
                 {
                  digitalWrite(RISE, LOW);
                  t1_prev=t1_curr;
                  ui_RunMode=3;
                 }
                 break;
            }

            case 3:
            {
                if ((t1_curr - t1_prev) < 4150)
                {
                    digitalWrite(FRONT_RACK_SMALL_RETRACT, HIGH);
                    digitalWrite(REAR_RACK_SMALL_EXTEND, HIGH);
                }
                else
                {
                    digitalWrite(FRONT_RACK_SMALL_RETRACT, LOW);
                    digitalWrite(REAR_RACK_SMALL_EXTEND, LOW);
                    t1_prev=t1_curr;
                    ui_RunMode=4;
                }
                break;
            }
            case 4:
            {
                if((t1_curr - t1_prev) < 2100)
                {
                    digitalWrite(FRONT_RACK_LARGE_RETRACT, HIGH);
                    digitalWrite(REAR_RACK_LARGE_EXTEND, HIGH);
                }
                else if((t1_curr - t1_prev) <= 4150)
                {
                  digitalWrite(REAR_RACK_LARGE_EXTEND, LOW);
                }
                else
                {
                    digitalWrite(FRONT_RACK_LARGE_RETRACT, LOW);
                    digitalWrite(REAR_RACK_LARGE_EXTEND, LOW);
                    t1_prev=t1_curr;
                    ui_RunMode=5;
                }
                break;
            }
            case 5:
            {
                if((t1_curr - t1_prev) < 4550)
                {
                    digitalWrite(REAR_RACK_LARGE_RETRACT, HIGH);
                    digitalWrite(REAR_RACK_SMALL_RETRACT, HIGH);
                }
                else
                {
                    digitalWrite(REAR_RACK_LARGE_RETRACT, LOW);
                    digitalWrite(REAR_RACK_SMALL_RETRACT, LOW);
                    t1_prev=t1_curr;
                    ui_RunMode=6;
                }
                break;
            }
            case 6:
            {
                if((t1_curr - t1_prev) < 19000)
                {
                    digitalWrite(FALL, HIGH);
                }
                else
                {
                    digitalWrite(FALL, LOW);
                    t1_prev=t1_curr;
                    ui_RunMode=7;
                }
                break;
            }
            case 7:
            {
                    
              if (t3_curr - t3_prev >= 20)
              {
                digitalWrite(ul_U_steer_ping, HIGH);
                digitalWrite(ul_U_check_ping, HIGH);
                t3_prev = t3_curr;
                t2_prev = t2_curr;
                delayMicroseconds(10);
              }

              if(digitalRead(ul_U_steer_ping)==HIGH && digitalRead(ul_U_check_ping)==HIGH)
              {
               
                  digitalWrite(ul_U_steer_ping, LOW);
                  digitalWrite(ul_U_check_ping, LOW);
                  ul_echo_check=pulseIn(ul_U_check_data, HIGH, 4000);
                  ul_echo_steer=pulseIn(ul_U_steer_data, HIGH, 7000);
                  t2_prev = t2_curr;
                  numOfPings++;
                
               }
              if(numOfPings != 0)
              {
                if (ul_echo_steer >= (ul_echo_steer_ref+70))
                {
                    Bot.Forward("D1",170,255);
                } //if more than 2 cm deviation to the right assuming the wall is to the left
                else if (ul_echo_steer <= (ul_echo_steer_ref-70))
                {
                    Bot.Forward("D1",255,170);
                }//if more than 2 cm deviation to the left assuming the wall is to the left
                else
                {
                    Bot.Forward("D1",255,255);
                }
                if (ul_echo_check==0)
                {
                 t1_prev=t1_curr;
                 ui_RunMode=8;
                 numOfPings=0;
                }
              }
                break;
            }
            case 8:
            {
              if (t3_curr - t3_prev >= 20)
              {
                digitalWrite(ul_U_steer_ping, HIGH);
                digitalWrite(ul_U_check_ping, HIGH);
                t3_prev = t3_curr;
                t2_prev = t2_curr;
                delayMicroseconds(10);
              }

              if(digitalRead(ul_U_steer_ping)==HIGH && digitalRead(ul_U_check_ping)==HIGH)
              {
               
                  digitalWrite(ul_U_steer_ping, LOW);
                  digitalWrite(ul_U_check_ping, LOW);
                  ul_echo_check=pulseIn(ul_U_check_data, HIGH, 4000);
                  ul_echo_steer=pulseIn(ul_U_steer_data, HIGH, 7000);
                  t2_prev = t2_curr;
               
              }
              if(numOfPings != 0)
              {
                if(t1_curr - t1_prev < 12000) // replace with a mapping function
                {
                    if (ul_echo_steer >= (ul_echo_steer_ref+70))
                    {
                        Bot.Reverse("D1",170,255);
                    }//if more than 2 cm deviation to the right assuming the wall is to the left
                    else if (ul_echo_steer <= (ul_echo_steer_ref-70))
                    {
                        Bot.Reverse("D1",255,170);
                    }//if more than 2 cm deviation to the left assuming the wall is to the left
                    else
                    {
                        Bot.Reverse("D1",255,255);
                    }
                }
                else
                {
                 t1_prev=t1_curr;
                 ui_RunMode=0;
                 ui_Robot_Mode_Index=0;
                 numOfPings=0;
                }
              }
                break;
            }
         }
      }
  }

      // Update brightness of heartbeat display on SmartLED
      ul_Display_Time++;                                                      // Count milliseconds
      if(ul_Display_Time > ci_Display_Update)                                 // When display update period has passed
      {
         ul_Display_Time = 0;                                                 // Reset display counter
         LEDBrightnessIndex++;                                                // Shift to next brightness level
         if(LEDBrightnessIndex > sizeof(LEDBrightnessLevels))                 // If all defined levels have been used
         {
            LEDBrightnessIndex = 0;                                           // Reset to starting brightness
         }
         SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);    // Set brightness of heartbeat LED
         Indicator();                                                         // Update LED
      }
   
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator()
{
  SmartLEDs.setPixelColor(0, ui_Mode_Indicator[ui_Robot_Mode_Index]);         // Set pixel colors to = mode
  SmartLEDs.show();                                                           // Send the updated pixel colors to the hardware
}