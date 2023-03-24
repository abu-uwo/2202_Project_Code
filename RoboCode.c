#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Uncomment keywords to enable debugging output
#define DEBUG_DRIVE_SPEED 1
#define DEBUG_ENCODER_COUNT 1

// Port pin constants

#define MODE_BUTTON         0   // GPIO0  pin 27 for Push Button 1

#define FORWARD         4  // GPIO35 pin 28 (J35) Motor 1 A
#define BACKWARD        5  // GPIO36 pin 29 (J36) Motor 1 B
#define RISE            45  // GPIO37 pin 30 (J37) Motor 2 A
#define FALL            35  // GPIO38 pin 31 (J38) Motor 2 B

// github test

#define FRONT_RACK_LARGE_EXTEND     6  // When DIP Switch S1-1 is on, Left encoder A signal is connected to pin 8 GPIO15 (J15)
                                // When DIP Switch S1-1 is off, J15 can be used as analog AD2-4
#define FRONT_RACK_LARGE_RETRACT    36  // When DIP Switch S1-2 is on, Left encoder B signal is connected to pin 9 GPIO16 (J16)
                                // When DIP Switch S1-2 is off, J16 can be used as analog AD2-5
#define FRONT_RACK_SMALL_EXTEND     7  // When DIP Switch S1-3 is on, Left encoder Direction signal is connected to pin 10 GPIO17 (J17)
                                // When DIP Switch S1-3 is off, J17 can be used as analog AD2-6
#define FRONT_RACK_SMALL_RETRACT    9  // When DIP Switch S1-4 is on, Left encoder Speed signal is connected to pin 11 GPIO18 (J18)
                                // When DIP Switch S1-4 is off, J18 can be used as analog AD2-7


#define REAR_RACK_LARGE_EXTEND      37  // When DIP Switch S1-7 is on, Right encoder A signal is connected to pin 19 GPIO11 (J11)
                                // When DIP Switch S1-7 is off, J11 can be used as analog AD2-0
#define REAR_RACK_LARGE_RETRACT     38  // When DIP Switch S1-8 is on, Right encoder B signal is connected to pin 20 GPIO12 (J12)
                                // When DIP Switch S1-8 is off, J12 can be used as analog AD2-1
#define REAR_RACK_SMALL_EXTEND      8  // When DIP Switch S1-9 is on, Right encoder Direction signal is connected to pin 21 GPIO13 (J13)
                                // When DIP Switch S1-9 is off, J13 can be used as analog AD2-2
#define REAR_RACK_SMALL_RETRACT     2  // When DIP Switch S1-10 is on, Right encoder Speed signal is connected to pin 22 GPIO14 (J14)
                                // When DIP Switch S1-10 is off, J14 can be used as analog AD2-3

#define SMART_LED           21  // When DIP Switch S1-11 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1   // Number of SMART LEDs in use



const int ci_Display_Update = 100;                                            // Update interval for Smart LED in milliseconds

boolean bt_3_S_Time_Up = false;                                               // 3 second timer elapsed flag
unsigned int ui_Mode_PB_Debounce;                                             // Pushbutton debounce timer count9

unsigned long ul_3_Second_timer = 0;                                          // 3 second timer count in milliseconds
unsigned long ul_Display_Time;                                                // Heartbeat LED update timer
unsigned long ul_Previous_Micros;                                             // Last microsecond count
unsigned long ul_Current_Micros;                                              // Current microsecond count
unsigned long t1_curr=0;
unsigned long t1_prev=0;


Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};

unsigned int  ui_Robot_Mode_Index = 0;                                        // Robot operational state
unsigned int  ui_Mode_Indicator[7] = {                                        // Colours for different modes
  SmartLEDs.Color(255,0,0),                                                   //   Red - Stop
  SmartLEDs.Color(0,255,0),                                                   //   Green - Run
  SmartLEDs.Color(0,0,255),                                                   //   Blue - Test stepper
  SmartLEDs.Color(255,255,0),                                                 //   Yellow - Test claw servo
  SmartLEDs.Color(255,0,255),                                                 //   Magenta - Test shoulder servo
  SmartLEDs.Color(0,255,255),                                                 //   Cyan - Test IR receiver
  SmartLEDs.Color(255,165,0)                                                  //   Orange - empty case
};

void Indicator();                                                             // For mode/heartbeat on Smart LED

void setup() {
  // put your setup code here, to run once:
   SmartLEDs.begin();                                                         // Initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                         // Clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                         // Set pixel colors to 'off'
   SmartLEDs.show();                                                          // Send the updated pixel colors to the hardware

   ui_Mode_PB_Debounce = 0;                                                   // Reset debounce timer count
   pinMode(FORWARD, OUTPUT);
   pinMode(BACKWARD, OUTPUT);

   pinMode(RISE, OUTPUT);
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

   ul_Current_Micros = micros();                                              // Get current time in microseconds
   if ((ul_Current_Micros - ul_Previous_Micros) >= 1000)                      // Enter when 1 ms has elapsed
   {
      ul_Previous_Micros = ul_Current_Micros;                                 // Record current time in microseconds

      // 3 second timer, counts 3000 milliseconds
      ul_3_Second_timer = ul_3_Second_timer + 1;                              // Increment 3 second timer count
      if(ul_3_Second_timer > 3000)                                            // If 3 seconds have elapsed
      {
         ul_3_Second_timer = 0;                                               // Reset 3 second timer count
         bt_3_S_Time_Up = true;                                               // Indicate that 3 seconds have elapsed
      }

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
               ul_3_Second_timer = 0;                                         // Reset 3 second timer count
               bt_3_S_Time_Up = false;                                        // Reset 3 second timer
            }
         }
      }



      // modes
      // 0 = Default after power up/reset. Robot is stopped.
      // 1 = Press mode button once to enter. Run robot.
      // 2 = Press mode button twice to enter. Test stepper motor.
      // 3 = Press mode button three times to enter. Test claw servo.
      // 4 = Press mode button four times to enter. Test arm shoulder servo.
      // 5 = Press mode button five times to enter. Test IR receiver.
      // 6 = Press mode button six times to enter.  //add your code to do something
      switch(ui_Robot_Mode_Index)
      {
         case 0: // Robot stopped
         {

            digitalWrite(FORWARD, LOW);
            digitalWrite(BACKWARD, LOW);

            digitalWrite(RISE, LOW); //HIGH POWER
            digitalWrite(FALL, LOW); //HIGH POWER

            digitalWrite(FRONT_RACK_LARGE_EXTEND, LOW);
            digitalWrite(FRONT_RACK_LARGE_RETRACT, LOW); //HIGH POWER
            digitalWrite(FRONT_RACK_SMALL_EXTEND, LOW);
            digitalWrite(FRONT_RACK_SMALL_RETRACT, LOW);

            digitalWrite(REAR_RACK_LARGE_EXTEND, LOW);  //HIGH POWER
            digitalWrite(REAR_RACK_LARGE_RETRACT, LOW); //HIGH POWER
            digitalWrite(REAR_RACK_SMALL_EXTEND, LOW);
            digitalWrite(REAR_RACK_SMALL_RETRACT, LOW); //HIGH POWER

            break;


         }

         case 1: // Run robot
         {
          t1_curr = millis();

           if((t1_curr-t1_prev)>=50000)
           {
             t1_prev = t1_curr;
             ui_Robot_Mode_Index=0;
           }

           else
           {
             if((t1_curr-t1_prev)>=1000 && (t1_curr-t1_prev)<5000)
             {
              digitalWrite(FORWARD, HIGH);

             }
             else if ((t1_curr-t1_prev)>=5000 && (t1_curr-t1_prev)<8000)
             {
              digitalWrite(FORWARD, LOW);

              digitalWrite(FRONT_RACK_LARGE_EXTEND, HIGH);
              digitalWrite(FRONT_RACK_SMALL_EXTEND, HIGH);


             }
             else if((t1_curr-t1_prev)>=8000 && (t1_curr-t1_prev)<15000)
             {
              digitalWrite(FRONT_RACK_LARGE_EXTEND, LOW);
              digitalWrite(FRONT_RACK_SMALL_EXTEND, LOW);

              digitalWrite(RISE, HIGH);


             }
             else if ((t1_curr-t1_prev)>=15000 && (t1_curr-t1_prev)<31000)
             {
              digitalWrite(RISE, LOW);

              digitalWrite(FRONT_RACK_LARGE_RETRACT, HIGH);
              digitalWrite(REAR_RACK_LARGE_EXTEND, HIGH);



             }
             else if ((t1_curr-t1_prev)>=31000 && (t1_curr-t1_prev)<41000)
             {
              digitalWrite(FRONT_RACK_LARGE_RETRACT, LOW);
              digitalWrite(REAR_RACK_LARGE_EXTEND, LOW);

              digitalWrite(FRONT_RACK_SMALL_RETRACT, HIGH);
              digitalWrite(REAR_RACK_SMALL_EXTEND, HIGH);


             }
             else if ((t1_curr-t1_prev)>=41000 && (t1_curr-t1_prev)<46000)
             {
              digitalWrite(FRONT_RACK_SMALL_RETRACT, LOW);
              digitalWrite(REAR_RACK_SMALL_EXTEND, LOW);

              digitalWrite(REAR_RACK_LARGE_RETRACT, HIGH);
              digitalWrite(REAR_RACK_SMALL_RETRACT, HIGH);

             }
             else if ((t1_curr-t1_prev)>=46000 && (t1_curr-t1_prev)<46500)
             {
              digitalWrite(REAR_RACK_LARGE_RETRACT, LOW);
              digitalWrite(REAR_RACK_SMALL_RETRACT, LOW);

              digitalWrite(FALL, HIGH);

             }
             else if ((t1_curr-t1_prev)>=46000 && (t1_curr-t1_prev)<47000)
             {
              digitalWrite(FALL, LOW);

              digitalWrite(FORWARD, HIGH);

             }
             else if ((t1_curr-t1_prev)>=46000 && (t1_curr-t1_prev)<48000)
             {
               digitalWrite(FORWARD, LOW);
               digitalWrite(BACKWARD, HIGH);

             }
             else
             {
               digitalWrite(BACKWARD, LOW);
             }

           }

           break;
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
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator()
{
  SmartLEDs.setPixelColor(0, ui_Mode_Indicator[ui_Robot_Mode_Index]);         // Set pixel colors to = mode
  SmartLEDs.show();                                                           // Send the updated pixel colors to the hardware
}
