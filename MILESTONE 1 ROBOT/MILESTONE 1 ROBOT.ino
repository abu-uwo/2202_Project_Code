#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Uncomment keywords to enable debugging output
#define DEBUG_DRIVE_SPEED 0
#define DEBUG_ENCODER_COUNT 0

// Port pin constants

#define MODE_BUTTON         0   // GPIO0  pin 27 for Push Button 1
                                   
#define LEFT_MOTOR_A        35  // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36  // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37  // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38  // GPIO38 pin 31 (J38) Motor 2 B
                                   
#define FRONT_ACT_A         4  
#define FRONT_ACT_B         5
#define REAR_ACT_A          6
#define REAR_ACT_B          7  

#define FRONT_RACK_A         15  
#define FRONT_RACK_AA        16
#define FRONT_RACK_B         17 
#define FRONT_RACK_BB        18

#define REAR_RACK_A          11
#define REAR_RACK_AA         12
#define REAR_RACK_B          13
#define REAR_RACK_BB         14
                                                                

// Constants

const int ci_Display_Update = 100;                                            // Update interval for Smart LED in milliseconds



//
//=====================================================================================================================

// Variables


boolean bt_3_S_Time_Up = false;                                               // 3 second timer elapsed flag


unsigned int ui_Mode_PB_Debounce;                                             // Pushbutton debounce timer count

unsigned long ul_3_Second_timer = 0;                                          // 3 second timer count in milliseconds

unsigned long ul_Current_millis;
unsigned long ul_Previous_millis = 0;
unsigned long ul_Display_Time;                                                // Heartbeat LED update timer
unsigned long ul_Previous_Micros;                                             // Last microsecond count
unsigned long ul_Current_Micros;                                              // Current microsecond count

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
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



// Function Declarations
void Indicator();                                                             // For mode/heartbeat on Smart LED

void setup()
{
   Serial.begin(9600);
   // Set up SmartLED
   SmartLEDs.begin();                                                         // Initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                         // Clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                         // Set pixel colors to 'off'
   SmartLEDs.show();                                                          // Send the updated pixel colors to the hardware

   // Set up mode pushbutton
   ui_Mode_PB_Debounce = 0;                                                   // Reset debounce timer count
   
   //Stepper motor 
   pinMode(LEFT_MOTOR_A, OUTPUT);                         //LEFT FORWARD                        
   pinMode(LEFT_MOTOR_B, OUTPUT);                         //LEFT BACKWARD                  
   pinMode(RIGHT_MOTOR_A, OUTPUT);                        //RIGHT FORWARD                    
   pinMode(RIGHT_MOTOR_B, OUTPUT);                        //RIGHT BACKWARD                 

   pinMode(FRONT_RACK_A, OUTPUT);                                             
   pinMode(FRONT_RACK_AA, OUTPUT); 
   pinMode(FRONT_RACK_B, OUTPUT);                                             
   pinMode(FRONT_RACK_BB, OUTPUT);                                            
   pinMode(REAR_RACK_A, OUTPUT);                                            
   pinMode(REAR_RACK_AA, OUTPUT);                                              
   pinMode(REAR_RACK_B, OUTPUT);                                            
   pinMode(REAR_RACK_BB, OUTPUT); 

   pinMode(FRONT_ACT_A, OUTPUT);
   pinMode(FRONT_ACT_B, OUTPUT);
   pinMode(REAR_ACT_A, OUTPUT);
   pinMode(REAR_ACT_B, OUTPUT);   
                                                  // Set up stepper direction pin as output

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
               ul_3_Second_timer = 0;                                         // Reset 3 second timer count
               bt_3_S_Time_Up = false;                                        // Reset 3 second timer         
            }
         }
      }
  
      // check if drive motors should be powered
    

      // modes 
      // 0 = Default after power up/reset. Robot is stopped.
      // 1 = Press mode button once to enter. Run robot.
      ui_Robot_Mode_Index = ui_Robot_Mode_Index % 2 ;                 // Keep mode index between 0 and 1
      switch(ui_Robot_Mode_Index)
      {
         case 0: // Robot stopped
         {
              digitalWrite(LEFT_MOTOR_A, LOW);                                             
              digitalWrite(LEFT_MOTOR_B, LOW);                                            
              digitalWrite(RIGHT_MOTOR_A, LOW);                                            
              digitalWrite(RIGHT_MOTOR_B, LOW);                                          

              digitalWrite(FRONT_RACK_A, LOW);                                             
              digitalWrite(FRONT_RACK_AA, LOW; 
              digitalWrite(FRONT_RACK_B, LOW);                                             
              digitalWrite(FRONT_RACK_BB, LOW);                                            
              digitalWrite(REAR_RACK_A, LOW);                                            
              digitalWrite(REAR_RACK_AA, LOW;                                              
              digitalWrite(REAR_RACK_B, LOW);                                            
              digitalWrite(REAR_RACK_BB, LOW); 

              digitalWrite(FRONT_ACT_A, LOW);
              digitalWrite(FRONT_ACT_B, LOW);
              digitalWrite(REAR_ACT_A, LOW);
              digitalWrite(REAR_ACT_B,LOW);

            break;
         }  
      
         case 1: // Run robot
         {
            if(bt_3_S_Time_Up)                                                // Pause for 3 sec before running Case 1 code
            {
              
              digitalWrite(LEFT_MOTOR_A, HIGH);            //FORWARD MOTORS ARE TURNED ON
              digitalWrite(RIGHT_MOTOR_A, HIGH);
              delay(5000);
              digitalWrite(LEFT_MOTOR_A, LOW);             //FORWARD MOTORS ARE TURNED OFF
              digitalWrite(RIGHT_MOTOR_A, LOW);
              digitalWrite(FRONT_RACK_A, HIGH);            //EXTEND FRONT RACK 
              digitalWrite(FRONT_RACK_AA, HIGH);
              delay(8000);
              digitalWrite(FRONT_RACK_A, LOW);             //STOP EXTENDING FRONT RACK 
              digitalWrite(FRONT_RACK_AA, LOW);
              digitalWrite(FRONT_ACT_A, HIGH);             //EXTEND LINEAR ACTUATORS
              digitalWrite(REAR_ACT_A, HIGH);
              delay(10000);
              digitalWrite(FRONT_ACT_A, LOW);              //STOP EXTENDING LINEAR ACTUATORS
              digitalWrite(REAR_ACT_A, LOW);
              digitalWrite(REAR_RACK_A, HIGH);             //EXTEND REAR RACK, RETRACT FRONT RACK
              digitalWrite(REAR_RACK_AA, HIGH);
              digitalWrite(FRONT_RACK_B, HIGH);
              digitalWrite(FRONT_RACK_BB, HIGH);
              delay(8000);
              digitalWrite(REAR_RACK_A, LOW);             //STOP
              digitalWrite(REAR_RACK_AA, LOW);
              digitalWrite(FRONT_RACK_B, LOW);
              digitalWrite(FRONT_RACK_BB, LOW);
              digitalWrite(FRONT_ACT_B, HIGH);             //RETRACT LINEAR ACTUATORS
              digitalWrite(REAR_ACT_B, HIGH);
              delay(10000);
              digitalWrite(REAR_RACK_B, HIGH);            //RETRACT REAT RACK
              digitalWrite(REAR_RACK_BB, HIGH);
              delay(10000);
              digitalWrite(REAR_RACK_B, LOW);            //STOP RETRACTION
              digitalWrite(REAR_RACK_BB, LOW);
              digitalWrite(LEFT_MOTOR_A, HIGH);            //FORWARD MOTORS ARE TURNED ON
              digitalWrite(RIGHT_MOTOR_A, HIGH);
              delay(5000);
              digitalWrite(LEFT_MOTOR_A, LOW);            //FORWARD MOTORS ARE TURNED OFF
              digitalWrite(RIGHT_MOTOR_A, LOW);
              digitalWrite(LEFT_MOTOR_B, HIGH);            //BACKWARD MOTORS TURNED ON
              digitalWrite(RIGHT_MOTOR_B, HIGH);
              delay(2000);
              digitalWrite(LEFT_MOTOR_B, LOW);            //BACKWARD MOTORS TURNED OFF
              digitalWrite(RIGHT_MOTOR_B, LOW);             
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
}   

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator()
{
  SmartLEDs.setPixelColor(0, ui_Mode_Indicator[ui_Robot_Mode_Index]);         // Set pixel colors to = mode 
  SmartLEDs.show();                                                           // Send the updated pixel colors to the hardware
}


