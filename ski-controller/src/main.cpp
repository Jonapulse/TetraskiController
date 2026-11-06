#include "Arduino.h"
#include <AccelStepper.h>
#include <SipPuff.h>
#include <EEPROM.h>
#include <genieArduinoDEV.h>
#include "SkiUpdate.h"

#define VERSION_MAJOR 2
#define VERSION_MINOR 6
#define VERSION_REV 1

// BENCH ONLY. 1 adds the test tools at the end of this file (serial bursts,
// battery override, loop timing). Must be 0 for any release. Pair a bench ski
// build only with a bench radio build.
#define BENCH_TOOLS 1

SKI_UPDATE_VERSION_TAG(VERSION_MAJOR, VERSION_MINOR, VERSION_REV);

//Firmware version as text, e.g. "2.6.1"
String firmwareVersionString() {
  return String(VERSION_MAJOR) + "." + VERSION_MINOR + "." + VERSION_REV;
}

// Digital pins **FOR TEENSY 3.2 ONLY**
#define LSTEP 2     //left step pin
#define LDIR 3      //left direction pin
#define RSTEP 4     //right step pin
#define RDIR 5      //right direction pin
#define RELAY 6     //power relay control pin
#define LCD_RESET 23    //LCD Reset Pin
#define PWR_BUT 11  //power/mode button pin
#define JOY_UNIT_POWER 13  //power control for joystick module
#define CTRL_PWR 19 //Joystick Signal - Power Down if disconnect

// analog pins
#define SIP A0      //Sip and puff sensor pin
#define RPOT A1     //Right actuator position sensor pin
#define LPOT A2     //Left actuator position sensor pin
#define JOY_X A3    //Joystick X-axis pin
#define JOY_Y A4    //Joystick Y-axis pin

//TEENSY 4.1 SPECIFIC PINS
#ifdef TEENSY41
#define MICRO_BOARD_VERSION 41
#define BSTAT A8
#define TONE 9
#define LED_R 10
#define LED_G 12
#define SERIAL_XB2 Serial5
#define SERIAL_LCD Serial2
#define SIP2 A17
#else  //TEENSY 3.2 SPECIFIC PINS
#define MICRO_BOARD_VERSION 32
#define BSTAT A10
#define TONE 20
#define LED_R 21
#define LED_G 22
#define SERIAL_XB2 Serial2
#define SERIAL_LCD Serial3
#endif

// display objects
#define DISP_MODE 5
#define DISP_BATT 4
#define DISP_DIFF 3

//Defines for speed and position parameters
#define FAST_SPEED 3200    //Fast stepper run speed
#define MID_SPEED 3250     //Medium stepper run speed
#define SLOW_SPEED 1500    //Slow stepper run speed
#define REM_SPEED 3000    //Stepper run speed for remote control operation
#define JOY_THRESH 40     //"Dead zone" threshold for joystick neutral 
#define WINDOW_SIZE 100   //Sampling size for moving average functions
#define MODE1 60
#define MODE2 30
#define MODE3 0
#define MODE4 -50

//battery measurements
#define BATT_WARNING 20
#define BATT_LOW 10
#define BATT_SHUTDOWN 0

//Timer values for updating inputs
#define SHUTDOWN_TIMER 2000         //Length of button hold (in ms) to shut down TetraSki
#define IDLE_SHUTDOWN 1800000       //Length of time (in ms) before idle shutdown

// EEPROM addresses for stored values
#define RUNTIME 1         //total runtime (ms)
#define TOTALRUNS 5       //total runs (ski placed in lift mode)
#define TOTALTURNS 9      //total turns (used for actuator lifecycle stats)
#define HW_ID 13          //Serial number for Tetra Ski
#define RMAX 17           //Max value for left actuator potentiometer
#define RMIN 21           //Min value for left actuator potentiometer
#define RSTEPS 25         //Total steps for complete actuator stroke
#define LMAX 29           //Max value for left actuator potentiometer
#define LMIN 33           //Min value for left actuator potentiometer
#define LSTEPS 37         //Total steps for left actuator stroke
#define X_CTR 41          //neutral reading for x axis
#define X_MAX 45         //maximum reading for x axis
#define X_MIN 49         //minimum reading for x axis
#define Y_CTR 53         //neutral reading for y axis
#define Y_MAX 57          //maximum reading for y axis
#define Y_MIN 61         //minimum reading for y axis

//Define a left and right steppers and the pins they will use
AccelStepper rightSki(1, RSTEP, RDIR);
AccelStepper leftSki(1, LSTEP, LDIR);

//Define a sip/puff object on pin A0
SipPuff sip(0);
SipPuff sip2(A17);

Genie display;

//Timer variables
long shutDownTimer;
long battUpdateTimer=0;
long remoteConnectionTimer;
long remoteResponseTimer;
unsigned long joystickTimer=0;
long blinkTimer;
long sipTimer;
long buttonTimer;
long idleTimer=0;
long toneTimer=0;

//Bring EEPROM values to RAM
int x_min, x_ctr, x_max, y_min, y_ctr, y_max, r_min, r_max, r_steps, l_min, l_max, l_steps;

int INDEX = 0;
int X_SUM = 0;
int X_READINGS[WINDOW_SIZE];
int X_AVERAGED = 0;
int Y_SUM = 0;
int Y_READINGS[WINDOW_SIZE];
int Y_AVERAGED = 0;

//Battery level calculation
int batteryPercentage, lowestBatteryPercentage=101;
float batteryVoltage;

bool beeper=0;
int beepCount=0;
int melody=0;
unsigned long melodyTimer=0;

int modePositions[4] = {MODE4, MODE3, MODE2, MODE1};
//int actuatorSpeed[3] = {SLOW_SPEED, MID_SPEED, FAST_SPEED};


int x, y, xS, yS, xB, yB, xJoy, yJoy, lSki=-800, rSki=-800, oldActive, actuatorSpeed=SLOW_SPEED;
bool joyMode=1, xbee=0, oldXbee=0, lockMode=0, joySipPuffChange = 0, remoteOverrideChime=0, difficulty=0, oldDifficulty=0, debug=0, newJoyUnit=0, batteryWarning=0, batteryLow=0, batteryShutdown=0, customSpeed=0, emgControlMode=0;  //Flags for operational states

//Variables for counting turns (useful for instruction and calculating life of actuators)
int turnCount=0, totalTurns=0, oldXPos=0;

//EMG control variables
byte EMG=0;
byte oldEMG=0;
uint8_t emgSensitivity=0;

//Flag variables for slower routines that can only be called when actuators are stationary
bool calibrateActuators=false;    //
bool shutdown=false;        //flat to enter shutdown sequence, TRUE -> ski will enter shutdown next loop
bool updateScreen=false;   //flag to updaate screen, true -> screen will update next time actuators stationary
bool updateMode=false;      //flag to update mode, TRUE -> mode will update next time skis are stationary
byte remoteResponseFlag=0;
bool flipJoyAxes=false;
bool invertEMG=false;

//variables for updateButton()
long multiclickTime=300, longClickTime=1000, debounceTime=20;
int clicks;                   // button click counts to return
bool depressed;            // the currently debounced button (press) state (presumably it is not sad :)
bool btnState;            // Current appearant button state
bool lastState;           // previous button reading
int clickCount;              // Number of button clicks within multiclickTime milliseconds
long lastBounceTime;         // the last time the button input pin was toggled, due to noise or a press

//global variables for setStatusLED()
uint8_t LEDcode=1, oldLEDCode=0, runs=1;
bool blinker;

//global Variables for readWireless()
byte inputVars[2];
byte temp;
int count;

void startUpSeq();
void altStartUpSeq();
void shutDownSeq();
void updateBatteryStats(bool forceUpdate);
void joyRead();
void setPosition(int xPos, int yPos, int minSize, int speed);
void setExactPosition(int l, int r);
void runSkis();
void stopSkis();
void setSpeeds(int speed);
void readRemote();
void parseBytes();
//void sendPingResponse();
void increaseWedge(bool audioChime);
void decreaseWedge(bool audioChime);
void setStatusLED();
void sendMode(int mode);
void printString(String s);
void updatePosition();
void updateButton();
void lockSkis();
void unlockSkis();
void writeEEPROM(int address, long value);
void updateEEPROM(int address, long value);
long readEEPROM(int address);
void loadEEPROM();
void readSensors();
void displayEventHandler();
void displayDebug();
void updateDisplayStats();
void calibrateActuatorFit();
void calibrateJoystick();
void calibrateMenu();
void checkJoy();
void beepGenerator();
void interruptShutdown();
void optionsMenu32();
void optionsMenu41();
void EMGOptions();
void melodyGenerator();
void sendRemoteResponse();
void progRadio();
void processBTCommands();
#if BENCH_TOOLS
bool benchParseByte(char c);
void benchLoopTick();
int benchReportedBattery();
#endif



void setup() {

  //initialize Serial ports for remote, BT, LCD comms
  Serial.begin(115200);
  Serial1.begin(57600);
  SERIAL_LCD.begin(200000);
  SERIAL_XB2.begin(57600);
  
  // //MANUALLY WRITE STATISTIC VALUES
  // writeEEPROM(RUNTIME, 630420);
  // writeEEPROM(TOTALTURNS, 58874);
  // writeEEPROM(TOTALRUNS, 211);

  //UNCOMMENT TO CONVERT PREVIOUS RUNTIME FORMAT TO NEW FORMAT
  //writeEEPROM(RUNTIME, readEEPROM(RUNTIME)/1000);
  
  //UNCOMMENT TO RESET RUNTIME
  //writeEEPROM(RUNTIME, 0);

  //load the stored calibration values from EEPROM into sram
  loadEEPROM();

  // if(newJoyUnit) {
  //   pinMode(CTRL_PWR, INPUT_PULLUP);
  //   delay(10);
  //   attachInterrupt(digitalPinToInterrupt(CTRL_PWR), interruptShutdown, RISING);
  // }
 
  //initialize input/output pins and default states
  pinMode(RELAY, OUTPUT);
  pinMode(PWR_BUT, INPUT_PULLDOWN);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(JOY_UNIT_POWER, OUTPUT);
  pinMode(TONE, OUTPUT);
  pinMode(CTRL_PWR, INPUT);
  pinMode(LCD_RESET, OUTPUT);
  

  digitalWrite(JOY_UNIT_POWER, LOW);
  digitalWrite(RELAY, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);


  //Set acceleration parameters for each linear actuator
  rightSki.setAcceleration(12000);
  leftSki.setAcceleration(12000);
  setSpeeds(REM_SPEED);

  //Set calibration values for SipPuff library
  sip.setNominalPressure();
  sip.longStateTime=500;
  sip.multiStateTime=500;
  sip.deadZone=50;

  if(MICRO_BOARD_VERSION==41) {
    sip2.setNominalPressure();
    sip2.longStateTime=1000;
    sip2.multiStateTime=500;
    sip2.deadZone=50;
  }

  //calibrateActuatorFit();

  //Detect startup mode. If startup with no power button, backup
  if(PWR_BUT)
    startUpSeq();
  else
    altStartUpSeq();

  Serial.print("Firmware version: ");
  Serial.println(firmwareVersionString());


}


void loop() {

#if BENCH_TOOLS
  benchLoopTick();  // loop timing and ski-side serial bursts
#endif

  //Updating functions that need to run every loop
  setStatusLED();  //This is depreciated with non-illuminated buttons on new skis but exists for backwards compatibility
  sip.update();
  updateButton();
  display.DoEvents();
  beepGenerator();
  melodyGenerator();
  sendRemoteResponse();


  //Set status LED to red if remote connection times out
  if(millis()-remoteConnectionTimer>2000) {
    LEDcode=1;
  }

  //Update battery level if it hasn't been done for 30 seconds (ski has been idle)
  if(millis()-battUpdateTimer > 30000) {
    updateScreen=true;
  }

  //Shutdown ski if it has been sitting idle for longer than IDLE_SHUTDOWN
  if(millis()-idleTimer > IDLE_SHUTDOWN)
    shutDownSeq();


  //Check if incoming data from remote control and process data through readWireless()
  if (Serial1.available()) {
    readRemote();
  }

  // Begin timer for shutdown on long press of power button
  if(!digitalRead(PWR_BUT))
    shutDownTimer=millis();

  // Run shutdown routine on long press of power button 
  if(millis()-shutDownTimer>SHUTDOWN_TIMER) {
    shutDownSeq();
  }

    //For skis with Teensy4.1 uC, update and perform 4.1 hardware-specific functions
  if(MICRO_BOARD_VERSION==41) {

      sip2.update();

      //Check if data incoming from EMG radio and store in local EMG var
      if(SERIAL_XB2.available()) {
          processBTCommands();
      }

      //Sip2 wedge change
      if(sip2.finalPattern=="P") {
        increaseWedge(true);
      } else if(sip2.finalPattern=="S") {
        decreaseWedge(true);
      }

      // //EMG wedge change if set to wedge control
      // if(EMG==49 && oldEMG==48 && emgControlMode) {
      //   increaseWedge(true);
      // } else if(EMG==50 && oldEMG==48 && emgControlMode) {
      //   decreaseWedge(true);
      // }

  }



  // If not in chairlift mode, enter user control below
  if(!lockMode) {

    // If not in joystick control, enter sip/puff mode below 
    if(!joyMode && !joySipPuffChange && !difficulty) {

      //mode change calls for advanced sip and puff controls
      // if(difficulty) {
      //   if(sip.finalPattern=="PP") {
      //     increaseWedge(true);
      //   } else if(sip.finalPattern=="SS") {
      //     decreaseWedge(true);
      //   }  
      // }

      // //if a change in sip state is detected, start the timer (used for mode change for advanced users)
      // if((sip.active != oldActive) || (EMG != oldEMG)) {
      //   sipTimer=millis();
      // }

      //             //EMG wedge change if set to wedge control
      // if(EMG==51 && oldEMG!=51) {
      //   increaseWedge(true);
      // } else if(EMG==52 && oldEMG!=52) {
      //   decreaseWedge(true);
      // }

      if(sip.active==-1 || (EMG==1 && !emgControlMode)) {

        if(lSki>750 || (yS<0 && lSki>100)) {}     //Prevents skis from continuously "stuttering" at max range

        //else if(difficulty && (millis()-sipTimer<250)) {}     //200ms delay in Sip and Puff response to allow for mode change commands

        else {
          xS=-100;
          setPosition(xS, yS, 0, actuatorSpeed);
       }

      } else if(sip.active==1 || (EMG==2 && !emgControlMode)) {

        if(rSki>750 || (yS<0 && rSki>100)) {}  //Prevents skis from continuously "stuttering" at max range

        //else if(difficulty && (millis()-sipTimer<250)) {}

        else {
          xS=100;
          setPosition(xS, yS, 0, actuatorSpeed);
        }

      // If sip/puff not detected stop skis in place
      } else if((sip.active==0 && oldActive!=0) || (EMG==0 && oldEMG!=0 && !emgControlMode)) {
          stopSkis();
      }

    } else if(joySipPuffChange) {
       if(sip.finalPattern=="P") {
          increaseWedge(true);
        } else if(sip.finalPattern=="S") {
          decreaseWedge(true);
        }  
    }

    oldActive=sip.active;   //update stored sip and puff state value for comparison next loop
    oldEMG=EMG;

    runSkis(); //call runSkis() as frequently as possible to keep step pulses continuous

    // Update joystick values
    if(millis()>(joystickTimer+1)) {

      joyRead();
      runSkis();
      joystickTimer=millis();
      // readSensors();
      
      //Update debug display (used in here because of timer frequency)
      if(debug)
        displayDebug();

      //if a non-neutral joystick position is detected, write the new position to the actuators
      if(joyMode) {

        xS=0;    //reset sip and puff position to center
        setPosition(x, y, 5, actuatorSpeed);
          
       }
    }
  }

  // increment through modes with single press of the power button
  if(clicks==1) {

    //Clear Battery Warning
    if (batteryWarning && beepCount>10 && !batteryLow) {
       beepCount=0;
       noTone(TONE);
    }      
    else if(debug) {
      debug=false;
      setSpeeds(SLOW_SPEED);
      display.WriteObject(GENIE_OBJ_FORM, 1, 0); // Change to Form 1
      updateDisplayStats();
    } else if(!lockMode && !joyMode && !difficulty) {
      // Cycle to mode 4 instead of 'maxing out' 
      if(yS==60) {
        yS=-50;  //Set yS to mode 4
        remoteResponseFlag=2; 
        setPosition(xS,yS,0,FAST_SPEED);  //Write target position to actuators
      } else
        increaseWedge(false);
    }
  }

  // cycle between difficulty settings with double press of the power button
  if(clicks==2) {
    difficulty=1-difficulty;
    if(!customSpeed) {
      if(!difficulty)
        actuatorSpeed=SLOW_SPEED;
      else
        actuatorSpeed=FAST_SPEED;
    }
    tone(TONE,1900,150);
    updateDisplayStats();
  }

  //lock/unlock skis with triple press of power button
  if(clicks==3) {
    if(!lockMode) {
      lockSkis();
    } else {
      unlockSkis();
    }
  }
  else if( clicks==4) {

    if(MICRO_BOARD_VERSION==32)
      optionsMenu32();
    else  
      optionsMenu41();
  }

  if(clicks==5 && !debug) {
      debug=true;
      display.WriteObject(GENIE_OBJ_FORM, 2, 0); // Change to Form 2
  }

  if(clicks==6 && !debug)
    calibrateMenu();

  if(clicks==7) {
    flipJoyAxes=1-flipJoyAxes;
    tone(TONE,1900,150);
  }

  if(clicks==8) {
    SERIAL_XB2.print("/SL");
    SERIAL_XB2.print("Test Comms Log");
    SERIAL_XB2.print("/FL");
  }

  if(clicks==9) {
    SERIAL_XB2.print("/E");
    SERIAL_XB2.write(100);

  }



  leftSki.moveTo(lSki);
  rightSki.moveTo(rSki);

  if((leftSki.isRunning() || rightSki.isRunning())) {
    if(!calibrateActuators)
      calibrateActuators=true;

    if(!updateScreen)
      updateScreen=true;

    idleTimer=millis();

  }


  //Blocking actions that can only be performed if skis are not currently moving
  if(rightSki.distanceToGo()==0 && leftSki.distanceToGo()==0) {
    if(calibrateActuators) {
      updatePosition();
      calibrateActuators=false;
    }

    if(updateScreen && (beepCount==0 || batteryLow)) {
      if(updateMode) {
        updateDisplayStats();
        updateMode=false;
      } else if (!joyMode) {
        updateBatteryStats(false);
      }

      updateScreen=false;
    }
  }

  if(remoteOverrideChime) {
    if(xbee==1 && oldXbee==0 && !lockMode) //melody=1;
      tone(TONE,2100);
    else if(oldXbee==1 && xbee==0) //melody=2;
      noTone(TONE);

    oldXbee=xbee;
  }

  runSkis();

}



//**************************************** void joyRead() *******************************************
//***************************************************************************************************
//    Reads the X and Y pots in the joystick and converts them into a value from -100 to 100,
//    with neutral for each axis residing at 0. Also incorporates a "dead zone" in the center
//    of each axis which must be exceeded before any change is made. Joystick parameters can
//    be adjusted in the defines at the top, to account for variation in potentiometers.
void joyRead() {

  //Store previous joystick variables for comparison purposes (explained later)
  int oldX= X_AVERAGED;
  int oldY= Y_AVERAGED;

  if(flipJoyAxes) {
      // Read X & Y axis analog voltage from the chipKit's ADC for each pin
    xJoy=analogRead(JOY_Y);
    runSkis();  //Since ADC conversion is slow and CPU intensive, call runSkis() between reads
    yJoy=analogRead(JOY_X);
    runSkis();
  } else {
      // Read X & Y axis analog voltage from the chipKit's ADC for each pin
    xJoy=analogRead(JOY_X);
    runSkis();  //Since ADC conversion is slow and CPU intensive, call runSkis() between reads
    yJoy=analogRead(JOY_Y);
    runSkis();
  }

  // Ignore values if in "dead zone" for x/y axis
  if(abs(xJoy-x_ctr)<JOY_THRESH) xJoy=x_ctr;
  if(abs(yJoy-y_ctr)<JOY_THRESH) yJoy=y_ctr;

  X_SUM = X_SUM - X_READINGS[INDEX];       // Remove the oldest entry from the sum    // Read the next sensor value
  X_READINGS[INDEX] = xJoy;           // Add the newest reading to the window
  X_SUM = X_SUM + xJoy;         // Add the newest reading to the sum
  X_AVERAGED = X_SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result

  runSkis();

  Y_SUM = Y_SUM - Y_READINGS[INDEX];       // Remove the oldest entry from the sum    // Read the next sensor value
  Y_READINGS[INDEX] = yJoy;           // Add the newest reading to the window
  Y_SUM = Y_SUM + yJoy;         // Add the newest reading to the sum
  Y_AVERAGED = Y_SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result


  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size


  // The following code determines if the joystick is actively being used and sets joyMode accordingly
  if(X_AVERAGED!=x_ctr || Y_AVERAGED!=y_ctr || xbee) {  //If joystick values deviate from "neutral", update joyMode
    joyMode=1;                 //Set joyMode to 1, signifying joystick operation
    xbee=0;

  } else if(X_AVERAGED==x_ctr && oldX==x_ctr && Y_AVERAGED==y_ctr && oldY==y_ctr)
    joyMode=0;  //If an idle state is detected for two call of functn, set joyMode to permit sip/puff ops

  runSkis();

  // Map the X and Y axis readings to a value ranging from -100 to 100, with zero being neutral.
  // Utilizes the joystick parameters set in the defines above  
  
  if(X_AVERAGED>=x_ctr)
    x = map(X_AVERAGED,x_ctr,x_max,0,-100);
  else
    x = map(X_AVERAGED,x_ctr,x_min,0,100);

  if(flipJoyAxes) x=x*-1;


  runSkis();

  // If in advanced mode, control wedge amount with joystick
  if(difficulty || debug) {
    if(Y_AVERAGED>=y_ctr) {
      y = map(Y_AVERAGED,y_ctr,y_max,0,-100);
    } else {
      y = map(Y_AVERAGED,y_ctr,y_min,0,100);
    }

  // Set Y position based on mode from remote
  } else {
    y=yS;
  }

  
  // limit max y of on-ski joystick
  if(y>85) y=85;
  if(y<-100) y=-100;

  if(x>100) x=100;
  if(x<-100) x=-100;

}



//************************ void setPosition(int xPos, int yPos, int minSize) ******************************
//*********************************************************************************************************
//    Parameters: int xPos - the X position of the joystick/sip and puff variables
//                int yPos - the Y position of the joystick/sip and puff variables
//                int minSize - the minimum change in actuator position required to make a move
//    setPosition() takes input arguments corresponding to the X and Y position of the joystick and maps
//        them to actuator positions for the left and right skis. Additionally, the input parameter minSize
//        sets a minimum change (in steps) required in actuator position in order to execute the move. For
//        joystick operation, this is typically set to around 200 (to prevent jitters). For sip and puff,
//        where desired positions are more exact, this is kept at zero.
void setPosition(int xPos, int yPos, int minSize, int speed) {

  // If x value switches from positive/negative, increment turn count
  if((oldXPos>0 && xPos<0) || (oldXPos<0 && xPos>0))
    turnCount++;

  if(xPos!=0)
    oldXPos=xPos;

  int oldL, oldR, part1, part2;
  //Store old values for actuator position
  oldL=lSki;
  oldR=rSki;

  // The below code constrains the X variabe to a "diamond" range of motion more like a joystick has.
  // Sip/puff mode has potential positions forming a square, while the joystick range is circular. This
  // isnt a problem for wedging (as the maximum wedge position of an actuator is limited to 0 steps)
  // but when the skis are straightened (y=100) it was causing the inside ski to turn too far and give
  // the tetra ski a tendency to tip over. Ask Ross for clarification if necessary.
  if(yPos>0) {
   if(abs(xPos)>(100-yPos)) {
      if(xPos>0)
        xPos=100-yPos;
      else if(xPos<0)
        xPos=-(100-yPos);

      x=xPos;
   }
 }


  runSkis();

  part1 = map(xPos,0,100,-400,-2000);    //map the x-component of joystick position to the left ski

  runSkis();

  // map the y-component of joystick position to the left ski
  if(yPos>=0)
      part2 = map(yPos,0,-100,-400,-2000);
  else {
      part2 = map(yPos,0,100,-400, 800);
  }

  runSkis();

  //if minSize isnt exceeded, do not write new position to ski
  if(abs(oldL-(part1+part2))>minSize) {
    lSki = part1+part2;
    //calibrateActuators=false;
  }

  part1 = map(xPos,0,-100,-400,-2000); // map the X-component of joystick position to the right ski

  runSkis();

  //if minSize isnt exceeded for the right ski, do not write new position
  if(abs(oldR-(part1+part2))>minSize) {
    rSki = part1+part2;
  }

  //Range limits. Prevents actuators from traveling outside desired range for skiing
  if(rSki<-2000)
    rSki=-2000;
  if(rSki>1000)
    rSki=1000;

  if(lSki<-2000)
    lSki=-2000;
  if(lSki>1000)
    lSki=1000;
  if(debug || batteryShutdown)
    setSpeeds(200);
  else
    setSpeeds(speed);

}



// ********************************** setExactPosition(int l, int r) **********************************
//*****************************************************************************************************
//      Parameters: int l - desired position for left ski (measured on linear potentiometer)
//                 int r - desired position for right ski (measured on linear potentiometer)
//      Moves skis to an exact position as measured by the linear pots built into the actuators. This 
//          function is blocking, meaning that nothing else will execute until the skis have reached
//          their desired position. Unreliable if running at >1000 steps/sec (HOME_SPEED) due to 
//          frequent polling of thre ADC for each analog pin.
void setExactPosition(int l, int r) {

  int leftPot = analogRead(LPOT);
  int rightPot = analogRead(RPOT);

  

  setSpeeds(500);  //Set speed to HOME_SPEED (default 1000 steps/sec)

  //Determine which side of desired position each ski is on and set motor direction accordingly
  if(leftPot < l)
    leftSki.setSpeed(-500);
  else
    leftSki.setSpeed(500);

  if(rightPot < r)
    rightSki.setSpeed(-500);
  else
    rightSki.setSpeed(500);

  //Loop the runSpeed commands until each motor is within five units of desired position  
  while( abs(rightPot-r) > 5 || abs(leftPot-l) > 5) {
    

    //Read linear position
    rightPot=analogRead(RPOT);
    leftPot=analogRead(LPOT);

    //Stop right motor if at desired position
    if(abs(rightPot-r) < 5)
      rightSki.setSpeed(0);

    //Stop left motor if at desired position
    if(abs(leftPot-l) < 5)
      leftSki.setSpeed(0);


    //Run steppers until in position
    leftSki.runSpeed();
    rightSki.runSpeed();    

    if(digitalRead(PWR_BUT))
      break;

    display.DoEvents();

  }

  setSpeeds(FAST_SPEED);  //Set the speed back to the default joystick speed

}




//***********************************************************************************************************************
//
void updatePosition() {

  int lAvg=0, rAvg=0;

  //sample 25 times to account for system noise, which is amplified by the conversion equation below
  for(int i=0; i<25; i++) {
    rAvg+=analogRead(RPOT);
    lAvg+=analogRead(LPOT);
  }

  //calculate new position based on readings from potentiometer
  rSki = map((rAvg/25), r_min, r_max, 1250, r_steps);
  lSki = map((lAvg/25), l_min, l_max, 1250, l_steps);

  //Range limits. Prevents actuators from traveling outside desired range for skiing
  if(rSki<-2000)
    rSki=-2000;
  // if(rSki>1000)
  //   rSki=1000;

  if(lSki<-2000)
    lSki=-2000;
  // if(lSki>1000)
  //   lSki=1000;

  rightSki.setCurrentPosition(rSki);
  leftSki.setCurrentPosition(lSki);

}






//*********************************** void setSpeeds(int speed) *************************************************
//**************************************************************************************************************
//  Parameters: int speed - the desired speed at which to operate the motors.
//  Sets the speed of both motors to a desired value. References AccelStepper::setMaxSpeed().
void setSpeeds(int speed) {
  leftSki.setMaxSpeed(speed);
  runSkis();
  rightSki.setMaxSpeed(speed);
}




//*********************************** void  stopSkis() **********************************************
//***************************************************************************************************
// writes a new target position to each ski causing it to stop as quickly as possible.
// Does not interfere with home position.
void stopSkis() {
  leftSki.stop();
  rightSki.stop();
  lSki=leftSki.targetPosition();
  rSki=rightSki.targetPosition();
}




//************************************ void runSkis() ***********************************************
//***************************************************************************************************
// Check each actuator and generate a step if necessary. MUST BE CALLED AS OFTEN AS POSSIBLE
// TO GENERATE A CONTINUOUS WAVEFORM FOR THE MOTORS. Call between slow processes, such as
// analogReads or Serial read/writes.
void runSkis() {
  leftSki.run();
  rightSki.run();
}




//************************************* void readWireless() ******************************************
//****************************************************************************************************
// Reads an incoming byte and and does some basic error checking for missing packets. If no missing
// data is detected and the delimiter byte is detected (255), the parseBytes is referenced and the
// bytes are converted to type int
void readRemote() {

    temp = Serial1.read();    //Read incoming serial byte

    runSkis();

    //if incoming byte == delimiter, parse the bytes and process the coordinates
    if(temp==255 && count==2) {
      parseBytes();
      count=0;
    } else {
      //if three bytes recorded without parsing (missing data), discard data and wait for next packet
      if(count>1) {
        //Serial.println("Error");
        count=0;
      }
      inputVars[count]=temp;   //Store input bytes in array for processing
      count++;                 //increment count
    }
}




//************************************* void parseBytes() ********************************************
//****************************************************************************************************
void parseBytes() {

  //sendPingResponse();
  remoteResponseFlag=1;     ///***Beta remote response test
  remoteResponseTimer=millis();

  LEDcode=2;
  remoteConnectionTimer=millis();

  //Read input bytes from temp array and parse them to type (int)
  yB=(int)inputVars[1]-100;
  runSkis();
  xB=(int)inputVars[0]-100;

  //set all variables/parameters needed for joystick control
  if(xB==154 || yB==154 || xB==128 || yB==128) {
    //Serial.println("Ping");
  }
  else if(xB==155 || yB==155) {}
    //printString("Missing Bytes");


  else {

    if(xB==122 || yB==122 || xB==123 || yB==123) {
      
      if(lockMode)
        unlockSkis();
      else
        lockSkis();

    } else if((xB==125 || yB==125)) {

      if(lockMode)
        unlockSkis();
      else
        decreaseWedge(false);

    } else if((xB==124 || yB==124)) {

      if(lockMode)
        unlockSkis();
      else
        increaseWedge(false);

    } else if (lockMode==false && xB<105 && yB<105) {
      if(debug) {
        displayDebug();
      } else
      {
           //setSpeeds(REM_SPEED);
      }
      

      joyMode=1;
      xbee=1;
      xS=0;
      runSkis();
      setPosition(xB, yB, 25, REM_SPEED);
      joystickTimer=millis()+200;
      runSkis();
    }
  }
}


// // Send ping response to remote confirming connection
// void sendPingResponse() {

//   if(yS<0) {            //mode 4
//     Serial1.write(251);
//     Serial1.write(251);
//     Serial1.write(255);
//   } else if(yS==0) {    //mode 3
//     Serial1.write(252);
//     Serial1.write(252);
//     Serial1.write(255);
//   } else if(yS==30) {   //mode 2
//     Serial1.write(253);
//     Serial1.write(253);
//     Serial1.write(255);
//   } else if(yS>30) {    //mode 1
//     Serial1.write(254);
//     Serial1.write(254);
//     Serial1.write(255);
//   }
// }



void increaseWedge(bool audioChime) {
  if(!difficulty) {
        //Chime TetraSki buzzer if called in arguments
    if(audioChime) {
      if(yS<0) beepCount+=7;
      if(yS==0) beepCount+=5;
      if(yS==30) beepCount+=3;
      if(yS>30) beepCount+=2;
    }
    
    if(yS<0) {
        yS=0;
        stopSkis();
    }//Check if skis are at "full wedge", and update to "neutral"
    else if(yS>=0 && yS<60) {
        yS+=30;
        stopSkis();
    }//If wedge is neutral or faster, increment by 50

    remoteResponseFlag=2; 

    if(leftSki.distanceToGo()==0 && rightSki.distanceToGo()==0)
        setSpeeds(FAST_SPEED);   //Set actuator speed to fastest setting for best response

    if(xS==0 && yS<=60)
      setPosition(xS,yS,0,REM_SPEED);  //Write target position to actuators
      //Serial.println(yS);

    SERIAL_XB2.print("2");
    Serial.println("Sent Wedge");
  }
}



void decreaseWedge(bool audioChime) {
  if(!difficulty) {
    if(yS>0) {   //If skis are forward of neutral, decrement by 25
      yS-=30;
      stopSkis();
    }
    else if(yS==0) {  //If skis at neutral, decrement by 50
      yS=-50;
      stopSkis();
    }

    remoteResponseFlag=2; 

    if(audioChime) {
      if(yS<0) beepCount+=9;
      if(yS==0) beepCount+=7;
      if(yS==30) beepCount+=5;
      if(yS>30) beepCount+=2; 
     }

    if(leftSki.distanceToGo()==0 && rightSki.distanceToGo()==0)
      setSpeeds(FAST_SPEED);   //Set actuator speed to fastest setting for best response

    if(xS==0 && yS>=-50)
      setPosition(xS,yS,0,REM_SPEED);  //Write target position to actuators
      //Serial.println(yS);

    SERIAL_XB2.print("2");
        Serial.println("Sent Wedge");
  }
}


void sendMode(int mode) {
    if(mode==5) {
      Serial1.write(234);
      runSkis();
      Serial1.write(234);
      runSkis();
      Serial1.write(255);
      //printString("Skis Locked");
    }
    else if(mode==1) {
      Serial1.write(230);
      runSkis();
      Serial1.write(230);
      runSkis();
      Serial1.write(255);
      runSkis();
      //printString("Mode 1");
    }
    else if (mode==2) {
      Serial1.write(231);
      runSkis();
      Serial1.write(231);
      runSkis();
      Serial1.write(255);
      runSkis();
      //printString("Mode 2");
    }
    else if (mode==3) {
      Serial1.write(232);
      runSkis();
      Serial1.write(232);
      runSkis();
      Serial1.write(255);
      runSkis();
      //printString("Mode 3");
    }
    else if (mode==4) {
      Serial1.write(233);
      runSkis();
      Serial1.write(233);
      runSkis();
      Serial1.write(255);
      runSkis();
      //printString("Mode 4");
    }

    // if(leftSki.distanceToGo()==0 && rightSki.distanceToGo()==0)
    //   updateDisplayStats();
    // else
    // {
        updateMode=true;
        updateScreen=true;
    //}
    


}



void printString(String s) {
  for(unsigned int i=0; i<s.length(); i++) {
    Serial.print(s.charAt(i));
    runSkis();
  }
  Serial.print("\n");
}



void startUpSeq() {

  
  delay(1500);
  digitalWrite(JOY_UNIT_POWER, HIGH);
  digitalWrite(RELAY, HIGH);
  digitalWrite(LED_R, HIGH);
  setStatusLED();


    // startup tone
  tone(TONE,1975);
  delay(125);
  tone(TONE,2093);
  delay(125);
  tone(TONE,2349);
  delay(125);
  noTone(TONE);


  //Display boot sequence
  digitalWrite(LCD_RESET, 0);
  delay(500);
  digitalWrite(LCD_RESET, 1);
  delay(3000);

  //Each Begin() waits up to 2 s. Give up after 5 tries so the ski still runs (and answers
  //the radio) without a display. The library keeps pinging and brings a late display online.
  Serial.println("Startup: waiting for display");
  bool displayFound=false;
  for(int i=0; i<5 && !displayFound; i++) {
    displayFound=display.Begin(SERIAL_LCD);
  }
  if(displayFound)
    Serial.println("Startup: display found");
  else
    Serial.println("Startup: no display, continuing without it");
  if(display.IsOnline())
    display.AttachEventHandler(displayEventHandler); 

  display.WriteContrast(15);

  display.SetForm(0); // Change to main screen
  display.DoEvents();

  String versionNo;

  if(newJoyUnit)
    versionNo = "HW# TS0";
  else  
    versionNo = "HW# TS00";

  if(MICRO_BOARD_VERSION==32)  
    versionNo = versionNo + readEEPROM(13) + "   FW Ver. " + firmwareVersionString() + "a";
  else 
    versionNo = versionNo + readEEPROM(13) + "   FW Ver. " + firmwareVersionString() + "b";

  String runtimeHoursMinutesSeconds = "Runtime: ";
  runtimeHoursMinutesSeconds = runtimeHoursMinutesSeconds + ((int) ((readEEPROM(RUNTIME) / (60*60)))) + "h " + ((int) ((readEEPROM(RUNTIME) / (60))%60)) + 'm';
 
  String runCount ="Runs: ";
  runCount = runCount + readEEPROM(TOTALRUNS) + "  Turns: " + readEEPROM(TOTALTURNS);

  display.WriteStr(0, versionNo);
  display.DoEvents();
  display.WriteStr(1, runtimeHoursMinutesSeconds);
  display.DoEvents();
  display.WriteStr(2, runCount);
  
  for(int i=0; i<WINDOW_SIZE*3; i++) {
    joyRead();
  }

  while(millis()<10000) { display.DoEvents(); }

  display.SetForm(1); // Change to main screen
  display.DoEvents();
  updateDisplayStats();
  display.DoEvents();
  updatePosition();

}


//***********************************************************************************************************************



void altStartUpSeq() {



}


//***********************************************************************************************************************


void shutDownSeq() {
  //Serial.println("Shutting down");

  totalTurns+=turnCount;

  // update EEPROM with statistics from skiing session
  // this can only happen once per boot to protect volatile EEPROM memory
  // shutdown flag prevents writing multiple times if power button is held down too long
  if(!shutdown) {
    updateEEPROM(RUNTIME, millis()/1000);
    updateEEPROM(TOTALRUNS, runs/3);
    updateEEPROM(TOTALTURNS, totalTurns);
  }

  shutdown=true;

  tone(TONE,2349);
  delay(125);
  tone(TONE,2093);
  delay(125);
  tone(TONE,1975);
  delay(125);
  noTone(TONE);

  if(analogRead(JOY_Y) > 350) {
    setPosition(0, -100, 5, 1000);
    leftSki.moveTo(lSki);
    rightSki.moveTo(rSki);
  } else {
    setPosition(0, 100, 5, 1000);
    leftSki.moveTo(lSki);
    rightSki.moveTo(rSki);
  }

  digitalWrite(JOY_UNIT_POWER, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);

  while(leftSki.distanceToGo()!=0 || rightSki.distanceToGo()!=0) {
    runSkis();
    display.DoEvents();
  }


  digitalWrite(RELAY, LOW);


  delay(15000);

}

//***********************************************************************************************************************


void updateDisplayStats() {
  int mode;  

  if(difficulty && !lockMode) {
    display.WriteStr(5, "0");
  } else if(!lockMode) {

    if(yS<0) mode=4;
    if(yS==0) mode=3;
    if(yS==30) mode=2;
    if(yS>30) mode=1;

    display.WriteStr(5, mode);
  }  else 
    display.WriteStr(5, "L");

  String controlStatus;

  if(difficulty) {
    controlStatus = "O - ";
  } else {
    controlStatus = "I - ";
  }

  switch(actuatorSpeed) {
    case SLOW_SPEED:
      controlStatus = controlStatus + "SLOW";
      break;
    case FAST_SPEED:
      controlStatus = controlStatus + "FAST";
      break;
    default:
      controlStatus = controlStatus + "MEDIUM";
  }      

  display.WriteStr(3, controlStatus);

  oldDifficulty=difficulty;

  updateBatteryStats(true);
}


//***********************************************************************************************************************


void updateBatteryStats(bool forceUpdate) {

  int rawBattery=0;

  for(int i=0; i<50; i++) {
    rawBattery=rawBattery+analogRead(BSTAT);
  }

  batteryVoltage=((rawBattery/50)*.0414)+.7455;
  batteryPercentage=(((batteryVoltage)*(11.111)-355.56));


  if(batteryPercentage>100)
    batteryPercentage=100;

  if(batteryPercentage<0)
    batteryPercentage=0;

  if(lowestBatteryPercentage>batteryPercentage) {
    lowestBatteryPercentage=batteryPercentage;
    String battPercent = String(lowestBatteryPercentage);
    battPercent.concat("%");
    display.WriteStr(4, battPercent);

    SERIAL_XB2.print("%");
#if BENCH_TOOLS
    SERIAL_XB2.print(benchReportedBattery());
#else
    SERIAL_XB2.print(lowestBatteryPercentage);
#endif
    Serial.println("Sent Battery");

  } else if(forceUpdate) {
    String battPercent = String(lowestBatteryPercentage);
    battPercent.concat("%");
    display.WriteStr(4, battPercent);

        SERIAL_XB2.print("%");
#if BENCH_TOOLS
    SERIAL_XB2.print(benchReportedBattery());
#else
    SERIAL_XB2.print(lowestBatteryPercentage);
#endif
    Serial.println("Sent Battery");
  }

  if(lowestBatteryPercentage<=BATT_WARNING && !batteryWarning) {
    batteryWarning=true;
    beepCount+=1000000;
  }

  if(lowestBatteryPercentage<=BATT_LOW && !batteryLow) {
    batteryLow=true;
    beepCount+=1000000;
  }

  if(lowestBatteryPercentage<=BATT_SHUTDOWN && !batteryShutdown) {
    batteryShutdown=true;
    beepCount=0;
  }

  battUpdateTimer=millis();
}


//***********************************************************************************************************************


void setStatusLED () {

    //RED (code=1)
    if(LEDcode==1 && oldLEDCode!=1) {
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_R, HIGH);
      //printString("Remote Disconnected");
    }

    //GREEN (code=2)
    else if(LEDcode==2 && oldLEDCode!=2) {
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, LOW);
      //printString("Remote Connected");
    }


  oldLEDCode=LEDcode;
}


//***********************************************************************************************************************


void lockSkis() {
  //xB=0;
  //yB=100;
  xbee=1;
  xS=0;
  //yS=0;
  lockMode=true;
  updateDisplayStats();
  runSkis();
  setPosition(0, 100, 200,SLOW_SPEED);
  joystickTimer=millis();
}


//***********************************************************************************************************************


void unlockSkis() {
  x=0;
  y=0;
  xS=0;
  yS=0;
  lockMode=false;
  //joyMode=0;
  xbee=1;
  runs++;
  totalTurns+=turnCount;
  turnCount=0;
  updateDisplayStats();
  runSkis();
  setPosition(xS, yS, 200, SLOW_SPEED);

  leftSki.moveTo(lSki);
  rightSki.moveTo(rSki);

  while(leftSki.distanceToGo()!=0 || rightSki.distanceToGo()!=0) {
    runSkis();
    display.DoEvents();
  }

  joystickTimer=millis();
}


//***********************************************************************************************************************


void updateButton() {
  buttonTimer = millis();      // get current time
  btnState = digitalRead(PWR_BUT);;  // current appearant button state

  // If the switch changed, due to noise or a button press, reset the debounce timer
  if (btnState != lastState) lastBounceTime = buttonTimer;


  // debounce the button (Check if a stable, changed state has occured)
  if (buttonTimer - lastBounceTime > debounceTime && btnState != depressed)
  {
    depressed = btnState;
    if (depressed) clickCount++;
  }

  // If the button released state is stable, report nr of clicks and start new cycle
  if (!depressed && (buttonTimer - lastBounceTime) > multiclickTime)
  {
    // positive count for released buttons
    clicks = clickCount;
    clickCount = 0;
  }

  // Check for "long click"
  if (depressed && (buttonTimer - lastBounceTime > longClickTime))
  {
    // negative count for long clicks
    clicks = 0 - clickCount;
    clickCount = 0;
  }

  lastState = btnState;

}



/***********************************************************************************************************************
 * Updates the incremental EEPROM variables by adding the new value
 * to the previously stored value
 * */
void updateEEPROM(int address, long value) {
  
    long total = readEEPROM(address)+value;
    writeEEPROM(address, total);
}



//***********************************************************************************************************************

void writeEEPROM(int address, long value) {
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}


//***********************************************************************************************************************

long readEEPROM(int address) {
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}


//***********************************************************************************************************************


//Write sensor values to USB Serial output **should only be called during debug, SLOW**
void readSensors() {

  // Print raw values (used for calibration only)
  Serial.print("RawX=");
  Serial.print(xJoy);
  Serial.print("  RawY=");
  Serial.print(yJoy);

  // Print processed values (used for calibration only)
  Serial.print("    X=");
  if(xJoy>=x_ctr)
    Serial.print(map(xJoy,x_ctr,x_max,0,-100));
  else
    Serial.print(map(xJoy,x_ctr,x_min,0,100));

  Serial.print("  Y=");
  if(yJoy>=y_ctr)
    Serial.print(map(yJoy,y_ctr,y_max,0,-100));
  else
    Serial.print(map(yJoy,y_ctr,y_min,0,100));


  Serial.print("    Rski=");
  Serial.print(analogRead(RPOT));
  Serial.print("  Lski=");
  Serial.println(analogRead(LPOT));

  Serial.print("    SP=");
  Serial.print(analogRead(0));

  Serial.print("    Batt=");
  Serial.println(analogRead(BSTAT));

  Serial.print("   ");    
  Serial.print(rSki);
  Serial.print("   ");
  Serial.println(lSki);
}



//***********************************************************************************************************************
void displayDebug() {
  display.WriteStr(6, xJoy);
  runSkis();
  display.WriteStr(7, yJoy);
  runSkis();
  if(xJoy>=x_ctr)
    display.WriteStr(8, map(xJoy,x_ctr,x_max,0,-100));
  else
    display.WriteStr(8, map(xJoy,x_ctr,x_min,0,100));

  runSkis();

  if(yJoy>=y_ctr)
    display.WriteStr(9, map(yJoy,y_ctr,y_max,0,-100));
  else
    display.WriteStr(9, map(yJoy,y_ctr,y_min,0,100));

  runSkis();
  display.WriteStr(10, analogRead(LPOT));
  runSkis();
  display.WriteStr(11, analogRead(RPOT));
  runSkis();
  display.WriteStr(12, lSki);
  runSkis();
  display.WriteStr(13, rSki);
  runSkis();
  display.WriteStr(14, analogRead(0));
  runSkis();
  display.WriteStr(15, analogRead(BSTAT));
  runSkis();
  display.WriteStr(16, batteryVoltage);
  runSkis();
  display.WriteStr(17, xB);
  runSkis();
  display.WriteStr(18, yB);
  runSkis();
  Serial.println(analogRead(A17));

}



void calibrateActuatorFit() {

  int currentValue=0;
  int lastValue=0;
  int leftMax=0, leftMin=0, rightMax=0, rightMin=0;
  bool update=true, cancelCalibrate=false;
  long menuTimeout=millis();

  //Wait for user to confirm calibration using button
  while(1) {
    updateButton();

    if(clicks==1) {
      cancelCalibrate=true;
      break;
    } else if (clicks==2) {
      display.WriteObject(GENIE_OBJ_STRINGS, 19, 2);  //switch display to "in progress" screen
      display.DoEvents();
      break;
    }

    if(millis()-menuTimeout>15000) {
      cancelCalibrate=true;
      break;
    }
    display.DoEvents();
  }

  
  if(!cancelCalibrate)
    setExactPosition(775, 775);

  Serial.println("Calibrating Right Max...");
  
  rightSki.setSpeed(-150);
  leftSki.setSpeed(-150);


  while(rightSki.speed()!=0 && !cancelCalibrate) {

    menuTimeout=millis();

    if(rightSki.currentPosition()%35==0 && update) {

      update=false;

      for(int i=0; i<25; i++) {
        currentValue+=analogRead(RPOT);
      }
      currentValue=currentValue/25;

      if(currentValue-lastValue<=3) {
        Serial.print(currentValue);
        rightSki.setSpeed(0);
        rightSki.setCurrentPosition(0);
        Serial.print("Right Max = ");
        Serial.println(lastValue);
        while(millis()-menuTimeout<2000) {
          display.DoEvents();
        }
        rightMax=lastValue;
        break;
      } 

    } else if (rightSki.currentPosition()%35!=0 && !update) {
      update=true;
      lastValue=currentValue;
      Serial.println(currentValue);
      currentValue=0;
    }

    rightSki.runSpeed();
    display.DoEvents();

    if(digitalRead(PWR_BUT)) {
      cancelCalibrate=true;
      display.WriteObject(GENIE_OBJ_STRINGS, 19, 3);
      display.DoEvents();
      while(millis()-menuTimeout<3000)
        display.DoEvents();
      break;  
    }
  }

  update=true;
  currentValue=0;
  lastValue=0;

  Serial.println("Calibrating Left Max...");
  
  while(leftSki.speed()!=0 && !cancelCalibrate) {

    menuTimeout=millis();

    if(leftSki.currentPosition()%35==0 && update) {

      update=false;

      for(int i=0; i<25; i++) {
        currentValue+=analogRead(LPOT);
      }
      currentValue=currentValue/25;

      if(currentValue-lastValue<=4) {
        //Serial.print(currentValue);
        leftSki.setSpeed(0);
        leftSki.setCurrentPosition(0);
        Serial.print("Left Max = ");
        Serial.println(lastValue);
        while(millis()-menuTimeout<2000)
          display.DoEvents();
        leftMax=lastValue;
        break;
      }

    } else if (leftSki.currentPosition()%35!=0 && !update) {
      update=true;
      lastValue=currentValue;
      Serial.println(currentValue);
      currentValue=0;
    }

    leftSki.runSpeed();
    display.DoEvents();

    if(digitalRead(PWR_BUT)) {
      cancelCalibrate=true;
      display.WriteObject(GENIE_OBJ_STRINGS, 19, 3);
      display.DoEvents();
      while(millis()-menuTimeout<3000)
        display.DoEvents();
      break;  
    }
  }

  if(!cancelCalibrate)
    setExactPosition(225, 225);

    Serial.println("Calibrating Right Min...");

  rightSki.setSpeed(150);  
  leftSki.setSpeed(150);

  update=true;
  currentValue=0;
  lastValue=250;

  while(rightSki.speed()!=0 && !cancelCalibrate) {

    menuTimeout=millis();

    if(rightSki.currentPosition()%35==0 && update) {

      update=false;

      for(int i=0; i<25; i++) {
        currentValue+=analogRead(RPOT);
      }
      currentValue=currentValue/25;

      if(currentValue-lastValue>=-4) {
        rightSki.setSpeed(0);
        Serial.print("Right Min = ");
        Serial.println(lastValue);
        while(millis()-menuTimeout<2000)
          display.DoEvents();
        rightMin=lastValue;
        break;
      }

    } else if (rightSki.currentPosition()%35!=0 && !update) {
      update=true;
      lastValue=currentValue;
      Serial.println(currentValue);
      currentValue=0;
    }

    rightSki.runSpeed();
    display.DoEvents();

    if(digitalRead(PWR_BUT)) {
      cancelCalibrate=true;
      display.WriteObject(GENIE_OBJ_STRINGS, 19, 3);
      display.DoEvents();
      while(millis()-menuTimeout<3000)
        display.DoEvents();
      break;  
    }
  }
  
  currentValue=0;
  lastValue=250;
  update=true;

  Serial.println("Calibrating Left Min...");

  while(leftSki.speed()!=0 && !cancelCalibrate) {

    menuTimeout=millis();

    if(leftSki.currentPosition()%35==0 && update) {

      update=false;

      for(int i=0; i<25; i++) {
        currentValue+=analogRead(LPOT);
      }
      currentValue=currentValue/25;

      if(currentValue-lastValue>=-4) {
        leftSki.setSpeed(0);
        Serial.print("Left Min = ");
        Serial.println(lastValue);
        while(millis()-menuTimeout<2000)
          display.DoEvents();
        leftMin=lastValue;
        break;
      } 

    } else if (leftSki.currentPosition()%35!=0 && !update) {
      update=true;
      lastValue=currentValue;
      Serial.println(currentValue);
      currentValue=0;
    }

    leftSki.runSpeed();
    display.DoEvents();

    if(digitalRead(PWR_BUT)) {
      cancelCalibrate=true;
      display.WriteObject(GENIE_OBJ_STRINGS, 19, 3);
      display.DoEvents();
      while(millis()-menuTimeout<3000)
        display.DoEvents();
      break;  
    }
  }

  menuTimeout=millis();

  if(!cancelCalibrate) {
    Serial.println("Writing Values to EEPROM");
    writeEEPROM(RMIN, rightMin);
    writeEEPROM(RMAX, rightMax);
    writeEEPROM(RSTEPS, (1250-rightSki.currentPosition()));
    writeEEPROM(LMIN, leftMin);
    writeEEPROM(LMAX, leftMax);
    writeEEPROM(LSTEPS, (1250-leftSki.currentPosition()));

    Serial.print(rightMax);
    Serial.print("  ");
    Serial.print(rightMin);
    Serial.print("  ");
    Serial.println(rightSki.currentPosition());

    Serial.print(leftMax);
    Serial.print("  ");
    Serial.print(leftMin);
    Serial.print("  ");
    Serial.println(leftSki.currentPosition());

    display.WriteObject(GENIE_OBJ_STRINGS, 19, 4);
    display.DoEvents();
    while(millis()-menuTimeout<3000)
      display.DoEvents();
}

  display.WriteObject(GENIE_OBJ_FORM, 1, 0); // Change to main screen
  display.DoEvents();
  updateDisplayStats();

  setSpeeds(REM_SPEED);
  loadEEPROM();
  updatePosition();
  setPosition(0,0,200,SLOW_SPEED);
  shutDownTimer=millis();

  
}


//***********************************************************************************************************************


void calibrateMenu() {
  display.WriteObject(GENIE_OBJ_STRINGS, 20, 0);
  display.WriteObject(GENIE_OBJ_STRINGS, 19, 0);
  display.DoEvents();
  display.SetForm(3);; // Change to calibrate screen
  display.DoEvents();

  idleTimer = millis();

  while(1) {
    updateButton();

    if(clicks==1 || millis() - idleTimer > 15000) {
      display.SetForm(1);
      display.DoEvents();
      updateDisplayStats();
      shutDownTimer=millis();
      break;
    }

    else if(clicks==2) {
      display.WriteObject(GENIE_OBJ_STRINGS, 19, 5);
      display.DoEvents();
      calibrateJoystick();
      break;
    }

    else if(clicks==3) {
      display.WriteObject(GENIE_OBJ_STRINGS, 19, 1);
      display.DoEvents();
      calibrateActuatorFit();
      break;
    }

    else if(clicks==7) {
      progRadio();
    }
    display.DoEvents();
  }
}


void calibrateJoystick() {

  long joyCalStartTime;
  int currentX=0, currentY=0, maxX=0, maxY=0, minX=1023, minY=1023, centX, centY;
  bool cancelCalibrate=0;

  idleTimer=millis();

  for(int i=0; i<100; i++) {
    currentX+=analogRead(JOY_X);
    currentY+=analogRead(JOY_Y);
  }
  centX=currentX/100;
  centY=currentY/100;

  do {
    updateButton();

    currentX=0;
    currentY=0;
  
    for(int i=0; i<20; i++) {
      currentX+=analogRead(JOY_X);
      currentY+=analogRead(JOY_Y);
    }
    currentX=currentX/20;
    currentY=currentY/20;

    if(clicks==1 || millis() - idleTimer > 15000) {
      joyCalStartTime=millis();
      display.WriteObject(GENIE_OBJ_STRINGS, 19, 3);
      display.DoEvents();
      cancelCalibrate=true;
      while(millis()-joyCalStartTime<3000) { display.DoEvents(); }
      break;  
    }

    display.DoEvents();
  } while(abs(currentX-centX)<30 || abs(currentY-centY)<30);



  joyCalStartTime=millis();
  
  if(!cancelCalibrate) {
    display.WriteObject(GENIE_OBJ_STRINGS, 19, 6);
    display.DoEvents();
  }



  while((millis()-joyCalStartTime < 10000) && !cancelCalibrate)
  {
    currentX=0;
    currentY=0;
  
    for(int i=0; i<20; i++) {
      currentX+=analogRead(JOY_X);
      currentY+=analogRead(JOY_Y);
    }
    currentX=currentX/20;
    currentY=currentY/20;

    if(currentX>maxX) maxX=currentX;
    if(currentX<minX) minX=currentX;

    if(currentY>maxY) maxY=currentY;
    if(currentY<minY) minY=currentY;

    display.DoEvents();

  }

  if(!cancelCalibrate) {
    display.WriteObject(GENIE_OBJ_STRINGS, 19, 7);
    display.DoEvents();

    writeEEPROM(X_MIN, minX);
    writeEEPROM(X_MAX, maxX);
    writeEEPROM(X_CTR, centX);
    writeEEPROM(Y_MAX, maxY);
    writeEEPROM(Y_MIN, minY);
    writeEEPROM(Y_CTR, centY);

    loadEEPROM();
    debug=true;
    while(millis()-joyCalStartTime < 13000) { display.DoEvents(); }
    display.WriteObject(GENIE_OBJ_FORM, 2, 0); // Change to Form 2
    display.DoEvents();
  } else
  {
    display.SetForm(1);
    display.DoEvents();
    updateDisplayStats(); 
  }
  
  shutDownTimer=millis();

}


void loadEEPROM() {

  r_min =(int) readEEPROM(RMIN);
  r_max =(int) readEEPROM(RMAX);
  r_steps =(int) readEEPROM(RSTEPS);
  l_min =(int) readEEPROM(LMIN);
  l_max =(int) readEEPROM(LMAX);
  l_steps =(int) readEEPROM(LSTEPS);
  x_min =(int) readEEPROM(X_MIN);
  x_max =(int) readEEPROM(X_MAX);
  x_ctr =(int) readEEPROM(X_CTR);
  y_min =(int) readEEPROM(Y_MIN);
  y_max =(int) readEEPROM(Y_MAX);
  y_ctr =(int) readEEPROM(Y_CTR);

  if((int) readEEPROM(13)>9)
    newJoyUnit=true;

}


void beepGenerator() {

  if(beepCount>0) {
    
    if(millis()-toneTimer > 100) {
      beeper=1-beeper;
      toneTimer=millis();
      beepCount--;
      if(beepCount==0)
        beeper=0;
    }
    if(beeper)
      tone(TONE, 1900);
    else
      noTone(TONE);
   }
}


void interruptShutdown() {
  digitalWrite(RELAY, LOW);
}


void optionsMenu32() {
  display.WriteObject(GENIE_OBJ_FORM, 3, 0); // Change to calibrate screen
  display.DoEvents();
  display.WriteStr(20, "   OPTIONS MENU");
  display.WriteStr(19,"\n  1 - Actuator Speed\n  2 - SipPuff Wedge Change\n  3 - Override Chime");

  display.DoEvents();
  long menuTimeout = millis();

  while(millis()-menuTimeout<15000) {

    if (Serial1.available()) {
        if((byte)Serial1.read()==255) {
          remoteResponseFlag=1;     ///***Beta remote response test
          remoteResponseTimer=millis();
        }
    }

    updateButton();

    if(clicks==1) {
      menuTimeout = millis()-11000;

      if(actuatorSpeed>=FAST_SPEED)
        actuatorSpeed=SLOW_SPEED;
      else 
        actuatorSpeed+=(FAST_SPEED-SLOW_SPEED)/2;

      tone(TONE,1900,150);

      switch(actuatorSpeed) {
        case SLOW_SPEED:
          display.WriteStr(19,"\n\n\n   Actuator speed is SLOW");
          break;
        case FAST_SPEED:
          display.WriteStr(19,"\n\n\n   Actuator speed is FAST");
          break;
        default:
          display.WriteStr(19,"\n\n\n   Actuator speed is MEDIUM");
      }
      //Serial.println(actuatorSpeed);
      customSpeed=1;
    }

    if(clicks==2) {

      menuTimeout = millis()-11000;

      if(joySipPuffChange) {
        display.WriteStr(19,"\n\n\nSip/Puff Wedge Change INACTIVE");
        joySipPuffChange = 0;
        tone(TONE,1900,150);
      } else {
        display.WriteStr(19,"\n\n\n  Sip/Puff Wedge Change ACTIVE  ");
        joySipPuffChange = 1;
        tone(TONE,1900,150);
      }

      while(millis()-menuTimeout<15000) {

        updateButton();
        if (Serial1.available()) {
            if((byte)Serial1.read()==255) {
              remoteResponseFlag=1;     ///***Beta remote response test
              remoteResponseTimer=millis();
            }
        }
        

        if(clicks==1 || clicks==2) {

          menuTimeout = millis()-11000;

          if(joySipPuffChange) {
            display.WriteStr(19,"\n\n\nSip/Puff Wedge Change INACTIVE");
            joySipPuffChange = 0;
            tone(TONE,1900,150);
          } else {
            display.WriteStr(19,"\n\n\n  Sip/Puff Wedge Change ACTIVE  ");
            joySipPuffChange = 1;
            tone(TONE,1900,150);
          }
        }

        sendRemoteResponse();
        display.DoEvents();

      }
    }

    if(clicks==3) {

      menuTimeout = millis()-11000;

      if(remoteOverrideChime) {
        display.WriteStr(19,"\n\n\n Remote override chime INACTIVE");
        remoteOverrideChime = 0;
        tone(TONE,1900,150);
      } else {
        display.WriteStr(19,"\n\n\n Remote override chime ACTIVE ");
        remoteOverrideChime = 1;
        tone(TONE,1900,150);
      }

      while(millis()-menuTimeout<15000) {

        updateButton();
        if (Serial1.available()) {
            if((byte)Serial1.read()==255) {
              remoteResponseFlag=1;     ///***Beta remote response test
              remoteResponseTimer=millis();
            }
        }

        if(clicks==1 || clicks==2) {

          menuTimeout = millis()-11000;

          if(remoteOverrideChime) {
            display.WriteStr(19,"\n\n\n Remote override chime INACTIVE");
            remoteOverrideChime = 0;
            tone(TONE,1900,150);
          } else {
            display.WriteStr(19,"\n\n\n Remote override chime ACTIVE ");
            remoteOverrideChime = 1;
            tone(TONE,1900,150);
          }
        }

        sendRemoteResponse();
        display.DoEvents();

      }
    }
    sendRemoteResponse();
    display.DoEvents();
  }

  display.WriteObject(GENIE_OBJ_FORM, 1, 0); // Change to Form 2
  display.DoEvents();
  updateDisplayStats(); 

}



void optionsMenu41() {
  display.WriteObject(GENIE_OBJ_FORM, 3, 0); // Change to calibrate screen
  display.DoEvents();
  display.WriteStr(20, "   OPTIONS MENU");
  display.WriteStr(19,"\n  1 - Actuator Speed\n  2 - EMG Options\n  3 - Override Chime");

  display.DoEvents();
  long menuTimeout = millis();

  while(millis()-menuTimeout<15000) {

    if (Serial1.available()) {
        if((byte)Serial1.read()==255) {
          remoteResponseFlag=1;     ///***Beta remote response test
          remoteResponseTimer=millis();
        }
    }


    updateButton();

    if(clicks==1) {
      menuTimeout = millis()-11000;

      if(actuatorSpeed>=FAST_SPEED)
        actuatorSpeed=SLOW_SPEED;
      else 
        actuatorSpeed+=(FAST_SPEED-SLOW_SPEED)/2;

      tone(TONE,1900,150);

      switch(actuatorSpeed) {
        case SLOW_SPEED:
          display.WriteStr(19,"\n\n\n   Actuator speed is SLOW");
          break;
        case FAST_SPEED:
          display.WriteStr(19,"\n\n\n   Actuator speed is FAST");
          break;
        default:
          display.WriteStr(19,"\n\n\n   Actuator speed is MEDIUM");
      }
      //Serial.println(actuatorSpeed);
      customSpeed=1;
    }



    if(clicks==2) {

      //check for EMG connection status and enter EMG options menu if confirmed
      if(SERIAL_XB2.available()) {
        menuTimeout=millis()-15000;
        EMGOptions();
      } else {

        //Change timer to display connection error for four seconds
        menuTimeout=millis()-11000;

        //write connection error to screen
        display.WriteObject(GENIE_OBJ_FORM, 3, 0); // Change to calibrate screen
        display.DoEvents();
        display.WriteStr(20, "      EMG OPTIONS");
        display.WriteStr(19,"\n\n\n     EMG NOT CONNECTED!");

        //block any other actions from menu screen
        while(millis()-menuTimeout<15000) display.DoEvents();
      }
    }    



    if(clicks==3) {

      menuTimeout = millis()-11000;

      if(remoteOverrideChime) {
        display.WriteStr(19,"\n\n\n Remote override chime INACTIVE");
        remoteOverrideChime = 0;
        tone(TONE,1900,150);
      } else {
        display.WriteStr(19,"\n\n\n Remote override chime ACTIVE ");
        remoteOverrideChime = 1;
        tone(TONE,1900,150);
      }

      while(millis()-menuTimeout<15000) {

        //Update button and send remote response if received
        updateButton();
        if (Serial1.available()) {
            if((byte)Serial1.read()==255) {
              remoteResponseFlag=1;     ///***Beta remote response test
              remoteResponseTimer=millis();
            }
        }

        if(clicks==1 || clicks==2) {

          menuTimeout = millis()-11000;

          if(remoteOverrideChime) {
            display.WriteStr(19,"\n\n\n Remote override chime INACTIVE");
            remoteOverrideChime = 0;
            tone(TONE,1900,150);
          } else {
            display.WriteStr(19,"\n\n\n Remote override chime ACTIVE ");
            remoteOverrideChime = 1;
            tone(TONE,1900,150);
          }
        }

        sendRemoteResponse();
        display.DoEvents();

      }
    }


    //
    if(clicks==5) {
      menuTimeout = millis();
      display.WriteStr(19,"\n\n\n   Entering WiFi OTA...");

      if(SERIAL_XB2.available()) SERIAL_XB2.read();     //flush EMG Serial buffer

      SERIAL_XB2.print('w');   //initiate Wifi OTA
      
      while(millis()-menuTimeout<120000) {

        updateButton();
        if(clicks==1)  {
          menuTimeout=millis()-120000;
        }

        if(SERIAL_XB2.available()) {

          char tempChar = SERIAL_XB2.read();
          Serial.write(tempChar);
          
          if(tempChar=='@') {
            display.WriteStr(19,"\n\n\n     WiFi OTA Ready");
          }

          if(tempChar=='#') {
            display.WriteStr(19,"\n\n\n     WiFi OTA Started");
          }
        }
        display.DoEvents();
      }
    }

    sendRemoteResponse();
    display.DoEvents();
  }

  display.WriteObject(GENIE_OBJ_FORM, 1, 0); // Change to Form 2
  display.DoEvents();
  updateDisplayStats(); 

}






void EMGOptions() {

  //begin menu timer for selection loops
  long menuTimeout = millis();


  display.WriteObject(GENIE_OBJ_FORM, 3, 0); // Change to calibrate screen
  display.DoEvents();
  display.WriteStr(20, "     EMG OPTIONS");
  display.WriteStr(19,"\n  1 - Sensitivity\n  2 - Turn/Wedge\n  3 - Invert Controls\n  4 - Calibrate");

  display.DoEvents();


  while(millis()-menuTimeout<15000) {

    if (Serial1.available()) {
        if((byte)Serial1.read()==255) {
          remoteResponseFlag=1;     ///***Beta remote response test
          remoteResponseTimer=millis();
        }
    }


    updateButton();

    if(clicks==1) {
      menuTimeout = millis()-11000;

      tone(TONE,1900,100);

      //increment sensitivity levels
      if(emgSensitivity<2) 
        emgSensitivity++;
      else 
        emgSensitivity=0;

      //write sensitivity to EMG controller
      SERIAL_XB2.print(emgSensitivity);

      //wait for confirmation from EMG controller
      while(millis()-menuTimeout<11100) {

        if(SERIAL_XB2.available()) {
          if(SERIAL_XB2.read()=='c')
            Serial.print("Sensitivity Confirmed");

            switch(emgSensitivity) {
              case 0:
                display.WriteStr(19,"\n\n\n      HIGH Sensitivity");
                break;
              case 1:
                display.WriteStr(19,"\n\n\n      MEDIUM Sensitivity");
                break;
              default:
                display.WriteStr(19,"\n\n\n      LOW Sensitivity");
            }

        }
        display.DoEvents();
      }   
    }



    if(clicks==2) {
      menuTimeout = millis()-11000;

      if(emgControlMode) {
        display.WriteStr(19,"\n\n\n EMG control set to TURN");
        emgControlMode = 0;
        tone(TONE,1900,100);
      } else {
        display.WriteStr(19,"\n\n\n EMG control set to WEDGE ");
        emgControlMode = 1;
        tone(TONE,1900,100);
      }

      while(millis()-menuTimeout<15000) {

        updateButton();
        if (Serial1.available()) {
            if((byte)Serial1.read()==255) {
              remoteResponseFlag=1;     ///***Beta remote response test
              remoteResponseTimer=millis();
            }
        }

        if(clicks==1 || clicks==2) {

          menuTimeout = millis()-11000;

          if(emgControlMode) {
            display.WriteStr(19,"\n\n\n EMG control set to TURN");
            emgControlMode = 0;
            tone(TONE,1900,100);
          } else {
            display.WriteStr(19,"\n\n\n EMG control set to WEDGE");
            emgControlMode = 1;
            tone(TONE,1900,100);
          }
        }
        
        sendRemoteResponse();
        display.DoEvents();

      }
    }    



    if(clicks==3) {

      menuTimeout = millis()-11000;

      if(invertEMG) {
        display.WriteStr(19,"\n\n\n EMG Controls NORMAL");
        invertEMG = 0;
        tone(TONE,1900,100);
      } else {
        display.WriteStr(19,"\n\n\n EMG Controls INVERTED ");
        invertEMG = 1;
        tone(TONE,1900,100);
      }

      SERIAL_XB2.print((int)invertEMG+3);

      while(millis()-menuTimeout<15000) {

        //Update button and send remote response if received
        updateButton();
        if (Serial1.available()) {
            if((byte)Serial1.read()==255) {
              remoteResponseFlag=1;     ///***Beta remote response test
              remoteResponseTimer=millis();
            }
        }

        if(clicks==1 || clicks==2) {

          menuTimeout = millis()-11000;

          if(invertEMG) {
            display.WriteStr(19,"\n\n\n    EMG Controls NORMAL");
            invertEMG = 0;
            tone(TONE,1900,150);
          } else {
            display.WriteStr(19,"\n\n\n    EMG Controls INVERTED ");
            invertEMG = 1;
            tone(TONE,1900,150);
          }
          SERIAL_XB2.print((int)invertEMG+3);
        }

        sendRemoteResponse();
        display.DoEvents();

      }
    }

    if(clicks==4) {

      menuTimeout = millis();

      SERIAL_XB2.print(5);


      while(millis()-menuTimeout<50000) {

        if(SERIAL_XB2.available()) {
          char incomingChar = SERIAL_XB2.read();

          if(incomingChar=='c')
            display.WriteStr(19,"\n\n\n    Starting Calibration... ");
          else if(incomingChar=='f') {
            display.WriteStr(19,"\n\n\n    Calibration Successful!");
            display.DoEvents();

            tone(TONE,1975);
            delay(100);
            tone(TONE,2093);
            delay(100);
            tone(TONE,2349);
            delay(100);
            noTone(TONE);
            menuTimeout=millis()-46000;
          }
            


        }
        display.DoEvents();
      }
    }


    sendRemoteResponse();
    display.DoEvents();
  }

  // display.WriteObject(GENIE_OBJ_FORM, 1, 0); // Change to Form 2
  // display.DoEvents();
  // updateDisplayStats(); 

}

void melodyGenerator() {
  if(melody!=0 && millis()>melodyTimer) {
    melodyTimer=millis()+300;
    Serial.println("MELODY");
  }

  if((melodyTimer-millis())>203 && (melodyTimer-millis())<=300) {
    
    if(melody==1) tone(TONE,1900);
    else if(melody==2) tone(TONE,2100);

  } else if ((melodyTimer-millis())>200 && (melodyTimer-millis())<=203) {
    noTone(TONE);
  } else if ((melodyTimer-millis())>100 && (melodyTimer-millis())<=200) {

     if(melody==1) tone(TONE,2100);
     else if(melody==2) tone(TONE,1900);

  } else if(melodyTimer-millis()<100 && melody) {
    noTone(TONE);
    melody=0;
  }
}


void sendRemoteResponse() {

  if(remoteResponseFlag !=0 && millis()-remoteResponseTimer>25) {

    if(remoteResponseFlag == 1) {     //indicates simple ping response

      if(difficulty) {
        Serial1.write(254);
        Serial1.write(254);
        Serial1.write(255);        
      } else {

        if(yS<0) {            //mode 4
          Serial1.write(251);
          Serial1.write(251);
          Serial1.write(255);
        } else if(yS==0) {    //mode 3
          Serial1.write(252);
          Serial1.write(252);
          Serial1.write(255);
        } else if(yS==30) {   //mode 2
          Serial1.write(253);
          Serial1.write(253);
          Serial1.write(255);
        } else if(yS>30) {    //mode 1
          Serial1.write(254);
          Serial1.write(254);
          Serial1.write(255);
        }
      }

      remoteResponseFlag = 0;

    } else if (remoteResponseFlag == 2) {   //indicates mode change response

       //Send the current wedge mode to the remote control
      if(yS<0) sendMode(4);
      if(yS==0) sendMode(3);
      if(yS==30) sendMode(2);
      if(yS>30) sendMode(1);

      remoteResponseFlag = 0;
    }
  }
}

void progRadio() {

  while(Serial1.available()) {
    Serial1.read();
  }

  display.DoEvents();
  display.WriteStr(20, "   RADIO PROGRAM");
  display.WriteStr(19,"\n\n   Press button for HWID#\n\n   Press once to cancel");

  display.DoEvents();
  long menuTimeout = millis();

  while(millis()-menuTimeout<15000) {
    updateButton();
    display.DoEvents();
    if(clicks!=0)
      break;
  }

  if(clicks > 1) {

    String tempString = "\n\n  Programming Ski for HWID# ";
    tempString = tempString + clicks + "\n\n **Ski will power down after";
    display.WriteStr(19,tempString);

    Serial.print("Programming radio for HWID# ");
    Serial.println(clicks);
    delay(1000);
    Serial.println("Opening AT command mode on radio...");
    Serial1.print("+++");
    delay(1000);

    if(Serial1.available()) {

      //Display output of AT Serial connection to radio
      while(Serial1.available()) {
        Serial.write(Serial1.read());
      }

     
      if(clicks>0 && clicks<10) {
        Serial1.print("ATID2005");
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATBD6");
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATDTB00");
        Serial1.print(clicks);
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATMYA00");
        Serial1.print(clicks);
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATWR");
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATCN");
        Serial1.print("\r");
      } else if(clicks>=10 && clicks<100) {
        Serial1.print("ATID2005");
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATBD6");
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATDTB0");
        Serial1.print(clicks);
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATMYA0");
        Serial1.print(clicks);
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATWR");
        Serial1.print("\r");
        delay(200);
        Serial1.print("ATCN");
        Serial1.print("\r");
      }
     
      delay(500);


      while(Serial1.available()) {
        Serial1.read();
      }


      Serial.println("\nRadio programming successful");
      writeEEPROM(HW_ID, clicks);
      shutdown=true;
      shutDownSeq();
    } else {
      Serial.println("\nError During Programming");
    }
  }

  display.WriteObject(GENIE_OBJ_FORM, 1, 0); // Change to Form 2
  display.DoEvents();
  updateDisplayStats(); 

}

void processBTCommands() {
  char inChar = SERIAL_XB2.read();

#if BENCH_TOOLS
  if (benchParseByte(inChar)) return;  // bench commands relayed by a bench radio build
#endif

  switch(inChar) {
    case '0':
      EMG=0;
      break;

    case '1':
      EMG=1;
      break;

    case '2':
      EMG=2;
      break;

    case '3':
      EMG=3;
      break;

    case '4':
      EMG=4;
      break;

    case 'V':
      SERIAL_XB2.print('V');
      SERIAL_XB2.write(VERSION_MAJOR);
      SERIAL_XB2.write(VERSION_MINOR);
      SERIAL_XB2.write(VERSION_REV);

      Serial.println("Sent Version #");
      break;

    case SKI_CMD_ENTER_UPDATE:  // radio is about to relay a firmware image, see SkiUpdate.cpp
      stopSkis();
      skiUpdateRun(SERIAL_XB2);
      break;

    case SKI_CMD_EXIT_UPDATE:   // already out of update mode, a repeated EXIT still gets an ACK
      SERIAL_XB2.write(SKI_ACK);
      break;
  }
}




//Empty Event handler function (required for display library)
void displayEventHandler(void)
{


}


#if BENCH_TOOLS
//***********************************************************************************************************************
// BENCH ONLY: test tools, compiled only with BENCH_TOOLS 1. Commands arrive from a
// bench radio build on SERIAL_XB2:
//   &S<size><ms>  send <size> filler bytes (2 digits, 00 = stop) to the radio every
//                 <ms> ms (1 digit, 0 = every loop). Auto-stops after 30 s.
//   *<nn>         report battery as nn% to the radio from now on (display and the
//                 ski's own battery warnings keep using the real reading). Sent once
//                 right away, then with every normal battery report. Until reboot.
//   #             report loop timing to the radio and USB, then reset it.
// Reports go to the radio as 0x81, text with the high bit set on every byte, 0x82,
// so no byte of a report can be read as a command by any radio firmware.
// Filler byte 0x80 is ignored by every radio firmware.

#define BENCH_FILLER      0x80
#define BENCH_TEXT_START  0x81
#define BENCH_TEXT_END    0x82
#define BENCH_BURST_MAX   99
#define BENCH_BURST_LIMIT_MS 30000

int benchBatteryOverride = -1;  // -1 = report the real reading

uint8_t benchBurstSize = 0;     // 0 = no burst running
uint8_t benchBurstIntervalMs = 0;
unsigned long benchBurstStartMs = 0;
unsigned long benchBurstLastMs = 0;
uint32_t benchBurstCount = 0;
uint32_t benchBurstMaxWriteUs = 0;
uint8_t benchBurstBuf[BENCH_BURST_MAX];

uint32_t benchLastLoopUs = 0;
uint32_t benchMaxLoopUs = 0;      // longest loop() while a ski was moving
uint32_t benchMovingLoops = 0;

char benchCmd = 0;                // bench command being parsed, 0 if none
char benchArg[4];
uint8_t benchGot = 0;
uint8_t benchNeed = 0;

int benchReportedBattery() {
  return benchBatteryOverride >= 0 ? benchBatteryOverride : lowestBatteryPercentage;
}

void benchSendText(const char* text) {
  SERIAL_XB2.write(BENCH_TEXT_START);
  for (const char* c = text; *c; c++) SERIAL_XB2.write((uint8_t)(*c | 0x80));
  SERIAL_XB2.write(BENCH_TEXT_END);
  Serial.print("Bench: ");
  Serial.println(text);
}

// Sends loop timing and burst results with a label, then starts counting again.
void benchReport(const char* label) {
  char text[120];
  snprintf(text, sizeof(text), "%s: loop max %lu us while moving (%lu loops), burst write max %lu us (%lu bursts), tx free %d",
           label, (unsigned long)benchMaxLoopUs, (unsigned long)benchMovingLoops,
           (unsigned long)benchBurstMaxWriteUs, (unsigned long)benchBurstCount, SERIAL_XB2.availableForWrite());
  benchSendText(text);
  benchMaxLoopUs = 0;
  benchMovingLoops = 0;
  benchBurstMaxWriteUs = 0;
  benchBurstCount = 0;
}

void benchBurstStart(uint8_t size, uint8_t intervalMs) {
  if (benchBurstSize > 0) {
    benchBurstSize = 0;
    benchReport("ski burst stopped");
  }
  if (size == 0) return;
  memset(benchBurstBuf, BENCH_FILLER, sizeof(benchBurstBuf));
  benchMaxLoopUs = 0;
  benchMovingLoops = 0;
  benchBurstMaxWriteUs = 0;
  benchBurstCount = 0;
  benchBurstIntervalMs = intervalMs;
  benchBurstStartMs = millis();
  benchBurstLastMs = 0;
  benchBurstSize = size;
  char text[60];
  if (intervalMs == 0) snprintf(text, sizeof(text), "ski burst: %u bytes every loop, 30 s", size);
  else snprintf(text, sizeof(text), "ski burst: %u bytes every %u ms, 30 s", size, intervalMs);
  benchSendText(text);
}

// Called at the top of every loop().
void benchLoopTick() {
  uint32_t now = micros();
  if (benchLastLoopUs != 0 && (leftSki.isRunning() || rightSki.isRunning())) {
    uint32_t dt = now - benchLastLoopUs;
    if (dt > benchMaxLoopUs) benchMaxLoopUs = dt;
    benchMovingLoops++;
  }
  benchLastLoopUs = now;

  if (benchBurstSize == 0) return;
  if (millis() - benchBurstStartMs >= BENCH_BURST_LIMIT_MS) {
    benchBurstSize = 0;
    benchReport("ski burst auto-stopped after 30 s");
    benchLastLoopUs = micros();  // the report itself is not a slow loop
    return;
  }
  if (benchBurstIntervalMs == 0 || millis() - benchBurstLastMs >= benchBurstIntervalMs) {
    benchBurstLastMs = millis();
    uint32_t t0 = micros();
    SERIAL_XB2.write(benchBurstBuf, benchBurstSize);
    uint32_t dt = micros() - t0;
    if (dt > benchBurstMaxWriteUs) benchBurstMaxWriteUs = dt;
    benchBurstCount++;
  }
}

static bool benchIsDigit(char c) { return c >= '0' && c <= '9'; }

// True if the byte belonged to a bench command, which then goes no further.
// No timeout: the radio writes a whole command at once.
bool benchParseByte(char c) {
  if (benchCmd != 0) {
    benchArg[benchGot++] = c;
    if (benchGot < benchNeed) return true;
    char cmd = benchCmd;
    benchCmd = 0;

    if (cmd == '*') {
      if (benchIsDigit(benchArg[0]) && benchIsDigit(benchArg[1])) {
        benchBatteryOverride = (benchArg[0] - '0') * 10 + (benchArg[1] - '0');
        SERIAL_XB2.print("%");
        SERIAL_XB2.print(benchBatteryOverride);
        char text[48];
        snprintf(text, sizeof(text), "battery reported as %d%% until reboot", benchBatteryOverride);
        benchSendText(text);
      } else {
        benchSendText("usage: *nn, two digits");
      }
    } else {  // '&'
      if (benchArg[0] == 'S' && benchIsDigit(benchArg[1]) && benchIsDigit(benchArg[2]) && benchIsDigit(benchArg[3])) {
        benchBurstStart((benchArg[1] - '0') * 10 + (benchArg[2] - '0'), benchArg[3] - '0');
      } else {
        benchSendText("usage: &S, size 00-99, interval 0-9");
      }
    }
    return true;
  }

  switch (c) {
    case '&': benchCmd = '&'; benchNeed = 4; benchGot = 0; return true;
    case '*': benchCmd = '*'; benchNeed = 2; benchGot = 0; return true;
    case '#': benchReport("stats"); return true;
  }
  return false;
}
#endif  // BENCH_TOOLS
