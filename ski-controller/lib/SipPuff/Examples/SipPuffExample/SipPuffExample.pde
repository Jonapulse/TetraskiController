//Sip & Puff Pattern Library - Written by Ross Imburgia - University of Utah ME Dept

/************************** IMPORTANT **********************************
 *  Do NOT call any delay() functions in your sketch, as this will interfere
 *  with the timing of the SipPuff class.
 *
 *  For examples of how to duplicate delay() functionality without blocking, see
 *  https://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
 */


#include <SipPuff.h>

SipPuff sip(0);							// create sip/puff object on pin A0

int8_t oldState=0;

void setup() {
  Serial.begin(9600);					// Open a new serial connection to PC

  sip.setNominalPressure();				// Calibrate the pressure to atmospheric


  //**** CONFIGURATION VARIABLES - Change as necessary for your project/user. Defaults are below:

  sip.deadZone        = 10;  //"Deadzone" around nominal pressure state where changes are ignored. Increase for noisy signals
  sip.debounceTime    = 80;  //Debouncing time (ms), used in tandem with deadZone to help filter noisy signals
  sip.multiStateTime  = 750; //Maximum "wait time" for additional inputs before pattern is finalized
  sip.longStateTime   = 500; //Minimum hold time for a Sip/Puff to register as a "LP/LS"

}

void loop() {

  sip.update();							// update the sip and puff detection function. CALL EVERY LOOP

  if(sip.finalPattern != "")			// if a Pattern is detected, print the output.
    Serial.println(sip.finalPattern);

  // For "continuous" operation, use the sip.active variable to get the real time state (-1 = active sip, 1 = active puff)

  if(sip.active != oldState)
    Serial.println(sip.active);

  oldState=sip.active;


}
  
