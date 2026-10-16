#ifndef SipPuff_H
#define SipPuff_H

#if (ARDUINO <  100)
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

class SipPuff
{
   public:
  SipPuff(int pin);    	//constructor declaration
  void update();						//primary update function
	bool setNominalPressure();			//call to zero out pressure gauge
	//void calibrate();
	String finalPattern;
  int8_t active;            // the currently debounced state
  long debounceTime;
  long multiStateTime;
  long longStateTime;
	int lastPressure;
	int deadZone;

  private:
  int8_t _state;            // Current appearant button state
  int8_t _lastState;           // previous button reading
  String _inputPattern;             // Number of button clicks within multiclickTime milliseconds
	int    inputPattern;
  long _lastBounceTime;         // the last time the button input pin was toggled, due to noise or a press
	int _nomPressure;
	int _pinNum;
};

#endif
