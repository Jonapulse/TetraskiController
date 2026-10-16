#include "SipPuff.h"

SipPuff::SipPuff(int pin)
{
  _state      	 = 0;  			  // initial sensor state (off=0, sip=-1, puff=1)
  _lastState     = _state;
  active      = 0;			  //public variable to check if sensor is active  (sipped or puffed)
  _inputPattern   = "";			  //temporary string to store input pattern
  finalPattern	 = "";			  //final pattern sequence
  _lastBounceTime= 0;
  debounceTime   = 80;            // debounce time (ms)
  multiStateTime = 750;           // time limit for multiple sip/puff inputs
  longStateTime  = 500;          // time until long sips/puffs register
  deadZone       = 10;            //size of pressure sensor "dead zone"
  _pinNum        = pin;
  _nomPressure   = analogRead(_pinNum);		//variable to store nominal atmospheric pressure
  lastPressure   = 0;			  //stores highest/lowest value of the the last input pattern for additional input criteria
}


//Call this function to zero out the pressure sensor to atmospheric ambient pressure
bool SipPuff::setNominalPressure() {
	_nomPressure=analogRead(_pinNum);

	if (_nomPressure == 0)
		return false;

	lastPressure = _nomPressure;
	return true;
}


void SipPuff::update() {
  long now = (long)millis();  //store current time
  int pressure = analogRead(_pinNum);

  // Detect Sip/Puff above deadzone threshold and change sensor state accordignly
  if(pressure<_nomPressure-deadZone) {

    _state=-1;

	//
	if(pressure<lastPressure)
		lastPressure=analogRead(_pinNum);

  } else if(pressure>_nomPressure+deadZone) {

    _state=1;

	if(pressure>lastPressure)
		lastPressure=pressure;

  } else _state=0;

  // If the state changed, due to noise or sip/puff, reset the debounce timer
  if(_state != _lastState) {
    _lastBounceTime = now;
  }

  //If a stable change has taken place, record the type of input and swap value of 'active'
  if( now-_lastBounceTime > debounceTime && _state != active) {

    active=_state;

    if(active) {

	  lastPressure = _nomPressure;

	  if (_state > 0){
		  _inputPattern += "P";

		  inputPattern = 0 ^ inputPattern;

	  }
	  if (_state<0){
		  _inputPattern += "S";
		  inputPattern = 1 ^ inputPattern;
	  }

	  inputPattern = inputPattern << 1;
	  }
  }
  //If a stable nominal state is present, record the final pattern into the finalPattern String
  if (!(abs(active)) && now - _lastBounceTime > multiStateTime) {
	  finalPattern = _inputPattern;
	  _inputPattern = "";
  }

  //If a long sip/puff is detected, append the current input with an underscore to indicate
  if (active && (now - _lastBounceTime) > longStateTime) {
	  _lastBounceTime = now;
	  _inputPattern.setCharAt(_inputPattern.length() - 1, 'L');

	  if (_state>0)
		  _inputPattern += "P";
	  if (_state<0)
		  _inputPattern += "S";
  }
  _lastState = _state;
}

//
// void SipPuff::calibrate() {
//   int puffTime=0, shortSipAvg=0, calCount=0;
//   _lastState = 1;
//  // Serial.println("\n**CALIBRATE**\nPlease give three 'Short Sips' into the machine.");
//
//   //SHORT SIP - useful to compare with a long sip to determine the difference
//   while(calCount<3) {
//
//     if(analogRead(_pinNum)<_nomPressure-deadZone)
// 		_state=0;    //Read in value of sip switch
// 	else
// 		_state=1;
//
//     //Detect sip, start debounce timer
//     if(_state==LOW && _lastState==HIGH)
//       _lastBounceTime=millis();
//
//     //Detect sip End, check against debounce timer, increment if successful reading
//     if(_state==HIGH && _lastState==LOW && (millis()-_lastBounceTime > debounceTime)) {
//       puffTime = millis()-_lastBounceTime;
//
// 	  o_Serial.println("good");
//       shortSipAvg+=puffTime;
//        calCount++;
//     }
//
//     _lastState = _state;    //Record old value of switch to compare next iteration
//   }
//   int shortAvrage = shortSipAvg / 3;
//   delay(200);
//   o_Serial.println( shortSipAvg/3);
//
//   delay(300);
//   o_Serial.println("Please give three 'long sips' into the machine");
//
//   //Reset counter and switch values for next test
//   _lastState=1;
//   calCount=0;
//
//
//   int longSipAvg=0;
//   //LONG SIP - Tests the average length of a long sip to determine the appropriate long sip threshold
//   while(calCount<3) {
//
//     if(analogRead(_pinNum)<_nomPressure-deadZone)
// 		_state=0;    //Read in value of sip switch
// 	else
// 		_state=1;
//
//     //Detect sip, start debounce timer
//     if(_state==LOW && _lastState==HIGH)
//       _lastBounceTime=millis();
//
//     //Detect sip end, check against debounce timer, increment if successful reading
//     if(_state==HIGH && _lastState==LOW && (millis()-_lastBounceTime > debounceTime ) && (millis() - _lastBounceTime)>shortSipAvg) {
//       puffTime = millis()-_lastBounceTime;  //Calculate length of puff
//       longSipAvg+=puffTime;
// 	  o_Serial.println("good");
//       calCount++;   //Increment counter
//     }
//
//     _lastState = _state;   //Record old value of switch to compare next iteration
//   }
//
//   int LongAverage = longSipAvg / 3;
//
//   if (LongAverage - shortAvrage > 200) {
//
//
// 	  longStateTime = shortAvrage + 200;
//   }
//
//   else {
// 	  longStateTime = LongAverage;
//
//
//   }
//   delay(200);
//   o_Serial.println(longSipAvg/3);
//   delay(300);
//   //Reset counter and switch value for next test
//   _lastState=1;
//   calCount=0;
//
//
//   o_Serial.println("Please give three 'double sips' into the machine");
//   puffTime=0;
//
//   bool count=false;
//   int doubleSipAvg=0;
//
//   //DOUBLE SIP - Tests the average length of a double sip to determine the appropriate double sip threshold
//   while(calCount<3) {
//     while(true) {      //Nested loops necessary because of nature of double puff
//
// 		if(analogRead(_pinNum)<_nomPressure-deadZone)
// 			_state=0;    //Read in value of sip switch
// 		else
// 			_state=1;
//
//       //Detect start of puff. If first sip, start debounce timer. If second sip, store time of first sip and reset debounce timer
//       if(_state==LOW && _lastState==HIGH) {
//         if(count)  //Second sip detected, store time of first puff
//           puffTime=_lastBounceTime;
//
//         _lastBounceTime=millis();
//         delay(5);  //Delay to allow switch to catch up
//       }
//
//       //Detect end of puff. If first, increment count. If second, determine spacing time of puffs and iterate external loop.
//       if(_state==HIGH && _lastState==LOW && (millis()-_lastBounceTime > debounceTime)) {
//         if(count) {     //Detect second puff
//           puffTime=millis()-puffTime;
//           count=false;  //Reset count for next round
//           calCount++;   //Increment counter for external loop
//           doubleSipAvg+=puffTime;  //Add to average
//
// 		  o_Serial.println("good");
//           break;    //Break out of internal loop
//         }
//         count=true;  //If first sip, set count to true. Second sip won't make it this far.
//       }
//
//       _lastState = _state;  //Record old value of switch to compare next iteration
//     }
//
//     _lastState=1;  //Reset for next round
// 	}
//   delay(200);
//   Serial.println(doubleSipAvg);
//   delay(300);
//   Serial.println("Done");
//
// }
