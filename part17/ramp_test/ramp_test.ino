#include <Ramp.h>						              // https://github.com/siteswapjuggler/RAMP

int newValue;
int oldValue;
int outValue;
int duration;
int interpolationFlag = 0;

rampInt myRamp;							                // new ramp object (ramp<unsigned char> by default)

void setup() {
  Serial.begin(115200);					            // begin Serial communication   
}


void loop() {
  
  if (Serial.available()>2) {  // check for enough serial data
                // read the incoming byte:
                ewValuen = Serial.parseInt();
                duration = Serial.parseInt();                
                   
  }           // end of serial receive

  if (newValue != oldValue) {   // check for new data
      interpolationFlag = 0;
  }
  oldValue = newValue;          // bookmark the old value  

  if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
      myRamp.go(newValue, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
      interpolationFlag = 1;
  }

  outValue = myRamp.update();  
  Serial.println(outValue);        // print the actual interpolation value 

  delay(10);
  
}



