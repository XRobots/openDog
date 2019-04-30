#include <Ramp.h>						              // https://github.com/siteswapjuggler/RAMP

int newValue;                 // used inmain loop
int outValue;                 // used in main loop
int duration;                 // used in main loop

int interpolationFlag = 0;    // used by function
int savedValue;               // used by function

rampInt myRamp;							                // new ramp object (ramp<unsigned char> by default)

void setup() {
  Serial.begin(115200);					            // begin Serial communication   
}


void loop() {
  
    if (Serial.available()>2) {  // check for enough serial data
                  // read the incoming byte:
                  newValue = Serial.parseInt();
                  duration = Serial.parseInt();                
                     
    }           // end of serial receive

    outValue = interpolation(newValue, duration); 
   
    Serial.println(outValue);        // print the actual interpolation value 
    delay(10);
  
}

// separate function for interpolation

int interpolation(int input, int duration) {

      if (input != savedValue) {   // check for new data
          interpolationFlag = 0;
      }
      savedValue = input;          // bookmark the old value  
    
      if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
          myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
          interpolationFlag = 1;
      }
    
      int output = myRamp.update(); 

      return output;  
}

