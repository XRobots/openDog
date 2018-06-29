#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <EasyTransfer.h>
#include <Math.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
SoftwareSerial odrive_serial(6, 7); //RX (ODrive TX), TX (ODrive RX)

// ODrive object
ODriveArduino odrive(odrive_serial);

//create Easy Transfer objects
EasyTransfer ET1;   // send serial
EasyTransfer ET2;   // rec serial

// knee calcs
#define DIGITLENGTH 360L    // length of each top/bottom leg
#define KNEEROD 184L       // length of push rod
#define KNEEROD2 100L        // other side of push rod triangle
#define KNEEACTANGLE 15.8L   // angle between actuator and upp leg joint

double legLength;           // required overall leg length
double kneeAngle;           // the actual angle of the knee between top and bottom sections
double kneeAngle2;          // angle between bottom of leg and actuator
double kneeAngle2a;          // angle between bottom of leg and actuator
double kneeAngle3;          // other angle
double kneeAngle3a;          // other angle
double kneeAngle4;          // other angle.
double kneeAngle4a;          // other angle
double kneeActuator;        // calculated length of actuator from joint

double kneeActuatorFiltered; // filtered output
double kneeActuatorFilteredPrev; // bookmarked previous value
double filter = 15;

int axis1;
int axis2;
int axis3;
int axis4;
int axis5;
int axis6;

int mode;
int count;

int menuFlag = 0;        

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    bool menuDown;      
    bool Select; 
    bool menuUp;  
    bool toggleBottom;  
    bool toggleTop; 
    int mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE mydata_remote;

unsigned long previousMillis = 0;
const long interval = 20;

long previousSafetyMillis;

void setup() {

  odrive_serial.begin(115200);

  Serial.begin(115200);
  Serial2.begin(57600);

  ET1.begin(details(mydata_send), &Serial2);
  ET2.begin(details(mydata_remote), &Serial2);

    // In this example we set the same parameters to both motors.
    // You can of course set them different if you want.  
  for (int motor = 0; motor < 2; ++motor) {
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_CURRENT_CONTROL_CURRENT_LIM, 30.0f);  // [A]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_VEL_LIMIT, 4800000.0f);                 // [counts/s]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_POS_GAIN, 10.0f);                     // [(counts/s) / counts]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_VEL_GAIN, 5.0f/10000.0f);            // [A/(counts/s)]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_VEL_INTEGRATOR_GAIN, 0.0f/10000.0f); // [A/(counts/s * s)]
  }


}

void loop() {

   unsigned long currentMillis = millis();
       if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
            previousMillis = currentMillis;

            if(ET2.receiveData()){                                        // main data receive
                previousSafetyMillis = currentMillis; 

                mydata_send.mode = mode;
                mydata_send.count = count;

                ET1.sendData();                                           // send data back to remote       
            
            } // end of receive data

            else if(currentMillis - previousSafetyMillis > 200) {         // safeties
                Serial.println("no data");
            }

            count = count+1;                                              // update count for remote monitoring
  
            if (mydata_remote.menuUp == 1 && menuFlag == 0) {             // menu select handling & debounce
              menuFlag = 1;
              mode = mode+1;
              mode = constrain(mode,0,5);
            }            
            else if (mydata_remote.menuDown == 1 && menuFlag == 0) {
              menuFlag = 1;
              mode = mode-1;
              mode = constrain(mode,0,5);
            }
            else if (mydata_remote.menuDown == 0 && mydata_remote.menuUp == 0){
              menuFlag = 0;
            }

            // initial leg length calcs

            legLength = map(mydata_remote.RT,0,1023,350,710);             // map control data to mm

            kneeAngle = acos ( (sq(DIGITLENGTH) + sq(DIGITLENGTH) - sq(legLength)) / (2 * DIGITLENGTH * DIGITLENGTH) );          
            kneeAngle = (kneeAngle * 4068) / 71;                          // convert radians to degrees
            kneeAngle2a = kneeAngle - KNEEACTANGLE;                       // take away known angle
            kneeAngle2 = (kneeAngle2a * 71) / 4068;                        // convert degrees to radians

            kneeAngle3 = asin ( (KNEEROD2 * sin(kneeAngle2)/KNEEROD));
            kneeAngle3a = (kneeAngle3 * 4068 / 71);                        // convert radians to degrees

            kneeAngle4a = 180 - kneeAngle2a - kneeAngle3a;                  // we now know all four angles
            kneeAngle4 = (kneeAngle4a * 71) / 4068;                        // convert degrees to radians

            kneeActuator = (((sin(kneeAngle4)) * KNEEROD)) / sin(kneeAngle2);

            kneeActuator = kneeActuator - 95;
            kneeActuator = constrain(kneeActuator,0,110);

            // filter           

            kneeActuatorFiltered = (kneeActuator + (kneeActuatorFilteredPrev * filter)) / (filter +1);
            kneeActuatorFilteredPrev = kneeActuatorFiltered;

            // write to Odrive
            
            odrive.SetPosition(0, kneeActuatorFiltered * 3490);

            Serial.print(legLength);
            Serial.print(" , ");
            Serial.print(kneeActuator); 
            Serial.print(" , ");
            Serial.println(kneeActuatorFiltered); 
          

       }  // end of timed event

}

