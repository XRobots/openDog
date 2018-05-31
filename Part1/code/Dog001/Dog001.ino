#include <EasyTransfer.h>
#include <Math.h>

//create object
EasyTransfer ET1;   // send serial
EasyTransfer ET2;   // rec serial

// knee calcs
#define DIGITLENGTH 330L    // length of each top/bottom leg
#define KNEEROD 180L       // length of push rod
#define KNEEROD2 94L        // other side of push rod triangle
#define KNEEACTANGLE 15L   // angle between actuator and upp leg joint

double legLength;           // required overall leg length
double kneeAngle;           // the actual angle of the knee between top and bottom sections
double kneeAngle2;          // angle between bottom of leg and actuator
double kneeAngle2a;          // angle between bottom of leg and actuator
double kneeAngle3;          // other angle
double kneeAngle3a;          // other angle
double kneeAngle4;          // other angle.
double kneeAngle4a;          // other angle
double kneeActuator;        // calculated length of actuator from joint

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

  Serial.begin(57600);
  Serial2.begin(57600);

  ET1.begin(details(mydata_send), &Serial2);
  ET2.begin(details(mydata_remote), &Serial2);


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

            legLength = map(mydata_remote.RT,0,1023,380,650);             // map control data to mm

            kneeAngle = acos ( (sq(DIGITLENGTH) + sq(DIGITLENGTH) - sq(legLength)) / (2 * DIGITLENGTH * DIGITLENGTH) );          
            kneeAngle = (kneeAngle * 4068) / 71;                          // convert radians to degrees
            kneeAngle2a = kneeAngle - KNEEACTANGLE;                       // take away known angle
            kneeAngle2 = (kneeAngle2a * 71) / 4068;                        // convert degrees to radians

            kneeAngle3 = asin ( (KNEEROD2 * sin(kneeAngle2)/KNEEROD));
            kneeAngle3a = (kneeAngle3 * 4068 / 71);                        // convert radians to degrees

            kneeAngle4a = 180 - kneeAngle2a - kneeAngle3a;                  // we now know all four angles
            kneeAngle4 = (kneeAngle4a * 71) / 4068;                        // convert degrees to radians

            kneeActuator = (((sin(kneeAngle4)) * KNEEROD)) / sin(kneeAngle2);

            Serial.print(legLength);
            Serial.print(" , ");
            Serial.println(kneeActuator);      
          

       }  // end of timed event

}

