#include <EasyTransfer.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address (addr, en, rw, rs, d4, d5, d6, d7, backlight, polarity)

//create Easy Transfer objects
//**************remote control****************
EasyTransfer ET1;   // send serial 
EasyTransfer ET2;   // rec serial

//**************Slave Arduinos****************
EasyTransfer ET3;   // slave 1 - back of robot
EasyTransfer ET4;   // slave 2 - front of robot

int axis1;
int axis2;
int axis3;
int axis4;
int axis5;
int axis6;

int mode;
int count;

int menuFlag = 0;  

//**************remote control****************
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
//**************remote control****************
struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};
//**************remote control****************
SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE mydata_remote;

//**************Slave Arduinos****************
struct SLAVE1_DATA_STRUCTURE{   // slave01 - back
  int16_t hipR;
  int16_t hipL;
  int16_t shoulderR;
  int16_t shoulderL;
  int16_t elbowR;
  int16_t elbowL; 
};
//**************Slave Arduinos****************
struct SLAVE2_DATA_STRUCTURE{   // slave02 - front
  int16_t hipR;
  int16_t hipL;
  int16_t shoulderR;
  int16_t shoulderL;
  int16_t elbowR;
  int16_t elbowL; 
};
//**************Slave Arduinos****************
SLAVE1_DATA_STRUCTURE mydata_back;
SLAVE2_DATA_STRUCTURE mydata_front;

unsigned long previousMillis = 0;
const long interval = 40;

long previousSafetyMillis;

void setup() {

  lcd.begin(20,4);   // Initialize the lcd for 20 chars 4 lines, turn on backlight

  // NOTE: Cursor Position: (CHAR, LINE) starts at 0  
  lcd.setCursor(0,0);
  lcd.print("openDog Robot         ");
  lcd.setCursor(0,1);
  lcd.print("XRobots.co.uk       ");

  Serial.begin(115200);
  Serial2.begin(57600);
  Serial1.begin(57600);         // slave01 - front of robot
  Serial3.begin(57600);         // slave02 - back of robot

  //**************remote control****************
  ET1.begin(details(mydata_send), &Serial2);
  ET2.begin(details(mydata_remote), &Serial2);

  //**************Slave Arduinos****************
  ET4.begin(details(mydata_front), &Serial1);
  ET3.begin(details(mydata_back), &Serial3);

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
                Serial.print("*no data* ");
                mydata_remote.RLR = 512;
                mydata_remote.RFB = 512;
                mydata_remote.RT = 512;
                mydata_remote.LLR = 512;
                mydata_remote.LFB = 512;
                mydata_remote.LT = 512;
            }

            count = count+1;                                              // update count for remote monitoring
            lcd.setCursor(0,2);
            lcd.print(count);
            lcd.setCursor(0,3);
            lcd.print("Mode - ");
            lcd.setCursor(7,3);
            lcd.print(mode);
  
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

            //********** manual kinematic / stick handling *********

             double RT = map(mydata_remote.RT,0,1023,490,700);      // Z axis
             double RFB = map(mydata_remote.RFB,0,1023,200,-200);   // Y axis
             double RLR = map(mydata_remote.RLR,0,1023,150,-150);   // X axis
             leg(RT, RFB, RLR, 0);                     // left legs      
             leg(RT, RFB, RLR, 1);                     // right legs  

       }  // end of timed event

}

double leg(double Z, double Y, double X2, int side) {

            #define HIPROD 118L           // offset from hip pivot to middle of leg
            #define HIPROD2 263L          // diagonal length from hip pivot to fixed actuator pivot 
            #define HIPROD3 141L          // length of hip actuator pivot 
            #define HIPOFFSETANGLE 32L    // DEGREES angle of diagnonal between hip pivot and actuator pivot
            double X;                     // X once offsets and inversion are taken ito account          
            double hipHypotenuse;         // the length between hip pivot and foot
            double hipAngle1;             // RADIANS - hip angle from vertical
            double hipAngle1a;             // DEGREES - hip angle from vertical
            double hipAngle2;             // RADIANS - angle at foot between middle of leg and hip pivot
            double hipAngle2a;             // DEGREES - angle at foot between middle of leg and hip pivot
            double hipAngle3;             // RADIANS - angle required from actual actuator
            double hipAngle3a;            // DEGREES - angle required from actual actuator
            double hipActuator;           // output length of ball screw
            double legHeight;             // height of leg in the 'sideways tilting plane'
  
            double shoulderOffset;        // RADIANS - offset angle to move the actual shoulder by, large angle moves the leg forward
            double shoulderOffset_a;       // DEGREES - offset angle to move the actual shoulder by, large angle moves the leg forward
            double legLength;
         
            #define DIGITLENGTH 363L    // length of each top/bottom leg
            #define ELBOWROD 185L       // length of push rod
            #define ELBOWROD2 95L        // other side of push rod triangle
            #define ELBOWACTANGLE 15.8L   // angle between actuator and upper leg joint            
            double elbowAngle;           // the actual angle of the elbow between top and bottom sections
            double elbowAngle2;          // angle between bottom of leg and actuator
            double elbowAngle2a;          // angle between bottom of leg and actuator
            double elbowAngle3;          // other angle
            double elbowAngle3a;          // other angle
            double elbowAngle4;          // other angle
            double elbowAngle4a;          // other angle
            double elbowActuator;        // calculated length of actuator from joint

            #define SHOULDERROD 149L
            #define SHOUDLERROD2 102L
            #define SHOULDERANGLE2 15.8L
            #define SHOULDERANGLE3 17L
            double shoulderAngle;     // initial demand for shoulder (half elbow angle plus offset for moving the leg
            double shoulderAngle2;    // RADIANS - the actual angle we need from the shoulder actuator
            double shoulderAngle2a;   // DEGREES - the actual angle we need from the shoulder actuator
            double shoulderAngle3;    // RADIANS - first ancgle to solve
            double shoulderAngle3a;   // DEGREES - first ancgle to solve
            double shoulderAngle4;    // RADIANS - second angle to solve
            double shoulderAngle4a;    // Degrees - second angle to solve
            double shoulderActuator;  // actual output length of actuator

            //************ X & Z axis calc ***************          
                          
            if (side == 1) {                                      //right leg
              X = HIPROD+X2;                                        
            }
            else if (side == 0) {                                 //left leg
              X = HIPROD-X2;                                     // turn around direction of leg for each side of the robot
            }

            if (X == 0) { X = 0.1; }                             // avoid divide by zero
  
            hipAngle1 = atan(X / Z);                             // calc hip angle from vertical
            hipAngle1a = (hipAngle1 * 4068) / 71;
            hipHypotenuse = X / sin(hipAngle1);                  // calc distance from hip pivot to foot

            hipAngle2 = asin(HIPROD / hipHypotenuse);             // calc angle at foot between middle of leg and hip pivot
            hipAngle2a = (hipAngle2 * 4068) / 71;                 // convert to degrees for debug

            legHeight = HIPROD / tan(hipAngle2);                  // leg height from 'sideways tilting plane' of leg

            hipAngle3a = (90 - HIPOFFSETANGLE) + (hipAngle1a + (180-90-hipAngle2a)) -90 ;        // DEGREES - angle required from actuator
            hipAngle3 = (hipAngle3a * 71) / 4068;                 // convert to radians

            //a2 = b2 + c2 − 2bc cosA            
            hipActuator =  sqrt( sq(HIPROD3)+sq(HIPROD2) - (2 * HIPROD3 * HIPROD2 * cos(hipAngle3)) )-223;      // work out actuator length relative to mid positon            

            //************ Y axis calcs **************

            if (Y == 0) { Y = 0.1; }                             // avoid divide by zero
            shoulderOffset = atan(Y / legHeight);                // calc shoulder angle offset
            shoulderOffset_a = (shoulderOffset * 4068) / 71;     // covert radians to degrees
            legLength = Y / sin(shoulderOffset);                 // calc hypotenuse of triangle to make actual leg length 
            
            //************ elbow calcs ***************
            
            elbowAngle = acos ( (sq(DIGITLENGTH) + sq(DIGITLENGTH) - sq(legLength)) / (2 * DIGITLENGTH * DIGITLENGTH) );   
            elbowAngle = (elbowAngle * 4068) / 71;                          // convert radians to degrees

            elbowAngle2a = elbowAngle - ELBOWACTANGLE;                       // take away known angle        
            elbowAngle2 = (elbowAngle2a * 71) / 4068;                        // convert degrees to radians

            elbowAngle3 = asin ( (ELBOWROD2 * sin(elbowAngle2)/ELBOWROD));
            elbowAngle3a = (elbowAngle3 * 4068 / 71);                        // convert radians to degrees

            elbowAngle4a = 180 - elbowAngle2a - elbowAngle3a;                  // we now know all four angles        
            elbowAngle4 = (elbowAngle4a * 71) / 4068;                        // convert degrees to radians

            elbowActuator = (((sin(elbowAngle4)) * ELBOWROD)) / sin(elbowAngle2); 

            //************ shoulder calcs **************
            
            shoulderAngle = (180-elbowAngle)/2 + shoulderOffset_a;    // initial demand for shoulder (half elbow angle plus offset for moving the leg)
            shoulderAngle2a = 90-shoulderAngle-SHOULDERANGLE2+SHOULDERANGLE3;      // calc actual angle required from shoulder
            shoulderAngle2 = (shoulderAngle2a * 71) / 4068;                        // convert degrees to radians

            shoulderAngle3 = asin ((SHOUDLERROD2 * sin(shoulderAngle2))  /  SHOULDERROD) ;     //calc first unknown angle
            shoulderAngle3a = (shoulderAngle3 * 4068 / 71);                     // convert radians to degrees

            shoulderAngle4a = 180 - shoulderAngle2a - shoulderAngle3a;             // calc out remaining angle
            shoulderAngle4 = (shoulderAngle4a * 71) / 4068;                     // convert degrees to radians
            
            shoulderActuator =  ( (sin(shoulderAngle4)) * SHOULDERROD ) / sin(shoulderAngle2); 

            if (side == 1) {                   // right leg                  
              Serial.print(legHeight);
              Serial.print(" , ");
              Serial.print(X);
              Serial.print(" , ");
              Serial.print(hipAngle3a);
              Serial.print(" , ");
              Serial.print(hipActuator);
              Serial.print(" ****** ");
              
              mydata_front.hipR = hipActuator;
              mydata_back.hipR = hipActuator;
              mydata_front.elbowR = elbowActuator;
              mydata_back.elbowR = elbowActuator;
              mydata_front.shoulderR = shoulderActuator;
              mydata_back.shoulderR = shoulderActuator;
            }

            else if (side == 0) {                // left leg
              Serial.print(legHeight);
              Serial.print(" , ");
              Serial.print(X);
              Serial.print(" , ");
              Serial.print(hipAngle3a);
              Serial.print(" , ");
              Serial.println(hipActuator);
              
              mydata_front.hipL = hipActuator;
              mydata_back.hipL = hipActuator;              
              mydata_front.elbowL = elbowActuator;            
              mydata_back.elbowL = elbowActuator;    
              mydata_front.shoulderL = shoulderActuator;
              mydata_back.shoulderL = shoulderActuator;              
            }


            


            
            if (mode == 1) {                                      // only send the data when I say to!
            ET3.sendData();
            ET4.sendData();  
            
            }     // end of send data function    

}   // end of leg fuction






            


