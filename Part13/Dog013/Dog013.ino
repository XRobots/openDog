#include <EasyTransfer.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

double Pk1 = 5;  //roll
double Ik1 = 0;
double Dk1 = 0;
double Setpoint1, Input1, Output1;  // PID variables, position - side to side

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

double Pk2 = 0;   // pitch
double Ik2 = 0;
double Dk2 = 0;
double Setpoint2, Input2, Output2;  // PID variables, position - front to back

PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);


// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address (addr, en, rw, rs, d4, d5, d6, d7, backlight, polarity)

//create Easy Transfer objects
//**************remote control****************
EasyTransfer ET1;   // send serial 
EasyTransfer ET2;   // rec serial

//**************Slave Arduinos****************
EasyTransfer ET3;   // slave 1 - back of robot
EasyTransfer ET4;   // slave 2 - front of robot

//**************IMU***************************
EasyTransfer ET5;   // IMU

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


//**************IMU****************
struct RECEIVE_DATA_STRUCTURE2{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO  
  float pitch;
  float roll;
};
RECEIVE_DATA_STRUCTURE2 mydata_IMU;     // IMU data from other Arduino

//**************Slave Arduinos****************
struct SLAVE1_DATA_STRUCTURE{   // slave01 - back
  int16_t hipR;
  int16_t hipL;
  int16_t shoulderR;
  int16_t shoulderL;
  int16_t elbowR;
  int16_t elbowL; 
  int16_t filter;
};
//**************Slave Arduinos****************
struct SLAVE2_DATA_STRUCTURE{   // slave02 - front
  int16_t hipR;
  int16_t hipL;
  int16_t shoulderR;
  int16_t shoulderL;
  int16_t elbowR;
  int16_t elbowL; 
  int16_t filter;
};
//**************Slave Arduinos****************
SLAVE1_DATA_STRUCTURE mydata_back;
SLAVE2_DATA_STRUCTURE mydata_front;

unsigned long previousMillis = 0;
const long interval = 50;

unsigned long gaitFlag1;
long previousGait1Millis;

long previousSafetyMillis;

// *** leg Gait vars ***

int leg1Z;
int leg2Z;
int leg3Z;
int leg4Z;

int leg1Y = 0;
int leg2Y = 0;
int leg3Y = 0;
int leg4Y = 0;

int leg1X = 0;
int leg2X = 0;
int leg3X = 0;
int leg4X = 0;

int Gait1X;

double pitch;
double roll;

double filter = 30;

void setup() {
  
  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(50);
  
  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(50);

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

  //**************Remote control****************
  ET1.begin(details(mydata_send), &Serial2);
  ET2.begin(details(mydata_remote), &Serial2);

  //**************Slave Arduinos****************
  ET4.begin(details(mydata_front), &Serial1);
  ET3.begin(details(mydata_back), &Serial3);        // only uses TX pin

  //**************IMU***************************
  ET5.begin(details(mydata_IMU), & Serial3);         // only uses RX pin
}

void loop() {

   unsigned long currentMillis = millis();
       if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
            previousMillis = currentMillis;

            mydata_back.filter = filter;        // set the slave filter values to the the filter
            mydata_front.filter = filter;

            if (ET5.receiveData()) {
              pitch = mydata_IMU.pitch;
              roll = mydata_IMU.roll;
            }

            
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
            //pitch = pitch +0.89;
            //roll = roll = roll -0.27;

                        

            

            
            Input1 = roll;
            Setpoint1 = 0;
            PID1.Compute();


            Input2 = pitch;
            Setpoint2 = 0;
            PID2.Compute();


            Serial.print(Output1);
            Serial.print(" , ");
            Serial.println(Output2);

             // ************ scale sticks ***************

             double RT = map(mydata_remote.RT,0,1023,490,700);      // Z axis
             double RFB = map(mydata_remote.RFB,0,1023,-200,200);   // Y axis
             double RLR = map(mydata_remote.RLR,0,1023,150,-150);   // X axis

             double LT = map(mydata_remote.LT,0,1023,-20,20);
             double LFB = map(mydata_remote.LFB,0,1023,-20,20);
             double LLR = map(mydata_remote.LLR,0,1023,-20,20);

            // ********** Gait 1 goes here ***************

            if (mode == 2) {

                  filter = 15;

                  int offset = -30;
                   
                  if (gaitFlag1 == 0) {         
                    //DO NOTHING HERE, just set up the timer
                    //but  make the legs slightly longer so they can lift up enough
                    leg1Z = Gait1Z(4);
                    leg2Z = Gait1Z(4);
                    leg3Z = Gait1Z(4);
                    leg4Z = Gait1Z(4);
                    //*******
                    leg1Y = offset;
                    leg2Y = offset;
                    leg3Y = offset;
                    leg4Y = offset;
                    //*******
                    leg1X = 50;
                    leg2X = 50;
                    leg3X = -50;
                    leg4X = -50;
                    gaitFlag1 = 1;
                    previousGait1Millis = currentMillis;                                      // when this is done the delay comes from the next 'if' statement
                    }
                    
                  else if (gaitFlag1 == 1 && currentMillis - previousGait1Millis > 500) {    // wait for this amount of time and then do the things in here
                    //lift left leg
                    leg1Z = Gait1Z(2);
                    leg2Z = Gait1Z(4);
                    leg3Z = Gait1Z(4);
                    leg4Z = Gait1Z(2);
                    /*
                    leg1Y = Gait1Y(2, RFB);
                    leg2Y = Gait1Y(4, RFB);
                    leg3Y = Gait1Y(4, RFB);
                    leg4Y = Gait1Y(2, RFB);
                    */
                    gaitFlag1 = 2;
                    previousGait1Millis = currentMillis;                                     // when this is done the delay comes from the next 'if' statement
                    }
                    
                  else if (gaitFlag1 == 2 && currentMillis - previousGait1Millis > 200){    // wait for this amount of time and then do the things in here
                    //do some stuff
                    leg1Z = Gait1Z(3);
                    leg2Z = Gait1Z(1);
                    leg3Z = Gait1Z(1);
                    leg4Z = Gait1Z(3);
                    /*
                    leg1Y = Gait1Y(3, RFB);
                    leg2Y = Gait1Y(1, RFB);
                    leg3Y = Gait1Y(1, RFB);
                    leg4Y = Gait1Y(3, RFB);
                    */
                    gaitFlag1 = 3;
                    previousGait1Millis = currentMillis;                                     // when this is done the delay comes from the next 'if' statement
                  }
                  
                  else if (gaitFlag1 == 3 && currentMillis - previousGait1Millis > 400){     // up/down
                    //do some stuff
                    leg1Z = Gait1Z(4);
                    leg2Z = Gait1Z(1);
                    leg3Z = Gait1Z(1);
                    leg4Z = Gait1Z(4);
                    /*
                    leg1Y = Gait1Y(4, RFB);
                    leg2Y = Gait1Y(1, RFB);
                    leg3Y = Gait1Y(1, RFB);
                    leg4Y = Gait1Y(4, RFB);
                    */
                    gaitFlag1 = 4;
                    previousGait1Millis = currentMillis;
                  }
                  
                  else if (gaitFlag1 == 4 && currentMillis - previousGait1Millis > 500){
                    //lift right leg
                    leg1Z = Gait1Z(4);
                    leg2Z = Gait1Z(2);
                    leg3Z = Gait1Z(2);
                    leg4Z = Gait1Z(4);
                    /*
                    leg1Y = Gait1Y(4, RFB);
                    leg2Y = Gait1Y(2, RFB);
                    leg3Y = Gait1Y(2, RFB);
                    leg4Y = Gait1Y(4, RFB);
                    */
                    gaitFlag1 = 5;
                    previousGait1Millis = currentMillis;
                  }

                  else if (gaitFlag1 == 5 && currentMillis - previousGait1Millis > 200){   
                    //do some stuff
                    leg1Z = Gait1Z(1);
                    leg2Z = Gait1Z(3);
                    leg3Z = Gait1Z(3);
                    leg4Z = Gait1Z(1);
                    /*
                    leg1Y = Gait1Y(1, RFB);
                    leg2Y = Gait1Y(3, RFB);
                    leg3Y = Gait1Y(3, RFB);
                    leg4Y = Gait1Y(1, RFB);
                    */
                    gaitFlag1 = 6;
                    previousGait1Millis = currentMillis;
                  }

                  else if (gaitFlag1 == 6 && currentMillis - previousGait1Millis > 400){  //up/down
                    //do some stuff
                    leg1Z = Gait1Z(1);
                    leg2Z = Gait1Z(4);
                    leg3Z = Gait1Z(4);
                    leg4Z = Gait1Z(1);
                    /*
                    leg1Y = Gait1Y(1, RFB);
                    leg2Y = Gait1Y(4, RFB);
                    leg3Y = Gait1Y(4, RFB);
                    leg4Y = Gait1Y(1, RFB);
                    */
                    gaitFlag1 = 1;
                    previousGait1Millis = currentMillis;
                  }

                  if (gaitFlag1 == 2 || gaitFlag1 == 3 ) {
                    leg1Y = offset-((Output1*1.5)*1);
                    leg2Y = offset-((Output1*1.5)*1);
                    leg3Y = offset-((Output1*1.5)*1);
                    leg4Y = offset-((Output1*1.5)*1);

                    leg1X = 50+Output1;
                    leg2X = 50+Output1;
                    leg3X = -50+Output1;
                    leg4X = -50+Output1;
                  }

                  else if (gaitFlag1 == 5 || gaitFlag1 == 6) {
                    leg1Y = offset+((Output1*1.5)*1);
                    leg2Y = offset+((Output1*1.5)*1);
                    leg3Y = offset+((Output1*1.5)*1);
                    leg4Y = offset+((Output1*1.5)*1);

                    leg1X = 50+Output1;
                    leg2X = 50+Output1;
                    leg3X = -50+Output1;
                    leg4X = -50+Output1;
                  }

           
                  
                  leg(leg1Z, leg1Y, leg1X-25, 0, 0, (Output1/5)*-1, 0, 1); // left front leg
                  leg(leg2Z, leg2Y, leg2X-25, 0, 0, (Output1/5)*-1, 0, 0); // left back leg
                  leg(leg3Z, leg3Y, leg3X-25, 0, 0, (Output1/5)*-1, 1, 1); // right front leg
                  leg(leg4Z, leg4Y, leg4X-25, 0, 0, (Output1/5)*-1, 1, 0); // right back leg  
               
              
            }

            else {      // if the mode is not 2, it is currently probably mode 1

                  filter = 20;
      
                  gaitFlag1 = 0; // reset the flag for Gait 1 to the first part of the sequence.
      
                  //********** manual kinematic / stick handling *********
                               
                  leg(RT, RFB, RLR+50, LT, LFB, LLR, 0, 0);                     // left back      
                  leg(RT, RFB, RLR+50, LT, LFB, LLR, 0, 1);                     // left front  
                  leg(RT, RFB, RLR-50, LT, LFB, LLR, 1, 0);                     // right back      
                  leg(RT, RFB, RLR-50, LT, LFB, LLR, 1, 1);                     // right front
             
            }           // end of manual control

       }  // end of timed event

}    // ************** end of main loop **************************





// *********************** function for positing feet for walking **************************

// *** Z axis ***
int Gait1Z(int pos){
  int Z;       // output Z axis postion. Mid point 590 +/- 100mm height from ground - small number makes the leg shorter.

  if (pos == 2 || pos == 3) {
    Z = 560;    // make leg shorter
  }
  else {
    Z = 600;    // make leg longer
  }
  return Z;     // return the Z position to make the leg get longer and shorter
}

// *** Y axis ***

int Gait1Y(int pos, int stick) {      // takes the postion in teh sequence, the current Y leg position, and the right hand FB stick
  int Y;      // output Y axis position based on zero point, current leg Y position and bias from RFB stick position
  if (pos == 4) {
    Y = stick -40;
  }
  else if (pos == 1) {
    Y = (stick*-1) -40;
  }
  else if (pos == 2) {
    Y = stick -40;
  }
  else if (pos == 3) {
    Y = (stick*-1) - 40;
  }


  return Y;     // return the Y position to make the leg move back and forward
}

// *********************** Kinematic model fuction ******************************************


double leg(double Z4, double Y4, double X4, double yaw, double pitch2, double roll, int side, int front) {
  
            // ************* XYZ consts and vars *****************************************************************
            #define HIPROD 118L           // offset from hip pivot to middle of leg
            #define HIPROD2 263L          // diagonal length from hip pivot to fixed actuator pivot 
            #define HIPROD3 141L          // length of hip actuator pivot 
            #define HIPOFFSETANGLE 32L    // DEGREES angle of diagnonal between hip pivot and actuator pivot
            double X;                     // X once offsets and inversion are taken ito account
            double X2;                    // X after yaw moves;
            double X3;                    // X at the roll calc stage  
            double Y;                     // Y to give to XYZ calcs
            double Y2;                    // Y after yaw moves
            double Z;                     // Z to give to XYZ cals
            double Z2;                    // Z after yaw moves
            double Z3;                     // Z at the roll calc stage
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

            // **************** Pitch Roll Yaw consts and vars **********************************************

            //  Pitch consts and vars - side view
            #define BODYRODY 287L      // half the distance from shoulder to shoulder on the Y side plane/view   
            double pitch;             // pitch after switching around for front/back of robot
            double differenceZ;         // difference in height between center point and new shoulder height, in Y axis
            double shoulderHeight;    // resulting new height of shoulder from ground as body tilts in pitch, in a vertical axis from the ground
            double legLength2;        // length of actual leg we need to calculate the eventual new Y and Z
            double distanceY;         // the new distance from the centre to the shoulder joint, taking into account the pitch angle
            double differenceY;       // the difference between foot placement and new distance from centre to shoulder joint
            double shoulderAngle5;    // RADIANS - angle of leg from vertical
            double shoulderAngle5a;    // DEGREES - angle of leg from vertical
            double shoulderAngle6;    // RADIANS - angle of leg from new Z
            double shoulderAngle6a;    // DEGREES - angle of leg from new Z

            // Roll consts and vars - end view
            #define BODYRODX 110L     // half the distance from hip pivot to hip pivot in the X axis
            double differenceZ2;      // difference in height between the centre point and new hip neight, in an X axis
            double hipHeight;         // resulting new height of hip from ground as the tits in roll, in a vertical line from the ground
            double legLength3;        // the new ***length of the VIRTUAL leg*** we need to calulate the new Y and Z
            double distanceX;         // the new distance from the centre to the hip joint, taking into account the roll angle
            double differenceX;       // the difference between foot placement and the new distance from the centre to the hip joint
            double hipAngle5;         // RADIANS the angle of the *VIRTUAL leg* from vertical
            double hipAngle5a;        // DEGREES the angle of the *VIRTUAL leg* from vertical
            double hipAngle6;         // RADIANS the angle of the *VIRTUAL* leg from the new Z
            double hipAngle6a;        // DEGREES the angle of the *VIRTUAL* leg from the new Z

            // Yaw consts and vars - top view
            double radius;            // radius/hypotenuse of triangle of leg from top XY view
            double yawAngle1;         // RADIANS - the origin angle of the radius using existing stick posotions
            double yawAngle1a;        // DEGREES - the origin angle of the radius using existing stick posotions
            double yawAngle2;         // RADIANS - the new demand angle of the radius 
            double yawAngle2a;        // DEGREES - the new demand angle of the radius 

            if (front == 1) {
              Y4 = Y4 + BODYRODY;
            }
            else if (front == 0) {
              Y4 = Y4 - BODYRODY;
            }
            if (side == 1) {
              X4 = X4 + (BODYRODX + HIPROD);
            }
            if (side == 0) {
              X4 = X4 - (BODYRODX + HIPROD);
            }

            yaw = (yaw * 71) / 4068;                // convert to RADIANS

            yawAngle1 = atan(X4/Y4);                  // calc existing angle of leg from centre
            yawAngle1a = (yawAngle1 * 4068) / 71;   // DEGREES - for debug            

            radius = X4/sin(yawAngle1);              // calc radius from centre

            yawAngle2 = yawAngle1 + yaw;
            yawAngle2a = (yawAngle2 * 4068) / 71;   // DEGREES - for debug

            X2 = radius * sin(yawAngle2);           // calc new X and Y based on new yaw angle
            Y2 = radius * cos(yawAngle2);
  
            if (front == 1) {                       // remnove offsets again to give results centred around zero
              Y2 = Y2 - BODYRODY;
            }
            else if (front == 0) {
              Y2 = Y2 + BODYRODY;
            }
            if (side == 1) {
              X2 = X2 - (BODYRODX + HIPROD);
            }
            else if (side == 0){
              X2 = X2 + (BODYRODX + HIPROD);
            }   

            Z2 = Z4;                              // pass through Z as it isn't needed in yaw calcs
                     
            // switch round X and roll for each side of the robot
            if (side == 1) {                                      //right leg
              X3 = HIPROD+X2;                                       
            }
            else if (side == 0) {                                 //left leg
              X3 = HIPROD-X2; 
            }

            // switch around pitch angle for back or front of robot
            if (front == 1) {                                     // front of robot
              pitch = pitch2*-1;
            }
            else if (front == 0) {                                // back of robot
              pitch = pitch2;
            }            

            // **************** Pitch calcs **********************************************

            pitch = (pitch * 71) / 4068;                        // convert pitch to RADIANS
            differenceZ = sin(pitch) * BODYRODY;                // calc opposite side of triangle
            distanceY = cos(pitch) * BODYRODY;                  // calc adjacent side of triangle

            shoulderHeight = Z2 + differenceZ;                   // calc new shoulder height as pitch changes
            differenceY = Y2 + (BODYRODY - distanceY);           // distance from original foot position and vertical line to shoulder

            if (front == 1) {                                     // switch around the data if it's the front of the robot
              differenceY = differenceY*-1;
            }

            shoulderAngle5 = atan(differenceY/shoulderHeight);    // RADIANS angle between leg and vetical
            shoulderAngle5a = (shoulderAngle5 * 4068) / 71;      // DEGREES convert for debug
            legLength2 = shoulderHeight/cos(shoulderAngle5);      // calc out new length of leg
            
            shoulderAngle6 = pitch+shoulderAngle5;               // RADIANS calc angle for new Z to 90' from body

            if (front == 1) {                                    // switch around the data if it's the front of the robot
              shoulderAngle6 = shoulderAngle6*-1;
            }
            

            shoulderAngle6a = (shoulderAngle6 * 4068) / 71;      // DEGREES convert for debug

            Z3 = legLength2*cos(shoulderAngle6);                  // new Z to pass on to roll calcs
            Y = legLength2*sin(shoulderAngle6);                   // new Y to pass on to roll cals 

            // **************** Roll calcs ***************************************************

            roll = ((roll * 71) / 4068)*-1;                       // convert roll to RADIANS
            differenceZ2 = sin(roll) * BODYRODX;                  // calc opposite side of triangle
            distanceX = cos(roll) * BODYRODX;                     // calc adjacent side of triangle
            
            if (side == 0) {                                      // *********** switch around for each side of the robot
                hipHeight = Z3 - differenceZ2;                    // new height of the hip from the ground taking into account roll
            }
            if (side == 1) {
                hipHeight = Z3 + differenceZ2;
            }
            
            differenceX = X3 + (BODYRODX - distanceX);            // diatance from original foot postion and vertical line to shoulder.

            if (side == 1) {                                      // *********** switch around for each side of the robot
              differenceX = differenceX*-1;
            }        

            hipAngle5 = atan(differenceX/hipHeight);              // RADIANS angle between *virtual* leg and vertical
            hipAngle5a = (hipAngle5 * 4068) / 71;                 // DEGREES convert for debug 
           
            legLength3 = hipHeight/cos(hipAngle5);                // calc new *virtual leg* length

            hipAngle6 = roll+hipAngle5;                          // RADIANS calc angle for new Z to 90' from body
            hipAngle6a = (hipAngle6 * 4068) / 71;                // DEGREES convert for debug
       
            if (side == 1) {                                     // *********** switch around for each side of the robot
              hipAngle6 = hipAngle6 *-1;
            }
                           
            X = legLength3*sin(hipAngle6);                        // new X to pass onto XYZ calcs        
            Z = legLength3*cos(hipAngle6);                        // new Z to pass onto XYZ calcs          
                      
            
            //************ X & Z axis calc for each leg ***************   
                         
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

            if (Y == 0) { Y = 0.1; }
            // avoid divide by zero
            shoulderOffset = atan((Y*-1) / legHeight);                // calc shoulder angle offset
            shoulderOffset_a = (shoulderOffset * 4068) / 71;     // covert radians to degrees
            legLength = (Y*-1) / sin(shoulderOffset);                 // calc hypotenuse of triangle to make actual leg length 
            
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

            

           // ********************** output variables *********************************************                

              if (side == 1 && front == 1) {                     // front right leg                    
                mydata_front.hipR = hipActuator;
                mydata_front.shoulderR = shoulderActuator;
                mydata_front.elbowR = elbowActuator;
                /*
                Serial.print(hipActuator);
                Serial.print(" , ");
                Serial.print(shoulderActuator);
                Serial.print(" , ");
                Serial.print(elbowActuator);
                Serial.print(" *** ");
                */
              }                             
              else if (side == 1 && front == 0) {               // back right leg
                mydata_back.hipR = hipActuator; 
                mydata_back.shoulderR = shoulderActuator;             
                mydata_back.elbowR = elbowActuator; 
                /*
                Serial.print(hipActuator);
                Serial.print(" , ");
                Serial.print(shoulderActuator);
                Serial.print(" , ");
                Serial.print(elbowActuator);
                Serial.print(" *** ");     
                */
              }              
              else if (side == 0 && front  == 1) {              // front right leg
                mydata_front.hipL = hipActuator;
                mydata_front.shoulderL = shoulderActuator;
                mydata_front.elbowL = elbowActuator;
                /*
                Serial.print(hipActuator);
                Serial.print(" , ");
                Serial.print(shoulderActuator);
                Serial.print(" , ");
                Serial.print(elbowActuator);
                Serial.print(" *** ");
                */
              }               
              else if (side == 0 && front == 0) {               // back right leg
                mydata_back.hipL = hipActuator; 
                mydata_back.shoulderL = shoulderActuator;             
                mydata_back.elbowL = elbowActuator; 
                /*
                Serial.print(hipActuator);
                Serial.print(" , ");
                Serial.print(shoulderActuator);
                Serial.print(" , ");
                Serial.print(elbowActuator);
                Serial.println(" *** ");   
                */
              } 

            // **************** only send the data when I say to! ***********************
                       
            if (mode == 1 || mode == 2) {                                      
            ET3.sendData();
            ET4.sendData();             
            }     // end of send data function    

}   // end of leg fuction






            


