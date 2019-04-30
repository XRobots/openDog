#include <EasyTransfer.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

#define LED_LEFT_BOTTOM_WHITE  33
#define LED_RIGHT_BOTTOM_WHITE 35
#define LED_LEFT_TOP_YELLOW    37
#define LED_RIGHT_TOP_YELLOW   39

#define BUTTON1_PIN 25
#define BUTTON2_PIN 27
#define BUTTON3_PIN 29
#define BUTTON4_PIN 31
        
#define HOME1_PIN 43
#define HOME2_PIN 45
#define HOME3_PIN 47
#define HOME4_PIN 49
#define HOME5_PIN 51
#define HOME6_PIN 53


//**************Slave Arduinos****************
EasyTransfer ET3;   // slave 1 - back of robot

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
SoftwareSerial odrive_serial(10, 11); //RX (ODrive TX), TX (ODrive RX)  // undercarriage

// ODrive objects

ODriveArduino odrive0(odrive_serial);    // undercarriage

ODriveArduino odrive2(Serial3);   // right leg
ODriveArduino odrive1(Serial2);   // left leg

//**************Slave Arduinos****************
struct SLAVE1_DATA_STRUCTURE{
  int16_t hipR;
  int16_t hipL;
  int16_t shoulderR;
  int16_t shoulderL;
  int16_t elbowR;
  int16_t elbowL; 
};
//**************Slave Arduinos****************
SLAVE1_DATA_STRUCTURE mydata_back;

// Slave 1 - Back End - bottom ctrl panel!

int but1;     // left bottom
int but2;     // right bottom
int but3;     // left top
int but4;     // right top

enum HomeSwitchIndex
{
  HOME_SWITCH_LEFT_ELBOW_X,
  HOME_SWITCH_LEFT_SHOULDER_X,
  HOME_SWITCH_RIGHT_ELBOW_X,
  HOME_SWITCH_RIGHT_SHOULDER_X,
  HOME_SWITCH_LEFT_SHOULDER_Y,
  HOME_SWITCH_RIGHT_SHOULDER_Y
};
int homeSwitchState[6]         = {1, 1, 1, 1, 1, 1};
int homeSwitchStateFiltered[6] = {1, 1, 1, 1, 1, 1};
long homeOffset[6];
long homePos[6];

int flag = 0;

int requested_state;

unsigned long previousFilterMillis;   // digital pin filter
int filterTime = 200;

unsigned long previousMillis = 0;
const long interval = 20;

double hipRFiltered;
double hipLFiltered;
double shoulderRFiltered;     // value in mm
double shoulderLFiltered;
double shoulderRFiltered2;    // value in encoder counts
double shoulderLFiltered2;
double elbowRFiltered;
double elbowLFiltered; 

void setup()
{
  // LEDs
  pinMode(LED_LEFT_BOTTOM_WHITE, OUTPUT);              // white left bottom
  pinMode(LED_RIGHT_BOTTOM_WHITE, OUTPUT);              // white right bottom
  pinMode(LED_LEFT_TOP_YELLOW, OUTPUT);              // yellow left top
  pinMode(LED_RIGHT_TOP_YELLOW, OUTPUT);              // yellow right top

  // control panel buttons 
  pinMode(BUTTON1_PIN, INPUT_PULLUP);        // left bottom
  pinMode(BUTTON2_PIN, INPUT_PULLUP);        // right bottom
  pinMode(BUTTON3_PIN, INPUT_PULLUP);        // left top
  pinMode(BUTTON4_PIN, INPUT_PULLUP);        // right top  

  // home switches
  pinMode(HOME1_PIN, INPUT_PULLUP);
  pinMode(HOME2_PIN, INPUT_PULLUP);
  pinMode(HOME3_PIN, INPUT_PULLUP);
  pinMode(HOME4_PIN, INPUT_PULLUP);
  pinMode(HOME5_PIN, INPUT_PULLUP);
  pinMode(HOME6_PIN, INPUT_PULLUP);

  Serial.begin(57600);
  Serial2.begin(115200);
  Serial3.begin(115200);
  odrive_serial.begin(115200);    // software serial

  //**************Slave Arduinos****************
  Serial1.begin(57600);
  ET3.begin(details(mydata_back), &Serial1);

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // ***set mtor parameters for initial setup***

  // right leg
  for (int axis = 0; axis < 2; ++axis)
  {
    Serial3 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    Serial3 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }
  // left leg
  for (int axis = 0; axis < 2; ++axis)
  {
    Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    Serial2 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }
  // undercariage
  for (int axis = 0; axis < 2; ++axis)
  {
    odrive_serial  << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    odrive_serial  << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }

  delay (500); // wait for everything to finish
  digitalWrite(LED_RIGHT_TOP_YELLOW, HIGH); // set initial homing LED yellow right top
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < interval)
  {
    return;
  }

  //start timed event
  previousMillis = currentMillis;       

  but1 = digitalRead(BUTTON1_PIN);
  but2 = digitalRead(BUTTON2_PIN);
  but3 = digitalRead(BUTTON3_PIN);
  but4 = digitalRead(BUTTON4_PIN);

  homeSwitchState[HOME_SWITCH_LEFT_ELBOW_X]     = digitalRead(HOME1_PIN);
  homeSwitchState[HOME_SWITCH_LEFT_SHOULDER_X]  = digitalRead(HOME2_PIN);
  homeSwitchState[HOME_SWITCH_RIGHT_ELBOW_X]    = digitalRead(HOME3_PIN);
  homeSwitchState[HOME_SWITCH_RIGHT_SHOULDER_X] = digitalRead(HOME4_PIN);
  homeSwitchState[HOME_SWITCH_LEFT_SHOULDER_Y]  = digitalRead(HOME5_PIN);
  homeSwitchState[HOME_SWITCH_RIGHT_SHOULDER_Y] = digitalRead(HOME6_PIN);  
  
  // *****************************right leg**********************************
  
  if (but4 == 0 && flag == 0)
  {
    digitalWrite(LED_RIGHT_TOP_YELLOW, LOW);
    FindStop (odrive2, 0, 10000, HOME_SWITCH_RIGHT_SHOULDER_X, HOME4_PIN, "Right");
    FindStop (odrive2, 1, 10000, HOME_SWITCH_RIGHT_ELBOW_X,    HOME3_PIN, "Right");
    flag = 1;
    digitalWrite(LED_LEFT_TOP_YELLOW, HIGH);     // Yellow top left
  }
  
  // *********************************left leg************************************
  
  else if (but3 == 0 && flag == 1)
  {
    digitalWrite(LED_LEFT_TOP_YELLOW, LOW);

    FindStop (odrive1, 0, 10000, HOME_SWITCH_LEFT_SHOULDER_X, HOME2_PIN, "Left");
    FindStop (odrive1, 1, 10000, HOME_SWITCH_LEFT_ELBOW_X,    HOME1_PIN, "Left");
  
    // time to boost the legs up
    digitalWrite(LED_LEFT_TOP_YELLOW, HIGH);   // yellow left
    digitalWrite(LED_RIGHT_TOP_YELLOW, HIGH);   // yellow right
    flag = 2;       
  }

  
  else if (but4 == 0 || but3 == 0 && flag == 2)
  {
    digitalWrite(LED_LEFT_TOP_YELLOW, LOW);
    digitalWrite(LED_RIGHT_TOP_YELLOW, LOW);           
    for (int axis = 0; axis < 2; ++axis)
    {
      Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 48000.0f << '\n';     // set motor speed to fast ODrive1               
    }
    for (int axis = 0; axis < 2; ++axis)
    {
      Serial3 << "w axis" << axis << ".controller.config.vel_limit " << 48000.0f << '\n';     // set motor speed to fast ODrive2         
    }
     
    odrive2.SetPosition(0, (homeOffset[HOME_SWITCH_RIGHT_SHOULDER_X] - 163840));      // move legs straight right
    odrive2.SetPosition(1, (homeOffset[HOME_SWITCH_RIGHT_ELBOW_X]    - 245760));      // move legs straight right
                   
    odrive1.SetPosition(0, (homeOffset[HOME_SWITCH_LEFT_SHOULDER_X]  - 163840));      // move legs straight left
    odrive1.SetPosition(1, (homeOffset[HOME_SWITCH_LEFT_ELBOW_X]     - 245760));      // move legs straight left     
  
    digitalWrite(LED_RIGHT_BOTTOM_WHITE, HIGH); // white LED right - ready for undercarriage calibration
    flag = 3;
  }
  
  // *****************************leg motor undercarriage**************************************
   
  else if (but2 == 0 && flag == 3)
  {
    // bend right leg
    digitalWrite(LED_RIGHT_BOTTOM_WHITE, LOW);
    odrive2.SetPosition(0, (homeOffset[HOME_SWITCH_RIGHT_SHOULDER_X] - 16384));      // move leg bent right
    odrive2.SetPosition(1, (homeOffset[HOME_SWITCH_RIGHT_ELBOW_X]    - 16384));      // move leg bent right
    delay (2000); // wait for leg to bend

    FindStop (odrive0, 1, -10000, HOME_SWITCH_RIGHT_SHOULDER_Y, HOME6_PIN, "Right Leg undercarriage");
  /* TODO: Should this be done?
    odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << 48000.0f << '\n';     // set motor speed to fast for Odrive 0 / axis 1          
    odrive0.SetPosition(1, (home6Offset+87654));  // back off 25mm
    delay(3000);   // wait for leg to move out again       
  */    
    // put leg straight again
    odrive2.SetPosition(0, (homeOffset[HOME_SWITCH_RIGHT_SHOULDER_X] - 163840));      // move legs straight right
    odrive2.SetPosition(1, (homeOffset[HOME_SWITCH_RIGHT_ELBOW_X]    - 245760));      // move legs straight right        
    
    digitalWrite(LED_LEFT_BOTTOM_WHITE, HIGH);   // white LED left bottom - ready for the other side
    flag = 4;
  }
  //**************** do the other leg *****************
  
  else if (but1 == 0 && flag == 4)
  {
    digitalWrite(LED_LEFT_BOTTOM_WHITE, LOW);
    odrive1.SetPosition(0, (homeOffset[HOME_SWITCH_LEFT_SHOULDER_X] - 16384));      // move leg bent left
    odrive1.SetPosition(1, (homeOffset[HOME_SWITCH_LEFT_ELBOW_X]    - 16384));      // move leg bent left
    delay (2000); // wait for leg to bend
    
    FindStop (odrive0, 0, -10000, HOME_SWITCH_LEFT_SHOULDER_Y, HOME5_PIN, "Left Leg undercarriage");
  /* TODO: Should this be done?    
    odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 48000.0f << '\n';     // set motor speed to fast for Odrive 0 / axis 0          
    odrive0.SetPosition(0, (home6Offset+87654));  // back off 25mm
    delay(3000);   // wait for leg to move out again      
  */  
    // put leg straight again
    odrive1.SetPosition(0, (homeOffset[HOME_SWITCH_LEFT_SHOULDER_X] - 163840));      // move legs straight left
    odrive1.SetPosition(1, (homeOffset[HOME_SWITCH_LEFT_ELBOW_X]    - 245760));      // move legs straight left   
  
    delay(3000);    // wait for the leg to be straight          
    flag = 5;       // proceed to next stage
  }
  
  if (flag == 5)
  {        // main output to leg starts here
  
    // ***********serial receive from master***************
  
    ET3.receiveData();
            
    Serial.print(mydata_back.shoulderR);    // print original test value
    Serial.print(" , ");
    shoulderRFiltered = filter(mydata_back.shoulderR, shoulderRFiltered);
    Serial.print(shoulderRFiltered);
  
    Serial.print(" , ");
  
    Serial.print(mydata_back.shoulderL);    // print original test value
    Serial.print(" , ");
    shoulderLFiltered = filter(mydata_back.shoulderL, shoulderLFiltered);
    Serial.println(shoulderLFiltered);
  
    shoulderRFiltered2 = shoulderRFiltered*3490;   // work out encoder counts per milimeter
    shoulderLFiltered2 = shoulderLFiltered*3490;   // work out encoder counts per milimeter
  
    homePos[HOME_SWITCH_LEFT_SHOULDER_X]  = homeOffset[HOME_SWITCH_LEFT_SHOULDER_X]  - 163840;    // the current position it's at
    homePos[HOME_SWITCH_RIGHT_SHOULDER_X] = homeOffset[HOME_SWITCH_RIGHT_SHOULDER_X] - 163840;    // the current position it's at
  
    odrive2.SetPosition(0, (homePos[HOME_SWITCH_LEFT_SHOULDER_X] + shoulderRFiltered2));      // use the test remote data to move the actuator
    odrive1.SetPosition(0, (homePos[HOME_SWITCH_RIGHT_SHOULDER_X] + shoulderLFiltered2));      // use the test remote data to move the actuator     
  
  }                       // end of main output to leg
}




void FindStop (ODriveArduino odrive, int axis, long velocity, int homeSwitchIndex, int homeSwitchPin, String logPrefix)
{   
  Serial << logPrefix << " Motor 0\n";        
  requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
  odrive.run_state(axis, requested_state, true);      
  requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
  odrive.run_state(axis, requested_state, true);      
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
  odrive.run_state(axis, requested_state, false); // don't wait

  // TODO: If necessary, add a parameter that determines if we wait for 500 milliseconds here. As was done for the undercarriage calibration.

  while (homeSwitchStateFiltered[homeSwitchIndex] == 1)
  {
    homeSwitchState[homeSwitchIndex] = digitalRead(homeSwitchPin);
    if (homeSwitchState[homeSwitchIndex] == 1)
    {
      previousFilterMillis = millis();
    }
    else if (homeSwitchState[homeSwitchIndex] == 0 && millis() - previousFilterMillis > filterTime)
    {
      homeSwitchStateFiltered[homeSwitchIndex] = 0;
    }  
    // move motor 0
    odrive.SetVelocity(axis, velocity);
  }
  // stop motor 0
  odrive.SetVelocity(axis, 0);
  delay(300);
  //save zero position and back off two revolutions
  Serial3 << "Axis" << axis << ".encoder.pos_estimate\n";
  homeOffset[homeSwitchIndex] = odrive.readInt();
  Serial.println(homeOffset[homeSwitchIndex]);

  // TODO: If necessary, add a parameter that determines if we need to "filter" the position.
  // TODO: If necessary, change the velocity limit to 48000.0f here?

  odrive.SetPosition(axis, (homeOffset[homeSwitchIndex] - (8192 * 2)));  // back off two revolutions

  //TODO: If necessary, add a parameter to alter the time we wait as it was different for the undercarriage calibration.
  delay (500);    // wait for that to properly finish
}


//***************filter joint motions*****************

double filter(double lengthOrig, double currentValue)
{
  double filter = 15;
  double lengthFiltered =  (lengthOrig + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;  
}






