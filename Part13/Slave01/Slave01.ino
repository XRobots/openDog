#include <EasyTransfer.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

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
  int16_t filter;
};
//**************Slave Arduinos****************
SLAVE1_DATA_STRUCTURE mydata_back;

// Slave 1 - Back End - bottom ctrl panel!

int but1;     // left bottom
int but2;     // right bottom
int but3;     // left top
int but4;     // right top

int home1 = 1;
int home1a = 1;
int home2 = 1;
int home2a = 1;
int home3 = 1;
int home3a = 1;
int home4 = 1;
int home4a = 1;
int home5 = 1;
int home5a = 1;
int home6 = 1;
int home6a = 1;

long home1Offset;
long home2Offset;
long home3Offset;
long home4Offset;
long home5Offset;
long home6Offset;

long home1Home;
long home2Home;
long home3Home;
long home4Home;
long home5Home;
long home6Home;

int flag = 0;

int motorSpeedFlag = 0;

int requested_state;

unsigned long previousFilterMillis;   // digital pin filter
int filterTime = 200;

unsigned long previousStartupmillis;
int startupFlag = 0;

unsigned long previousMillis = 0;
const long interval = 10;

double hipRFiltered;          // value in mm
double hipLFiltered;
double hipRFiltered2;         // value in encoder counts  
double hipLFiltered2;
double shoulderRFiltered;     // value in mm
double shoulderLFiltered;
double shoulderRFiltered2;    // value in encoder counts
double shoulderLFiltered2;
double elbowRFiltered;        // value in mm
double elbowLFiltered; 
double elbowRFiltered2;       // value in encoder counts
double elbowLFiltered2;

double hipROutput;            // actual output in encoder counts
double hipLOutput;
double shoulderROutput;       
double shoulderLOutput;
double elbowROutput;
double elbowLOutput;

void setup() {

  // LEDs

  pinMode(33, OUTPUT);              // white left bottom
  pinMode(35, OUTPUT);              // white right bottom
  pinMode(37, OUTPUT);              // yellow left top
  pinMode(39, OUTPUT);              // yellow right top

  // control panel buttons 

  pinMode(25, INPUT_PULLUP);        // left bottom
  pinMode(27, INPUT_PULLUP);        // right bottom
  pinMode(29, INPUT_PULLUP);        // left top
  pinMode(31, INPUT_PULLUP);        // right top  

  // home switches

  pinMode(43, INPUT_PULLUP);
  pinMode(45, INPUT_PULLUP);
  pinMode(47, INPUT_PULLUP);
  pinMode(49, INPUT_PULLUP);
  pinMode(51, INPUT_PULLUP);
  pinMode(53, INPUT_PULLUP);

  Serial.begin(115200);
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
  for (int axis = 0; axis < 2; ++axis) {
    Serial3 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    Serial3 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }
  // left leg
  for (int axis = 0; axis < 2; ++axis) {
    Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    Serial2 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }
  // undercariage
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial  << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    odrive_serial  << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  }

  delay (500); // wait for everything to finish
  digitalWrite(39, HIGH); // set initial homing LED yellow right top

}

void loop() {

   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {  //start timed event
      previousMillis = currentMillis;       

        but1 = digitalRead(25);
        but2 = digitalRead(27);
        but3 = digitalRead(29);
        but4 = digitalRead(31);
              
        home1 = digitalRead(43);
        home2 = digitalRead(45);
        home3 = digitalRead(47);
        home4 = digitalRead(49);
        home5 = digitalRead(51);
        home6 = digitalRead(53); 


        // *****************************right leg**********************************
      
        if (but4 == 0 && flag == 0) {
          digitalWrite(39, LOW);
          
          Serial.println("Right Motor 0");        
          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(0, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(0, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(0, requested_state, false); // don't wait

          while (home4a  == 1) {
            home4 = digitalRead(49);
            if (home4 == 1) {
              previousFilterMillis = millis();
            }
            else if (home4 == 0 && millis() - previousFilterMillis > filterTime) {
              home4a = 0;
            }  
            // move motor 0
            odrive2.SetVelocity(0, 10000);
          }
          // stop motor 0
          odrive2.SetVelocity(0, 0);
          delay(300);
          //save zero position and back off two revolutions
          Serial3 << "r axis" << 0 << ".encoder.pos_estimate\n";
          home4Offset = odrive2.readInt();
          Serial.println(home4Offset);
          odrive2.SetPosition(0, (home4Offset-(8192*2)));  // back off two revolutions
          delay (500);    // wait for that to properly finish

          Serial.println("Right Motor 1");                  
          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(1, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(1, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(1, requested_state, false); // don't wait 
                         
          while (home3a == 1) {
            home3 = digitalRead(47);
            if (home3 == 1) {
              previousFilterMillis = millis();
            }
            else if (home3 == 0 && millis() - previousFilterMillis > filterTime) {
              home3a = 0;
            }
            // move motor 1
            odrive2.SetVelocity(1, 10000);
          }
          //stop motor 1
          odrive2.SetVelocity(1, 0);
          delay(300);
          //save zero position and back off two revolutions
          Serial3 << "r axis" << 1 << ".encoder.pos_estimate\n";
          home3Offset = odrive2.readInt();
          Serial.println(home3Offset);
          odrive2.SetPosition(1, (home3Offset-(8192*2)));  // back off one revolution
          flag = 1;
          digitalWrite(37, HIGH);     // Yellow top left
        }

        
      
        // *********************************left leg************************************
        
        else if (but3 == 0 && flag == 1) {
          digitalWrite(37, LOW);
          
          Serial.println("Left Motor 0");      
          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(0, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(0, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(0, requested_state, false); // don't wait

          while (home2a  == 1) {
            home2 = digitalRead(45);
            if (home2 == 1) {
              previousFilterMillis = millis();
            }
            else if (home2 == 0 && millis() - previousFilterMillis > filterTime) {
              home2a = 0;
            }  
            // move motor 0
            odrive1.SetVelocity(0, 10000);
          }
          // stop motor 0
          odrive1.SetVelocity(0, 0);
          delay(300);
          //save zero position and back off two revolutions
          Serial2 << "r axis" << 0 << ".encoder.pos_estimate\n";
          home2Offset = odrive1.readInt();
          Serial.println(home2Offset);
          odrive1.SetPosition(0, (home2Offset-(8192*2)));  // back off two revolutions
          delay (500);    // wait for that to properly finish

          Serial.println("Left Motor 1");            
          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(1, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(1, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(1, requested_state, false); // don't wait

          while (home1a == 1) {
            home1 = digitalRead(43);
            if (home1 == 1) {
              previousFilterMillis = millis();
            }
            else if (home1 == 0 && millis() - previousFilterMillis > filterTime) {
              home1a = 0;
            }
            // move motor 1
            odrive1.SetVelocity(1, 10000);
            }
          //stop motor 1
          odrive1.SetVelocity(1, 0);
          delay(300);
          //save zero position and back off two revolutions
          Serial2 << "r axis" << 1 << ".encoder.pos_estimate\n";
          home1Offset = odrive1.readInt();
          Serial.println(home1Offset);
          odrive1.SetPosition(1, (home1Offset-(8192*2)));  // back off one revolution 

          /* ********** commenting out so calibration takes place on the stand with the legs bent

          // time to boost the legs up
          digitalWrite(37, HIGH);   // yellow left
          digitalWrite(39, HIGH);   // yellow right
          flag = 2;       
        }

        else if (but4 == 0 || but3 == 0 && flag == 2) {
           digitalWrite(37, LOW);
           digitalWrite(39, LOW);           

           for (int axis = 0; axis < 2; ++axis) {
                Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 48000.0f << '\n';     // set motor speed to fast ODrive1               
           }
           for (int axis = 0; axis < 2; ++axis) {
                Serial3 << "w axis" << axis << ".controller.config.vel_limit " << 48000.0f << '\n';     // set motor speed to fast ODrive2         
           }
           
           odrive2.SetPosition(0, (home4Offset - 163840));      // move legs straight right
           odrive2.SetPosition(1, (home3Offset - 245760));      // move legs straight right
                         
           odrive1.SetPosition(0, (home2Offset - 163840));      // move legs straight left
           odrive1.SetPosition(1, (home1Offset - 245760));      // move legs straight left     
        */
           digitalWrite(35, HIGH); // white LED right - ready for undercarriage calibration
           flag = 3;
           
        }
        

        // *****************************leg motor undercarriage**************************************
          
        else if (but2 == 0 && flag == 3) {
          // bend right leg
          digitalWrite(35, LOW);

          /*
          
          odrive2.SetPosition(0, (home4Offset - 16384));      // move leg bent right
          odrive2.SetPosition(1, (home3Offset - 16384));      // move leg bent right
          delay (2000); // wait for leg to bend

          */
          
          // calibrate right leg undercarriage, ODrive axis 1
          Serial.println("Right Leg undercarriage");          
          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive0.run_state(1, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive0.run_state(1, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive0.run_state(1, requested_state, false); // don't wait
          delay (500);    // wait for that to properly finish

          /*     COMMENTING OUT -  CALIBRATION IS DONE WITH MANUAL ALIGNMENT

          // move undercarriage ODrive axis 1 under it hits the home switch 6

          while (home6a == 1) {
            home6 = digitalRead(53);
            if (home6 == 1) {
              previousFilterMillis = millis();
            }
            else if (home6 == 0 && millis() - previousFilterMillis > filterTime) {
              home6a = 0;
            }
            // move motor 1
            odrive0.SetVelocity(1, -10000);
            }
          //stop motor 1
          odrive0.SetVelocity(1, 0);
          
          //save zero position         
          odrive_serial << "r axis" << 1 << ".encoder.pos_estimate\n";
          home6Offset = odrive0.readInt(); 
          Serial.print(home6Offset);
          Serial.print(" , ");
          while ((home6Offset < -122000) || (home6Offset > -52000)) {         // filter because software serial is a bit rubbish
            odrive_serial << "r axis" << 1 << ".encoder.pos_estimate\n";  
            home6Offset = odrive0.readInt();
          }
          Serial.println(home6Offset);                                      // print filtered value         
          
          odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << 48000.0f << '\n';     // set motor speed to fast for Odrive 0 / axis 1          
          odrive0.SetPosition(1, (home6Offset+87654));  // back off 25mm
          delay(3000);   // wait for leg to move out again   

          

          // put leg straight again
          odrive2.SetPosition(0, (home4Offset - 163840));      // move legs straight right
          odrive2.SetPosition(1, (home3Offset - 245760));      // move legs straight right    
          */    
          
          digitalWrite(33, HIGH);   // white LED left bottom - ready for the other side
          flag = 4;
          }

        //**************** do the other leg *****************
        
        else if (but1 == 0 && flag == 4) {
          digitalWrite(33, LOW);
          /*
          odrive1.SetPosition(0, (home2Offset - 16384));      // move leg bent left
          odrive1.SetPosition(1, (home1Offset - 16384));      // move leg bent left
          delay (2000); // wait for leg to bend
          
          */
                   
          // calibrate right leg undercarriage, ODrive axis 0
          Serial.println("Right Leg undercarriage");          
          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive0.run_state(0, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive0.run_state(0, requested_state, true);      
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive0.run_state(0, requested_state, false); // don't wait
          delay (500);    // wait for that to properly finish

          

          /*     COMMENTING OUT -  CALIBRATION IS DONE WITH MANUAL ALIGNMENT
                
          // move undercarriage ODrive axis 0 under it hits the home switch 5

          while (home5a == 1) {
            home5 = digitalRead(51);
            if (home5 == 1) {
              previousFilterMillis = millis();
            }
            else if (home5 == 0 && millis() - previousFilterMillis > filterTime) {
              home5a = 0;
            }
            // move motor 0
            odrive0.SetVelocity(0, -10000);
            }
            
          //stop motor 0
          odrive0.SetVelocity(0, 0);
          
          //save zero position         
          odrive_serial << "r axis" << 0 << ".encoder.pos_estimate\n";
          home5Offset = odrive0.readInt(); 
          Serial.print(home5Offset);
          Serial.print(" , ");
          while ((home5Offset < -122000) || (home5Offset > -52000)) {         // filter because software serial is a bit rubbish
            odrive_serial << "r axis" << 0 << ".encoder.pos_estimate\n";  
            home5Offset = odrive0.readInt();
          }
          Serial.println(home5Offset);                                      // print filtered value         
          
          odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 48000.0f << '\n';     // set motor speed to fast for Odrive 0 / axis 0          
          odrive0.SetPosition(0, (home5Offset+87654));  // back off 25mm
          delay(3000);   // wait for leg to move out again

          

    
          // put leg straight again
          odrive1.SetPosition(0, (home2Offset - 163840));      // move legs straight left
          odrive1.SetPosition(1, (home1Offset - 245760));      // move legs straight left   

          delay(3000);    // wait for the leg to be straight 

          */
          flag = 5;       // proceed to next stage
        }

          if (flag == 5) {        // main output to leg starts here

                if (motorSpeedFlag == 0) {   // only send the data once ever
                
                        // turn up motor speeds a bit
                        odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 192000.0f << '\n';   
                        odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << 192000.0f << '\n';     
        
                        Serial2 << "w axis" << 0 << ".controller.config.vel_limit " << 440000.0f << '\n';
                        Serial2 << "w axis" << 1 << ".controller.config.vel_limit " << 400000.0f << '\n';
        
                        Serial3 << "w axis" << 0 << ".controller.config.vel_limit " << 440000.0f << '\n';
                        Serial3 << "w axis" << 1 << ".controller.config.vel_limit " << 400000.0f << '\n';
                        
                        motorSpeedFlag = 1;  // set the flag after this has run once so it never runs again

                }

                // ***********serial receive from master***************
                if (ET3.receiveData()) {     

                  if(startupFlag == 0) {
                    previousStartupmillis = currentMillis;  // start filter settling timer 
                    startupFlag = 1; // only do this once ever the first time you get data
                  }

                mydata_back.shoulderR = constrain(mydata_back.shoulderR, 80,200);
                mydata_back.shoulderL = constrain(mydata_back.shoulderL, 80,200);
                mydata_back.elbowR = constrain(mydata_back.elbowR, 80,200);
                mydata_back.elbowL = constrain(mydata_back.elbowL, 80,200);
                mydata_back.hipR = constrain(mydata_back.hipR, -35,35);
                mydata_back.hipL = constrain(mydata_back.hipL, -35,35);

                shoulderRFiltered = filter(mydata_back.shoulderR, shoulderRFiltered);  
                shoulderLFiltered = filter(mydata_back.shoulderL, shoulderLFiltered);           
                elbowRFiltered = filter(mydata_back.elbowR, elbowRFiltered);
                elbowLFiltered = filter(mydata_back.elbowL, elbowLFiltered);

                // ********* LEGS OUTPUT *********
        
                shoulderRFiltered2 = shoulderRFiltered*3490;   // work out encoder counts per milimeter
                shoulderLFiltered2 = shoulderLFiltered*3490;   // work out encoder counts per milimeter
                elbowRFiltered2 = elbowRFiltered*3490;   // work out encoder counts per milimeter
                elbowLFiltered2 = elbowLFiltered*3490;   // work out encoder counts per milimeter  

                shoulderROutput = home4Offset - (715450 - shoulderRFiltered2);      // use offset from homing minus the actual postion of the leg from the joint
                elbowROutput = home3Offset - (715450 - elbowRFiltered2);            
                
                shoulderLOutput = home2Offset - (715450 - shoulderLFiltered2);      // use offset from homing minus the actual postion of the leg from the joint
                elbowLOutput = home1Offset - (715450 - elbowLFiltered2);             
   
                // ******** HIP - UNDERCARRIAGE OUTPUT *********

                hipRFiltered = filter(mydata_back.hipR, hipRFiltered);    // filtered value in mm
                hipLFiltered = filter(mydata_back.hipL, hipLFiltered);    // filtered value in mm
                hipRFiltered2 = hipRFiltered * 3490;                      // work out encoder counts per mm
                hipLFiltered2 = hipLFiltered * 3490;                      // work out encoder counts per mm

                // calibration is done by manually aligning
                //hipROutput = (home6Offset+87654)+hipRFiltered;            // +/- mid actuator postion with new data
                //hipLOutput = (home5Offset+87654)+hipLFiltered;            // +/- mid actuator postion with new data

                hipROutput = hipRFiltered2;            // +/- mid actuator postion with new data
                hipLOutput = hipLFiltered2;            // +/- mid actuator postion with new data

                if (currentMillis - previousStartupmillis > 3000) {   // makes sure filter has settled the first time                
                    odrive2.SetPosition(0, shoulderROutput);     // output position to actuator RIGHT LEG
                    odrive2.SetPosition(1, elbowROutput);        // output position to actuator RIGHT LEG
                    odrive1.SetPosition(0, shoulderLOutput);     // output position to actuator LEFT LEG
                    odrive1.SetPosition(1, elbowLOutput);        // output position to actuator LEFT LEG 
                    odrive0.SetPosition(0, hipLOutput);                        // output data to ODrive
                    odrive0.SetPosition(1, hipROutput);                        // output data to ODrive
                }
                    
                
                } // end of receive data

               
          }   // end of main output to leg

   } // end of timed event


}


//***************filter joint motions*****************

double filter(double lengthOrig, double currentValue) {
  double filter = mydata_back.filter;
  double lengthFiltered =  (lengthOrig + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;  
}






