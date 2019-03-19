#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

double Pk1 = 5000;  
double Ik1 = 0;
double Dk1 = 0;
double Setpoint1, Input1, Output1, Output1a;    // PID variables

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

#include <SoftwareSerial.h>
#include <ODriveArduino.h>

int pot1;
int pot2;

unsigned long previousMillis = 0;
const long interval = 20;

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
SoftwareSerial odrive_serial(10, 11); //RX (ODrive TX), TX (ODrive RX)

// ODrive object

//ODriveArduino odrive(odrive_serial);


 //ODriveArduino odrive(Serial2);
 ODriveArduino odrive(Serial1);

void setup() {

    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-4800000, 4800000);
    PID1.SetSampleTime(20);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

 
  Serial2.begin(115200);
  Serial1.begin(115200);

  analogReadResolution(13);

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    Serial1 << "w axis" << axis << ".motor.config.current_lim " << 40.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }


  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'v' to read bus voltage");
  Serial.println("Send the character 'p' to read motor position");
  Serial.println("Send the character 'a' do the test1");
  Serial.println("Send the character 'b' do the test2");
  Serial.println("Send the character 'c' turn down speed");
  Serial.println("Send the character 'd' turn up speed");
  Serial.println("Send the character 'e' small step");
  Serial.println("Send the character 'f' small step");
  Serial.println("Send the character 'g' stop");
}

void loop() {

       unsigned long currentMillis = millis();
         if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
            previousMillis = currentMillis; 

            Setpoint1 = pot2;
            Input1 = pot1;
            PID1.Compute();
          
            pot1 = analogRead(0);
            pot2 = analogRead(1);
            pot1 = map(pot1, 0,8191,0,1023);
            pot2 = map(pot2, 0,8191,0,1023);
            Serial.print("analog 0 is: ");
            Serial.print(pot1);
            Serial.print(", analog 1 is: ");
            Serial.print(pot2);
            Serial.print(", Output is ");
            Serial.println(Output1);
            
            //odrive.SetVelocity(0, Output1);

            
          
            if (Serial.available()) {
              char c = Serial.read();
          
              // Run calibration sequence
              if (c == '0' || c == '1') {
                int motornum = c-'0';
                int requested_state;     
          
                requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
                Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
                odrive.run_state(motornum, requested_state, true);
          
                requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
                odrive.run_state(motornum, requested_state, true);
          
                requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
                Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
                odrive.run_state(motornum, requested_state, false); // don't wait
              }
          
              if (c == 'a') {
                  odrive.SetPosition(0, 349000);      // move 100mm
              }
              if (c == 'b') {
                  odrive.SetPosition(0, 00000);      
              }
          
              if (c == 'c') {       
                  for (int axis = 0; axis < 2; ++axis) {
                      Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 5000.0f << '\n';
                  }
              }
              
              if (c == 'd') {
                  for (int axis = 0; axis < 2; ++axis) {
                      Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 4800000.0f << '\n';
                  }                 
              }
              
              if (c == 'e') {
                  odrive.SetPosition(0, 0);
                  delay(1000);
                  Serial1 << "r axis" << 0 << ".encoder.pos_estimate\n";
                  Serial << odrive.readFloat() << '\n';
              }
              if (c == 'f') {
                  odrive.SetPosition(0, 40000); 
                  delay(1000);     
                  Serial1 << "r axis" << 0 << ".encoder.pos_estimate\n";
                  Serial << odrive.readFloat() << '\n';
                            
              }
              if (c == 'g') {
                  odrive.SetVelocity(0, 0);               
              }
          
          
              if (c == 'p') {
                for (int motor = 0; motor < 2; ++motor) {
                      Serial1 << "r axis" << motor << ".encoder.pos_estimate\n";
                      Serial << odrive.readFloat() << '\n';
                    }
              }
          
          
              // Sinusoidal test move
              if (c == 's') {
                Serial.println("Executing test move");
                for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
                  float pos_m0 = 20000.0f * cos(ph);
                  float pos_m1 = 20000.0f * sin(ph);
                  odrive.SetPosition(0, pos_m0);
                  odrive.SetPosition(1, pos_m1);
                  delay(5);
                }
              }
          
              // Read bus voltage
              if (c == 'v') {
                Serial1 << "r vbus_voltage\n";
                Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
              }
          
             
        }   // end of serial available

       

        
      }       // end of timed event

}   // end of main loop



