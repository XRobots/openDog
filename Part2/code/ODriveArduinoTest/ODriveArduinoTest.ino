
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
SoftwareSerial odrive_serial(8, 9); //RX (ODrive TX), TX (ODrive RX)

// ODrive object
ODriveArduino odrive(odrive_serial);

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino alpha.");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  for (int motor = 0; motor < 2; ++motor) {
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_CURRENT_CONTROL_CURRENT_LIM, 30.0f);  // [A]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_VEL_LIMIT, 4800000.0f);                 // [counts/s]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_POS_GAIN, 20.0f);                     // [(counts/s) / counts]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_VEL_GAIN, 15.0f/10000.0f);            // [A/(counts/s)]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_VEL_INTEGRATOR_GAIN, 0.0f/10000.0f); // [A/(counts/s * s)]
  }

  Serial.println("Ready!");
  
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

  if (Serial.available()) {
    char c = Serial.read();


    // test 1

    if (c == 'a') {
        odrive.SetPosition(0, 349000);      // move 100mm
    }
    if (c == 'b') {
        odrive.SetPosition(0, 00000);      
    }
    
    if (c == 'c') {
        odrive.SetParameter(0, odrive.PARAM_FLOAT_VEL_LIMIT, 5000.0f);    
    }
    if (c == 'd') {
        odrive.SetParameter(0, odrive.PARAM_FLOAT_VEL_LIMIT, 4800000.0f);              
    }
    if (c == 'e') {
        odrive.SetPosition(0, 0);
    }
    if (c == 'f') {
        odrive.SetPosition(0, 40000);                
    }
    if (c == 'g') {
        odrive.SetVelocity(0, 0);               
    }


    // Sinusoidal test move
    if (c == 's') {
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
      Serial << "Vbus voltage: " << odrive.getBusVoltage() << '\n';
    }

    // print motor position
    if (c == 'p') {
        long encoder = odrive.GetParameter(0, odrive.PARAM_FLOAT_ENCODER_PLL_POS);
        Serial.print("encoder position is: ");
        Serial.println(encoder);

    }
  }
}

