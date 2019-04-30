#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x3F,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// ODrive objects

ODriveArduino odrive1(Serial1);   // front right leg
ODriveArduino odrive2(Serial2);   // front left leg
ODriveArduino odrive3(Serial3);   // front undercarriage
ODriveArduino odrive4(Serial4);   // rear right leg
ODriveArduino odrive5(Serial5);   // rear left leg
ODriveArduino odrive6(Serial6);   // rear undercarriage

RF24 radio(25, 24); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    int16_t menuDown;      
    int16_t Select; 
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t mode;  
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

// *************** RAW values from the kinematic model in mm ******************

// back of robot
struct BACK_DATA_STRUCTURE{   // back of robot
  double hipR;
  double hipL;
  double shoulderR;
  double shoulderL;
  double elbowR;
  double elbowL; 
};
// front of robot
struct FRONT_DATA_STRUCTURE{   // front of robot
  double hipR;
  double hipL;
  double shoulderR;
  double shoulderL;
  double elbowR;
  double elbowL; 
};

BACK_DATA_STRUCTURE mydata_back;
FRONT_DATA_STRUCTURE mydata_front;

// ************** Actual encoder counts to move the axis by******************************

// back of robot
struct COUNT_BACK_DATA_STRUCTURE{   // back of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};
// front of robot
struct COUNT_FRONT_DATA_STRUCTURE{   // front of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};

COUNT_BACK_DATA_STRUCTURE mydata_back_count;
COUNT_FRONT_DATA_STRUCTURE mydata_front_count;

// ************** CORRECTED Actual encoder counts to move the axis by ******************************
// ************** Offset for physical zero position removed

// back of robot
struct COUNT_CORR_BACK_DATA_STRUCTURE{   // back of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};
// front of robot
struct COUNT_CORR_FRONT_DATA_STRUCTURE{   // front of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};

COUNT_CORR_BACK_DATA_STRUCTURE mydata_back_count_corr;
COUNT_CORR_FRONT_DATA_STRUCTURE mydata_front_count_corr;

// *************** Bookmarked values from last time (used to calculate ratio of motor speeds)

// back of robot
struct COUNT_CORR_BM_BACK_DATA_STRUCTURE{   // back of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};
// front of robot
struct COUNT_CORR_BM_FRONT_DATA_STRUCTURE{   // front of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};

COUNT_CORR_BM_BACK_DATA_STRUCTURE mydata_back_bm;
COUNT_CORR_BM_FRONT_DATA_STRUCTURE mydata_front_bm;

// **************** difference in joint positions between each cycle *******************

// back of robot
struct COUNT_CORR_DIFF_BACK_DATA_STRUCTURE{   // back of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};
// front of robot
struct COUNT_CORR_DIFF_FRONT_DATA_STRUCTURE{   // front of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};

COUNT_CORR_DIFF_BACK_DATA_STRUCTURE mydata_back_diff;
COUNT_CORR_DIFF_FRONT_DATA_STRUCTURE mydata_front_diff;

// *************** Final FILTERED values in counts ******************

// back of robot
struct FILTERED_BACK_DATA_STRUCTURE{   // back of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};
// front of robot
struct FILTERED_FRONT_DATA_STRUCTURE{   // front of robot
  long hipR;
  long hipL;
  long shoulderR;
  long shoulderL;
  long elbowR;
  long elbowL; 
};

FILTERED_BACK_DATA_STRUCTURE mydata_back_filtered;
FILTERED_FRONT_DATA_STRUCTURE mydata_front_filtered;



unsigned long currentMillis;

unsigned long previousMillis = 0;       // main loop
const long interval = 5;

unsigned long previousSafetyMillis = 0;

unsigned long previousStartupmillis;
int startupFlag = 0;

int remoteFlag = 0;
int menuFlag = 0;
int mode = 0;

int ODriveFlag = 0;

int requested_state;

void setup() {

  lcd.init();  // initialize the lcd 
  lcd.backlight();
  lcd.begin(20,4);   // Initialize the lcd for 20 chars 4 lines, turn on backlight

  lcd.setCursor(0,0);
  lcd.print("XRobots openDog     ");

  lcd.setCursor(0,3);
  lcd.print("Mode - ");
  lcd.setCursor(7,3);
  lcd.print(mode);  
  lcd.print("   ");      // clear any character left after the mode value

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);

  Serial.begin(115200);   // USB debug
  Serial1.begin(115200);  // front right leg 
  Serial2.begin(115200);  // front left leg 
  Serial3.begin(115200);  // front undercarriage 
  Serial4.begin(115200);  // rear right leg
  Serial5.begin(115200);  // rear left leg
  Serial6.begin(115200);  // rear undercarriage

}

void loop() {

        currentMillis = millis();   
        if (currentMillis - previousMillis >= 10) {     // start of 10ms cycle

        previousMillis = currentMillis;   // reset the clock so the loop timing is correct

        radio.startListening();
            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));
                    previousSafetyMillis = currentMillis; 
        }    

        //  ************** menu handling & debounce *******************
  
        if (mydata_remote.menuUp == 1 && menuFlag == 0) {           
          menuFlag = 1;
          mode = mode+1;
          mode = constrain(mode,0,10);
          lcd.setCursor(0,3);
          lcd.print("Mode - ");
          lcd.setCursor(7,3);
          lcd.print(mode);  
          lcd.print("   ");      // clear any character left after the mode value
        }            
        else if (mydata_remote.menuDown == 1 && menuFlag == 0) {
          menuFlag = 1;
          mode = mode-1;
          mode = constrain(mode,0,10);
          lcd.setCursor(0,3);
          lcd.print("Mode - ");
          lcd.setCursor(7,3);
          lcd.print(mode);  
          lcd.print("   ");      // clear any character left after the mode value
        }
        else if (mydata_remote.menuDown == 0 && mydata_remote.menuUp == 0){
        menuFlag = 0;

        } 

       // check if remote has become disconnected

        if(currentMillis - previousSafetyMillis > 200) {         
            Serial.println("*no data* ");
            mydata_remote.RLR = 512;
            mydata_remote.RFB = 512;
            mydata_remote.RT = 512;
            mydata_remote.LLR = 512;
            mydata_remote.LFB = 512;
            mydata_remote.LT = 512;
        }  

        // Do the ODrive startup when I say

        if (mode == 1 && mydata_remote.Select == 1 && ODriveFlag == 0) {     
          ODriveSetup();
          ODriveFlag = 1;
        }

        // print control data, count and mode to terminal
        /*
        Serial.print(mydata_remote.menuDown);
        Serial.print(" , ");
        Serial.print(mydata_remote.Select);
        Serial.print(" , ");
        Serial.print(mydata_remote.menuUp);
        Serial.print(" , ");
        Serial.print(mydata_remote.toggleTop);
        Serial.print(" , ");
        Serial.print(mydata_remote.toggleBottom);
        Serial.print(" , ");
        Serial.print(mode);
        Serial.print(" , ");
        Serial.print(count);
        Serial.print(" *** ");
        Serial.print(mydata_remote.RLR);
        Serial.print(" , ");
        Serial.print(mydata_remote.RFB);
        Serial.print(" , ");
        Serial.print(mydata_remote.RT);
        Serial.print(" , ");
        Serial.print(mydata_remote.LLR);
        Serial.print(" , ");
        Serial.print(mydata_remote.LFB);
        Serial.print(" , ");
        Serial.println(mydata_remote.LT);
        */

        // run Kinematic Model function for each leg if we are in mode 2

        if (mode == 2) {   
          
            // scale stick values to mm    
            double RT = map(mydata_remote.RT,0,1023,490,700);      // Z axis
            double RFB = map(mydata_remote.RFB,0,1023,-200,200);   // Y axis
            double RLR = map(mydata_remote.RLR,0,1023,150,-150);   // X axis
            double LT = map(mydata_remote.LT,0,1023,-20,20);
            double LFB = map(mydata_remote.LFB,0,1023,-20,20);
            double LLR = map(mydata_remote.LLR,0,1023,-20,20);   

            // send the values to the function        
            leg(RT, RFB, RLR, LT, LFB, LLR, 0, 0);                     // left back      
            leg(RT, RFB, RLR, LT, LFB, LLR, 0, 1);                     // left front  
            leg(RT, RFB, RLR, LT, LFB, LLR, 1, 0);                     // right back      
            leg(RT, RFB, RLR, LT, LFB, LLR, 1, 1);                     // right front        
        }
        
          
    } // end of 10ms cycle
    

 } // end of main loop


             

