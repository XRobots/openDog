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

            //********** print data from sticks *********

            mydata_back.shoulderR = map(mydata_remote.RT,0,1023,-50,50);     // test send data scaled to milimeters
            mydata_front.shoulderR = map(mydata_remote.RT,0,1023,-50,50);
            mydata_back.shoulderL = map(mydata_remote.RT,0,1023,-50,50);     // test send data scaled to milimeters
            mydata_front.shoulderL = map(mydata_remote.RT,0,1023,-50,50);
            
            ET3.sendData();
            ET4.sendData();

       }  // end of timed event

}

