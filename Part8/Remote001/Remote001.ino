#include <EasyTransfer.h>

//create object
EasyTransfer ET1;   // send serial
EasyTransfer ET2;   // rec serial

#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address (addr, en, rw, rs, d4, d5, d6, d7, backlight, polarity)

bool but1;
bool but2;
bool but3;
bool but4;
bool but5;

int axis1;
int axis2;
int axis3;
int axis4;
int axis5;
int axis6;

String count;

struct SEND_DATA_STRUCTURE{
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

struct RECEIVE_DATA_STRUCTURE_REMOTE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE_REMOTE mydata_remote;

int state; // BT state

unsigned long previousMillis = 0;
const long interval = 40;

unsigned long previousDispMillis = 0;
const long Dispinterval = 10;

void setup() {

  lcd.begin(20,4);   // Initialize the lcd for 20 chars 4 lines, turn on backlight

  Serial.begin(57600);
  Serial2.begin(57600);

  ET1.begin(details(mydata_send), &Serial2);
  ET2.begin(details(mydata_remote), &Serial2);

  // NOTE: Cursor Position: (CHAR, LINE) starts at 0  
  lcd.setCursor(0,0);
  lcd.print("openDog Remote        ");
  lcd.setCursor(0,1);
  lcd.print("XRobots.co.uk       ");
  
  pinMode(44, INPUT_PULLUP);
  pinMode(46, INPUT_PULLUP);
  pinMode(48, INPUT_PULLUP);
  pinMode(50, INPUT_PULLUP);
  pinMode(52, INPUT_PULLUP);

  


  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);

  pinMode(3, INPUT); // BT state

  digitalWrite(2, LOW); // turn off LED

}

  void pair() {
    state = digitalRead(3);
    while(state == 0) {
        lcd.setCursor(0,2);
        lcd.print("Waiting to Pair BT  ");
        lcd.setCursor(0,3);
        lcd.print("               ");
        state = digitalRead(3);    
    }

    delay(500);  // wait before sending data
}

void loop() {


 unsigned long currentMillis = millis();
 if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
    previousMillis = currentMillis;


// check to see if BT is paired
state = digitalRead(3);
if (state == 0) {
  pair();
}
else {
lcd.setCursor(0,2);
lcd.print("Paired to openDog!  ");
}


but1 =  digitalRead(44);
but2 =  digitalRead(46);
but3 =  digitalRead(48);
but4 =  digitalRead(50);
but5 =  digitalRead(52);

if (but1 == 0) {
  mydata_send.menuDown = 1;
}
else {
  mydata_send.menuDown = 0;
}

if (but2 == 0) {
  mydata_send.Select = 1;
}
else {
  mydata_send.Select = 0;
}

if (but3 == 0) {
  mydata_send.menuUp = 1;
}
else {
  mydata_send.menuUp = 0;
}

if (but4 == 0) {
  mydata_send.toggleBottom = 1;
}
else {
  mydata_send.toggleBottom = 0;
}

if (but5 == 0) {
  mydata_send.toggleTop = 1;
}
else {
  mydata_send.toggleTop = 0;
}

axis1 = analogRead(A0);
axis2 = analogRead(A1);
axis3 = analogRead(A2);
axis4 = analogRead(A3);
axis5 = analogRead(A4);
axis6 = analogRead(A5);

mydata_send.RLR = axis1;
mydata_send.RFB = axis2;
mydata_send.RT = axis3;
mydata_send.LLR = axis4;
mydata_send.LFB = axis5;
mydata_send.LT = axis6;

ET1.sendData();

 } // end of timed event send

if (currentMillis - previousDispMillis >= Dispinterval) {  // start timed event for read and send
  previousDispMillis = currentMillis;  

        if(ET2.receiveData()){
          count = String(mydata_remote.count);
          lcd.setCursor(0,3);
          lcd.print(count);

          if (mydata_remote.mode == 0) {
            lcd.setCursor(0,0);
            lcd.print("Mode 0              ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          else if (mydata_remote.mode == 1) {
            lcd.setCursor(0,0);
            lcd.print("Mode 1              ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          else if (mydata_remote.mode == 2) {
            lcd.setCursor(0,0);
            lcd.print("Mode 2              ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          else if (mydata_remote.mode == 3) {
            lcd.setCursor(0,0);
            lcd.print("Mode 3              ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          else if (mydata_remote.mode == 4) {
            lcd.setCursor(0,0);
            lcd.print("Mode 4              ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          else if (mydata_remote.mode == 5) {
            lcd.setCursor(0,0);
            lcd.print("Mode 5              ");
            lcd.setCursor(0,1);
            lcd.print("                    ");
          }
          

        }

        
}  // end of second timed event






}
