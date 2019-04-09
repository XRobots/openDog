#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

unsigned long previousMillis = 0;
const long interval = 20;

int but1;
int but2;
int but3;
int but4;
int but5;

int axis1;
int axis2;
int axis3;
int axis4;
int axis5;
int axis6;

String count;

struct SEND_DATA_STRUCTURE{
//struct __attribute__((__packed__)) SEND_DATA_STRUCTURE{
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

struct RECEIVE_DATA_STRUCTURE_REMOTE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

int remoteFlag = 0;

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE_REMOTE mydata_remote;

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};


void setup() {  

  lcd.init();  // initialize the lcd 
  lcd.backlight();

  pinMode(38, INPUT_PULLUP);
  pinMode(40, INPUT_PULLUP);
  pinMode(42, INPUT_PULLUP);
  pinMode(44, INPUT_PULLUP);
  pinMode(46, INPUT_PULLUP);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00001
  radio.openReadingPipe(1, addresses[0]); // 00002
  radio.setPALevel(RF24_PA_MIN);

  lcd.begin(20,4);   // Initialize the lcd for 20 chars 4 lines, turn on backlight

  lcd.setCursor(0,0);
  lcd.print("openDog Remote      ");
  lcd.setCursor(0,1);
  lcd.print("XRobots.co.uk       ");
    
}

void loop() {

     unsigned long currentMillis = millis();
         if (remoteFlag == 0 && currentMillis - previousMillis >= 5) { 

              but1 =  digitalRead(38);
              but2 =  digitalRead(40);
              but3 =  digitalRead(42);
              but4 =  digitalRead(44);
              but5 =  digitalRead(46);                     
              
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
              
              //delay(5);
              //radio.startListening();              
              //radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE_REMOTE));

              count = String(mydata_remote.count);


              /*
              
              lcd.setCursor(0,3);
              lcd.print(count);
    
              if (mydata_remote.mode == 0) {
                lcd.setCursor(0,0);
                lcd.print("Mode 0 - Safe       ");
                lcd.setCursor(0,1);
                lcd.print("                    ");
              }
              else if (mydata_remote.mode == 1) {
                lcd.setCursor(0,0);
                lcd.print("Mode 1 - Kin Test   ");
                lcd.setCursor(0,1);
                lcd.print("                    ");
              }
              else if (mydata_remote.mode == 2) {
                lcd.setCursor(0,0);
                lcd.print("Mode 2 -            ");
                lcd.setCursor(0,1);
                lcd.print("                    ");
              }
              else if (mydata_remote.mode == 3) {
                lcd.setCursor(0,0);
                lcd.print("Mode 3 -            ");
                lcd.setCursor(0,1);
                lcd.print("                    ");
              }
              else if (mydata_remote.mode == 4) {
                lcd.setCursor(0,0);
                lcd.print("Mode 4 -            ");
                lcd.setCursor(0,1);
                lcd.print("                    ");
              }
              else if (mydata_remote.mode == 5) {
                lcd.setCursor(0,0);
                lcd.print("Mode 5 -            ");
                lcd.setCursor(0,1);
                lcd.print("                    ");
              }

              */

              radio.stopListening();
              radio.write(&mydata_send, sizeof(SEND_DATA_STRUCTURE));
                
              previousMillis = currentMillis;  
         }        
        


      



  }  // end of main loop

