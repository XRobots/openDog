#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

unsigned long previousMillis = 0; // The timestamp at which we last sent a radio packet.
const long interval = 20;

const int BUTTON1_PIN = 38;
const int BUTTON2_PIN = 40;
const int BUTTON3_PIN = 42;
const int BUTTON4_PIN = 44;
const int BUTTON5_PIN = 46;

const int AXIS1_PIN = A0;
const int AXIS2_PIN = A1;
const int AXIS3_PIN = A2;
const int AXIS4_PIN = A3;
const int AXIS5_PIN = A4;
const int AXIS6_PIN = A5;

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
    lcd.init();
    lcd.backlight();

    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    pinMode(BUTTON4_PIN, INPUT_PULLUP);
    pinMode(BUTTON5_PIN, INPUT_PULLUP);

    pinMode(AXIS1_PIN, INPUT);
    pinMode(AXIS2_PIN, INPUT);
    pinMode(AXIS3_PIN, INPUT);
    pinMode(AXIS4_PIN, INPUT);
    pinMode(AXIS5_PIN, INPUT);
    pinMode(AXIS6_PIN, INPUT);

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

        mydata_send.menuDown = !digitalRead(BUTTON1_PIN);
        mydata_send.Select = !digitalRead(BUTTON2_PIN);
        mydata_send.menuUp = !digitalRead(BUTTON3_PIN);
        mydata_send.toggleBottom = !digitalRead(BUTTON4_PIN);
        mydata_send.toggleTop = !digitalRead(BUTTON5_PIN);

        mydata_send.RLR = analogRead(AXIS1_PIN);
        mydata_send.RFB = analogRead(AXIS2_PIN);
        mydata_send.RT = analogRead(AXIS3_PIN);
        mydata_send.LLR = analogRead(AXIS4_PIN);
        mydata_send.LFB = analogRead(AXIS5_PIN);
        mydata_send.LT = analogRead(AXIS6_PIN);

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
