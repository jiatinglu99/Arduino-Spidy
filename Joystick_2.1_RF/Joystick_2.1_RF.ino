#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RF24.h"
#include "nRF24L01.h"
#include "printf.h"

RF24 radio(7, 8);

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

struct infoData{
  unsigned long _micros;// save response times
  int command;
  int joy1x, joy1y, joy2x, joy2y;
  int leg, part, potChange;
  bool joy1Button, joy2Button;
} myData;

int displayMode = 0;
int moveMode = 0;
int pButton1 = 0;
int pButton2 = 0;
int pButton3 = 0;
int pButton4 = 0;
int pJoyB1 = 0, pJoyB2 = 0;
int connectionRetrial = 0, timeoutCount = 0;;
int potent, potSave, potChange = 0, leg = 1, part = 1;
int csor = 1;
bool up = false, quit = false, down = false, enter = false, lpSwitch = true, joyLeft = false, joyRight = false;
bool buttonTesting = false;
bool Connected = false;
bool Sit = false, Walk = false;

const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
typedef enum { role_ping_out = 1, role_pong_back } role_e;
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};
role_e role = role_ping_out;

void setup() {
  Serial.begin(57600);
  printf_begin();
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  
  //display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(2000);
  display.clearDisplay();
  displayMode = 0;
  displayCurrent();

  //radio
  radio.begin();
  radio.setChannel(108);
  role = role_ping_out;
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setRetries(15,15);
  radio.startListening();
  radio.printDetails();
}

void loop() {
  //SerialTest();
  buttons();
  displayCurrent();
  radioLoop();
}

void radioLoop(){//only pingout
    radio.stopListening();                                    // First, stop listening so we can talk.
  /*********************( Read the Joystick positions )*************************/
  myData.command = moveMode;
  myData.joy1x = analogRead(0);
  myData.joy1y = analogRead(1);
  myData.joy1Button = !digitalRead(10);
  myData.joy2x = analogRead(2);
  myData.joy2y = analogRead(3);
  myData.joy2Button = !digitalRead(9);
  myData.leg = leg;
  myData.part = part;
  myData.potChange = potChange;
  myData._micros = micros();  // Send back for timing

  //printf("Now sending %lu...",myData._micros);
  bool ok = radio.write( &myData, sizeof(myData) );
    
//  if (ok)
//    printf("ok...");
//  else
//    printf("failed.\n\r");

  // Now, continue listening
  radio.startListening();

  // Wait here until we get a response, or timeout (250ms)
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout )
    if (millis() - started_waiting_at > 200 )
      timeout = true;

  if ( timeout )
  {
    //printf("Failed, response timed out.\n\r");
    //delay(200);
    timeoutCount++;
    if (timeoutCount > 3)Connected = false;
  }
  else
  {
    timeoutCount = 0;
    radio.read( &myData, sizeof(myData) );
    //printf("Got response %lu, round-trip delay: %lu\n\r",myData._micros,millis()-myData._micros);
    Connected = true;
  }
  delay(40);
}
void serialReceive() {
  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    display.println(c);
  }
}

void connectionDisplay(){
  display.setCursor(0,56);
  display.setTextSize(1);
  display.setTextColor(BLACK,WHITE);
  display.print("Status:");
  if (Connected) display.print("Connected");
  else display.print("Not Connected");
}

void displayCurrent(){
  switch (displayMode) {
    case 0:{
      moveMode = 0;
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.println("  WELCOME");
      display.setTextSize(1);

      display.print(" Local Test Mode");
      if (csor == 1) display.println("<-");
      else display.println();
      display.print(" Walking Algorithms");
      if (csor == 2) display.println("<-");
      else display.println();
      display.print(" Calibration Test");
      if (csor == 3) display.println("<-");
      else display.println();
      display.print(" Simultaneous Test");
      if (csor == 4) display.println("<-");
      else display.println();

      connectionDisplay();

      display.display();
      display.clearDisplay();
      if (enter) {
        displayMode = csor; //abbreviate for cursor
        if (displayMode == 2) csor = 1;//reset cursor
        if (displayMode == 3) {
          leg = 1;
          part = 1;
          potSave = map(analogRead(6), 0, 1023, 0, 180);
        }
        if (displayMode == 4) {
          csor = 1;
          leg = 1;
          part = 1;
          potSave = map(analogRead(6), 0, 1023, 0, 180);
        }
      }
      if (up) csor--;
      if (down) csor++;
      if (csor < 1) csor += 5;
      if (csor > 5) csor -= 5;
      break;
    }
    case 1:{//Local Test Mode
      display.setCursor(0,0);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      displayTest();
      display.display();
      display.clearDisplay();
      if (quit) displayMode = 0;
      break;
    }
    case 2:{//Walking Algorithms Select
      display.setCursor(0,0);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      
      display.display();
      display.clearDisplay();
      if (quit) displayMode = 0;
      break;
    }
    case 3:{//Calibration
      moveMode = 10;
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      
      display.print("Leg:  ");
      display.print(leg);
      if (lpSwitch) display.println(" <-");
      else display.println();
      
      display.print("Part: ");
      display.print(part);
      if (!lpSwitch) display.println(" <-");
      else display.println();
      
      potChange = map(analogRead(6), 0, 1023, 0, 180) - potSave;//For Now temporarily not the change
      display.print("Change:");
      display.println(potChange);

      if (down || up) lpSwitch = !lpSwitch;
      if (lpSwitch && enter){
        leg++;
        if (leg > 8) leg = 1;
        potSave = map(analogRead(6), 0, 1023, 0, 180);
      }
      else if (!lpSwitch && enter){
        part++;
        if (part > 3) part = 1;
        potSave = map(analogRead(6), 0, 1023, 0, 180);
      }

      display.display();
      display.clearDisplay();
      if (quit) {
        displayMode = 0;
        leg = 1;
        part = 1;
      }
      break;
    }
    case 4:{
      moveMode = 9;
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(WHITE);
      
      display.print("Part: ");
      display.print(part);
      display.println(" <-");
      
      potChange = map(analogRead(6), 0, 1023, 0, 180) - potSave;//For Now temporarily not the change
      display.print("Change:");
      display.println(potChange);

      connectionDisplay();

      if (enter){
        part++;
        if (part > 3) part = 1;
        potSave = map(analogRead(6), 0, 1023, 0, 180);
      }

      display.display();
      display.clearDisplay();
      if (quit) {
        displayMode = 0;
        moveMode = 0;
        leg = 1;
        part = 1;
      }
    }
    default:
    break;
  }
  if (joyLeft) Sit = !Sit;
  if (Sit) moveMode = 1;
  if (Sit && joyRight) Walk = !Walk;
  if (Walk) moveMode = 2;
}

void displayTest(){
  int x1 = SerialRound(analogRead(0), 1);
  display.print("x1: ");
  display.print(x1);
  display.print("    ");
  int y1 = SerialRound(analogRead(1), 0);
  display.print("y1: ");
  display.println(y1);
  int x2 = SerialRound(analogRead(2), 1);
  display.print("x2: ");
  display.print(x2);
  display.print("    ");
  int y2 = SerialRound(analogRead(3), 0);
  display.print("y2: ");
  display.println(y2);
  
  potent = map(analogRead(6), 0, 1023, 0, 180);
  display.print("Potentiometer: ");
  display.println(potent);
  
  display.print("Up: ");
  display.print(digitalRead(6));
  display.print("  ");
  display.print("Quit: ");
  display.print(digitalRead(2));
  display.print("  ");
  display.print("Down: ");
  display.print(digitalRead(3));
  display.print("  ");
  display.print("Enter: ");
  display.print(digitalRead(5));
  display.println("  ");


  display.print("joy1: ");
  display.print(!digitalRead(10));
  display.print("  ");
  display.print("joy2: ");
  display.print(!digitalRead(9));
  //delay(100);
}

void buttons() {
  int button1 = digitalRead(6);
  int button2 = digitalRead(2);
  int button3 = digitalRead(3);
  int button4 = digitalRead(5);
  int joyB1 = !digitalRead(10);
  int joyB2 = !digitalRead(9);

  if (pButton1 == 0 && button1 == 1) up = true;
  else up = false;
  if (pButton2 == 0 && button2 == 1) quit = true;
  else quit = false;
  if (pButton3 == 0 && button3 == 1) down = true;
  else down = false;
  if (pButton4 == 0 && button4 == 1) enter = true;
  else enter = false;
  if (pJoyB1 == 0 && joyB1 == 1) joyLeft = true;
  else joyLeft = false;
  if (pJoyB2 == 0 && joyB2 == 1) joyRight = true;
  else joyRight = false;
  
  pButton1 = button1;
  pButton2 = button2;
  pButton3 = button3;
  pButton4 = button4;
  pJoyB1 = joyB1;
  pJoyB2 = joyB2;
}

void SerialTest() {
  int x1 = SerialRound(analogRead(0),1);
  Serial.print("x1:");
  Serial.print(x1);
  Serial.print(" ");
  int y1 = SerialRound(analogRead(1), 0);
  Serial.print("y1: ");
  Serial.print(y1);
  Serial.print(" ");
  int x2 = SerialRound(analogRead(2), 1);
  Serial.print("x2: ");
  Serial.print(x2);
  Serial.print(" ");
  int y2 = SerialRound(analogRead(3), 0);
  Serial.print("y2: ");
  Serial.print(y2);
  Serial.print(" ");
  Serial.print("b1: ");
  Serial.print(digitalRead(6));
  Serial.print(" ");
  Serial.print("b2: ");
  Serial.print(digitalRead(2));
  Serial.print(" ");
  Serial.print("b3: ");
  Serial.print(digitalRead(3));
  Serial.print(" ");
  Serial.print("b4: ");
  Serial.print(digitalRead(5));
  Serial.print(" ");
  int potent = map(analogRead(6), 0, 1023, 0, 179);
  Serial.print("Potent: ");
  Serial.println(potent);
//  for (int i = 0; i <12; i++) Serial.print(digitalRead(i));
//  Serial.println();
  //delay(500);
}

int SerialRound(int input, int type){
  int answer;
  if (input < 100) answer = 0;
  else if (input < 200) answer = 1;
  else if (input < 300) answer = 2;
  else if (input < 400) answer = 3;
  else if (input < 550) answer = 4;
  else if (input < 600) answer = 5;
  else if (input < 700) answer = 6;
  else if (input < 800) answer = 7;
  else answer = 8;
  if (type == 0) answer = 4 - answer;
  else answer = answer - 4;
  return answer;
}


