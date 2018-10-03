#include <SPI.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "math.h"

void(* resetFunc) (void) = 0;

RF24 radio(53, 9);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
typedef enum { role_ping_out = 1, role_pong_back } role_e;
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};
role_e role = role_ping_out;
bool hasBeenConnected = false;
unsigned long lastConnection;
int walkStart = 0, radioCount = 0;

struct infoData{
  unsigned long _micros;// save response times
  int command;
  int joy1x, joy1y, joy2x, joy2y;
  int leg, part,potChange;
  bool joy1Button, joy2Button;
} myData;

class Leg{
public:
  Servo servo[3];
  double cal[3];
  bool flip[3];
  double presentAngle[3] = {0, 0, 0};
  double goalAngle[3] = {0, 0, 0};
  double a = 44.85, b = 64.5, d = 12.0, m = 0.0, n = 0.0, f = 0.0;
  int type, front;
  double spd = 0.08;
  double positiveR, negativeR;
  double currentForward;
  bool stepInProcess = false, stepDone = false;
  Leg ();
  void Init (int pin1, int pin2, int pin3, int cal1, int cal2, int cal3, int def1, int def2, int def3, int ttype, int tfront) {
    servo[0].attach(pin1);
    servo[1].attach(pin2);
    servo[2].attach(pin3);
    if (def1 < 0) flip[0] = true;
    else flip[0] = false;
    if (def2 < 0) flip[1] = true;
    else flip[1] = false;
    if (def3 < 0) flip[2] = true;
    else flip[2] = false;
    cal[0] = cal1;
    cal[1] = cal2;
    cal[2] = cal3 - 28;
    backToDefault();
    servoWrite();
    type = ttype;//0 Normal,1 Corner 
    front = tfront;//0 Back, 1 Front
  }
  void rangeInit(double tpositive,double tnegative)
  {
    positiveR = tpositive;
    negativeR = tnegative;
  }
  void backToDefault(){
    servoGoal(1, 0);
    servoGoal(2, 0);
    servoGoal(3, 40);
  }
  void servoGoal(int part, double degree){
    goalAngle[part - 1] = degree;
  }
  void servoWrite(){
    angleProcess();
    for (int i = 0; i < 3; i++) {
      if (flip[i]) servo[i].write(180 - cal[i] - presentAngle[i]);
      else servo[i].write(cal[i] + presentAngle[i]);
    }
  }
  void angleProcess(){
    spd = 0.15;
    for (int i = 0; i < 3; i++)
      if (presentAngle[i] != goalAngle[i]) presentAngle[i] = lerp(presentAngle[i], goalAngle[i], spd);
  }
  double lerp(double start, double ending, double amt){
    double answer = start + (ending - start) * 0.4;
    return answer;
  }
  void setPosition(double tf, double tm, double tn){
    currentForward = tf;
    f = tf;
    m = sqrt(square(d+tm)+square(f))-d;
    n = tn;
    double x = rtdegree(atan2(f, d+tm));
    double y = -rtdegree(M_PI - atan(m/n)-acos((square(a)-square(b)+square(m)+square(n))/(2.0*a*sqrt(square(m)+square(n)))));
    double z = rtdegree(acos((square(a)+square(b)-square(m)-square(n))/(2.0*a*b)));
    servoGoal(1, x);
    servoGoal(2, y);
    servoGoal(3, z);
  }
  double rtdegree(double radian){
    return radian/M_PI*180.0;
  }
  void Walk(double tempd, double side, double height){
    tempd*=4;
    if (tempd <= 0.01 && tempd >= -0.01) tempd = 0;
    double Direction = 0.3 * tempd;
    if (!stepInProcess){
      stepDone = false;
      if (currentForward + Direction < positiveR && currentForward + Direction > negativeR) {
        currentForward += Direction;
        setPosition(currentForward, side, height);
      }
      //else Step(-1);
    }
    else {
      double temp = 2.1;
      if (currentForward - temp*tempd < positiveR && currentForward - temp*tempd > negativeR) {
        currentForward -= temp*tempd;
        int th = -40;//temporary height
        if (tempd <= 0.02 && tempd >= -0.02) th = 0;
        //Serial.println(tempd);
        setPosition(currentForward, side, height + th);
      }
      else {
        if (Direction < 0) currentForward = positiveR;
        else currentForward = negativeR;
        stepDone = true;
        stepInProcess = false;
        setPosition(currentForward, side, height);
      }
    }
  }
  void Step(int Direction){
    stepInProcess = true;
    stepDone = false;
  }
  bool StepFinished(){
    return stepDone;
  }
};

Leg::Leg (){}
Leg legs[8];


void setup(void)
{  
  //noInterrupts();
  legs[0].Init(18, 19, 20, 64, 178, -16, -1, -1, 1, 1, 1);
  legs[1].Init(14, 15, 16, 66, 154, -15, -1, -1, 1, 0, 1);
  legs[2].Init(26, 27, 28, 130, 160, -5, -1, 1, -1, 0, 0);
  legs[3].Init(22, 23, 24, 152, 150, -5, -1, 1, -1, 1, 0);
  legs[4].Init(42, 43, 45, 49, 170, -16, 1, 1, -1, 1, 1);
  legs[5].Init(38, 39, 40, 96, 160, -8, 1, 1, -1, 0, 1);
  legs[6].Init(34, 35, 36, 125, 174, 6, 1, -1, 1, 0, 0);
  legs[7].Init(30, 31, 32, 130, 162, -6, 1, -1, 1, 1, 0);
  Serial.begin(115200);
  //Serial2.begin(9600);
  printf_begin();
  radio.begin();
  
  radio.setRetries(15,15);

  role = role_ping_out;
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.startListening();
  radio.printDetails();
  //delay(1000);
  legs[0].rangeInit(75, 25);
  legs[1].rangeInit(50, -10);
  legs[2].rangeInit(10, -50);
  legs[3].rangeInit(-25, -75);
  legs[4].rangeInit(75, 25);
  legs[5].rangeInit(50, -10);
  legs[6].rangeInit(10, -50);
  legs[7].rangeInit(-25, -75);
}

void loop(void){
  if (radioCount == 0) radioLoop();
  radioCount++;
  if (radioCount >= 5) radioCount = 0;
  processData();
}

void processData(){
  switch (myData.command){
    case 0:{
      for(int i = 0; i < 8; i++) legs[i].backToDefault();
      break;
    }
    case 1:{//Standup
      double forward = (map(myData.joy2y - 525, -525, 498, -40, 40) - 1)/2;
      double height = (-map(myData.joy1y - 487, -487, 536, -60, 0) - 15)/2+ 20;
      double side = (map(myData.joy2x - 487, -487, 536, -30, 30) + 2)/2;
      double tilt = (map(myData.joy1x - 525, -525, 498, -5, 20) - 8);
      //Serial.println(height);
      legs[0].setPosition(forward+30, 45+side, 30+height+tilt);//normal height = 47
      legs[1].setPosition(forward+10, 45+side, 30+height+tilt);
      legs[2].setPosition(forward-10, 45+side, 30+height+tilt);
      legs[3].setPosition(forward-30, 45+side, 30+height+tilt);
      legs[4].setPosition(forward+30, 45-side, 30+height-tilt);
      legs[5].setPosition(forward+10, 45-side, 30+height-tilt);
      legs[6].setPosition(forward-10, 45-side, 30+height-tilt);
      legs[7].setPosition(forward-30, 45-side, 30+height-tilt);
      walkStart = 0;
      break;
    }
    case 2:{//Walk Basic
      int height = 47;
      int side = 35;
      double jspd = (myData.joy2y - 525.0)/600;
      double sped = -jspd;
      double turn = (myData.joy1x - 533.0)/1000;
      if (sped <= 0.01 && sped >= -0.01) sped = 0;
      if (turn <= 0.01 && turn >= -0.01) turn = 0;
      //Serial.println(jspd);
      legs[0].Walk(-sped + turn, 20+side-15, height+5);
      legs[1].Walk(-sped + turn, 20+side-5, height+18);
      legs[2].Walk(-sped + turn, 20+side+0, height+15);
      legs[3].Walk(-sped + turn, 20+side-15, height+5);
      legs[4].Walk(-sped - turn, 20+side-15, height+15);
      legs[5].Walk(-sped - turn, 20+side+0, height+5);
      legs[6].Walk(-sped - turn, 20+side+0, height+5);
      legs[7].Walk(-sped - turn, 20+side-15, height+8);
      Serial.println(turn);
      if (walkStart == 0) {
        legs[3].Step(-1);
        //legs[6].Step(1);
      }
      //Serial.println(walkStart);
      if (walkStart < 1) walkStart++;
      if (sped > 0 || turn > 0){
        if (legs[4].StepFinished())legs[1].Step(-1);
        if (legs[1].StepFinished())legs[6].Step(-1);
        if (legs[6].StepFinished())legs[3].Step(-1);
        if (legs[3].StepFinished())legs[0].Step(-1);
        if (legs[0].StepFinished())legs[5].Step(-1);
        if (legs[5].StepFinished())legs[2].Step(-1);
        if (legs[2].StepFinished())legs[7].Step(-1);
        if (legs[7].StepFinished())legs[4].Step(-1);
      }
      else if (sped < 0 || turn < 0){
        if (legs[3].StepFinished())legs[6].Step(-1);
        if (legs[6].StepFinished())legs[1].Step(-1);
        if (legs[1].StepFinished())legs[4].Step(-1);
        if (legs[4].StepFinished())legs[7].Step(-1);
        if (legs[7].StepFinished())legs[2].Step(-1);
        if (legs[2].StepFinished())legs[5].Step(-1);
        if (legs[5].StepFinished())legs[0].Step(-1);
        if (legs[0].StepFinished())legs[3].Step(-1);
      }
      break;
    }
    case 3:{
      int height = 47;
      int side = 35;
      double jspd = (myData.joy2y - 525.0)/350;
      double sped = -jspd;
      //Serial.println(jspd);
      legs[0].Walk(-sped+SerialRound(myData.joy2x, 0)*10, side-10, height);
      legs[1].Walk(-sped+SerialRound(myData.joy2x, 0)*10, side, height);
      legs[2].Walk(-sped+SerialRound(myData.joy2x, 0)*10, side, height);
      legs[3].Walk(-sped+SerialRound(myData.joy2x, 0)*10, side-10, height);
      legs[4].Walk(-sped-SerialRound(myData.joy2x, 0)*10, side-10, height);
      legs[5].Walk(-sped-SerialRound(myData.joy2x, 0)*10, side, height);
      legs[6].Walk(-sped-SerialRound(myData.joy2x, 0)*10, side, height);
      legs[7].Walk(-sped-SerialRound(myData.joy2x, 0)*10, side-10, height);
      //Serial.println(SerialRound(myData.joy2x, 0));
      if (walkStart == 0) {
        legs[3].Step(-1);
        legs[4].Step(-1);
      }
      //Serial.println(walkStart);
      if (walkStart < 1) walkStart++;
      if (legs[3].StepFinished())legs[5].Step(-1);
      if (legs[5].StepFinished())legs[1].Step(-1);
      if (legs[1].StepFinished())legs[7].Step(-1);
      if (legs[7].StepFinished())legs[3].Step(-1);
      
      if (legs[4].StepFinished())legs[2].Step(-1);
      if (legs[2].StepFinished())legs[6].Step(-1);
      if (legs[6].StepFinished())legs[0].Step(-1);
      if (legs[0].StepFinished())legs[4].Step(-1);
      break;
    }
    case 9:{
      for(int i = 0; i < 8; i++) legs[i].servoGoal(myData.part, myData.potChange);
      break;
    }
    case 10:{
      legs[myData.leg - 1].servoGoal(myData.part, myData.potChange);
      break;
    }
    default:
    break;
  }
  if (myData.command == 0) for (int i = 0; i < 8; i++) legs[i].spd = 0.01;
  else for (int i = 0; i < 8; i++) legs[i].spd = 0.08;
  for(int i = 0; i < 8; i++) legs[i].servoWrite();
}

void radioLoop()
{
  if (role == role_ping_out)//only pingout once
  {
    radio.stopListening();
    unsigned long time = millis();
    //printf("Now sending %lu...",time);
    bool ok = radio.write( &time, sizeof(unsigned long) );
    
//    if (ok)
//      printf("ok...");
//    else
//      printf("failed.\n\r");

    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 200 )
        timeout = true;

    // Describe the results
    if ( timeout )
    {
      //printf("Failed, response timed out.\n\r");
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      unsigned long got_time;
      radio.read( &got_time, sizeof(unsigned long) );
      //printf("Got response %lu, round-trip delay: %lu\n\r",got_time,millis()-got_time);
    }
  }
  if ( role == role_pong_back )
  {
    // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      unsigned long got_time;
      bool done = false;
      while (!done)
      {
        done = radio.read( &myData, sizeof(myData) );

        //printf("Got payload %lu...",myData._micros);
        //Serial.print(myData.joy1x);
	    //delay(30);
      }

      // First, stop listening so we can talk
      radio.stopListening();

      // Send the final one back.
      radio.write( &myData, sizeof(myData) );
      //printf("Sent response.\n\r");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
      hasBeenConnected = true;
      lastConnection = millis();
    }
    else if (hasBeenConnected && millis() - lastConnection > 10000) resetFunc();
  }
  if (role != role_pong_back){
    role = role_pong_back;
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
  }
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
