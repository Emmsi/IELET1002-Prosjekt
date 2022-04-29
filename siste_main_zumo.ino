#include <Wire.h>
#include <Zumo32U4.h>
Zumo32U4OLED display;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;

#include "TurnSensor.h"
//lineSensorValues
#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

//globalTimeTicker()
unsigned long lastTimeForGlobalTimeTicker;
const int timePerLoop = 1;

//waitForInputFromESP32()
char inputFromESP32;
bool readyToCharge;


//switch case:
int state;
//states in switch case
#define ToTrack1 1
#define InsideTrack1 2
#define FindLine 3
#define HomeTrack 4
#define ExitTrack1 5
#define waitForNewCommand 6
#define ToTrack2 7
#define InsideTrack2 8
#define ExitTrack2 9

//flags for switch case

bool readyToTurn = false;
bool track1 = false;
bool track2 = false;
bool objectSeen = false;

//lineFollower()
int16_t lastError = 0;
const uint16_t maxSpeed = 400;
float Kp;
float Td;

//turn_around()
#define right -90
#define left 90
#define fullTurn 180

//track1_executotion()
int leftTurnSpeed = 200;
int rightTurnSpeed = 130;
int track1_counter;
bool track1Executor = false;

//exit_track1()
int proximitytreshold = 3;

//track2_execution
int turnSpeed = 150;
int straightSpeed = 150;
int action;
bool goingRight;
bool initialValues;
int countForWaiting;
int countForExit;
#define straight 1
#define turn 2
#define inBetweenTurns 3
#define slowAcellerationError 4
unsigned long encoderLeftForTrack2;
unsigned long encoderRightForTrack2;
int correctionSpeedRight = 150;
int correctionSpeedLeft = 150;
bool turning;
int differance;

//speedometer
// 909.7 counts per revolution, wheel dcircumference = 122.52211349mm
//millimetersPerCount = 122.52211349/909.7 = 0.1346840865
const float millimetersPerCount = 0.1346840865;
unsigned long lastRightCount;
unsigned long lastLeftCount;
int maxSpeedInCm = 65; // found in datasheet for Zumo.
int averageSpeed;
unsigned long totalTime;
unsigned long totalEncoder;
unsigned long totalDistance;
int speedInCmPerSec;

//findLine
int countForFindLine;

//HomeTrack
int countForHomeTrack;

//updateTimeOverSeventy
int timeOverSeventy;

//battery
int fullBattery = 8800;
int newBattery = 100;
float currentLevel = 8800;

void setup() {
  turnSensorSetup();
  delay(1000);
  turnSensorReset();
  zumo32U4Setup();
}

void loop() {
  waitForInputFromESP32();
  trackingValues();
  globalTimeTicker();
  batteryLevel();
  speedometer();
  Serial.print(batteryLevel());
  Serial.print(speedometer());

  switch (state) {
    case waitForNewCommand:
    waitForInputFromESP32()
    if (readyToCharge){                             
      chargingBattery();
    }
    else;
      break;
    case ToTrack1:
      lineFollower();
      if (t_intersection()) {       // bytt tilbake til t_intersection()
        turnAround(left-10);
        track1Executor = true;
        state = InsideTrack1;
      }
      break;

    case ToTrack2:
      lineFollower();
      if (t_intersection()) {
        motors.setSpeeds(150,150);
        state = InsideTrack2;
        setValuesForTrack2();
      }
      break;

    case InsideTrack1:
      if (track1Executor) {
        track1_execution();
      }
      else {
        state = ExitTrack1;
      }
      break;

    case InsideTrack2:
    if (countForExit > 8){
        motors.setSpeeds(150,150);
        state = FindLine;
        countForExit = 0;
        resetWaiting();
       }
       else{
       turntrack2();
       }
      break;

    case ExitTrack1:
      exit_track1(proximitytreshold);
      if (objectSeen){
    turnAround(right);
    motors.setSpeeds(200,200);
    state = FindLine;
  }
      break;

    case FindLine:
      if (waiting(500)){
        //do nothing
      }
      else{
        if (find_line()) {
          if (track1) {
            turnAround(right);
          }
          else if (track2) {
            turnAround(left);
          }
          state = HomeTrack;
          Kp = 3;
          Td = 6;
        }
      }
      break;

    case HomeTrack:
    resetWaiting();
      lineFollower();
      if (t_intersection()) {
        turnAround(180);
        state = waitForNewCommand;
        sendReportForLap();
        track1 = false;
        track2 = false;
      }
      break;

  }
}
//calibrating lightSensors for linefollower
void calibrateLineSensors() {
  delay(1000);
  motors.setSpeeds(100, -100);
  for (int i = 0; i < 180; i++) {
    lineSensors.calibrate();
    delay(10);
  }
  motors.setSpeeds(0, 0);
}

void zumo32U4Setup() {
  delay(500);
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();
  calibrateLineSensors();
  Serial.begin(9600);
  Serial1.begin(9600);
  state = waitForNewCommand;
}

//period defines millis delay for each iteration through void loop
void globalTimeTicker() {
  while (millis() < lastTimeForGlobalTimeTicker + timePerLoop); //do nothing
  lastTimeForGlobalTimeTicker = millis();
}

void waitForInputFromESP32() {
  inputFromESP32 = Serial1.read();
  if (inputFromESP32 == 'a'){//initiate track1
    state = ToTrack1;
    track1 = true;
    turnAround(left);
    Kp = 4;
    Td = 6;
    SetAndresetValuesForNewLap();
  }
  else if ((inputFromESP32 =='b')) { 
    track2 = true;
    state = ToTrack2;
    resetWaiting();
    turnAround(right);
    action = straight;
    Kp = 3;
    Td = 6;
    SetAndresetValuesForNewLap();
  }
  else if (inputFromESP32 == 'c') { //initiate charging
    readyToCharge = true;
  }
  else;
}

void lineFollower() {
  // Les linjefølgersensor
  int16_t position = lineSensors.readLine(lineSensorValues);

  // Finn avvik, og lagre i en ny variabel kalt error
  int16_t error = position - 2000;

  // Opprett en ny variabel, med et navn som «speed_difference»
  int16_t speedDifference = (error * Kp) + Td * (error - lastError);
  lastError = error;

  // Opprett to nye variabler, kalt noe som left_speed og right_speed.
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  // constrain(x, bunnverdi, toppverdi), tvinger verdien til å være 0-400
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

  // Setter motorhastigheter
  motors.setSpeeds(leftSpeed, rightSpeed);
}

void turnAround(int deg) {
  motors.setSpeeds(0, 0);
  int left_counts = encoders.getCountsLeft();
  int right_counts = encoders.getCountsRight();
  if (deg < 0) {
    while (encoders.getCountsLeft() < left_counts + deg * -7.5) {
      motors.setSpeeds(300, -300);
    }
  }
  if (deg > 0) {
    while (encoders.getCountsRight() < right_counts + deg * 7.5) {
      motors.setSpeeds(-300, 300);
    }
  }
  motors.setSpeeds(0, 0);
}

bool t_intersection() {
  lineSensors.read(lineSensorValues);
  if (lineSensorValues[0] && lineSensorValues[4] > 1500) return true;
  else return false;
}
bool find_line() {
  lineSensors.read(lineSensorValues);
  if (lineSensorValues[1] && lineSensorValues[3] > 1500) return true;
  else return false;
}

void track1_execution() {
  if (rightTurnSpeed == -100) {
    track1Executor = false;
  }
  if (waiting(3500)) {
    motors.setSpeeds(leftTurnSpeed, rightTurnSpeed);
  }
  else {
    resetWaiting();
    leftTurnSpeed -= 5;
    rightTurnSpeed -= 10;
    if (rightTurnSpeed < 40) {
      rightTurnSpeed = -100;
    }
  }
}

void exit_track1(int threshold) {
  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
  if (leftValue == rightValue && leftValue > threshold) {
    objectSeen = true;
  }
}

int speedometer() {
  unsigned long rightCount = encoders.getCountsRight();
  unsigned long leftCount = encoders.getCountsLeft();
  int motorCounts = ((rightCount - lastRightCount) + (leftCount - lastLeftCount)) / 2;
  int SpeedInCmPerSec = motorCounts * millimetersPerCount * 10;
  lastRightCount = rightCount;
  lastLeftCount = leftCount;
  return SpeedInCmPerSec;
}

void updateTimeOverSeventy(){
  if (speedInCmPerSec > maxSpeedInCm*0.7){
    timeOverSeventy += timePerLoop/1000; // in seconds
  }
}

void totalencoder(){
  totalEncoder += ((encoders.getCountsRight() + encoders.getCountsLeft())/2);
}

void totalDistanceUpdate(){
  totalDistance = totalEncoder * millimetersPerCount; //in mm
}

void averageSpeedFunction(){
  totalTime += timePerLoop;
  averageSpeed = totalDistance/totalTime;
  }

void sendReportForLap(){
  Serial.print(averageSpeed);
  Serial.print(totalDistance);
  Serial.print(timeOverSeventy);
}

void SetAndresetValuesForNewLap(){
  totalTime = 0;
  averageSpeed = 0;
  totalDistance = 0;
  totalEncoder = 0;
  timeOverSeventy = 0;
}

void trackingValues(){
  totalencoder();
  averageSpeedFunction();
  totalDistanceUpdate();
  updateTimeOverSeventy();
}

int batteryLevel(){
  if (speedometer() > 65*0.7){
    currentLevel = constrain(currentLevel, 0, fullBattery); 
    (currentLevel -= 0.0005);
  }
  else if (speedometer() == 0)
  {
    currentLevel = currentLevel;
  }
  else{
    currentLevel -= 0.0002;
  }
  return currentLevel;
}

void chargingBattery(){
  fullBattery -=10;
  currentLevel = fullBattery;
  readyToCharge = false;
}

void turntrack2() {
  switch (action) {

    case straight:
      if (waiting(2000)) {
        driveStraight();
      }
      else {
        resetWaiting();
        action = turn;
        goingRight = !goingRight;
        turning = !turning;
        initialValues = false;
      }
      break;

    case turn: //move in turn
    countForExit ++;
      if (goingRight) {
        turnRight();
      }
      else {
        turnLeft();
      }
      initialValues = false;
      action = slowAcellerationError;
      break;

    case inBetweenTurns:
      if (waiting(500)) {
        driveStraight();
      }
      else {
        resetWaiting();
        action = turn;
        turning = !turning;
       initialValues = false;
      }
      break;
    case slowAcellerationError:
      stopDriving();
      if (turning) {
        action = inBetweenTurns;
       initialValues = false;
      }
      else {
        action = straight;
       initialValues = false;
      }
      break;
  }
}


void driveStraight() {
  if (!initialValues) {
    encoderLeftForTrack2 = encoders.getCountsLeft();
    encoderRightForTrack2 = encoders.getCountsRight();
    differance = encoderLeftForTrack2 - encoderRightForTrack2;
    initialValues = true;
  }
  else {

    if ((encoders.getCountsLeft() - encoders.getCountsRight()) > differance) {
      correctionSpeedRight += 5;
      correctionSpeedLeft -= 5;
    }
    else if ((encoders.getCountsLeft() - encoders.getCountsRight()) < differance) {
      correctionSpeedRight -= 5;
      correctionSpeedLeft += 5;
    }
    correctionSpeedLeft = constrain(correctionSpeedLeft, (straightSpeed - 40), (straightSpeed + 40));
    correctionSpeedRight = constrain(correctionSpeedRight, (straightSpeed - 40), (straightSpeed + 40));
    motors.setSpeeds(correctionSpeedLeft, correctionSpeedRight);
  }
}

void turnRight() {
  motors.setSpeeds(-turnSpeed, turnSpeed);
  turnSensorReset();
  while ((int32_t)turnAngle < 1.8 * turnAngle45)
  {
    turnSensorUpdate();
  }
}

void turnLeft() {
  motors.setSpeeds(turnSpeed, -turnSpeed);
  turnSensorReset();
  while ((int32_t)turnAngle > 1.8 * -turnAngle45) // - turnAngle1 * 20)
  {
    turnSensorUpdate();
  }
}


bool waiting(int milliseconds) {
  if (countForWaiting > milliseconds) {
    return false;
  }
  else {
    countForWaiting += timePerLoop;
    return true;
  }
}

void resetWaiting() {
  countForWaiting = 0;
}
void stopDriving() {
  if (waiting(100)) {
    motors.setSpeeds(0, 0);
  }
  else {
    resetWaiting();
  }
}

void setValuesForTrack2(){
  goingRight = false;
  initialValues = false;
  turning = false;
  action = straight;
}
