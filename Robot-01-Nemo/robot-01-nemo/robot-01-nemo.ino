// ----- LIBRARIES ----- //
// Motor Shield
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
// Servo
#include <Servo.h>
// Bluefruit LE 
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

// ----- GLOBAL VARIABLES ----- //
// --- Motor Shield Related --- //
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Create individual DC motor objects
Adafruit_DCMotor * rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor * leftMotor = AFMS.getMotor(2);
// Create servo object for the servo that rotates the US distance sensor
Servo servoHead;  // create servo object to control a servo
const int servoHeadPin = 10;

// --- Variables to Control the DC Motors' Speed --- //
// need to slow down the left motor so that its speed matches up with the speed of the right motor
// so I'll use the below factor to multiply this variable with the speed of the left motor to slow it down
float speedReductionFactorForward = 0.48; //0.465; // 0.448 0.45 with battery
float speedReductionFactorBackward = 0.48;// 0.465; // 0.45 with battery
int turningRightTime90Degrees = 950; // amount of time (ms) it takes to turn 90 degrees
int turningLeftTime90Degrees = 950; // amount of time (ms) it takes to turn 90 degrees
// --- Variables for Measuring Distance with the Ultrasonic HC SR04 Range Finder --- //
int trigPin = 6;    //Trig
int echoPin = 5;    //Echo
long duration, cm;
// --- Variables to Determine which Way to Go --- //
boolean moveTowardsObjects = true;
int distanceForward = 3000; // distance in ms for the robot to move forward
int delayAfterMoving = 5000; // amount of time to wait before checking distance and moving again
int safeDistance = 60; // if the robot is within this distance from the object, don't move
// --- Variables for Using the Photoresistor --- //
const int numReadings = 10;     // length of the array to store the photoresistor readings
int total = 0;                  // the total of the photoresistor readings
int averageLightLevel = 0;                // the average of the photoresistor readings
int photoresistorPin = A0;
// --- Variables Storing the State and Timings of the Robot's Movements --- //
int robotState = 5; // 1: forward, 2: backward, 3: left, 4: right, 5: stop, 6: leftForward, 7: rightFoward, 8: spinBackward, 9: spinTurn, 10: moveServo
long moveTime = 2000; // the amount of time the robot moves forward/backward
long turnTime = 950/2; // the amount of time the robot turns for 90 degree turn
long spinBackwardTime = 1000; // the amount of time the robot moves backward before spinning
long spinTurnTime = 4000; // the amount of time the robot spins
long pauseTime = 1000; // the amount of time the robot waits before determining which way to move
long pauseServoTime = 500; // the amount of time to wait for the servo to reach its new position
unsigned long previousRobotMovingMillis = 0; // for comparing time intervals
int motorSpeed = 0;
bool letRobotMoveByItself = false;

// --- Bluefruit LE Stuff --- //
/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0 // 1 will reset, 0 won't (important if you made a custom name for the bluetooth chip)
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

// ----- SETUP ----- //
void setup(void) {
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  
  // Other Stuff
  
  //Serial.begin(9600);    // set up Serial library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz (for the motor shield)

  servoHead.attach(10);  // attaches the servo on pin 10 to the servo object
  servoHead.write(90);   // makes sure that the servo and US sensor are facing forward upon startup

  //Define inputs and outputs for the ultrasonic range finder
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Calculate the average ambient light reading using the photoresistor
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    total = total + analogRead(photoresistorPin);
    delay(100);
  }
  // Calculate the average reading from the photoresistor:
  averageLightLevel = total/numReadings;
  constrain(averageLightLevel, 0, 1023);
  Serial.println(averageLightLevel);
    
  Serial.println("Starting...");
}

// Bluefruit LE Stuff 
/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

// ----- LOOP ----- //
void loop(void) {
  if (letRobotMoveByItself == true) {
    updateMotion();
  }
  
  // Read data coming from the Bluefruit LE module
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }

    // ADDITIONS FOR MOTOR SHIELD 
    if (pressed ) {
      if (buttnum == 5) {
        Serial.println(" Forward");
        goForward();
      }
      if (buttnum == 6) {
        Serial.println(" Backward");
        goBackward();
      }
      if (buttnum == 7) {
        Serial.print(" Left");
        turnLeft();
      }
      if (buttnum == 8) {
        Serial.print(" Right");
        turnRight();
      }
      if (buttnum == 1) {
        Serial.print(" Slow down left motor: ");
        Serial.print("Speed Reduction Factor: ");
        speedReductionFactorForward -= 0.01;
        speedReductionFactorBackward -= 0.01;
        Serial.println(speedReductionFactorForward);
      }
      if (buttnum == 2) {
        Serial.print(" Speed up left motor");
        Serial.print("Speed Reduction Factor: ");
        speedReductionFactorForward += 0.01;
        speedReductionFactorBackward += 0.01;
        Serial.println(speedReductionFactorForward);
      }
      if (buttnum == 3) {
        if (letRobotMoveByItself == true) {
          letRobotMoveByItself = false;
          Serial.println(" Stop. Do nothing. "); 
        } else {
          letRobotMoveByItself = true;
          Serial.println(" Move according to distance. ");
        }
        stop();
      } else {
        letRobotMoveByItself = false;
      }
      if (buttnum == 4) {
        Serial.println(" Random movement. ");
        int directionMode = random(1,20);
        if ((directionMode % 4) == 0) {
          Serial.println(" forward.");
          goForward();
        } else if ((directionMode % 4) == 1){
          Serial.println(" backward.");
          goBackward();
        } else if ((directionMode % 4) == 2) {
          Serial.println(" left.");
          turnLeft();
        } else if ((directionMode % 4) == 3){
          Serial.println(" right.");
          turnRight();
        }          
      }
    } else { // button has been released
      Serial.println(" Stop");
      stop();
    }
  } // end of button event

  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer+2);
    lon = parsefloat(packetbuffer+6);
    alt = parsefloat(packetbuffer+10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Accel\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Mag\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
  
  //updateMotion();
}

// --- Functions to Update Motion of the Robot --- //
void updateMotion() {
  unsigned long currentRobotMovingMillis = millis();
  // if the robot was turning left/right,
  // and the amount of time to turn has completed
  // then tell the robot to stop or go forward depending on the turn state
  if ((robotState == 3 || robotState == 4) && (currentRobotMovingMillis - previousRobotMovingMillis >= turnTime)) {
    Serial.println("I turned left/right. I shall stop now.");
    Serial.print("Turn Interval (950): ");
    Serial.println(currentRobotMovingMillis - previousRobotMovingMillis);
    robotState = 5;
    stop();
    previousRobotMovingMillis = currentRobotMovingMillis;
  }
  // if the robot was turning left/right,
  // and the amount of time to turn has completed
  // then tell the robot to stop or go forward depending on the turn state
  else if ((robotState == 6 || robotState == 7) && (currentRobotMovingMillis - previousRobotMovingMillis >= turnTime)) {
    Serial.println("I turned left/right. I shall move forward now.");
    Serial.print("Turn Interval (950): ");
    Serial.println(currentRobotMovingMillis - previousRobotMovingMillis);
    robotState = 1;
    goForward();
    previousRobotMovingMillis = currentRobotMovingMillis;
  }
  // else if the robot was on pause mode (not moving), 
  // and the amount of time to wait before taking its next action has passed,
  // then tell the robot to determine the next movement direction
  else if ((robotState == 5) && (currentRobotMovingMillis - previousRobotMovingMillis >= pauseTime)) {
    Serial.println("I shall think about which way to move.");
    Serial.print("Pause Interval (5000): ");
    Serial.println(currentRobotMovingMillis - previousRobotMovingMillis);
    determineMotion();  
    previousRobotMovingMillis = currentRobotMovingMillis;
  }
  // else if the robot was moving forward or backward,
  // and the amount of time to move has completed,
  // then tell the robot to stop 
  else if ((robotState == 1 || robotState == 2) && (currentRobotMovingMillis - previousRobotMovingMillis >= moveTime)) {
    Serial.println("I moved a lot forward/backward. I shall stop now.");
    Serial.print("Move Interval (2000): ");
    Serial.println(currentRobotMovingMillis - previousRobotMovingMillis);
    robotState = 5;
    stop();
    previousRobotMovingMillis = currentRobotMovingMillis;
  }
  // else if the robot was moving backward before needing to spin,
  // and the amount of time to move has completed,
  // then tell the robot to stop 
  else if ((robotState == 8) && (currentRobotMovingMillis - previousRobotMovingMillis >= (spinBackwardTime))) {
    Serial.println("I moved a lot backward. I shall spin now.");
    Serial.print("Spin Backward Interval (1000): ");
    Serial.println(currentRobotMovingMillis - previousRobotMovingMillis);
    //robotState = random(3,5);
    //Serial.println(robotState);
    robotState = 9;
    turnRight(); 
    previousRobotMovingMillis = currentRobotMovingMillis;
  }
  // else if the robot was spinning,
  // and the amount of time to turn has completed
  // then tell the robot to stop 
  else if ((robotState == 9) && (currentRobotMovingMillis - previousRobotMovingMillis >= spinTurnTime)) {
    Serial.println("I turned left/right. I shall stop now.");
    Serial.print("Spin Turn Interval (4000): ");
    Serial.println(currentRobotMovingMillis - previousRobotMovingMillis);
    robotState = 5;
    stop();
    previousRobotMovingMillis = currentRobotMovingMillis;
  }
  /*// else if the robot was changing the servo head position,
  // and the amount of time for the servo to reach the new position has completed
  // then tell the robot to do something...
  else if ((robotState == 10) && (currentRobotMovingMillis - previousRobotMovingMillis >= pauseServoTime)) {
    Serial.println("Confused!!!!");
    previousRobotMovingMillis = currentRobotMovingMillis;
  }*/
}

// --- Functions to Check Distance --- //
int checkDistanceAtServoAngle(int servoAngle) {
  // move the servo to the desired angle
  servoHead.write(servoAngle);
  delay(250);  // wait for the servo to reach the new position
  // Check the distance at this angle  
  // The ultrasonic range finder sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5); //one millionith of a second
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echoPin, HIGH);
 
  // convert the time into a distance
  cm = (duration/2) / 29.1;

  Serial.print("Distance at ");
  Serial.print(servoAngle);
  Serial.print(" deg is: "); 
  Serial.print(cm);
  Serial.print(" cm.");
  Serial.println();
  
  delay(250);
  return cm;
}

void determineMotionAwayFromObjects() {
  // measure the distances in front, left of, and right of the robot
  int distanceAt10Degrees = checkDistanceAtServoAngle(10);
  int distanceAt90Degrees = checkDistanceAtServoAngle(90);
  int distanceAt170Degrees = checkDistanceAtServoAngle(170);
  servoHead.write(90); // return servo and ultrasonic sensor to face forward
  // if the distance left of the robot is greatest, move left
  if (distanceAt10Degrees > distanceAt90Degrees && distanceAt10Degrees > distanceAt170Degrees) {
    robotState = 6;
    turnLeft();
    /*turnLeft(turningLeftTime90Degrees); 
    goForward(); 
    delay(distanceForward);
    stop();*/  
  } 
  // else if the distance forward of the robot is greatest, move forward
  else if (distanceAt90Degrees > distanceAt10Degrees && distanceAt90Degrees > distanceAt170Degrees) {
    robotState = 1;
    goForward();
    /*goForward(); 
    delay(distanceForward);
    stop();*/
  } 
  // else if the distance right of the robot is greatest, turn right
  else {
    robotState = 7;
    turnRight();
    /*turnRight(turningRightTime90Degrees);
    goForward(); 
    delay(distanceForward);
    stop();*/
  }
}

void determineMotionTowardsObjects() {
  // measure the distances in front, left of, and right of the robot
  int distanceAt10Degrees = checkDistanceAtServoAngle(10);
  int distanceAt90Degrees = checkDistanceAtServoAngle(90);
  int distanceAt170Degrees = checkDistanceAtServoAngle(170);
  servoHead.write(90); // return servo and ultrasonic sensor to face forward
  // if the distance left of the robot is least, turn left
  if (distanceAt10Degrees < distanceAt90Degrees && distanceAt10Degrees < distanceAt170Degrees && distanceAt10Degrees > safeDistance) {
    robotState = 6;
    turnLeft();
    /*turnLeft(turningLeftTime90Degrees);  
    goForward(); 
    delay(distanceForward);
    stop();*/ 
  } 
  // else if the distance forward of the robot is least, move forward
  else if (distanceAt90Degrees < distanceAt10Degrees && distanceAt90Degrees < distanceAt170Degrees && distanceAt90Degrees > safeDistance) {
    robotState = 1;
    goForward();
    /*goForward(); 
    delay(distanceForward);
    stop();*/ 
  } 
  // else if the distance right of the robot is least, turn right
  else if (distanceAt170Degrees < distanceAt10Degrees && distanceAt170Degrees < distanceAt90Degrees && distanceAt170Degrees > safeDistance) {
    robotState = 7;
    turnRight();
    /*turnRight(turningRightTime90Degrees);
    goForward(); 
    delay(distanceForward);
    stop();*/ 
  } 
  // else if the robot is too close to an object, do something different
  else {
    Serial.println("Begin spin movement.");
    robotState = 8;
    goBackward();
    /*goBackward(); 
    delay(distanceForward/2);
    turnRight(4000);
    stop(); 
    Serial.println("End spin movement.")*/
  }
  
}

void determineMotion() {
  // if the environment is darker than the initial average light level, 
  // then the robot should become more comfortable and thus move towards objects/people
  int currentLightLevel = analogRead(photoresistorPin);
  Serial.print("Current Light Level: ");
  Serial.print(currentLightLevel);
  Serial.print(" Average Light Level: ");
  Serial.println(averageLightLevel);
  // - 10 to avoid noisy fluctuations
  if (currentLightLevel < (averageLightLevel - 10)) { 
    Serial.println("Dark - move towards objects");
    determineMotionTowardsObjects();
  } 
  // else if the environment is bright
  // then the robot should be uncomfortable and thus move away from objects/people
  else {
    Serial.println("Bright - move away from objects");
    determineMotionAwayFromObjects();
  }
  // wait a bit before sensing and moving again
  //delay(delayAfterMoving);
}

// --- Functions to Control Basic Movement of the Robot --- //
// Function to make the robot go forward
// This function does not stop the motor
void goForward() {
  Serial.println("goForward");
  leftMotor->setSpeed(150*speedReductionFactorForward);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(150);
  rightMotor->run(FORWARD);
}

// Function to make the robot go backwards
void goBackward() {
  Serial.println("goBackward");
  leftMotor->setSpeed(150*speedReductionFactorBackward);
  leftMotor->run(BACKWARD);
  rightMotor->setSpeed(150);
  rightMotor->run(BACKWARD);
}

// stop both motors
void stop() {
  Serial.println("stop");
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

// Function to make the robot turn right
// for a certain amount of time
// This function stops the motor once the
// time is up
// Note that this function takes one parameter,
// namely, the amount of time in millliseconds
// to turn
//void turnRight(int amount) { // amount is the parameter
void turnRight() {
  Serial.println("turnRight");
  leftMotor->setSpeed(250*speedReductionFactorForward);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(50);
  rightMotor->run(BACKWARD);
  //delay(amount); // here we are using the parameter
  //stop();
  //delay(250);
}

// Function to make the robot turn left
// for a certain amount of time 
//void turnLeft(int amount) {
void turnLeft() {
  Serial.println("turnLeft");
  rightMotor->setSpeed(250);
  rightMotor->run(FORWARD);
  leftMotor->setSpeed(50*speedReductionFactorBackward);
  leftMotor->run(BACKWARD);
  //delay(amount); 
  //stop();
  //delay(250);
}

// I used this function at the begining to test the
// motors and confirm that they were turning in the
// right direction, but once I was done I didn't
// need this anymore; however I'm keeping it in here
// (a) as more examples and (b) in case I need to use
// it again
void testRobot() {
  // do a little test:
  // Go forward for a bit
  leftMotor->setSpeed(150*speedReductionFactorForward);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(150);
  rightMotor->run(FORWARD);
  delay (500);

  // stop for a moment before changing direction
  // to avoid stressing the motor
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay (250);

  // Go backwards for a bit
  leftMotor->setSpeed(150*speedReductionFactorBackward);
  leftMotor->run(BACKWARD);
  rightMotor->setSpeed(150);
  rightMotor->run(BACKWARD);
  delay (500);

  // and stop.
  // Note that the  comment in the example is wrong:
  // RELEASE turns the motor OFF
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

/*void square() {
  // Try to do a square:
  goForward();
  delay (2000); // amount of time to go forward
  stop();       // what it says
  delay (250);  // let motors actually stop

  turnRight(turningRightTime90Degrees); // turn right for this much time

  goForward();
  delay (2000);
  stop();
  delay (250);

  turnRight(turningRightTime90Degrees); // 300 is the argument

  goForward();
  delay (2000);
  stop();
  delay (250);

  turnRight(turningRightTime90Degrees);

  goForward();
  delay (2000);
  stop();
  delay (250); // totally unnecessary  
}
*/
