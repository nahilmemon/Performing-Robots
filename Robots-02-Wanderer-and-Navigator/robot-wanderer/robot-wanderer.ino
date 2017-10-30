// ----- LIBRARIES ----- //
// XBee 
// We'll use SoftwareSerial to communicate with the XBee:
#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX
// Motor Shield
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
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

// --- Variables to Control the DC Motors' Speed --- //
// need to slow down the left motor so that its speed matches up with the speed of the right motor
// so I'll use the below factor to multiply this variable with the speed of the left motor to slow it down
float speedReductionFactorForward = 0.48; //0.465; // 0.448 0.45 with battery
float speedReductionFactorBackward = 0.48;// 0.465; // 0.45 with battery
int turningRightTime90Degrees = 950; // amount of time (ms) it takes to turn 90 degrees
int turningLeftTime90Degrees = 950; // amount of time (ms) it takes to turn 90 degrees
// --- Variables Storing the State and Timings of the Robot's Movements --- //
bool servoOnMode = false;
int lastButtNum = 0;
int robotState = 0; // Actual movements: 0: pause, 1: forward, 2: backward, 3: left, 4: right
int robotNextState = 0;
long pauseTime = 250; // the amount of time the robot waits before determining which way to move
unsigned long previousRobotMovingMillis = 0; // for comparing time intervals
int motorSpeed = 0;
// --- Variables Storing the Information to Send Through Xbee to Robot N --- //
int codeForRobotNServo = 5;
int codeForRobotNDirection = 0;

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
  
  XBee.begin(9600);
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
  
  // --- Other Stuff --- //
  AFMS.begin();  // create with the default frequency 1.6KHz (for the motor shield)
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
  // Stuff that should always happen in the loop:
  //Serial.println("looping");
//  Serial.print("----------------------------------Step 0: code to send is: ");
//  Serial.print(codeForRobotNServo);
//  Serial.print(" -- ");
//  Serial.println(codeForRobotNDirection);

  // Sending stuff from Xbee W to Xbee N
  String codeForRobotNServoString = String(codeForRobotNServo);
  String codeForRobotNDirectionString = String(codeForRobotNDirection);
  XBee.print(codeForRobotNServoString);
  XBee.print(',');
  XBee.print(codeForRobotNDirectionString);
  XBee.print('\n');

  // If both robots should move, then update the motion of Robot W with a delay
  if (servoOnMode == true) {
    Serial.println("------");
    Serial.println("Step 4a: Let's update motion.");
    updateMotionIfServoOnModeTrue(); 
  }

  
  // Read data coming from the Bluefruit LE module
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  
  // if no input is given from the phone controller, then exit the loop immediately
  if (len == 0) return;

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("****Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released****");
    }

    // If button is pressed
    if (pressed) {
      // Step 1: first check which servo mode to use
      if (buttnum == 3) {
        Serial.println("------");
        Serial.println("Step 1: ");
        if (servoOnMode == true) {
          servoOnMode = false;
          codeForRobotNServo = 5;
          Serial.println("  servoOnMode = false. Only Robot W moves.");  
        } else {
          servoOnMode = true;
          codeForRobotNServo = 6;
          Serial.println("  servoOnMode = true. Both robots must move together.");  
        }
      }

      // Step 2: if servoOnMode is false, then let only robot W move with no delays.
      if (servoOnMode == false) {
        Serial.println("------");
        Serial.println("Step 2: Only Robot W moves.");
        if (buttnum == 5) {
          Serial.println("  Move forward");
          codeForRobotNDirection = 1;
          goForward();
        }
        if (buttnum == 6) {
          Serial.println("  Move backward");
          codeForRobotNDirection = 2;
          goBackward();
        }
        if (buttnum == 7) {
          Serial.println("  Move left");
          codeForRobotNDirection = 3;
          turnLeft();
        }
        if (buttnum == 8) {
          Serial.println("  Move right");
          codeForRobotNDirection = 4;
          turnRight();
        }
      }

      // Step 3: if servoOnMode is true, then both robots move. Robot W must have a delay first.
      if (servoOnMode == true) {
        Serial.println("------");
        Serial.println("Step 3: Both robots move. Robot W has a delay.");
        // Step 3a: Was a new button command given? If so, create a pause before movement. Else just move.
        if ((buttnum != 3) && (buttnum != lastButtNum)) {
          Serial.println("  New command given. Commencing the pause.");
          // Set pause mode to true.
          robotState = 0; // pause mode is true
          // reset the clock's previous millis to check against
          previousRobotMovingMillis = millis();
          // Set the next robot state for when the pause has been completed.
          if (buttnum == 5) {
            codeForRobotNDirection = 1;
            robotNextState = 1;  // forward
          } else if (buttnum == 6) {
            codeForRobotNDirection = 2;
            robotNextState = 2;  // backward
          } else if (buttnum == 7) {
            codeForRobotNDirection = 3;
            robotNextState = 3;  // left
          } else if (buttnum == 8) {
            codeForRobotNDirection = 4;
            robotNextState = 4;  // right
          }
          Serial.print("  Next robot state: ");
          Serial.println(robotNextState);
        }
      } 

      // Step Last: recalibrate left motor speed reduction factor
      if (buttnum == 1) {
        Serial.println("------");
        Serial.println("Step Last: Recalibrate left motor speed reduction factor.");
        Serial.print("  Slow down left motor: ");
        Serial.print("Speed Reduction Factor: ");
        speedReductionFactorForward -= 0.01;
        speedReductionFactorBackward -= 0.01;
        Serial.println(speedReductionFactorForward);
      }
      if (buttnum == 2) {
        Serial.println("------");
        Serial.println("Step Last: Recalibrate left motor speed reduction factor.");
        Serial.print("  Speed up left motor");
        Serial.print("Speed Reduction Factor: ");
        speedReductionFactorForward += 0.01;
        speedReductionFactorBackward += 0.01;
        Serial.println(speedReductionFactorForward);
      }
      if (buttnum == 4) {
        Serial.println("Still stop.");
        codeForRobotNDirection = 0;
        stop();
        robotNextState = 0;
      }
    } 
    // If button is released
    else { // button has been released
      // Step 2: if servoOnMode is false, then let only robot W move with no delays.
      if (servoOnMode == false) {
        Serial.println("------");
        Serial.println("Step 2: Only Robot W moves.");
        Serial.println("  Stop.");
        codeForRobotNDirection = 0;
        stop();
      } 
    }
  } // end of button event
}

// --- Functions to Update Motion of the Robot --- //
void updateMotionIfServoOnModeTrue() {
  Serial.println("Step 4b: Which state and motion. ");
  Serial.println(robotState);
  unsigned long currentRobotMovingMillis = millis();
  
  // For when the robot needs to pause first before moving
  if (robotState == 0) {
    // stop the robot
    stop();
    // if the robot has paused long enough, then set the new robot state
    if (currentRobotMovingMillis - previousRobotMovingMillis >= pauseTime) {
      robotState = robotNextState;
      Serial.print("  New state: ");
      Serial.println(robotState);
//      previousRobotMovingMillis = currentRobotMovingMillis;
    }
  }
  // Else if the robot no longer needs to pause (robotState != 0), then move
  else {
    if (robotState == 1) {
      goForward();
    } else if (robotState == 2) {
      goBackward();
    } else if (robotState == 3) {
      turnLeft();
    } else if (robotState == 4) {
      turnRight();
    }
  }
}

// -------------------------------------------------------- //

// --- Functions to Control Basic Movement of the Robot --- //
// Function to make the robot go forward
// This function does not stop the motor
void goForward() {
  //Serial.println("goForward");
  leftMotor->setSpeed(150*speedReductionFactorForward);
  leftMotor->run(FORWARD);
  rightMotor->setSpeed(150);
  rightMotor->run(FORWARD);
}

// Function to make the robot go backwards
void goBackward() {
  //Serial.println("goBackward");
  leftMotor->setSpeed(150*speedReductionFactorBackward);
  leftMotor->run(BACKWARD);
  rightMotor->setSpeed(150);
  rightMotor->run(BACKWARD);
}

// stop both motors
void stop() {
  //Serial.println("stop");
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
  //Serial.println("turnRight");
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
  //Serial.println("turnLeft");
  rightMotor->setSpeed(250);
  rightMotor->run(FORWARD);
  leftMotor->setSpeed(50*speedReductionFactorBackward);
  leftMotor->run(BACKWARD);
  //delay(amount); 
  //stop();
  //delay(250);
}
