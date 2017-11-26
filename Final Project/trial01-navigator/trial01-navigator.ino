// ------ LIBRARIES ------ //
// We'll use SoftwareSerial to communicate with the XBee:
#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX
// Servo Library
#include <Servo.h>

// ------ GLOBAL VARIABLES ------ //
// Servo Objects
Servo leftServo;  // create servo object to control left servo
Servo rightServo;  // create servo object to control right servo
int servoDelay = 500;
const int servoMaxDelay = 500;
// Navigator Message Stuff
const int lengthOfNavigatorMessagesArray = 6;
String navigatorMessagesArray[lengthOfNavigatorMessagesArray] = {"Hello, I'm string 1.", "Bonjour, je suis string 2.", 
"Hi, I'm string 3.", "Salut, je suis string 4.", "Goodbye, said string 5.", "Au revoir, said string 6"};
int navigatorMessagesIndex = 0;
int messageReadyToBePlayed = 0;
// Unison mode stuff
int directionCodeFromWanderer = 0;

// ------ SETUP ------ //
void setup() {
  XBee.begin(9600);
  Serial.begin(9600);
  leftServo.attach(9);  // attaches the servo on pin 9 to the servo object
  rightServo.attach(10); // attaches the servo on pin 10 to the servo object
  Serial.println("Navigator Starting...");
}

// ------ LOOP ------ //
void loop() {
  // A. read data coming from XBee C
    // 0. Input 0: record the index of the array
    // 1. Input 1: record whether the message is ready to be played
    // 2. Input 2: record the direction code
  // B. play the message based on the unison mode

  // A. read data coming from XBee C
  // read incoming data from Xbee C
  if (XBee.available()) { // if data comes from the XBee
    //Serial.write(XBee.read());
    // --- Navigator Stuff --- //
    // 0. Input 0: record the index of the array
    navigatorMessagesIndex = XBee.parseInt();
    // 1. Input 1: record whether the message is ready to be played
    messageReadyToBePlayed = XBee.parseInt();
    // 2. Input 2: record the direction code
    directionCodeFromWanderer = XBee.parseInt();
    // --- Wanderer Stuff --- //
    // 3. Input 3: record the fan state
    int fanOn = XBee.parseInt();
    // 4. Input 4: record the joystick x value
    int joystickXValue = XBee.parseInt();
    // 5. Input 5: record the joystick y value
    int joystickYValue = XBee.parseInt();
    // 6. Input 6: record the speed reduction factor
    int speedReductionFactor = XBee.parseInt();

    Serial.println(String(navigatorMessagesIndex) + ", " + String(messageReadyToBePlayed) + ", " + String(directionCodeFromWanderer) + ", " + String(fanOn) + ", " + String(joystickXValue) + ", " + String(joystickYValue) + ", " + String(speedReductionFactor));
    
    // After all the inputs have been parsed, perform the desired operations with these inputs
    if (XBee.read() == '\n') {
      //B. play the message based on the unison mode
      playMessageBasedOnMode(directionCodeFromWanderer);
//      Serial.println("XBee Message Complete --------------------------------------------");
      
    }
  }

//  Serial.println("Outside XBee availability");
}

// ------ CUSTOM FUNCTIONS ------ //
// ------ NAVIGATOR ------ //
// 1. Input: left and right flag positions in degrees
// Output: move the servo paddles with flags attached to the desired positions
void moveFlags(int leftFlagPos, int rightFlagPos) {
  leftServo.write(leftFlagPos);
  rightServo.write(rightFlagPos);
  Serial.print("Flag Positions: ");
  Serial.print(leftFlagPos/30+1);
  Serial.print(", ");
  Serial.println(rightFlagPos/30+1);
  delay(servoDelay);
}

// 2. Input: a character
// Output: uses a switch case to determine which flag positions to use based on the character
// Then it calls the moveFlags function
void convertCharToFlagMovement(char inputCharacter) {
  // Possible Servo Positions
  int leftServoPos1 = 0;
  int leftServoPos2 = 30;
  int leftServoPos3 = 60;
  int leftServoPos4 = 90;
  int leftServoPos5 = 120;
  int leftServoPos6 = 150;
  int leftServoPos7 = 180;
  int rightServoPos1 = 0;
  int rightServoPos2 = 30;
  int rightServoPos3 = 60;
  int rightServoPos4 = 90;
  int rightServoPos5 = 120;
  int rightServoPos6 = 150;
  int rightServoPos7 = 180;

  // Convert input character to moveFlags(leftServoPos, rightServoPos)
  switch(inputCharacter) {
    case 'a': moveFlags(leftServoPos1, rightServoPos1);
              break;
    case 'b': moveFlags(leftServoPos1, rightServoPos2);
              break;
    case 'c': moveFlags(leftServoPos1, rightServoPos3);
              break;
    case 'd': moveFlags(leftServoPos1, rightServoPos4);
              break;
    case 'e': moveFlags(leftServoPos1, rightServoPos5);
              break;
    case 'f': moveFlags(leftServoPos1, rightServoPos6);
              break;
    case 'g': moveFlags(leftServoPos1, rightServoPos7);
              break;
    case 'h': moveFlags(leftServoPos2, rightServoPos1);
              break;
    case 'i': moveFlags(leftServoPos2, rightServoPos2);
              break;
    case 'j': moveFlags(leftServoPos2, rightServoPos3);
              break;
    case 'k': moveFlags(leftServoPos2, rightServoPos4);
              break;
    case 'l': moveFlags(leftServoPos2, rightServoPos5);
              break;
    case 'm': moveFlags(leftServoPos2, rightServoPos6);
              break;
    case 'n': moveFlags(leftServoPos2, rightServoPos7);
              break;
    case 'o': moveFlags(leftServoPos3, rightServoPos1);
              break;
    case 'p': moveFlags(leftServoPos3, rightServoPos2);
              break;
    case 'q': moveFlags(leftServoPos3, rightServoPos3);
              break;
    case 'r': moveFlags(leftServoPos3, rightServoPos4);
              break;
    case 's': moveFlags(leftServoPos3, rightServoPos5);
              break;
    case 't': moveFlags(leftServoPos3, rightServoPos6);
              break;
    case 'u': moveFlags(leftServoPos3, rightServoPos7);
              break;
    case 'v': moveFlags(leftServoPos4, rightServoPos1);
              break;
    case 'w': moveFlags(leftServoPos4, rightServoPos2);
              break;
    case 'x': moveFlags(leftServoPos4, rightServoPos3);
              break;
    case 'y': moveFlags(leftServoPos4, rightServoPos4);
              break;
    case 'z': moveFlags(leftServoPos4, rightServoPos5);
              break;
    case '0': moveFlags(leftServoPos4, rightServoPos6);
              break;
    case '1': moveFlags(leftServoPos4, rightServoPos7);
              break;
    case '2': moveFlags(leftServoPos5, rightServoPos1);
              break;
    case '3': moveFlags(leftServoPos5, rightServoPos2);
              break;
    case '4': moveFlags(leftServoPos5, rightServoPos3);
              break;
    case '5': moveFlags(leftServoPos5, rightServoPos4);
              break;
    case '6': moveFlags(leftServoPos5, rightServoPos5);
              break;
    case '7': moveFlags(leftServoPos5, rightServoPos6);
              break;
    case '8': moveFlags(leftServoPos5, rightServoPos7);
              break;
    case '9': moveFlags(leftServoPos6, rightServoPos1);
              break;
    case ' ': moveFlags(leftServoPos6, rightServoPos2);
              break;
    case '.': moveFlags(leftServoPos6, rightServoPos3);
              break;
    case ',': moveFlags(leftServoPos6, rightServoPos4);
              break;
    case '?': moveFlags(leftServoPos6, rightServoPos5);
              break;
    case '!': moveFlags(leftServoPos6, rightServoPos6);
              break;
    case 'U': moveFlags(leftServoPos6, rightServoPos7);
              break;
    case 'D': moveFlags(leftServoPos7, rightServoPos1);
              break;
    case 'L': moveFlags(leftServoPos7, rightServoPos2);
              break;
    case 'R': moveFlags(leftServoPos7, rightServoPos3);
              break;
    case 'N': moveFlags(leftServoPos7, rightServoPos4);
              break;
    case 'E': moveFlags(leftServoPos7, rightServoPos5);
              break;
    case 'S': moveFlags(leftServoPos7, rightServoPos6);
              break;
    case 'W': moveFlags(leftServoPos7, rightServoPos7);
              break;
  }
}

// 3. Input: A string
// Output: Split up the string into individual characters using the convertCharToFlagMovement function.
// For each character, convert this into flag positions.
void displayMessageFromString(String inputString) {
  inputString.toLowerCase();
  for (int i=0; i<inputString.length(); i++) {
    Serial.print(inputString[i]);
    Serial.print(" ");
    convertCharToFlagMovement(inputString[i]);
  }
  Serial.println("*****************************************************");
}

// 4. Input: An array of strings, index, state of the ready to display message button
// Output: if the ready to display message button was pressed, extract one string using the index.
// Display the message of this string using the displayMessageFromString function.
void chooseStringFromArrayToDisplayMessage(String arrayOfStrings[], int arrayIndex, int buttonStateChanged) {
  if (buttonStateChanged == 1) {
    displayMessageFromString(arrayOfStrings[arrayIndex]);
    Serial.println("----------------------------------------------");
  }
} 

void moveFlagsToRestingPosition(int buttonStateChanged) {
  if (buttonStateChanged == 0) {
    servoDelay = 0;
    convertCharToFlagMovement(' ');
  }
}

void playMessageBasedOnMode(int directionCode) {
  if (directionCode != 0) {
    servoDelay = 0;
    // only look at the direction codes
    // north position
    if (directionCode == 1) {
      convertCharToFlagMovement('U');
    }
    // south position
    else if (directionCode == 2) {
      convertCharToFlagMovement('D');
    }
    // west position
    else if (directionCode == 3) {
      convertCharToFlagMovement('L');
    }
    // east position
    else if (directionCode == 4) {
      convertCharToFlagMovement('R');
    }
    // north-east position
    else if (directionCode == 6) {
      convertCharToFlagMovement('E');
    }
    // south-east position
    else if (directionCode == 7) {
      convertCharToFlagMovement('S');
    }
    // north-west position
    else if (directionCode == 8) {
      convertCharToFlagMovement('N');
    }
    // south-west position
    else if (directionCode == 9) {
      convertCharToFlagMovement('W');
    }
    // rest/stop position
    else if (directionCode == 5) {
      convertCharToFlagMovement(' ');
    }
  } else {
    servoDelay = servoMaxDelay;
    // play message if ready
    chooseStringFromArrayToDisplayMessage(navigatorMessagesArray, navigatorMessagesIndex, messageReadyToBePlayed);
    // otherwise, move flags to resting position
    moveFlagsToRestingPosition(messageReadyToBePlayed);
  }
}
