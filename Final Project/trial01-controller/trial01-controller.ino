// ------ LIBRARIES ------ //
// We'll use SoftwareSerial to communicate with the XBee:
#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX
// ------ GLOBAL VARIABLES ------ //
// --- NAVIGATOR --- //
// Navigator Button Stuff
// for each button, the relevant info is stored in an array
// 0: pin number, 1: current button state, 2: last button state, 3: tell other functions to do stuff
int buttonIncrementMessagesIndex[4] = {4, 0, 0, 0};
int buttonDecrementMessagesIndex[4] = {5, 0, 0, 0};
int buttonReadyToDisplayMessage[4] = {6, 0, 0, 0};
// Navigator Message Stuff
const int lengthOfNavigatorMessagesArray = 6;
String navigatorMessagesArray[lengthOfNavigatorMessagesArray] = {"Hello, I'm string 1.", "Bonjour, je suis string 2.", 
"Hi, I'm string 3.", "Salut, je suis string 4.", "Goodbye, said string 5.", "Au revoir, said string 6"};
int navigatorMessagesIndex = 0;

// --- WANDERER --- //
// --- Variables to Control the Wanderer's Movements and Fan --- //
// Wanderer Inputs
const int joystickXPin = 0;
const int joystickYPin = 1;
int joystickXValue = 0;
int joystickYValue = 0;
// for each button, the relevant info is stored in an array
// 0: pin number, 1: current button state, 2: last button state, 3: tell other functions to do stuff
int buttonToggleFan[4] = {7, 0, 0, 0};
int buttonIncrementSpeedReductionFactor[4] = {8, 0, 0, 0};
int buttonDecrementSpeedReductionFactor[4] = {9, 0, 0, 0};
int buttonToggleUnisonMode[4] = {10, 0, 0, 0};

// Fan
int fanOn = 0;
// Move in unison mode
int moveInUnisonMode = 0;
//0: unison mode off, 1-5: unison mode on
//1: up, 2: down, 3: left, 4: right, 5: stop
int navigatorDirectionCode = 0;  

// --- Variables to Control the DC Motors' Speed --- //
// need to slow down the left motor so that its speed matches up with the speed of the right motor
// so I'll use the below factor to multiply this variable with the speed of the left motor to slow it down
int speedReductionFactor = 48; //0.465; // 0.448 0.45 with battery

// ------ SETUP ------ //
void setup() {
  XBee.begin(9600);
  Serial.begin(9600);
  Serial.println("Controller commencing...");
  // Set up button pins as inputs
  pinMode(buttonIncrementMessagesIndex[0],INPUT);
  pinMode(buttonDecrementMessagesIndex[0],INPUT);
  pinMode(buttonReadyToDisplayMessage[0],INPUT);
  pinMode(buttonToggleFan[0],INPUT);
  pinMode(buttonIncrementSpeedReductionFactor[0],INPUT);
  pinMode(buttonDecrementSpeedReductionFactor[0],INPUT);
  pinMode(buttonToggleUnisonMode[0],INPUT);
}

// ------ LOOP ------ //
void loop() {
//  Serial.print(digitalRead(buttonIncrementMessagesIndex[0]));
//  Serial.print(digitalRead(buttonDecrementMessagesIndex[0]));
//  Serial.print(digitalRead(buttonReadyToDisplayMessage[0]));
//  Serial.print(digitalRead(buttonToggleFan[0]));
//  Serial.print(digitalRead(buttonIncrementSpeedReductionFactor[0]));
//  Serial.print(digitalRead(buttonDecrementSpeedReductionFactor[0]));
//  Serial.print(digitalRead(buttonToggleUnisonMode[0]));
//  Serial.print(analogRead(joystickXPin));
//  Serial.println(analogRead(joystickYPin));

  // --- NAVIGATOR --- //
  // 1. increment the array index 
  // 2. decrement the array index
  // 3. check if message is ready to be sent
  // 4. send info through xbee

  // 1. increment the array index 
  buttonEdgeStateDetection(buttonIncrementMessagesIndex);
  navigatorMessagesIndex = incrementNavigatorMessagesIndex(navigatorMessagesIndex, buttonIncrementMessagesIndex[3], lengthOfNavigatorMessagesArray-1);    
  // 2. decrement the array index
  buttonEdgeStateDetection(buttonDecrementMessagesIndex);
  navigatorMessagesIndex = decrementNavigatorMessagesIndex(navigatorMessagesIndex, buttonDecrementMessagesIndex[3]);
  // 3. check if message is ready to be sent
  buttonEdgeStateDetection(buttonReadyToDisplayMessage);
  // 4. send info through xbee
  // Sending stuff to XBee N for the navigator
  String navigatorMessagesIndexString = String(navigatorMessagesIndex);
  String buttonReadyToDisplayMessageString = String(buttonReadyToDisplayMessage[3]);
  XBee.print(navigatorMessagesIndexString);
  XBee.print(',');
  XBee.print(buttonReadyToDisplayMessageString);
  XBee.print(',');

  Serial.print(navigatorMessagesIndexString + ", " + buttonReadyToDisplayMessageString + ", ");

  // --- WANDERER --- //
  // 1. toggle fan on/off
  // 2. toggle unison mode on/off
  // 3. increment speed reduction factor
  // 4. decrement speed reduction factor
  // 5. multiply speed reduction by 100
  // 6. read joystick x 
  // 7. read joystick y
  // 8. figure out direction code
  // 9. send info through xbee

  // 1. toggle fan on/off
  buttonEdgeStateDetection(buttonToggleFan);
  fanOn = toggleFanBasedOnButtonInput(buttonToggleFan[3], fanOn);
  // 2. toggle unison mode on/off
  buttonEdgeStateDetection(buttonToggleUnisonMode);
  moveInUnisonMode = toggleUnisonModeBasedOnButtonInput(buttonToggleUnisonMode[3], moveInUnisonMode);
  // 3. increment speed reduction factor
  buttonEdgeStateDetection(buttonIncrementSpeedReductionFactor);
  speedReductionFactor = incrementSpeedReductionFactor(buttonIncrementSpeedReductionFactor[3], speedReductionFactor);
  // 4. decrement speed reduction factor
  buttonEdgeStateDetection(buttonDecrementSpeedReductionFactor);
  speedReductionFactor = decrementSpeedReductionFactor(buttonDecrementSpeedReductionFactor[3], speedReductionFactor); 
  // 5. multiply speed reduction by 100
//  speedReductionFactor *= 100;
  // 6. read joystick x
  joystickXValue = readAndSendJoystickXValue(joystickXPin);
  // 7. read joystick y
  joystickYValue = readAndSendJoystickYValue(joystickYPin);
  // 8. figure out direction code
  readJoystickXYValues(joystickXPin, joystickYPin);
  // 9. send info through xbee
  // Sending stuff to XBee W for the wanderer
  String navigatorDirectionCodeString = String(navigatorDirectionCode);
  String fanOnString = String(fanOn);
  String joystickXValueString = String(joystickXValue);
  String joystickYValueString = String(joystickYValue);
  String speedReductionFactorString = String(speedReductionFactor);
  XBee.print(navigatorDirectionCodeString);
  XBee.print(',');
  XBee.print(fanOnString);
  XBee.print(',');
  XBee.print(joystickXValueString);
  XBee.print(',');
  XBee.print(joystickYValueString);
  XBee.print(',');
  XBee.print(speedReductionFactorString);
  XBee.print('\n');

  Serial.println(navigatorDirectionCodeString + ", " + fanOnString + ", " + joystickXValueString + ", " + joystickYValueString + ", " + speedReductionFactorString);
}

// ------ CUSTOM FUNCTIONS ------ //
// ------ NAVIGATOR ------ //
void buttonEdgeStateDetection(int buttonInfoArray[]) {
  // read the pushbutton input pin:
  int buttonState = digitalRead(buttonInfoArray[0]);
  int lastButtonState = buttonInfoArray[2];
  buttonInfoArray[3] = 0;
  
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
//      Serial.println("on");
      buttonInfoArray[3] = 1;
    } else {
      // if the current state is LOW then the button went from on to off:
//      Serial.println("off");
      buttonInfoArray[3] = 0;
    }
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;

  buttonInfoArray[1] = buttonState;
  buttonInfoArray[2] = lastButtonState;
}

// 6. Input: 1. index of the array, state of the button that will affect the index, length of the array - 1
// Output: update the counter whenever the relevant button is pressed
int incrementNavigatorMessagesIndex(int counter, int buttonStateChanged, int counterMaxLimit) {
  // update the counter if the button was pressed
  if (buttonStateChanged == 1) {
    counter++;
    Serial.println("----------------------------------");
  }
  // don't let the counter go beyond the limits of the array's length
  if (counter >= counterMaxLimit) {
    counter=counterMaxLimit;
  }
  //Serial.println(counter);
  return counter;
}

// 7. Input: 1. index of the array, state of the button that will affect the index, length of the array - 1
// Output: update the counter whenever the relevant button is pressed
int decrementNavigatorMessagesIndex(int counter, int buttonStateChanged) {
  // update the counter if the button was pressed
  if (buttonStateChanged == 1) {
    counter--;
    Serial.println("********************************"); 
  }
  // don't let the counter go beyond the limits of the array's length
  if (counter <= 0) {
    counter=0;
  }
//  Serial.print("Counter: ");
//  Serial.println(counter);
  return counter;
}

// -------------------------------------------------------- //
// --- WANDERER CONTROLS --- //
// --- Functions to use the joystick's x and y axis inputs --- //
int readAndSendJoystickXValue(int inputPinX) {
  int inputValueX = analogRead(inputPinX);
  return inputValueX/4;
}

int readAndSendJoystickYValue(int inputPinY) {
  int inputValueY = analogRead(inputPinY);
  return inputValueY/4;
}

void readJoystickXYValues(int inputPinX, int inputPinY) {
  // x-value stuff
  int inputValueX = analogRead(inputPinX);
  int leftMinValue = 0;
  int leftMaxValue = 490;
  int rightMinValue = 530;
  int rightMaxValue = 1023;
  // y-value stuff
  int inputValueY = analogRead(inputPinY);
  int backwardMinValue = 0;
  int backwardMaxValue = 490;
  int forwardMinValue = 530;
  int forwardMaxValue = 1023;

  // if the x- and y-values are withing resting state, then stop the robot
  if (inputValueX < rightMinValue && inputValueX > leftMaxValue && inputValueY < forwardMinValue && inputValueY > backwardMaxValue) {
    if (moveInUnisonMode == 1) {
      navigatorDirectionCode = 5;
    } else {
      navigatorDirectionCode = 0;
    }
//    Serial.print("stop.");
  }
  // else, let the robot move in the desired direction(s)
  else {
    // if the x-value is greater than the resting value, then turn right
    if (inputValueX > rightMinValue) {
      if (moveInUnisonMode == 1) {
        navigatorDirectionCode = 4;
      } else {
        navigatorDirectionCode = 0;
      }
//      Serial.print("turn right.");
    } 
    // else if the x-value is less than the resting value, then turn left
    if (inputValueX < leftMaxValue) {
      if (moveInUnisonMode == 1) {
        navigatorDirectionCode = 3;
      } else {
        navigatorDirectionCode = 0;
      }
//      Serial.print("turn left.");
    } 
    // if the y-value is greater than the resting value, then move forward
    if (inputValueY > forwardMinValue) {
      if (moveInUnisonMode == 1) {
        navigatorDirectionCode = 1;
      } else {
        navigatorDirectionCode = 0;
      }
//      Serial.print("go forward.");
    } 
    // else if the y-value is less than the resting value, then move backward
    if (inputValueY < backwardMaxValue) {
      if (moveInUnisonMode == 1) {
        navigatorDirectionCode = 2;
      } else {
        navigatorDirectionCode = 0;
      }
//      Serial.print("go backward.");
    } 
  }
}

// --- Functions for reading buttons, and their consequences --- //
int toggleFanBasedOnButtonInput(int buttonStateChanged, int fanIsOn) {
  // update the counter if the button was pressed
  if (buttonStateChanged == 1) {
    if (fanIsOn == 1) {
      fanIsOn = 0;
      // turn off the fan  
      //fanMotor->run(RELEASE);
    } else {
      fanIsOn = 1;
      // turn on the fan
      //fanMotor->run(FORWARD);
    }
  }
//  Serial.print(" Fan state: ");
//  Serial.print(fanIsOn);
  return fanIsOn;
}

int toggleUnisonModeBasedOnButtonInput(int buttonStateChanged, int unisonModeIsOn) {
  // update the counter if the button was pressed
  if (buttonStateChanged == 1) {
    unisonModeIsOn = !unisonModeIsOn;
  }
//  Serial.print(" Unison mode: ");
//  Serial.print(unisonModeIsOn);
  return unisonModeIsOn;
}

int incrementSpeedReductionFactor(int buttonStateChanged, float reductionFactor) {
  int upperLimit = 100;
  // increase the reduction factor if the button was pressed
  if (buttonStateChanged == 1) {
    reductionFactor += 1;
  }
  // prevent the reduction factor from becoming too high
  if (reductionFactor >= upperLimit) {
    reductionFactor = upperLimit;
  }
  return reductionFactor;
}

int decrementSpeedReductionFactor(int buttonStateChanged, float reductionFactor) {
  int lowerLimit = 0;
  // decrease the reduction factor if the button was pressed
  if (buttonStateChanged == 1) {
    reductionFactor -= 1;
  }
  // prevent the reduction factor from becoming too low
  if (reductionFactor <= lowerLimit) {
    reductionFactor = lowerLimit;
  }
  return reductionFactor;
}
