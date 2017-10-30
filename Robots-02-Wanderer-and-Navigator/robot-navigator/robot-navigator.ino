/* Robot N
*/
// We'll use SoftwareSerial to communicate with the XBee:
#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX

#include <Servo.h>

Servo leftServo;  // create servo object to control left servo
Servo rightServo;  // create servo object to control right servo

int servoMode = 0;
int flagDirection = 0; // store the position command received from Robot W's Xbee W

void setup() {
  XBee.begin(9600);
  Serial.begin(9600);
  leftServo.attach(9);  // attaches the servo on pin 9 to the servo object
  rightServo.attach(10); // attaches the servo on pin 10 to the servo object
}

void loop() {
  
  // read incoming data from Xbee W onto Xbee N
  if (XBee.available())
  { // If data comes in from XBee, send it out to serial monitor
    //Serial.write(XBee.read());
    servoMode = XBee.parseInt();
    flagDirection = XBee.parseInt();
    if (XBee.read() == '\n') {
      if (servoMode == 6) {
        moveFlagPosition(flagDirection);
      } else {
        // move servos to resting position  
          leftServo.write(135);
          rightServo.write(45); 
      }
      Serial.print(servoMode);
      Serial.print('-');
      Serial.println(flagDirection);
    }
  }

}

void moveFlagPosition(int flagPosition) {
  if (flagPosition == 1) { // forward // appears as both flags at top position
    Serial.print("Move forward.");
    leftServo.write(90);
    rightServo.write(90);  
  } else if (flagPosition == 2) { // backward // appears as both flags outward position
    Serial.print("Move backward.");
    leftServo.write(0);
    rightServo.write(180);  
  } else if (flagPosition == 3) { // left // right flag: top // left flag: outward
    Serial.print("Move left.");
    leftServo.write(0);
    rightServo.write(90);  
  } else if (flagPosition == 4) { // right // right flag: outward // left flag: top
    Serial.print("Move right.");
    leftServo.write(90);
    rightServo.write(180);  
  } else if (flagPosition == 0) { // stop // both flags: inwards at 45 degree angle
    Serial.print("Stop.");
    leftServo.write(135);
    rightServo.write(45);  
  } 
}

