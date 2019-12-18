#include <Arduino.h> //Standard Arduino libary
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h> // måske ikke nødvendigt
#include "Packets.h"

#define SERVO_SET_Baudrate  57600  //Baudrate is set to the right number

PacketsClass pck;

LiquidCrystal_I2C lcd(0x27, 20, 4);

void modeOne(bool modeOne) {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd. print("Mode One");
  while (modeOne == true) {


    String Gesture;
    if (Serial.available() > 0) {
      Gesture = Serial.readStringUntil('\n');
      delay (100);
    }

    if (Gesture.equals("waveIn")) {
      lcd.clear();
      lcd.setCursor(6, 1);
      lcd.print("Mode One");
      lcd.setCursor(6, 2);
      lcd.print("WaveIn");

      int p1 = 2048; //Servo 1 goal position
      int p2 = 2316; //Servo 2 goal position
      int p3 = 1743; //Servo 3 goal position
      int p4 = -1; //Servo 4 goal position
      int p5 = -1; //Servo 5 goal position

      pck.setNGoalPositionPacket(p1, p2, p3, p4, p5);

    }

    if (Gesture.equals("waveOut")) {

      lcd.clear();
      lcd.setCursor(6, 1);
      lcd.print("Mode One");
      lcd.setCursor(6, 2);
      lcd.print("WaveOut");

      int p1 = 626; //Servo 1 -125 deg
      int p2 = 2316; //Servo 2 2316 deg
      int p3 = 1333; //Servo 3 133 deg
      int p4 = -1; //Servo 4 is not set
      int p5 = -1; //Servo 5 is not set

      pck.setNGoalPositionPacket(p1, p2, p3, p4, p5);
      delay(2000);
      int p23 = 1860; //Servo 3 1333 de
      pck.setNGoalPositionPacket(p1, p2, p23, p4, p5);
    }


    if (Gesture.equals("fist")) {

      lcd.clear();
      lcd.setCursor(6, 1);
      lcd.print("Mode One");
      lcd.setCursor(6, 2);
      lcd.print("Fist");

      int p1 = -1; //Servo 1 goal position
      int p2 = -1; //Servo 2 goal position
      int p3 = -1; //Servo 3 goal position
      int p4 = 2050; //Servo 4 goal position
      int p5 = 2050; //Servo 5 goal position

      pck.setNGoalPositionPacket(p1, p2, p3, p4, p5);
    }


    if (Gesture.equals("fingersSpread")) {

      lcd.clear();
      lcd.setCursor(6, 1);
      lcd.print("Mode One");
      lcd.setCursor(6, 2);
      lcd.print("Open");

      int p1 = -1; //Servo 1 goal position
      int p2 = -1; //Servo 2 goal position
      int p3 = -1; //Servo 3 goal position
      int p4 = 2500; //Servo 4 goal position
      int p5 = 2500; //Servo 5 goal position


      pck.setNGoalPositionPacket(p1, p2, p3, p4, p5);
      delay(1000);
      int p22 = 1126; //Servo 2 goal position
      pck.setNGoalPositionPacket(p1, p22, p3, p4, p5);
      delay(500);
      int p21 = 2048; //Servo 1 goal position
      int p23 = 2048; //Servo 3 goal position
      pck.setNGoalPositionPacket(p21, p2, p23, p4, p5);
    }

    if (Gesture.equals("modetwo")) {
      modeOne = false;
    }
  }
}


void modeTwo(bool modeTwo) {
  pck.SetPGain(1, 200);
  pck.SetPGain(2, 200);
  pck.SetPGain(3, 200);
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd. print("Mode Two");
  while (modeTwo == true) {


    String Gesture;

    if (Serial.available() > 0) {
      Gesture = Serial.readStringUntil('\n');
      delay (100);
    }
  
if (Gesture.equals("waveIn")) {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Mode Two");
  lcd.setCursor(7, 1);
  lcd.print("waveIn");
  lcd.setCursor(2, 2);
  lcd.print("Shoulder rotation");
  for (int x = 0; x < 5; x++) {
    int p1 = 2048; //Servo 1 goal position
    int p2 = 2048; //Servo 2 goal position
    int p3 = 1024; //Servo 3 goal position
    int p4 = 2048; //Servo 4 goal position
    int p5 = 2048; //Servo 5 goal position

    pck.setNGoalPositionPacket(p1, p2, p3, p4, p5);
    delay(1000);
    int p12 = 1024; //Servo 1 goal position


    pck.setNGoalPositionPacket(p12, p2, p3, p4, p5);
    delay(1000);
  }
}

if (Gesture.equals("waveOut")) {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Mode Two");
  lcd.setCursor(6, 1);
  lcd.print("waveOut");
  lcd.setCursor(4, 2);
  lcd.print("Tricep Rehab");
  for (int x = 0; x < 5; x++) {
    int p1 = 2048; //Servo 1 goal position
    int p2 = 1125; //Servo 2 goal position
    int p3 = 1024; //Servo 3 goal position
    int p4 = 2048; //Servo 4 goal position
    int p5 = 2048; //Servo 5 goal position

    pck.setNGoalPositionPacket(p1, p2, p3, p4, p5);
    delay(1000);
    int p32 = 2048;

    pck.setNGoalPositionPacket(p1, p2, p32, p4, p5);
    delay(1000);
  }
}

if (Gesture.equals ("fist")) {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Mode Two");
  lcd.setCursor(8, 1);
  lcd.print("Fist");
  lcd.setCursor(4, 2);
  lcd.print("Bicep Rehab");
  for (int x = 0; x < 5; x++) {
    int p1 = 2048; //Servo 1 goal position
    int p2 = 2048; //Servo 2 goal position
    int p3 = 2048; //Servo 3 goal position
    int p4 = 2048; //Servo 4 goal position
    int p5 = 2048; //Servo 5 goal position

    pck.setNGoalPositionPacket(p1, p2, p3, p4, p5);
    delay(1000);

    int p32 = 1024; //Servo 3 goal position


    pck.setNGoalPositionPacket(p1, p2, p32, p4, p5);
    delay(1000);
  }
}

if (Gesture.equals ("fingersSpread")) {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Mode Two");
  lcd.setCursor(3, 1);
  lcd.print("FingersSpread");
  lcd.setCursor(3, 2);
  lcd.print("Shoulder Press");
  for (int x = 0; x < 5; x++) {
    int p1 = 2048; //Servo 1 goal position
    int p2 = 2048; //Servo 2 goal position
    int p3 = 1024; //Servo 3 goal position
    int p4 = 2048; //Servo 4 goal position
    int p5 = 2048; //Servo 5 goal position

    pck.setNGoalPositionPacket(p1, p2, p3, p4, p5);
    delay(1000);

    int p22 = 1107; //Servo 3 goal position
    int p32 = 2048;

    pck.setNGoalPositionPacket(p1, p22, p32, p4, p5);
    delay(1000);
  }
}


if (Gesture.equals("modeone")) {
  modeTwo = false;
}
}
}




void setup() {

  Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
  pck.mySerial.begin(SERVO_SET_Baudrate);
  Serial.begin(SERVO_SET_Baudrate);



  pck.setProfileVelocity(100);
  pck.setProfileAcceleration(100);

  pck.setNTorquePacket(true);

  lcd.begin();
  lcd.backlight();

}

void loop() {
  String Gesture;
  if (Serial.available() > 0) {
    Gesture = Serial.readStringUntil('\n');
    delay(100);
  }
  lcd.clear();

  if (Gesture.equals("modeone")) {
    modeOne(true);
  };

  if (Gesture.equals("modetwo")) {
    modeTwo(true);
  };

}
