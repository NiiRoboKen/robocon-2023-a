#include <PS4BT.h>
#include <usbhub.h>
#include <SPI.h>

USB Usb;
BTD Btd(&Usb);  // You have to create the Bluetooth Dongle instance like so

PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
// PS4BT PS4(&Btd);

struct Motor {
  int dir_pin;
  int pwm_pin;
};

struct Relay {
  int pin;
};

enum ChoseRelay {
  EnumStart,
  Front,
  Center,
  Rear,
  EnumEnd
};

enum Mode {
  Normal,
  Caterpillar
};

ChoseRelay GetNextEnumValue(ChoseRelay current) {
  if (current + 1 != EnumEnd) {
    return static_cast<ChoseRelay>(current + 1);
  } else {
    return Front;
  }
}

ChoseRelay GetBeforeEnumValue(ChoseRelay current) {
  if (current - 1 != EnumStart) {
    return static_cast<ChoseRelay>(current - 1);
  } else {
    return Rear;
  }
}


Mode ChangeMode(Mode current) {
  if (current == Normal) {
    return Caterpillar;
  } else if (current == Caterpillar) {
    return Normal;
  }
}


// WARNNING: Specify here the motor to be used for mecanum
/*
  Front
1       2
        
3       4
        
5       6
*/
// WARNNING: Connect A to the red conductor
// WARNNING: Connect B to the black conductor
Motor front_left_motor = Motor{ 26, 8 };
Motor front_right_motor = Motor{ 36, 2 };
Motor center_left_motor = Motor{ 29, 7 };
Motor center_right_motor = Motor{ 38, 3 };
Motor rear_left_motor = Motor{ 31, 6 };
Motor rear_right_motor = Motor{ 39, 5 };

Motor arm_pair1 = Motor{ 34, 45 };
Motor arm_pair2 = Motor{ 48, 46 };

Motor caterpillar_left_motor = Motor{ 27, 4 };
Motor caterpillar_right_motor = Motor{ 37, 44 };

Relay front_air_cylinder = Relay{ 32 };
Relay center_air_cylinder = Relay{ 35 };
Relay rear_air_cylinder = Relay{ 33 };

// Convert from degrees to radiansc:\Users\ロボ研\Documents\Arduino\hoge\hoge.ino
float degreeToRadian(int degree) {
  return (degree * 71) / 4068;
}


// Control motors (0~255)
void driveMotor(struct Motor motor, int power) {
  if (power > 0) {
    digitalWrite(motor.dir_pin, HIGH);
    analogWrite(motor.pwm_pin, abs(power));
  } else {
    digitalWrite(motor.dir_pin, LOW);
    analogWrite(motor.pwm_pin, abs(power));
  }
}


// Control air cylinders
void upRelay(ChoseRelay current) {
  switch (current) {
    case Front:
      digitalWrite(front_air_cylinder.pin, HIGH);
      break;
    case Center:
      digitalWrite(center_air_cylinder.pin, HIGH);
      break;
    case Rear:
      digitalWrite(rear_air_cylinder.pin, HIGH);
      break;
    default:
      break;
  }
}


void downRelay(ChoseRelay current) {
  switch (current) {
    case Front:
      digitalWrite(front_air_cylinder.pin, LOW);
      break;
    case Center:
      digitalWrite(center_air_cylinder.pin, LOW);
      break;
    case Rear:
      digitalWrite(rear_air_cylinder.pin, LOW);
      break;
    default:
      break;
  }
}


// Control mecanum from coordinate values
void moveMecanum(int Vx, int Vy) {
  int wheel1, wheel2, wheel3, wheel4, wheel5, wheel6;

  // Whether to rotate or not (to avoid moving while rotating)
  // Calculate for each mecanum wheel
  wheel1 = Vy - Vx;  // left front
  wheel2 = Vy + Vx;  // right front
  wheel5 = Vy + Vx;  // left back
  wheel6 = Vy - Vx;  // right back

  wheel3 = Vy;  // left center
  wheel4 = Vy;  // right center

  // Clip speed values to a range of -255 to 255 (prevents overflow)
  wheel1 = constrain(wheel1, -150, 150);
  wheel2 = constrain(wheel2, -150, 150);
  wheel3 = constrain(wheel3, -150, 150);
  wheel4 = constrain(wheel4, -150, 150);
  wheel5 = constrain(wheel5, -150, 150);
  wheel6 = constrain(wheel6, -150, 150);

  // Controls each motor
  driveMotor(front_left_motor, wheel1 * 0.5);
  driveMotor(front_right_motor, wheel2 * 0.5);
  driveMotor(center_left_motor, wheel3 * 0.5);
  driveMotor(center_right_motor, wheel4 * 0.5);
  driveMotor(rear_left_motor, wheel5 * 0.5);
  driveMotor(rear_right_motor, wheel6 * 0.5);
}


void rotateMecanum(int Vr) {
  Vr = Vr * 0.5;
  driveMotor(front_left_motor, Vr);
  driveMotor(front_right_motor, -Vr);
  driveMotor(center_left_motor, Vr);
  driveMotor(center_right_motor, -Vr);
  driveMotor(rear_left_motor, Vr);
  driveMotor(rear_right_motor, -Vr);
}

void changeControllerLED(ChoseRelay choseRelay) {
  switch (choseRelay) {
    case Front:
      PS4.setLed(Red);
      break;
    case Center:
      PS4.setLed(Blue);
      break;
    case Rear:
      PS4.setLed(Yellow);
      break;
    default:
      break;
  }
}


void setup() {
  Serial.begin(115200);

  // motor 1
  pinMode(front_left_motor.dir_pin, OUTPUT);
  pinMode(front_left_motor.pwm_pin, OUTPUT);
  // motor 2
  pinMode(front_right_motor.dir_pin, OUTPUT);
  pinMode(front_right_motor.pwm_pin, OUTPUT);

  // motor 3
  pinMode(center_left_motor.dir_pin, OUTPUT);
  pinMode(center_left_motor.pwm_pin, OUTPUT);

  // motor 4
  pinMode(center_right_motor.dir_pin, OUTPUT);
  pinMode(center_right_motor.pwm_pin, OUTPUT);

  // motor 5
  pinMode(rear_left_motor.dir_pin, OUTPUT);
  pinMode(rear_left_motor.pwm_pin, OUTPUT);

  // motor 6
  pinMode(rear_right_motor.dir_pin, OUTPUT);
  pinMode(rear_right_motor.pwm_pin, OUTPUT);

  // caterpillar left motor
  pinMode(caterpillar_left_motor.dir_pin, OUTPUT);
  pinMode(caterpillar_left_motor.pwm_pin, OUTPUT);
  
  // caterpillar right motor
  pinMode(caterpillar_right_motor.dir_pin, OUTPUT);
  pinMode(caterpillar_right_motor.pwm_pin, OUTPUT);

  // arm motor pair1
  pinMode(arm_pair1.dir_pin, OUTPUT);
  pinMode(arm_pair1.pwm_pin, OUTPUT);

  // arm motor pair2
  pinMode(arm_pair2.dir_pin, OUTPUT);
  pinMode(arm_pair2.pwm_pin, OUTPUT);

  // air cylinder
  pinMode(front_air_cylinder.pin, OUTPUT);
  pinMode(center_air_cylinder.pin, OUTPUT);
  pinMode(rear_air_cylinder.pin, OUTPUT);

#if !defined(__MIPSEL__)
  while (!Serial);  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1);  // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}

void loop() {
  
  Mode mode = Normal;
  ChoseRelay choseRelay = Front;

  while (true) {

    Usb.Task();
    if (PS4.connected()) {
      // Receive controller input
      int Vx = map(PS4.getAnalogHat(RightHatX), 0, 255, -255, 255);
      int Vy = map(PS4.getAnalogHat(RightHatY), 0, 255, -255, 255);
      int Vr = map(PS4.getAnalogHat(LeftHatX), 0, 255, -255, 255) * -1;

      int L2_value = PS4.getAnalogButton(L2);
      int R2_value = PS4.getAnalogButton(R2);

      changeControllerLED(choseRelay);      
        
      if (PS4.getButtonClick(TRIANGLE)) {
        choseRelay = GetNextEnumValue(choseRelay);
      }
      if (PS4.getButtonClick(CROSS)) {
        choseRelay = GetBeforeEnumValue(choseRelay);
      }

      if (PS4.getButtonClick(UP)) {
        upRelay(choseRelay);
      }
      if (PS4.getButtonClick(DOWN)) {
        downRelay(choseRelay);
      }

      if (mode == Normal) {
        if (L2_value > 30 || R2_value > 30) {
          if (L2_value > 30) {
            driveMotor(arm_pair1, L2_value * -1);
            driveMotor(arm_pair2, L2_value * -1);
          } else if (R2_value > 30) {
            driveMotor(arm_pair1, R2_value);
            driveMotor(arm_pair2, R2_value);
          }
        } else {
          driveMotor(arm_pair1, 0);
          driveMotor(arm_pair2, 0);
        }
      } else if (mode == Caterpillar) {
        if (PS4.getButtonPress(L1)) {
          driveMotor(caterpillar_left_motor, -255);
        } else {
          if (L2_value > 30) {
            driveMotor(caterpillar_left_motor, 255);
          } else {
            driveMotor(caterpillar_left_motor, 0);
          }
        }
        if (PS4.getButtonPress(R1)) {
          driveMotor(caterpillar_right_motor, 255);
        } else {
          if (R2_value > 30) {
            driveMotor(caterpillar_right_motor, 255);
          } else {
            driveMotor(caterpillar_right_motor, 0);
          }
        }
      }

      if (PS4.getButtonClick(OPTIONS)) {
        mode = ChangeMode(mode);
      }

      if (PS4.getButtonClick(SHARE) && choseRelay == Front) {
        PS4.setLedFlash(10, 10);
        while (true) {
          Usb.Task();
          if (PS4.getButtonClick(UP)) {
            digitalWrite(center_air_cylinder.pin, LOW);
            digitalWrite(rear_air_cylinder.pin, LOW);

            while (true) {
              Usb.Task();
              if (PS4.getButtonPress(L1)) {
                driveMotor(caterpillar_left_motor, -255);
              } else {
                if (L2_value > 30) {
                  driveMotor(caterpillar_left_motor, 255);
                } else {
                  driveMotor(caterpillar_left_motor, 0);
                }
              }
              if (PS4.getButtonPress(R1)) {
                driveMotor(caterpillar_right_motor, -255);
              } else {
                if (R2_value > 30) {
                  driveMotor(caterpillar_right_motor, 255);
                } else {
                  driveMotor(caterpillar_right_motor, 0);
                }
              }
              if (PS4.getButtonClick(DOWN)) {
                digitalWrite(front_air_cylinder.pin, HIGH);
                digitalWrite(center_air_cylinder.pin, HIGH);
                digitalWrite(rear_air_cylinder.pin, HIGH);
                break;
              }
            }
            PS4.setLedFlash(0, 0);
            break;
          }

          
          
          if (PS4.getButtonClick(SHARE)) {
            PS4.setLedFlash(0, 0);
            break;
          }
        }
      }

      if (PS4.getButtonPress(SQUARE) && PS4.getButtonPress(CROSS)) {
        digitalWrite(front_air_cylinder.pin, LOW);
        digitalWrite(center_air_cylinder.pin, LOW);
        digitalWrite(rear_air_cylinder.pin, LOW);
      }

      if (PS4.getButtonPress(TRIANGLE) && PS4.getButtonPress(CIRCLE)) {
        digitalWrite(front_air_cylinder.pin, HIGH);
        digitalWrite(center_air_cylinder.pin, HIGH);
        digitalWrite(rear_air_cylinder.pin, HIGH);
      }

      if (PS4.getButtonClick(PS)) {
        driveMotor(front_left_motor, 0);
        driveMotor(front_right_motor, 0);
        driveMotor(center_left_motor, 0);
        driveMotor(center_right_motor, 0);
        driveMotor(rear_left_motor, 0);
        driveMotor(rear_right_motor, 0);
        driveMotor(arm_pair1, 0);
        driveMotor(arm_pair2, 0);
        driveMotor(caterpillar_left_motor, 0);
        driveMotor(caterpillar_right_motor, 0);
        PS4.disconnect();
        continue;
      }

      // If the input is too small, ignore it(Input range is -30 to 30)
      if (abs(Vx) > 30 || abs(Vy) > 30) {
        moveMecanum(Vx, Vy);
      } else if (abs(Vr) > 30) {
        rotateMecanum(Vr);
      } else {
        driveMotor(front_left_motor, 0);
        driveMotor(front_right_motor, 0);
        driveMotor(center_left_motor, 0);
        driveMotor(center_right_motor, 0);
        driveMotor(rear_left_motor, 0);
        driveMotor(rear_right_motor, 0);
      }
    }
  }
}
