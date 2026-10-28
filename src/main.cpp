#include <Arduino.h>
#include <math.h>
#include <Adafruit_BNO08x.h>
#include <USB.h>
#include <USBHIDGamepad.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

USBHIDGamepad Gamepad;

int pinTx = 5;
int pinRx = 6;

float maxTilt = 30.0;
float deadZone = 3.0;

void setReports() {
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
}

void sendJoystick(int x, int y) {
  String message = String(x) + "," + String(y) + "\n";

  Serial1.write(
    (const uint8_t *)message.c_str(),
    message.length()
  );
}

void setup() {
  USB.productName("Klask Joystick");
  Gamepad.begin();
  USB.begin();

  Serial.begin(115200);

  Serial1.setPins(pinRx, pinTx);
  Serial1.begin(9600);

  delay(2000);

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x");

    while (1) {
      delay(10);
    }
  }

  Serial.println("BNO08x found!");
  setReports();
}

void loop() {
  delay(20);

  if (bno08x.wasReset()) {
    Gamepad.leftStick(0, 0);
    sendJoystick(0, 0);
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  if (sensorValue.sensorId != SH2_ACCELEROMETER) {
    return;
  }

  float x = sensorValue.un.accelerometer.x;
  float y = sensorValue.un.accelerometer.y;
  float z = sensorValue.un.accelerometer.z;

  float tiltX = atan2(x, sqrt(y * y + z * z)) * RAD_TO_DEG;
  float tiltY = atan2(y, sqrt(x * x + z * z)) * RAD_TO_DEG;

  if (abs(tiltX) < deadZone) {
    tiltX = 0;
  }

  if (abs(tiltY) < deadZone) {
    tiltY = 0;
  }

  int joystickX = constrain(tiltX / maxTilt * 127, -127, 127);
  int joystickY = constrain(tiltY / maxTilt * 127, -127, 127);

  joystickX = -joystickX;

  Gamepad.leftStick(joystickX, joystickY);
  sendJoystick(joystickX, joystickY);

  Serial.print("X: ");
  Serial.print(joystickX);
  Serial.print("  Y: ");
  Serial.println(joystickY);
}