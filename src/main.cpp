#include <Arduino.h>
#include <math.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

const int pinTx = 5;
const int pinRx = 6;

const float maxTilt = 30.0;
const float deadZone = 3.0;
const float smoothing = 0.25;

const unsigned long sendInterval = 20;
const unsigned long sensorTimeout = 250;

float filteredX = 0;
float filteredY = 0;

bool haveReading = false;

unsigned long lastSensorTime = 0;
unsigned long lastSendTime = 0;

bool setReports() {
  return bno08x.enableReport(SH2_ACCELEROMETER, 20000);
}

int tiltToJoystick(float angle) {
  float magnitude = fabsf(angle);

  if (magnitude <= deadZone) {
    return 0;
  }

  float output =
    (magnitude - deadZone) / (maxTilt - deadZone) * 127.0;

  output = constrain(output, 0.0f, 127.0f);

  if (angle < 0) {
    output = -output;
  }

  return (int)roundf(output);
}

void sendJoystick(int x, int y) {
  char message[20];

  int length = snprintf(
    message,
    sizeof(message),
    "%d,%d\n",
    x,
    y
  );

  Serial1.write((const uint8_t *)message, length);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, pinRx, pinTx);

  delay(1000);

  if (!bno08x.begin_I2C()) {
    if (Serial) {
      Serial.println("Failed to find BNO08x");
    }

    while (true) {
      sendJoystick(0, 0);
      delay(20);
    }
  }

  if (!setReports()) {
    if (Serial) {
      Serial.println("Could not enable accelerometer");
    }

    while (true) {
      sendJoystick(0, 0);
      delay(20);
    }
  }

  if (Serial) {
    Serial.println("Joystick ready");
  }
}

void loop() {
  if (bno08x.wasReset()) {
    haveReading = false;
    sendJoystick(0, 0);
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      float x = sensorValue.un.accelerometer.x;
      float y = sensorValue.un.accelerometer.y;
      float z = sensorValue.un.accelerometer.z;

      float tiltX =
        atan2f(x, sqrtf(y * y + z * z)) * RAD_TO_DEG;

      float tiltY =
        atan2f(y, sqrtf(x * x + z * z)) * RAD_TO_DEG;

      if (!haveReading) {
        filteredX = tiltX;
        filteredY = tiltY;
      } else {
        filteredX += smoothing * (tiltX - filteredX);
        filteredY += smoothing * (tiltY - filteredY);
      }

      haveReading = true;
      lastSensorTime = millis();
    }
  }

  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();

    int joystickX = 0;
    int joystickY = 0;

    if (haveReading &&
        millis() - lastSensorTime <= sensorTimeout) {
      joystickX = -tiltToJoystick(filteredX);
      joystickY = tiltToJoystick(filteredY);
    }

    sendJoystick(joystickX, joystickY);
  }
}