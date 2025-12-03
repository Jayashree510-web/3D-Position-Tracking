#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250_asukiaaa imu;
unsigned long startTime;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("MPU9250 Initialized");

#ifdef _ESP32_HAL_I2C_H_
  Wire.begin(SDA_PIN, SCL_PIN);
  imu.setWire(&Wire);
#endif

  imu.beginAccel();
  imu.beginGyro();
  imu.beginMag();

  startTime = millis(); // Record the start time
}

void loop() {
  imu.accelUpdate();
  imu.gyroUpdate();

  unsigned long elapsedTime = millis() - startTime;
  Serial.print(elapsedTime);
  Serial.print(",");

  // Apply offsets to acceleration values
  Serial.print(imu.accelX() - 0.01);
  Serial.print(",");
  Serial.print(imu.accelY() - 0.03);
  Serial.print(",");
  Serial.print(imu.accelZ() - 0.12);
  Serial.print(",");

  // Print gyro values
  Serial.print(imu.gyroX());
  Serial.print(",");
  Serial.print(imu.gyroY());
  Serial.print(",");
  Serial.println(imu.gyroZ());

  delay(10); // Small delay to prevent excessive serial output
}
