/*
ref:
https://docs.arduino.cc/tutorials/nano-33-ble/imu-accelerometer

IMU: LSM9DS1

ref: Libreria Arduino
https://github.com/arduino-libraries/Arduino_LSM9DS1

ref:
https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

ref
https://learn.sparkfun.com/tutorials/lsm9ds1-breakout-hookup-guide#lsm9ds1-overview

ref
https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions/


https://learn.adafruit.com/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout/overviewv
https://learn.adafruit.com/comparing-gyroscope-datasheetsvhttps://learn.adafruit.com/comparing-gyroscope-datasheets
https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration
https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration



https://github.com/sparkfun/LSM9DS1_Breakout/blob/master/Libraries/Arduino/examples/LSM9DS1_Basic_I2C/LSM9DS1_Basic_I2C.ino


ref:
How to Write your own Flight Controller Software — Part 5
Calculating Roll, Pitch and Yaw from IMU Rates
https://reefwing.medium.com/how-to-write-your-own-flight-controller-software-part-5-a59bf9ed8c69

*/
/*
  Arduino LSM9DS1 - Accelerometer Application

  This example reads the acceleration values as relative direction and degrees,
  from the LSM9DS1 sensor and prints them to the Serial Monitor or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE

  Created by Riccardo Rizzo

  Modified by Jose García
  27 Nov 2020

  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>
#include<Wire.h>

float x, y, z;
int degreesX = 0;
int degreesY = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
}

void loop() {

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

  }
  float roll = atan2(y, z);
  float pitch = atan2(-x, sqrt(y * y + z * z));

  // Convert everything from radians to degrees:
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  Serial.println("====================");
  Serial.print("Roll ");
  Serial.print(roll);
  Serial.println("  degrees");

  Serial.print("Pitch ");
  Serial.print(pitch);
  Serial.println("  degrees");
  

  if (x > 0.1) {
    Serial.println("---------------------");
    Serial.print("Tilting up ");
    Serial.print(x);
    Serial.println("  mg == mm/s^2");
    
    x = 100 * x;
    degreesX = map(x, 0, 97, 0, 90);
    Serial.print("Tilting up ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }

  
  if (x < -0.1) {
    Serial.println("---------------------");
    Serial.print("Tilting down ");
    Serial.print(x);
    Serial.println("  mg == mm/s^2");
    
    x = 100 * x;
    degreesX = map(x, 0, -100, 0, 90);
    Serial.print("Tilting down ");
    Serial.print(degreesX);
    Serial.println("  degrees");
  }

  
  if (y > 0.1) {
    Serial.println("++++++++++++++++++++++++");
    Serial.print("Tilting left ");
    Serial.print(y);
    Serial.println("  mg == mm/s^2");
    
    y = 100 * y;
    degreesY = map(y, 0, 97, 0, 90);

    Serial.print("Tilting left ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }
  if (y < -0.1) {
    Serial.println("++++++++++++++++++++++++");
    Serial.print("Tilting right ");
    Serial.print(y);
    Serial.println("  mg == mm/s^2");
    
    y = 100 * y;
    degreesY = map(y, 0, -100, 0, 90);
    Serial.print("Tilting right ");
    Serial.print(degreesY);
    Serial.println("  degrees");
  }


  
  delay(1000);
}
