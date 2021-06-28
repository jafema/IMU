/*
 * Author: jafema
 * Date: 27/06/2021
 * 
 * References:
 * [1] Accessing accelerometer data on Nano 33 BLE
 * https://docs.arduino.cc/tutorials/nano-33-ble/imu_accelerometer
 * Warning: sample frecuency for Accelerometer and Gyroscope is 119 Hz according code from ref [3] and checked with 
 * 
 * [2] LSM9DS1 datasheet
 * https://www.st.com/resource/en/datasheet/lsm9ds1.pdf
 * 
 * [3] arduino-libraries/Arduino_LSM9DS1
 * https://github.com/arduino-libraries/Arduino_LSM9DS1
 * 
 * [4] Measure Tilt Angle Using MPU6050 Gyro/Accelerometer & Arduino
 * https://how2electronics.com/measure-tilt-angle-mpu6050-arduino/
 * 
 * [5] How to calculate the angle from sparkfun lsm9ds1 IMU sensor ?
 * https://forum.arduino.cc/t/how-to-calculate-the-angle-from-sparkfun-lsm9ds1-imu-sensor/702695/5
 * 
 * [6]kriswiner/LSM9DS1 ---> angle calculation at line 539
 * https://github.com/kriswiner/LSM9DS1/blob/master/LSM9DS1_MS5611_BasicAHRS_t3.ino
 * 
 * [7] How to Fuse Motion Sensor Data into AHRS Orientation (Euler/Quaternions) 
 * https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions/
 * 
 *  
 * Sparkfun
 * [8] LSM9DS1 Breakout Hookup Guide
 * https://learn.sparkfun.com/tutorials/lsm9ds1-breakout-hookup-guide/all
 * 
 * [9] SparkFun_LSM9DS1_Arduino_Library
 * https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
 * 
 * ST
 * ST Drivers ---> C-Driver-MEMS. Standard C platform-independent drivers for MEMS motion and environmental sensors
 * https://www.st.com/content/st_com/en/products/embedded-software/mems-and-sensors-software/drivers-for-mems/c-driver-mems.html
 * https://github.com/STMicroelectronics/
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers
 * 
 * lsm9ds1_STdC
 * [10] https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm9ds1_STdC
 * 
*/
#include <Arduino_LSM9DS1.h>
#include <Wire.h> // to include constant macro PI, RAD_TO_DEG

#define SERIAL_PLOT 0
String output;

//int getAngle(float *x, float *y, float *z);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (!IMU.begin()){
    Serial.println("Failed to initalize IMU!");
    while(1);
  }

  /* Gyroscope data */
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.print(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("GyX\tGyY\tGyZ");

  /* Accelerometer */
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("AcX\tAcY\tAcZ");

}

void loop() {
  // put your main code here, to run repeatedly:
  float GyX, GyY, GyZ;
  float AcX, AcY,AcZ;
  float AngX, AngY,AngZ;

  if (IMU.gyroscopeAvailable()){
    IMU.readGyroscope(GyX, GyY, GyZ);
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AcX, AcY, AcZ);
  }


  int16_t x = map(AcX,-4,4,-90,90);
  int16_t y = map(AcY,-4,4,-90,90);
  int16_t z = map(AcZ,-4,4,-90,90);

  AngX = RAD_TO_DEG * (atan2(-y,-z)+PI);
  AngY = RAD_TO_DEG * (atan2(-x,-z)+PI);
  AngZ = RAD_TO_DEG * (atan2(-y,-x)+PI);
  
  

    
  #if SERIAL_PLOT
    Serial.print(GyX);
    Serial.print('\t');
    Serial.print(GyY);
    Serial.print('\t');
    Serial.print(GyZ);
    Serial.print('\t');
    
    Serial.print(AcX);
    Serial.print('\t');
    Serial.print(AcY);
    Serial.print('\t');
    Serial.println(AcZ);
  #else
  /*
    output = "Gyro data. GyX: " + String(GyX) + "\tGyY: " + String(GyY) + "\tGyZ: " + String(GyZ) + "\tdps";  
    Serial.println(output);
    */

    output = "Accelerometer data. AcX: " + String(AcX) + "\tAcY: " + String(AcY) + "\tAcZ: " + String(AcZ) + "\t G";  
    Serial.println(output);

    output = "Angle data. AngX: " + String(AngX) + "\tAngY: " + String(AngY) + "\tAngZ: " + String(AngZ) + "\t degrees";  
    Serial.println(output);
    
  #endif    
  

}

/*
int getAngle(float *x, float, *y, float *z)
{
  int ret = 0;
  
  x = map()


  return 
}
*/
