/*
   Author: jafema
   Date: 27/06/2021

   References:
   [1] Accessing accelerometer data on Nano 33 BLE
   https://docs.arduino.cc/tutorials/nano-33-ble/imu_accelerometer
   Warning: sample frecuency for Accelerometer and Gyroscope is 119 Hz according code from ref [3] and checked with

   [2] LSM9DS1 datasheet
   https://www.st.com/resource/en/datasheet/lsm9ds1.pdf

   [3] arduino-libraries/Arduino_LSM9DS1
   https://github.com/arduino-libraries/Arduino_LSM9DS1

   [4] Measure Tilt Angle Using MPU6050 Gyro/Accelerometer & Arduino
   https://how2electronics.com/measure-tilt-angle-mpu6050-arduino/

   [5] How to calculate the angle from sparkfun lsm9ds1 IMU sensor ?
   https://forum.arduino.cc/t/how-to-calculate-the-angle-from-sparkfun-lsm9ds1-imu-sensor/702695/5

   [6]kriswiner/LSM9DS1 ---> angle calculation at line 539
   https://github.com/kriswiner/LSM9DS1/blob/master/LSM9DS1_MS5611_BasicAHRS_t3.ino

   [7] How to Fuse Motion Sensor Data into AHRS Orientation (Euler/Quaternions)
   https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions/


   Sparkfun
   [8] LSM9DS1 Breakout Hookup Guide
   https://learn.sparkfun.com/tutorials/lsm9ds1-breakout-hookup-guide/all

   [9] SparkFun_LSM9DS1_Arduino_Library
   https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library

   ST
   ST Drivers ---> C-Driver-MEMS. Standard C platform-independent drivers for MEMS motion and environmental sensors
   https://www.st.com/content/st_com/en/products/embedded-software/mems-and-sensors-software/drivers-for-mems/c-driver-mems.html
   https://github.com/STMicroelectronics/
   https://github.com/STMicroelectronics/STMems_Standard_C_drivers

   lsm9ds1_STdC
   [10] https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm9ds1_STdC



   TODO
   How to Implement an Inertial Measurement Unit (IMU) Using an Accelerometer, Gyro, and Magnetometer
   https://www.youtube.com/watch?v=T9jXoG0QYIA

   TODO
   https://roboticsclubiitk.github.io/2017/12/21/Beginners-Guide-to-IMU.html
   http://www.starlino.com/dcm_tutorial.html


*/


#include <Arduino_LSM9DS1.h>
#include <Wire.h> // to include constant macro PI, RAD_TO_DEG
#include <math.h>

#define SERIAL_PLOT_GYRO 1
#define SERIAL_PLOT_ACC 0
#define MONITOR_SERIE 0
#define PLOT_ROLL_YAW_HEADING 0

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.


String output;

enum e_axis {e_X, e_Y, e_Z, E_COUNT_OF_AXIS};

void printAttitude(float ax, float ay, float az, float mx, float my, float mz);
void getRollPitchYaw(float ax, float ay, float az, float mx, float my, float mz);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initalize IMU!");
    while (1);
  }

  /* Gyroscope data */
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.print(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("GyX\tGyY\tGyZ");

  /* Accelerometer data*/
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("AcX\tAcY\tAcZ");

  /* Magnetometer data*/
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" uT");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");

}

void loop() {
  // put your main code here, to run repeatedly:
  float AcX, AcY, AcZ;
  float Gy[E_COUNT_OF_AXIS];
  float magnet[E_COUNT_OF_AXIS];

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Gy[e_X], Gy[e_Y], Gy[e_Z]);
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AcX, AcY, AcZ);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(magnet[e_X], magnet[e_Y], magnet[e_Z]);
  }



#if SERIAL_PLOT_GYRO
  Serial.print(Gy[e_X]);
  Serial.print('\t');
  Serial.print(Gy[e_Y]);
  Serial.print('\t');
  Serial.print(Gy[e_Z]);
  Serial.print('\t');
#endif

#if SERIAL_PLOT_ACC
  Serial.print(AcX);
  Serial.print('\t');
  Serial.print(AcY);
  Serial.print('\t');
  Serial.println(AcZ);
#endif

#if MONITOR_SERIE
  output = "Gyro data. GyX: " + String(Gy[e_X]) + "\tGyY: " + String(Gy[e_Y]) + "\tGyZ: " + String(Gy[e_Z]) + "\tdps";
  Serial.println(output);


  output = "Accelerometer data. AcX: " + String(AcX) + "\tAcY: " + String(AcY) + "\tAcZ: " + String(AcZ) + "\t G";
  Serial.println(output);
  /*
      output = "Angle data. AngX: " + String(AngX) + "\tAngY: " + String(AngY) + "\tAngZ: " + String(AngZ) + "\t degrees";
      Serial.println(output);
  */
  Serial.println();

#endif

  //printAttitude(AcX, AcY, AcZ,-magnet[e_Y], -magnet[e_X], magnet[e_Z]);

  getRollPitchYaw(AcX, AcY, AcZ, magnet[e_X], magnet[e_Y], magnet[e_Z]);

  Serial.println();


}



// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
#if MONITOR_SERIE
  Serial.print("Pitch (rotation over Y axis), Roll (rotation over X axis): ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading (yaw) (rotation over Z axis): "); Serial.println(heading, 2);
#endif

#if PLOT_ROLL_YAW_HEADING
  Serial.print(pitch, 2);
  Serial.print('\t');
  Serial.println(roll, 2);
  //Serial.print('\t');
  //Serial.println(heading, 2);
#endif
}

/*
Maths get from

https://roboticsclubiitk.github.io/2017/12/21/Beginners-Guide-to-IMU.html
*/

void getRollPitchYaw(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, sqrt(ax * ax + az * az)); // rotation over X
  float pitch = atan2(ax, sqrt(ay * ay + az * az)); // rotation over Y
  float yaw = atan2(-my, mx); // rotation over Z


  // Convert everything from radians to degrees:
  yaw *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

#if MONITOR_SERIE
  Serial.print("Roll (rotation over X axis), Pitch (rotation over Y axis): ");
  Serial.print(roll, 2);
  Serial.print(", ");
  Serial.println(pitch, 2);
  Serial.print("Yaw (rotation over Z axis): "); Serial.println(yaw, 2);
#endif

#if PLOT_ROLL_YAW_HEADING
  Serial.print(roll, 2);
  Serial.print('\t');
  Serial.print(pitch, 2);
  Serial.print('\t');
  Serial.println(yaw, 2);
#endif
}
