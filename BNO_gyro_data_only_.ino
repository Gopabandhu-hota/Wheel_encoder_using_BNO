#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include<SoftwareSerial.h>
SoftwareSerial BT(2,3);
/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();
float rpm = 0;
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  BT.begin(9600);
  BT.println("Orientation Sensor Raw Data Test"); BT.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    BT.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  BT.print("Current Temperature: ");
  BT.print(temp);
  BT.println(" C");
  BT.println("");

  bno.setExtCrystalUse(true);

  BT.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* Display the floating point data */
  BT.print("X:rpm  ");
  rpm = euler.x()*60/(2*PI);
  BT.println(rpm );
  
  /*BT.print(" Y: ");
  BT.print(euler.y());
  BT.print(" Z: ");
  BT.print(euler.z());
  BT.print("\t\t");*/

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  BT.print("qW: ");
  BT.print(quat.w(), 4);
  BT.print(" qX: ");
  BT.print(quat.y(), 4);
  BT.print(" qY: ");
  BT.print(quat.x(), 4);
  BT.print(" qZ: ");
  BT.print(quat.z(), 4);
  BT.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /*BT.print("CALIBRATION: Sys=");
  BT.print(system, DEC);
  BT.print(" Gyro=");
  BT.print(gyro, DEC);
  BT.print(" Accel=");
  BT.print(accel, DEC);
  BT.print(" Mag=");
  BT.println(mag, DEC);*/

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
