#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include<SoftwareSerial.h>
SoftwareSerial BT(2, 3);
float recent_time = 0;
float last_time = 0;
float change_in_time = 0 ;
const int numReadings = 10;

float readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average_rpm = 0;                // the average

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
float ang_vel = 0;
float old_ang = 0;
float new_ang = 0;
float total_ang = 0;
float change_in_ang = 0;
float wheel_radius =  0.20;//in m
float distance_moved = 0 ;
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  /*BT.println("------------------------------------");
    BT.print  ("Sensor:       "); BT.println(sensor.name);
    BT.print  ("Driver Ver:   "); BT.println(sensor.version);
    BT.print  ("Unique ID:    "); BT.println(sensor.sensor_id);
    BT.print  ("Max Value:    "); BT.print(sensor.max_value); BT.println(" xxx");
    BT.print  ("Min Value:    "); BT.print(sensor.min_value); BT.println(" xxx");
    BT.print  ("Resolution:   "); BT.print(sensor.resolution); BT.println(" xxx");
    BT.println("------------------------------------");
    BT.println("");*/
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the BT Monitor */
  /*BT.println("");
    BT.print("System Status: 0x");
    BT.println(system_status, HEX);
    BT.print("Self Test:     0x");
    BT.println(self_test_results, HEX);
    BT.print("System Error:  0x");
    BT.println(system_error, HEX);
    BT.println("");*/
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  BT.print("\t");
  if (!system)
  {
    BT.print("! ");
  }

  /* Display the individual values */
  /*BT.print("Sys:");
    BT.print(system, DEC);
    BT.print(" G:");
    BT.print(gyro, DEC);
    BT.print(" A:");
    BT.print(accel, DEC);
    BT.print(" M:");
    BT.print(mag, DEC);*/
}
void get_ang_vel(void)
{
  if (new_ang != 0 )
  {
    if (old_ang != 0)
    {
      change_in_ang = new_ang - old_ang;
    }
    else change_in_ang = 0;
    if (change_in_ang > 180)
    {
      change_in_ang = change_in_ang - 360;
    }
    if (change_in_ang < -180)
    {
      change_in_ang = change_in_ang + 360;
    }
    total_ang = total_ang + (change_in_ang);
    //BT.print("total angle covered in degrees");BT.print(total_ang);
    //BT.println("");
    distance_moved = total_ang * 2 * PI * wheel_radius / 360; //calculate distance moved in metre
    //BT.print("total distance travelled");BT.print(distance_moved);
    //BT.println("");
    recent_time = millis();
    change_in_time = recent_time - last_time;
    ang_vel = change_in_ang * 1000 * 60  / (change_in_time * 360 );
    smoothing();
    old_ang = new_ang ;
    //BT.print("ang_speed in rpm"); BT.print(ang_vel);
    last_time = recent_time ;
  }
}
void smoothing()
{
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = ang_vel;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average_rpm = total / numReadings;
  // send it to the computer as ASCII digits
  BT.print("average_rpm: ");
  BT.println(average_rpm);
  

}
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  BT.begin(9600);
  BT.println("Orientation Sensor Test"); BT.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    BT.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  //BT.print("X: ");
  //BT.print(event.orientation.x, 4);
  new_ang = event.orientation.z ;

  //BT.print("\tY: ");
  //BT.print(event.orientation.y, 4);
  //BT.print("\tZ: ");
  //BT.print(event.orientation.z, 4);


  /* Optional: Display calibration status */
  displayCalStatus();
  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();
  BT.println("");
  get_ang_vel(); //finds angular velocity
  /* New line for the next sample */
  BT.println("");
  
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

}


