#include "BNO055_Naboris.h"


Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Accelerometer & gyroscope only for getting relative orientation, subject to gyro drift
// Adafruit_BNO055 bno = Adafruit_BNO055(0x08); // OPERATION_MODE_IMUPLUS

// Accelerometer & magnetometer only for getting relative orientation
// Adafruit_BNO055 bno = Adafruit_BNO055(0x0a);  // OPERATION_MODE_M4G

// Gets heading only from compass
// Adafruit_BNO055 bno = Adafruit_BNO055(0x09); // OPERATION_MODE_COMPASS

// OPERATION_MODE_NDOF without fast magnetometer calibration
// Adafruit_BNO055 bno = Adafruit_BNO055(OPERATION_MODE_NDOF_FMC_OFF);

imu::Quaternion quat;
imu::Vector<3> euler;
imu::Vector<3> mag;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> linaccel;


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
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

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
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
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}



/**************************************************************************/
/*
Initialize the BNO055
*/
/**************************************************************************/
void initIMU() {
    delay(1000);
    // Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    delay(500);

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    Serial.print("BNO055 sensor ID: "); Serial.println(sensor.sensor_id);
    Serial.print("EEPROM sensor ID: "); Serial.println(bnoID);

    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

    //Crystal must be configured AFTER loading calibration data into BNO055.
    bno.setExtCrystalUse(true);

    // sensors_event_t event;
    // bno.getEvent(&event);
    // if (foundCalib){
    //     Serial.println("Move sensor slightly to calibrate magnetometers");
    //     while (!bno.isFullyCalibrated())
    //     {
    //         bno.getEvent(&event);
    //         delay(BNO055_SAMPLERATE_DELAY_MS);
    //     }
    // }
    //  else
    //  {
    //      Serial.println("Please Calibrate Sensor: ");
    //      while (!bno.isFullyCalibrated())
    //      {
    //          bno.getEvent(&event);
    //
    //          Serial.print("X: ");
    //          Serial.print(event.orientation.x, 4);
    //          Serial.print("\tY: ");
    //          Serial.print(event.orientation.y, 4);
    //          Serial.print("\tZ: ");
    //          Serial.print(event.orientation.z, 4);
    //
    //          /* Optional: Display calibration status */
    //          displayCalStatus();
    //
    //          /* New line for the next sample */
    //          Serial.println("");
    //
    //          /* Wait the specified delay before requesting new data */
    //          delay(BNO055_SAMPLERATE_DELAY_MS);
    //      }
    //  }

    // Serial.println("\nFully calibrated!");
    // Serial.println("--------------------------------");
    // Serial.println("Calibration Results: ");
    // adafruit_bno055_offsets_t newCalib;
    // bno.getSensorOffsets(newCalib);
    // displaySensorOffsets(newCalib);

    //  Serial.println("\n\nStoring calibration data to EEPROM...");
    //
    //  eeAddress = 0;
    //  bno.getSensor(&sensor);
    //  bnoID = sensor.sensor_id;
    //
    //  EEPROM.put(eeAddress, bnoID);
    //
    //  eeAddress += sizeof(long);
    //  EEPROM.put(eeAddress, newCalib);
    //  Serial.println("Data stored to EEPROM.");
    //
    //  Serial.println("\n--------------------------------\n");
     // delay(500);
}

float qw, qx, qy, qz;
float ex, ey, ez;
float mx, my, mz;
float gx, gy, gz;
float ax, ay, az;
float lx, ly, lz;
uint8_t sys_stat, gyro_stat, accel_stat, mag_stat = 0;

#ifdef INCLUDE_FILTERED_DATA
uint16_t imu_skip_counter = 0;
#endif

void updateIMU() {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2

    sensors_event_t event;
    bno.getEvent(&event);

    Serial.print("imu\tt");
    Serial.print(millis());

    linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    quat = bno.getQuat();
    bno.getCalibration(&sys_stat, &gyro_stat, &accel_stat, &mag_stat);
    //
    qw = quat.w();
    qx = quat.x();
    qy = quat.y();
    qz = quat.z();

    Serial.print("\tqw");
    Serial.print(qw, 4);

    Serial.print("\tqx");
    Serial.print(qx, 4);

    Serial.print("\tqy");
    Serial.print(qy, 4);

    Serial.print("\tqz");
    Serial.print(qz, 4);

    gx = gyro.x();
    gy = gyro.y();
    gz = gyro.z();

    Serial.print("\tgx");
    Serial.print(gx, 4);

    Serial.print("\tgy");
    Serial.print(gy, 4);

    Serial.print("\tgz");
    Serial.print(gz, 4);

    float lx = linaccel.x();
    float ly = linaccel.y();
    float lz = linaccel.z();

    Serial.print("\tlx");
    Serial.print(lx, 4);

    Serial.print("\tly");
    Serial.print(ly, 4);

    Serial.print("\tlz");
    Serial.print(lz, 4);

    /* Display calibration status for each sensor. */
    Serial.print("\tss");
    Serial.print(sys_stat, DEC);
    Serial.print("\tsg");
    Serial.print(gyro_stat, DEC);
    Serial.print("\tsa");
    Serial.print(accel_stat, DEC);
    Serial.print("\tsm");
    Serial.print(mag_stat, DEC);

    Serial.print('\n');

}
