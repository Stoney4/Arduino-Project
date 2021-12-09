//Near Space Arduino Project 
// By: Andrew Amidei

#include <SPI.h> // SPI storage protocol
#include <SD.h> // SD card library

#define cardSelect 4 // pin that microSD is plugged into

File dataFile;

#include <Adafruit_APDS9960.h> //For debug purposes- also needed for the proximity, color, and light sensor

//__Temperature and barometric pressure__//
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp280;
float temperature, pressure, altitude;
int altitudeZero = 0; // used to set the starting altitude to zero

//__Magnetometer (Compass)__//
#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;
float magnetic_x, magnetic_y, magnetic_z;

//__Accelerometer and gyroscope__//
#include <Adafruit_LSM6DS33.h>
Adafruit_LSM6DS33 lsm6ds33;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

//__Humidity__//
#include <Adafruit_SHT31.h>
Adafruit_SHT31 sht30;
float humidity;

const int recordDelay = 5000; // Time delay between readings in miliseconds
int runTime = 0; // Time in seconds the board has been running for

#define VBATPIN A6 //pin for measuring voltage

////////////////////////////////////////////////////////////////////////////////////////////////SETUP////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  digitalWrite(LED_BUILTIN, LOW); // Turn off led while in setup
  
  Serial.begin(115200);
  while(!Serial); // MUST comment out line or code will not log data, but need this for debug

  Serial.println("Hello World!");

  bmp280.begin();  // Temperature and barometric pressure
  lis3mdl.begin_I2C();  // Magnetomater (Compass)
  lsm6ds33.begin_I2C();  // Accelerometer and gyroscope
  sht30.begin();  // Humidity

  altitudeZero = bmp280.readAltitude(1013.25); // Read altitude and use current altitude as the ground, aka 0

  
  // SD card Setup
  pinMode(13, OUTPUT);

  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
  }
    else {
    // Create a new file name if there's alredy a file with the same name
    char filename[10];
    strcpy(filename, "LOG_00.TXT");
    for (int i = 0; i < 100; i++) {
      filename[4] = '0' + i/10;
      filename[5] = '0' + i%10;
      
      if (! SD.exists(filename)) {
        break;
      }
    }
  
    dataFile = SD.open(filename, FILE_WRITE);
    if( ! dataFile ) {
      Serial.print("Could not create "); Serial.println(filename);
    }
    else {
      Serial.print("Writing to "); Serial.println(filename);
    
      pinMode(13, OUTPUT);
      pinMode(8, OUTPUT);
      Serial.println("Ready!");
    }
    dataFile.println("Runtime(seconds),Battery(v),Temperature(F),Pressure(Pa),Altitude(m),Humidity(%),Compass Bearing(uTesla),Accelerometer(m/s^2),Gyroscope(dps)");
    
    digitalWrite(LED_BUILTIN, HIGH); // Turn on led after setup
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////LOOP////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  if (dataFile){
    Serial.println("___________________________"); // Just to add some space
  
    /*************************************************************************/
    //timekeeping
    runTime = runTime + (recordDelay/1000);
    
    Serial.print("Time since start: "); Serial.print(runTime - (recordDelay/1000)); Serial.println(" s"); // must also subtract (recordDelay/1000) seconds off to count for 0 seconds as the first value
    
    dataFile.print(runTime - (recordDelay/1000)); dataFile.print(",");
    /*************************************************************************/
  
    /*************************************************************************/
    // Arduino Battery voltage measure (Example from Adafruit)
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.6;  // Multiply by 3.6V, our reference voltage
    measuredvbat /= 1024; // Convert to voltage
    
    Serial.print("Battery: " ); Serial.print(measuredvbat); Serial.println("v");
    
    dataFile.print(measuredvbat); dataFile.print(",");
    /*************************************************************************/
  
  
    /*************************************************************************/
    // Temperature, barometric pressure, and altitude
    temperature = bmp280.readTemperature();
    pressure = bmp280.readPressure();
    altitude = bmp280.readAltitude(1013.25) - altitudeZero; // Subtract starting altitude so we know how far we are from the ground
  
    Serial.print("Temperature: "); Serial.print((temperature * 1.8) + 32); Serial.println(" F");
    Serial.print("Pressure: "); Serial.print(pressure); Serial.println("Pa");
    Serial.print("Altitude: "); Serial.print(altitude); Serial.println("m");
    
    dataFile.print((temperature * 1.8) + 32); dataFile.print(",");
    dataFile.print(pressure); dataFile.print(",");
    dataFile.print(altitude); dataFile.print(",");
    /*************************************************************************/
  
    
    /*************************************************************************/
    // Humidity
    humidity = sht30.readHumidity();
  
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println("%");
    
    dataFile.print(humidity); dataFile.print(",");
    /*************************************************************************/
  
  
    /*************************************************************************/
    // Magnetomater (Compass)
    lis3mdl.read();
    magnetic_x = lis3mdl.x;
    magnetic_y = lis3mdl.y;
    magnetic_z = lis3mdl.z;
  
    Serial.print("Compass Bearing(uTesla):"); Serial.print(" X:"); Serial.print(magnetic_x); Serial.print(" Y:"); Serial.print(magnetic_y); Serial.print(" Z:"); Serial.println(magnetic_z);
    
    dataFile.print(magnetic_x); dataFile.print(":"); dataFile.print(magnetic_y); dataFile.print(":"); dataFile.print(magnetic_z); dataFile.print(",");
    /*************************************************************************/
  
  
    /*************************************************************************/
    // Accelerometer and gyroscope
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lsm6ds33.getEvent(&accel, &gyro, &temp);
    accel_x = accel.acceleration.x;
    accel_y = accel.acceleration.y;
    accel_z = accel.acceleration.z;
    gyro_x = gyro.gyro.x;
    gyro_y = gyro.gyro.y;
    gyro_z = gyro.gyro.z;
  
    Serial.print("Accelerometer(m/s^2):"); Serial.print(" X:"); Serial.print(accel_x); Serial.print(" Y:"); Serial.print(accel_y); Serial.print(" Z:"); Serial.println(accel_z);
    Serial.print("Gyroscope(dps):"); Serial.print(" X:"); Serial.print(gyro_x); Serial.print(" Y:"); Serial.print(gyro_y); Serial.print(" Z:"); Serial.println(gyro_z);
    
    dataFile.print(accel_x); dataFile.print(":"); dataFile.print(accel_y); dataFile.print(":"); dataFile.print(accel_z); dataFile.print(",");
    dataFile.print(gyro_x); dataFile.print(":"); dataFile.print(gyro_y); dataFile.print(":"); dataFile.print(gyro_z); dataFile.print(",");
    /*************************************************************************/
  
    dataFile.println(); // New line after data write
    dataFile.flush(); // Save all data
    
    delay(recordDelay);
  }
  else {
      // If there is any problem the onboard LED will blink red
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }