//  ==============================================================================
//  PATHOS (Pressure, Acceleration, Temperature, Humidity, and Orentation Sensors)
//  ==============================================================================

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_SHTC3.h>
#include <Adafruit_LIS3DH.h>

#define SEALEVELPRESSURE_HPA (1013.25)  //Altitude in meters

const int chipSelect = 4; //pin micro SD is hooked up to on m0 feather

Adafruit_BMP3XX bmp; // definition for BMP390

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3(); // definition for SHTC3

Adafruit_LIS3DH lis = Adafruit_LIS3DH(); // definition for LIS3DH

File dataFile;

//     /------------------------------------\
//     | SHTC3 = Temperature & Pressure     |
//     | BMP390 = Humidity & Temperature    |
//     | LIS3DH = Acceleration & Orentation |
//     \------------------------------------/

///////////////////////////////////////////////////////DIVIDER////////////////////////////////////////////////////////

void setup() {

  pinMode(LED_BUILTIN, OUTPUT); // define onboard led

  Serial.begin(115200); // or 9600?

  Serial.println("Test for SHTC3 sensor, BMP3 sensor, and LIS3DH sensor");

// ___BMP390___ //
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a BMP3 sensor, check wiring!");
    while (1); delay(1);
  }
  Serial.println("Found BMP3 sensor");

// ___SHTC3___ //
  if (! shtc3.begin()) {
    Serial.println("Couldn't find SHTC3 sensor, check wiring!");
    while (1) delay(1);
  }
  Serial.println("Found SHTC3 sensor");

// ___LIS3DH___ //
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
  Serial.println("Could not find a LIS3DH sensor, check wiring!");
  while (1) yield();
  }
  Serial.println("Found LIS3DH sensor");
  
  // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G
  Serial.print("Range = "); Serial.print(2 << lis.getRange()); Serial.println("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;
  }
  
  Serial.println();

  // __SD card initialization__ //
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SS, OUTPUT);
  
    if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    while (1) ;
  }
  Serial.println("card initialized.");

  //make new file name if there is a duplicate
    char filename[15]; // create an array with only 15 chars in it
  strcpy(filename, "DATALOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[7] = '0' + i/10;
    filename[8] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  dataFile = SD.open(filename, FILE_WRITE);
  if( ! dataFile ) {
    Serial.print("Couldn't create ");
    Serial.println(filename);
  }
  else {
    Serial.print("Writing to ");
    Serial.println(filename);
    dataFile.println("Temperature #1 (F),Pressure (psi),Altitude (ft),Temperature (F),Humidity (rH),X,Y,Z,X accel,Y accel,Z accel"); //Write the first row of the data/excel file
  }
}

///////////////////////////////////////////////////////DIVIDER////////////////////////////////////////////////////////

void loop() {
//  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile){
    /*
    // ___BMP390___ //
      // temperature from BMP390
      Serial.print("Temperature: ");
      Serial.println((bmp.temperature*1.8)+32);
      //Serial.println(" *F");
    
      // pressure from BMP390
      Serial.print("Pressure: "); 
      Serial.println((bmp.pressure / 100.0)*0.0145037738); 
      //Serial.println(" psi");
    
      // altitude from BMP390
      Serial.print("Approx-Altitude: ");
      Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA)/0.3048);
      //Serial.println(" ft");
    
    // ___SHTC3___ //
    */
      sensors_event_t humidity, temp; // idk what this is
      shtc3.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    
    /*
      // temperature from SHTC3
      Serial.print("Temperature: "); 
      Serial.println((temp.temperature*1.8)+32); 
      //Serial.println(" *F");
    
      // humidity from SHTC3
      Serial.print("Humidity: "); 
      Serial.println(humidity.relative_humidity); 
      //Serial.println("% rH");
    
    // ___LIS3DH___ //
      lis.read();      // get X Y and Z data at once
      
      // Then print out the raw data
      Serial.print("X:  "); Serial.print(lis.x);
      Serial.print("\tY:  "); Serial.print(lis.y);
      Serial.print("\tZ:  "); Serial.println(lis.z);
    
      // Or....get a new sensor event, normalized
    */
      sensors_event_t event;
      lis.getEvent(&event);
    /*
      // Display the results (acceleration is measured in m/s^2)
      Serial.print("X accel: "); Serial.print(event.acceleration.x);
      Serial.print("   Y accel: "); Serial.print(event.acceleration.y);
      Serial.print("  Z accel: "); Serial.print(event.acceleration.z);
      Serial.println();
      delay(100);
    
      Serial.println();
    */
    
    // __SD card loging__ //
    dataFile.write((bmp.temperature*1.8)+32);
    dataFile.write(",");
    dataFile.write((bmp.pressure / 100.0)*0.0145037738);
    dataFile.write(",");
    dataFile.write(bmp.readAltitude(SEALEVELPRESSURE_HPA)/0.3048);
    dataFile.write(",");
    dataFile.write((temp.temperature*1.8)+32);
    dataFile.write(",");
    dataFile.write(humidity.relative_humidity);
    dataFile.write(",");
    
    dataFile.write(lis.x);
    dataFile.write(",");
    dataFile.write(lis.y);
    dataFile.write(",");
    dataFile.write(lis.z);
    dataFile.write(",");
    dataFile.write(event.acceleration.x);
    dataFile.write(",");
    dataFile.write(event.acceleration.y);
    dataFile.write(",");
    dataFile.write(event.acceleration.z);
    dataFile.println();
    dataFile.flush(); // save all data
    delay(10000); // delay #/100 seconds
    }
  else {
   Serial.println("SD card writing failed");
   //flash red led slow
   digitalWrite(LED_BUILTIN, LOW);
   delay(1000);
   digitalWrite(LED_BUILTIN, HIGH);
   delay(1000);
  }
}
