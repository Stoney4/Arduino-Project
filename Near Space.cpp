//Near Space Arduino Project 
// By: Andrew Amidei

//#include <SPI.h> // SPI storage protocol
//#include <SD.h> // SD card library

#define cardSelect 10 // pin that microSD is plugged into

File dataFile;

//#include <Adafruit_APDS9960.h> //For debug purposes- also needed if you are using the proximity, color, and light sensor

//__Temperature and barometric pressure__//
//#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp280;
float temperature, pressure, altitude;
int altitudeZero = 0; // used to set the starting altitude to zero

//__Humidity__//
//#include <Adafruit_SHT31.h>
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
  //while(!Serial); // MUST comment out line or code will not continue, but need this for debug

  Serial.println("Hello World!");

  bmp280.begin();  // Temperature and barometric pressure
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
    dataFile.println("Runtime(seconds),Battery(v),Temperature(F),Pressure(Pa),Altitude(m),Humidity(%)");
    
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
    
    dataFile.println(humidity);
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
  