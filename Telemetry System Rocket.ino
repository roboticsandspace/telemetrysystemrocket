#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#define seaLevelPressure_hPa 1013.25

Adafruit_BMP085 bmp;
double previousaltitude = 0.17;
double currentaltitude = 0.17;
double velocity;
unsigned long previousMillis = 0;

int i=0;
float apogee;
float x,y,z;
float flat, flon;
    unsigned long age;
TinyGPS gps;
File myFile;
Servo myservo;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (!SD.begin(4)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  myservo.attach(6);
  myservo.write(90);
}

void loop() {
  unsigned long currentMillis = millis();  // store the current time
  delay(1000);
  Serial.print("Altitude = ");
  currentaltitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
  Serial.print(currentaltitude);
  Serial.println(" meters");
  // delay(1000);
  double time_sec = (currentMillis - previousMillis)/1000;
  previousMillis = currentMillis;
  velocity = (currentaltitude - previousaltitude) / time_sec;
  Serial.print("Velocity: ");
  Serial.print(velocity);
  Serial.println(" m/s");
  // previousaltitude = currentaltitude;
  
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;  
   for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      //Serial.print(c);
      if (gps.encode(c)) 
        newData = true;  
    }
  }

  if (newData)      //If newData is true
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);   
    Serial.print("Latitude = ");
    Serial.println(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print("Longitude = ");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);

  }
  x = ((analogRead(A0)*2.0)/1024.0)-1;  
  y = ((analogRead(A1)*2.0)/1024.0)-1;
  z = ((analogRead(A2)*2.0)/1024.0)-1;
  // x = analogRead(A0);  
  // y = analogRead(A1);
  // z = analogRead(A2);   
  Serial.print("gX = ");
  Serial.println(x);
  Serial.print("gY = ");
  Serial.println(y);
  Serial.print("gZ = ");
  Serial.println(z);
  delay(30);

  myFile = SD.open("telem.txt", FILE_WRITE);
   if (myFile) {
    // myFile.print();
    myFile.print("Altitude = ");
    myFile.print(currentaltitude);
    myFile.println(" meters");
    myFile.print("Velocity: ");
    myFile.print(velocity);
    myFile.println(" m/s");
    myFile.print("Latitude = ");
    myFile.println(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    myFile.print("Longitude = ");
    myFile.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    myFile.print("gX = ");
    myFile.println(x);
    myFile.print("gY = ");
    myFile.println(y);
    myFile.print("gZ = ");
    myFile.println(z);    
    // close the file:
    myFile.close();
    // Serial.println("load saved to SD card");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening telem.txt");
  }  
  if(previousaltitude>currentaltitude){
    i++;
    if(i==2){  

    apogee = previousaltitude;
   Serial.println("Maximum height reached");
  Serial.print("Apogee = ");
  Serial.print(apogee);
  Serial.println(" meters");
  myFile = SD.open("telem.txt", FILE_WRITE);
   if (myFile) {
  myFile.println("Maximum height reached");
  myFile.print("Apogee = ");
  myFile.print(apogee);
  myFile.println(" meters");     
  myFile.close();
    // Serial.println("load saved to SD card");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening telem.txt");
    }
    myservo.write(180);
    Serial.println("Parachute opened");
}
  }
previousaltitude = currentaltitude;  
}