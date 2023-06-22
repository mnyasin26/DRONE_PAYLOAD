#include <Arduino.h>

#include <NTPClient.h>
// change next line to use with another board/shield
#include <WiFi.h>
// #include <WiFi.h> // for WiFi shield
// #include <WiFi101.h> // for WiFi 101 shield or MKR1000
#include <WiFiUdp.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <Adafruit_INA219.h>
#include <TinyGPS++.h>

const char *ssid = "<SSID>";
const char *password = "<PASSWORD>";

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

unsigned long Clock;

SoftwareSerial GPSSerial;
SoftwareSerial OpenMVSerial;

TinyGPSPlus gps;

MPU6050 accelgyro;

Adafruit_INA219 ina219;

int16_t ax, ay, az;
int16_t gx, gy, gz;

String DataOpenMV = "0", timestamp, LAT, LON, ALT, SOG, COG, Humidity, Arus, Tegangan, Daya;
String ID = "DEV1";

int part1 = -1;
int part2 = -1;
int part3 = -1;
int part4 = -1;

String dataOpenMV[4] = {"a", "a", "a", "a"};
// int data1 = -1;
// int data2 = -1;
// int data3 = -1;
// int data4 = -1;
String tempOpenMV = "";
String notOpenMV = "-1,-1,-1,-1";

int counter = 0;

bool statusOpenMV = false;

void resetVal()
{
  part1 = -1;
  part2 = -1;
  part3 = -1;
  part4 = -1;
}

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial2.begin(9600);
  GPSSerial.begin(9600, SWSERIAL_8N1, 27, 26, false, 95, 11);
  OpenMVSerial.begin(9600, SWSERIAL_8N1, 15, 4, false, 95, 11);
  accelgyro.initialize();
  WiFi.begin(ssid, password);
  unsigned status;

  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  timeClient.begin();

  if (!ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
    while (1)
    {
      delay(10);
    }
  }
}

void loop()
{
  timeClient.update();

  if ((millis() - Clock) > 4000 && statusOpenMV == false)
  {
    OpenMVSerial.write('k');
  }
  
  if ((millis() - Clock) > 5000 && statusOpenMV == true)
  {
    Serial.println("MPU6050 : ");
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.println(String() + "AX =\t" + ax);
    Serial.println(String() + "AY =\t" + ay);
    Serial.println(String() + "AZ =\t" + az);
    Serial.println(String() + "GX =\t" + gx);
    Serial.println(String() + "GY =\t" + gy);
    Serial.println(String() + "GZ =\t" + gz);
    Serial.println();

    Serial.println(F("GPS Module : "));
    if (gps.location.isValid())
    {
      Serial.println(gps.location.lat(), 9);
      Serial.println(gps.location.lng(), 9);
      Serial.println(gps.altitude.meters(), 2);
      LAT = String(gps.location.lat(), 9);
      LON = String(gps.location.lng(), 9);
      SOG = String(gps.speed.kmph());
      COG = String(gps.course.deg());
    }
    else
    {
      LAT = String("0.000000000");
      LON = String("0.000000000");
      SOG = String("0");
      COG = String("0");
    }
    Serial.println("");
    Arus = ina219.getCurrent_mA();
    Tegangan = ina219.getBusVoltage_V();
    Daya = ina219.getPower_mW();
    timestamp = timeClient.getFormattedTime();
    ALT = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Humidity = bme.readHumidity();
    Serial.println(String() + "OpenMVData = " + DataOpenMV);
    Serial.println("");
    Serial.println(String() + "*" + timestamp + "," + LAT + "," + LON + "," + ALT + "," + SOG + "," + COG + "," + Humidity + "," + Arus + "," + Tegangan + "," + Daya + "," + tempOpenMV + "," + ax + "," + ay + "," + az + "," + gx + "," + gy + "," + gz + ",#");
    Serial2.println(String() + "*" + timestamp + "," + LAT + "," + LON + "," + ALT + "," + SOG + "," + COG + "," + Humidity + "," + Arus + "," + Tegangan + "," + Daya + "," + tempOpenMV + "," + ax + "," + ay + "," + az + "," + gx + "," + gy + "," + gz + ",#");

    Clock = millis();
    statusOpenMV = false;
    tempOpenMV = "";
  }

  while (GPSSerial.available() > 0)
  {
    gps.encode(GPSSerial.read());
    yield();
  }
  while (OpenMVSerial.available() > 0)
  {

    char c = OpenMVSerial.read();
    Serial.println("msg dari OpenMV: " + (String)c);
    if (c == '*')
    {
    }
    else if (c == '#')
    {
      statusOpenMV = true;
      counter = 0;
    }
    else
    {
      tempOpenMV += (String)c;
      // dataOpenMV[counter++] = (String)c;
    }

    Serial.print(dataOpenMV[0]);
    Serial.print(dataOpenMV[1]);
    Serial.print(dataOpenMV[2]);
    Serial.print(dataOpenMV[3]);
    Serial.println();

    yield();
  }
  while (Serial2.available() > 0)
  {
    char c = Serial2.read();
    Serial.write(c);
    OpenMVSerial.write(c);
    if (c == 'k')
    {
      /* code */
    }

    yield();
  }
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    GPSSerial.write(c);
    OpenMVSerial.write(c);
    yield();
  }
}
