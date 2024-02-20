#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <DS3231.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#define gpsPort ssGPS

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
RTClib myRTC;
TinyGPSPlus tinyGPS;

#define GPS_BAUD 9600

#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX 17 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 16 // GPS RX, Arduino TX pin
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX);


float btemp;
float bpres;
float balti;
float mpuaccelx;
float mpuaccely;
float mpuaccelz;
float mpugyrox;
float mpugyroy;
float mpugyroz;
float neolat;
float neolong;
float neosats;


int counter = 0;

void setup() 
{
  Serial.begin(9600);
  gpsPort.begin(GPS_BAUD);
  Wire.begin();
  while (!Serial);

  Serial.println("LoRa Sender");
  LoRa.setPins(5,32,33);
  LoRa.setTxPower(20);
    
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  //LoRa.setSpreadingFactor(6);
  //LoRa.setSyncWord(0xF4);
  
  //#BMP280
  if (!bmp.begin()) 
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);

  //#MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
  
}

void loop() 
{
  if (bmp.takeForcedMeasurement()) 
  {
    btemp=bmp.readTemperature();
    bpres=bmp.readPressure();
    balti=bmp.readAltitude(1013.25); /* Adjusted to local forecast! */
  } 
  else {
    Serial.println("BMP Forced measurement failed!");
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  
  mpuaccelx=a.acceleration.x;
  mpuaccely=a.acceleration.y;
  mpuaccelz=a.acceleration.z;
  mpugyrox=g.gyro.x;
  mpugyroy=g.gyro.y;
  mpugyroz=g.gyro.z; 

  
  printGPSInfo();
  smartDelay(500);

  DateTime now = myRTC.now();
  String rtc=String(now.hour())+":"+String(now.minute())+":"+String(now.second());
  
  Serial.println(String(counter)+", "+rtc+", "+String(balti)+", "+String(btemp)+", "+String(neolat,6)+", "+String(neolong,6)+", "+String(bpres)+", "+
                 String(mpuaccelx)+", "+String(mpuaccely)+", "+String(mpuaccelz)+", "+String(mpugyrox)+", "+String(mpugyroy)+", "+String(mpugyroz)+", "+String(neosats));  
  
  LoRa.beginPacket();
  LoRa.print(String(counter)+", "+rtc+", "+String(balti)+", "+String(btemp)+", "+String(neolat,6)+", "+String(neolong,6)+", "+String(bpres)+", "+
             String(mpuaccelx)+", "+String(mpuaccely)+", "+String(mpuaccelz)+", "+String(mpugyrox)+", "+String(mpugyroy)+", "+String(mpugyroz)+", "+String(neosats)); 
  LoRa.endPacket();

  counter++;

  delay(100);
}

void printGPSInfo()
{
  neolat=tinyGPS.location.lat();
  neolong=tinyGPS.location.lng();
  neosats=tinyGPS.satellites.value();
}

// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples.
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}
