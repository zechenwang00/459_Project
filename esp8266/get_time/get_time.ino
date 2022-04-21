#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <PolledTimeout.h>

const char *ssid     = "USC Guest Wireless";
const char *password = "";
// 8*60*60
const long utcOffsetInSeconds = 28800;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

const int16_t I2C_MASTER = 0x42;
const int16_t I2C_SLAVE = 0x08;

#define SDA_PIN 2
#define SCL_PIN 0

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // join i2c bus (address optional for master)

  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  timeClient.begin();
}

byte x = 0;

void loop() {
  timeClient.update();

//  Serial.print(daysOfTheWeek[timeClient.getDay()]);
//  Serial.print(", ");
//  Serial.print(timeClient.getHours());
//  Serial.print(":");
//  Serial.print(timeClient.getMinutes());
//  Serial.print(":");
//  Serial.println(timeClient.getSeconds());
//  Serial.write(daysOfTheWeek[timeClient.getDay()]);
//  Serial.write(daysOfTheWeek[timeClient.getHours()]);
//  Serial.write(daysOfTheWeek[timeClient.getMinutes()]);
//  Serial.write(daysOfTheWeek[timeClient.getSeconds()]);
  //Serial.println(timeClient.getFormattedTime());
  using periodic = esp8266::polledTimeout::periodicMs;
  static periodic nextPing(1000);

  if (nextPing) {
    Wire.beginTransmission(I2C_SLAVE); // transmit to device #8
    Wire.write("x is ");        // sends five bytes
    Wire.write(x);              // sends one byte
    Wire.endTransmission();    // stop transmitting

    x++;
  }

  delay(10000);
}
