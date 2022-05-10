#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <PolledTimeout.h>
#include "Adafruit_SGP40.h"
#include "Adafruit_SHT31.h"
#include <EEPROM.h>

//def
void sent_current_time(char, char, char);

//voc
Adafruit_SGP40 sgp;
Adafruit_SHT31 sht31;

const char *ssid     = "USC Guest Wireless";
const char *password = "";
// -7*60*60
const long utcOffsetInSeconds = -25200;

String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

// Text setup
String serverName = "http://textbelt.com/text/";
bool send_voc_alert = false;
bool alert_en = true;

const int16_t I2C_SLAVE = 0x00;

#define SDA_PIN 2
#define SCL_PIN 0

//eeprom
#define EEPROM_SIZE 512
int alert_count;
int ALERT_ADDR = 0;

struct alertData{ 
    char alert_time[9] = "";
    char alert_text[30] = "";
};

void setup(){
  Serial.println("/////////ESP3266 Program Start...////////////");
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin(SDA_PIN,SCL_PIN); // join i2c bus (address optional for master)
  
  Serial.println();

  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  //eeprom
  EEPROM.begin(EEPROM_SIZE);
  alertData ad;
  EEPROM.get(ALERT_ADDR, ad);
  Serial.println("Old values are: "+String(ad.alert_time)+" "+String(ad.alert_text));

//  Wire.beginTransmission(I2C_SLAVE); // transmit to device #0
//  Serial.println("Sending...");
//  Wire.write("Hello");              // sends one byte
//  Wire.endTransmission();    // stop transmitting

  timeClient.begin();

  //voc
  if (! sgp.begin()){
    Serial.println("SGP40 sensor not found :(");
    while (1);
  }

  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
  }

  Serial.print("Found SHT3x + SGP40 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
}

int voc_old = 0;
int counter=200;

void loop() {
  counter = counter+1;

  //voc
  uint16_t sraw;
  int32_t voc_index;
  
  float t = 15;
//  Serial.print("Temp *C = "); Serial.print(t); Serial.print("\t\t");
  float h = 70.3;

  sraw = sgp.measureRaw(t, h);
  Serial.print("Raw measurement: ");
  Serial.println(sraw);
  if(voc_old == 0){
    voc_old = sraw;  
  }

  voc_index = sgp.measureVocIndex(t, h);
  
  Serial.print("Voc Index: ");
  Serial.println(voc_index);

  if((voc_old-sraw) > 2000 && (counter >= 200)){ //no repeat send in 100*1 s
    send_voc_alert = true; 
    counter = 0;
  }

  voc_old = sraw;

  //ntp
  timeClient.update();

  String formattedTime = timeClient.getFormattedTime();

  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();
  int currentSecond = timeClient.getSeconds(); 
  String weekDay = weekDays[timeClient.getDay()];

  if((currentSecond%2) == 0){
//      Wire.beginTransmission(I2C_SLAVE); // transmit to device #0
//      Serial.println("Sending...");
//      Wire.write(day);        // sends five bytes
      char h;
      char m;
      char s;
      h = (char) currentHour;
      m = (char) currentMinute;
      s = (char) currentSecond;
//      Wire.write(h);              // sends one byte
//      Wire.write(m);
//      Wire.write(s);
//      Wire.endTransmission();    // stop transmitting
//      sent_current_time(h,m,s);
//      Serial.println(timeClient.getFormattedTime());
    }
    
    delay(1000);
    
    if(send_voc_alert && alert_en){
        if(WiFi.status()== WL_CONNECTED){
          Serial.println("Sending http...");
          WiFiClient client;
          HTTPClient http;
      
          // Your Domain name with URL path or IP address with path
          http.begin(client, serverName);
      
          http.addHeader("Content-Type", "application/json");
          int httpResponseCode = http.POST("{\"phone\":\"8607728539\",\"message\":\"Your trash can smells!\",\"key\":\"35d02c1aeb034e66961c872a36738590b8caf8d2R5GNhLLfZFeXgHJmiQ3Aifcka\"}");
          alertData ad;
          
          int n = formattedTime.length();
          strcpy(ad.alert_time, formattedTime.c_str());

          String text = "Your trash can smells!";
          strcpy(ad.alert_text, text.c_str());
          
          EEPROM.put(ALERT_ADDR, ad);
          EEPROM.commit(); 
          
          String payloadS = http.getString();
          Serial.print("HTTP Response code: ");
          Serial.println(httpResponseCode);
          Serial.println(payloadS);
        
      // Free resources
          http.end();
      }
      else {
        Serial.println("WiFi Disconnected");
      }
      send_voc_alert = false;

    }
}

void receive_response(){
   while (Serial.available()){
      Serial.read();
   }  
}

void sent_current_time(char h, char m, char s){
  while (Serial.available() <= 0) { //no data on bus
//    Serial.print(h); 
//    Serial.print(m);
    Serial.print(s);
  }  
}
