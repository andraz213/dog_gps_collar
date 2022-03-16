#include <WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include "sending_queue.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_pm.h"
#include "nvs_flash.h"

#include <Adafruit_NeoPixel.h>

#include <HardwareSerial.h>

HardwareSerial in(1);
#include <FuGPS.h>
bool gpsAlive = false;
bool toSendSignal = false;
FuGPS fuGPS(in);

Adafruit_NeoPixel pixels(1, 39, NEO_GRB + NEO_KHZ800);


// IP address to send UDP data to.
// it can be ip address of the server or
// a network broadcast address
// here is broadcast address
const char * udpAddress = "192.168.4.1";
const int udpPort = 2390;
uint8_t mac_address [6];

//create UDP instance
WiFiUDP udp;


void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  in.setRxBufferSize(512);
  in.begin(9600, SERIAL_8N1, 16, 5);
  fuGPS.sendCommand("$PCAS01,3*1F");
  delay(100);
  in.end();
  in.setRxBufferSize(512);
  in.begin(38400, SERIAL_8N1, 16, 5);
  in.setRxBufferSize(512);
  fuGPS.sendCommand("$PCAS02,200*1D");
  fuGPS.sendCommand("$PCAS03,1,0,0,0,1,0,0,0,0,0,0,0,0,*02");
  // wl_status_t begin(const char* ssid, const char *passphrase = NULL, int32_t channel = 0, const uint8_t* bssid = NULL, bool connect = true);
  // wl_status_t begin(char* ssid, char *passphrase = NULL, int32_t channel = 0, const uint8_t* bssid = NULL, bool connect = true);
  mac_address[0] = 0x4C;
  mac_address[1] = 0xEB;
  mac_address[2] = 0xD6;
  mac_address[3] = 0x4C;
  mac_address[4] = 0x7D;
  mac_address[5] = 0xF9;

  pinMode(38, OUTPUT);
  digitalWrite(38, HIGH);
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(100, 0, 0));

  pixels.show();

  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  WiFi.begin("test", "bakabakabaka", 6, mac_address);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  while (WiFi.status() != WL_CONNECTED && millis() < 10000) {
    delay(500);
    //Serial.print(".");
  }

  psramInit();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 100, 0));

  pixels.show();




}
long prev = 0;

int valrgb = 0;
void loop() {

  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(valrgb, 0, valrgb));

  pixels.show();

  delay(1);


  get_data();
  sendData();
  //receive response from server, it will be HELLO WORLD

  //Wait for 1 second
  //delay(1000);
}



int prev_second = 0;
float prev_longitude = 0;
float prev_latitude = 0;
float prev_speed = 0;
float speed_data [5];
int speed_data_ptr = 0;
float max_speed = 0.0;

long prev_data = 0;

long avg_prev [100];
int ptr_avg = 0;



void get_data() {
  while (fuGPS.read()) {}

  Serial.println(fuGPS.Speed * 1.852);
  if(prev_speed != fuGPS.Speed){
    Serial.println("speedspeed");
    Serial.println(fuGPS.Speed * 1.852);
    prev_speed = fuGPS.Speed;
    if(fuGPS.Speed > max_speed){
      max_speed = fuGPS.Speed;
      Serial.println("new max speed-------------");
    }
    speed_data[speed_data_ptr%5] = fuGPS.Speed;
    speed_data_ptr++;
    speed_data_ptr % 5;
    //Serial.println("speed");
  }
  if (fuGPS.Seconds != prev_second) {
    prev_second = fuGPS.Seconds;
    //Serial.print("Satellites: ");
    //Serial.println(fuGPS.Satellites);

    float pressure = 0.0;
    float speed = 0.0;
    float altitude = 0.0;
    float latitude = 0.0;
    float longitude = 0.0;
    int satellites = fuGPS.Satellites;
    int rssi = -255;
    int id = 0;
    if ((WiFi.status() == WL_CONNECTED)) {
      rssi = (int)WiFi.RSSI();
    }
    setTime((int) fuGPS.Hours, (int) fuGPS.Minutes, (int) fuGPS.Seconds, (int) fuGPS.Days, (int) fuGPS.Months, (int) fuGPS.Years);
    unsigned long time = now();

    if (fuGPS.hasFix() == true) {

      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(50, 50, 0));

      pixels.show();
      //Serial.println(fuGPS.Seconds);
      //Serial.println(fuGPS.Minutes);
      //Serial.println(fuGPS.Hours);


      pressure = max_speed * 1.852;

      float n_speed = 0.0;
      for(int i = 0; i < speed_data_ptr; i++){
        speed += speed_data[i];
        speed_data[i] = 0.0;
        n_speed += 1.0;
      }
      speed_data_ptr = 0;
      max_speed = 0.0;
      if(n_speed>0){
        speed /= n_speed;
      }

      //Serial.println("hitrost I guess");
      //Serial.println(speed);

      speed *= 1.852;
      altitude = fuGPS.Altitude;
      latitude = fuGPS.Latitude;
      longitude = fuGPS.Longitude;
      satellites = fuGPS.Satellites;
      rssi = -255;
      if ((WiFi.status() == WL_CONNECTED)) {
        rssi = (int)WiFi.RSSI();
      }

      Serial.print("sending-------------");
      Serial.print(speed);
      Serial.print(" ");
      Serial.print(pressure);
    }
    add_to_sending_queue(speed, altitude, latitude, longitude, pressure, time, satellites, rssi, id);
    toSendSignal = true;
    //Serial.print("PREVIOUS DATA THIS LONG AGO: ");
    //Serial.println(millis() - prev_data);
    avg_prev[ptr_avg] = millis() - prev_data;
    ptr_avg++;
    ptr_avg%=100;
    prev_data = millis();

    long avg = 0;
    for(int i = 0; i<100; i++){
      avg += avg_prev[i];
    }

    //Serial.print("AVERAGE DATA THIS LONG AGO: ");
    //Serial.println(avg / 100);

  }
}




long nextSend = 0;
long prevSend = 0;
void sendData() {

  if ( millis() - prevSend > nextSend || toSendSignal) {
    toSendSignal = false;
    sending_list * to_send = get_first();
    uint8_t buffer[50];

    long sending = millis();
    while (to_send != 0 && millis() - sending < 100) {
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 5, 0));

      pixels.show();
      //Serial.println(WiFi.RSSI());
      //Serial.print("Free psram: ");
      //Serial.println(ESP.getFreePsram());

      //Serial.println("sending");
      if (WiFi.status() != WL_CONNECTED) {
        //Serial.println("Not connected");
        prevSend = millis();
        nextSend = 10000;
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        WiFi.begin("test", "bakabakabaka", 6, mac_address);
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        return;
      }
      if (WiFi.status() == WL_CONNECTED) {

        // Wait for connection
        if (WiFi.status() == WL_CONNECTED) {
          //Serial.print("IP address: ");
          //Serial.println(WiFi.localIP());
        }
        long started = millis();
        udp.beginPacket(udpAddress, udpPort);
        udp.write((uint8_t *) to_send, sizeof(sending_list));
        udp.endPacket();

        memset(buffer, 0, 50);
        //processing incoming packet, must be called before reading the buffer

        int size = udp.parsePacket();
        while (size == 0 && millis() - started < 200) {
          size = udp.parsePacket();
        }
        if (size) {
          if (udp.read(buffer, 50) > 0) {
            //Serial.println(millis() - prev);
            prev = millis();
            //Serial.println(millis() - started);
            //Serial.print("Server to client: ");
            //Serial.println((char *)buffer);
            String answer = (char *)buffer;
            if (answer.compareTo("acknowledged") == 0) {
              pixels.clear();
              pixels.setPixelColor(0, pixels.Color(0, 0, 50));

              pixels.show();
              //Serial.println("OK");
              remove_first();
              to_send = get_first();
            } else {
              return;
            }
          }
        }
      }
    }
    prevSend = millis();
    nextSend = 200 + random(100);
  }
}
