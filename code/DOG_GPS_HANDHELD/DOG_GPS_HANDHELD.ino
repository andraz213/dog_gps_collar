#include <Arduino_MKRIoTCarrier.h>
#include <Arduino_OplaUI.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "global_defines.h"
#include <SD.h>
#include <TimeLib.h>
#include <Arduino.h>
#include "variant.h"
#include "wiring_private.h"
#include <FuGPS.h>
FuGPS fuGPS(Serial1);

MKRIoTCarrier carrier;
CycleWidgetsApp app;
Gauge1_Widget w0;
Gauge1_Widget w1;
Gauge1_Widget w2;
String_Widget w3;
Gauge2_Widget w4;
String_Widget w5;

float distance = 0;
int num = 0;
float speed = 0.0;
float maxspeed = 0.0;
float realmaxspeed = 0.0;
float altitude = 0.0;
String rssisatstring = "";
String filename_dog = "";
String filename_handheld = "";
bool started_writing = false;
long prev_write_time = 0;

float dog_altitude = 0.0;
float handheld_altitude = 0.0;
float prev_dog_altitude = 0.0;
float prev_handheld_altitude = 0.0;

float dirandaway = 0.0;

float latd = 0.0;
float lond = 0.0;
float lath = 0.0;
float lonh = 0.0;

float latb = 46.102959;
float lonb = 14.520718;


File myFile;

int status = WL_IDLE_STATUS;
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
int keyIndex = 0;            // your network key index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[256]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;

String altitude_string;


void setup(){
  pinPeripheral(A6, PIO_ANALOG);
  pinPeripheral(A5, PIO_ANALOG);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);




  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }


  status = WiFi.beginAP("test", "bakabakabaka", 6);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    //while (true);
  }
  Serial.println("Connected to WiFi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

  carrier.begin();
  app.begin(carrier);
  app.addWidget(w0);
  app.addWidget(w1);
  app.addWidget(w2);
  app.addWidget(w3);
  app.addWidget(w4);
  app.addWidget(w5);
  w0.setTitle("SPEED");
  w0.setSuffix(" km/h");
  w0.attachValue(speed);

  w2.setTitle("TOP SPEED");
  w2.setSuffix(" km/h");
  w2.attachValue(maxspeed);

  w1.setTitle("altitude");
  w1.setSuffix(" m");
  w1.attachValue(altitude);

  w3.setTitle("rssi & satt");
  w3.attachValue(rssisatstring);

  w5.setTitle("dog vs you altitude");
  w5.attachValue(altitude_string);

  w4.setTitle("direction & away");
  w4.attachValue(dirandaway);
  w4.setRange(0,360);




  myFile = SD.open("test1.txt");
  if (myFile) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  }

  pinMode(13, INPUT);

  Serial1.begin(115200);

}

long prevapp = 0;



void loop() {

  if(millis() - prevapp > 250){
    prevapp = millis();
    app.loop();
  }
  while (fuGPS.read()) {}
  write_handheld_sd();

  altitude_string = String(dog_altitude) + " " + String(handheld_altitude);

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {

    w4.setTitle(String(getDistanceFromLatLonInM(latd, lond, lath, lonh)).c_str());
    dirandaway = direction(latd, lond, lath, lonh);
    Serial.println(getDistanceFromLatLonInM(latd, lond, lath, lonh));
    Serial.println(direction(latd, lond, lath, lonh));
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }

    sending_list new_item;

    if(len == sizeof(sending_list)){
      memcpy(&new_item, (char*)packetBuffer, (sizeof(sending_list)));

      Serial.println("Contents:");
      Serial.println(new_item.speed);
      Serial.println(new_item.altitude);
      Serial.println(new_item.pressure);
      Serial.println(new_item.latitude * 10000);
      Serial.println(new_item.longitude * 10000);
      Serial.println(new_item.time);
      Serial.println(new_item.satellites);
      Serial.println(new_item.rssi);

      latd = new_item.latitude;
      lond = new_item.longitude;

      speed = new_item.pressure;
      altitude = new_item.altitude;
      num++;

      distance += (new_item.speed / 3.6);

      if(prev_dog_altitude > 1.0){
        float d = new_item.altitude - prev_dog_altitude;
        if(d > 0.0){
          dog_altitude += d;
        }
      }

      prev_dog_altitude = new_item.altitude;

      rssisatstring = String(new_item.rssi) + " " + String(new_item.satellites) + " " + String(distance / 1000.0) + "km " + String(num);

      if(new_item.satellites > 0){
        if(!started_writing){
          String filename = String(month(new_item.time)) + String(day(new_item.time)) + String(hour(new_item.time))+ String(minute(new_item.time));
          filename_dog = filename + "d.txt";
          filename_handheld = filename + "h.txt";
          started_writing = true;
        }

        if(prev_write_time != new_item.time){
          prev_write_time = new_item.time;
          Serial.print("WRITING TO: ");
          Serial.println(filename_dog);

          myFile = SD.open(filename_dog, FILE_WRITE);

          if (myFile) {

            String to_print = "";
            to_print += String(new_item.time);
            to_print += ",";
            to_print += String(new_item.longitude, 7);
            to_print += ",";
            to_print += String(new_item.latitude, 7);
            to_print += ",";
            to_print += String(new_item.altitude);
            to_print += ",";
            to_print += String(new_item.speed);


            myFile.println(to_print);
            // close the file:
            myFile.close();
            Serial.println(to_print);
            Serial.println(new_item.satellites);
            Serial.println("done.");
          }else{
            Serial.println("it failed");
          }
        }
      }

      if(speed > realmaxspeed){
        realmaxspeed = speed;
      }
      maxspeed = realmaxspeed;
    }


    Serial.println("Contents:");
    Serial.println(packetBuffer);

    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
}


unsigned long  prev_handheld = 0;

void write_handheld_sd(){

  setTime((int) fuGPS.Hours, (int) fuGPS.Minutes, (int) fuGPS.Seconds, (int) fuGPS.Days, (int) fuGPS.Months, (int) fuGPS.Years);
  unsigned long currenttime = now();

  if(currenttime != prev_handheld && started_writing){
    prev_handheld = currenttime;

    if(prev_handheld_altitude > 1.0){
      float d = fuGPS.Altitude - prev_handheld_altitude;
      if(d > 0.0){
        handheld_altitude += d;
      }
    }

    prev_handheld_altitude = fuGPS.Altitude;

    Serial.print("WRITING TO: ");
    Serial.println(filename_handheld);

    myFile = SD.open(filename_handheld, FILE_WRITE);

    if (myFile) {

      String to_print = "";
      to_print += String(currenttime);
      to_print += ",";
      to_print += String(fuGPS.Longitude, 7);
      to_print += ",";
      to_print += String(fuGPS.Latitude, 7);
      to_print += ",";
      to_print += String(fuGPS.Altitude);
      to_print += ",";
      to_print += String(fuGPS.Speed * 1.852);

      lath = fuGPS.Latitude;
      lonh = fuGPS.Longitude;
      myFile.println(to_print);
      // close the file:
      myFile.close();
      Serial.println(to_print);
      Serial.println(fuGPS.Satellites);
      Serial.println("done.");
    }else{
      Serial.println("it failed");
    }

  }

}



void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/*
Serial.println(direction(46.102959, 14.520718, 46.103015, 14.519875)); // left 270

Serial.println(direction(46.102959, 14.520718, 46.102520, 14.520707)); // down 180

Serial.println(direction(46.102959, 14.520718, 46.102884, 14.521454)); // right 90

Serial.println(direction(46.102959, 14.520718, 46.103375, 14.520697)); // up 0
*/

float direction(float lat1,float lon1, float lat2, float lon2){
  float dlon = abs(lon1 - lon2);
  float dlat = abs(lat1 - lat2);

  if(dlon == 0.0){
    dlon = 0.001;
  }
  if(dlat == 0.0){
    dlat = 0.001;
  }

  float rads = atan(dlon / dlat);
  Serial.println(rads * 57.29577951);
  float degs = 0.0;
  if(lat1 > lat2){
    if(lon1 < lon2){
      degs = 360.0 - rads * 57.29577951;
    }else{
      degs = rads * 57.29577951;
    }
  }else{
    if(lon1 < lon2){
      degs = 180.0 + rads * 57.29577951;
    }else{
      degs = 180.0 - rads * 57.29577951;
    }
  }

  return degs;
}



float getDistanceFromLatLonInM(float lat1,float lon1, float lat2, float lon2) {
  float R = 6371.0; // Radius of the earth in km
  float dLat = deg2rad(lat2-lat1);  // deg2rad below
  float dLon = deg2rad(lon2-lon1);
  float a =
    sin(dLat/2) * sin(dLat/2) +
    cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
    sin(dLon/2) * sin(dLon/2)
    ;
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  float d = R * c * 1000.0; // Distance in km
  return d;
}

float deg2rad(float deg) {
  return deg / 57.29577951;
}
