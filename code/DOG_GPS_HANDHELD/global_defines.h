#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H
#include <Arduino.h>


typedef struct  {
  void * next;
  float speed;
  float altitude;
  float pressure;
  float latitude;
  float longitude;
  unsigned long time;
  int satellites;
  int rssi;
  int id;
} sending_list;

typedef struct  {
  float speed;
  float altitude;
  float pressure;
  float latitude;
  float longitude;
  unsigned long time;
  int satellites;
  int rssi;
  int id;
} data_message;

typedef struct  {
  int message_type;
  double battery_voltage;
} telemetry_message;

typedef struct  {
  char mac[6];
} espnow_telemetry_message;

typedef struct  {
  int message_type;
  uint8_t mac[6];
} gateway_time_request;

typedef struct  {
  void * next;
  char mac[6];
  int len;
  int type;
  char* message;
} message_queue;

typedef struct  {
  int message_type;
  uint64_t time;
  int interval;
} gateway_time_sync;


enum message_types {
  GATEWAY_TIME,
  SENSOR_TIME,
  SENSOR_READING,
  SENOSR_TELEMETRY,
  TIME_REQUEST,
  ESPNOW_GATEWAY_TELEMETRY
};




























#endif
