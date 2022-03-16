#ifndef SENDING_QUEUE_H
#define SENDING_QUEUE_H
#include <arduino.h>
#include "global_defines.h"
void add_to_sending_queue(float speed, float altitude, float latitude, float longitude, float pressure, unsigned long time, int satellites, int rssi, int id);
sending_list * get_first();
void remove_first();



#endif
