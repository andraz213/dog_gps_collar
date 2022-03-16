#include "global_defines.h"
#include "sending_queue.h"
#include <arduino.h>




sending_list* first_item;
sending_list* last_item;
sending_list* free_pool;
int in_list = 0;
int free_size = 0;

boolean inited = false;


void init_sending_queue() {
  if(!inited){
    inited = psramInit();
  }
  if(inited){
    sending_list *pool;
    pool = (sending_list*)ps_malloc(sizeof(sending_list) * 64);

    for (int i = 0; i < 64; i++) {
      pool[i].next = free_pool;
      free_pool = &pool[i];
    }

  }

}



void need_free_items() {
  if(!inited){
    init_sending_queue();
  }
  if (ESP.getFreePsram() < 5000) {
    remove_first();
    Serial.println((unsigned long) first_item);
  } else {
    init_sending_queue();
  }
}


sending_list * get_one_free_item_and_add_to_queue() {
  if(!inited){
    init_sending_queue();
  }
  sending_list *new_item = free_pool;
  free_pool = (sending_list*)new_item->next;
  return new_item;
}


void put_item_into_queue(sending_list * new_item) {
  if(!inited){
    init_sending_queue();
  }
  if (!first_item) {
    first_item = new_item;
    last_item = new_item;
  } else {
    last_item->next = new_item;
    last_item = new_item;
  }

  last_item->next = (sending_list*)0;
}




void add_to_sending_queue(float speed, float altitude, float latitude, float longitude, float pressure, unsigned long time, int satellites, int rssi, int id) {
  if(!inited){
    init_sending_queue();
  }
  if (!free_pool) {
    need_free_items();
  }
  sending_list *new_item = get_one_free_item_and_add_to_queue();

  new_item->next = (sending_list*)0;
  new_item->speed = speed;
  new_item->altitude = altitude;
  new_item->latitude = latitude;
  new_item->longitude = longitude;
  new_item->time = time;
  new_item->satellites = satellites;
  new_item->rssi = rssi;
  new_item->id = id;
  new_item->pressure = pressure;
  put_item_into_queue(new_item);

  in_list++;
}



sending_list * get_first() {
  if(!inited){
    init_sending_queue();
  }
  return (sending_list *)first_item;
}



void remove_first() {
  if(!inited){
    init_sending_queue();
  }
  if (first_item) {
    in_list --;
    sending_list* second = (sending_list*)first_item->next;
    first_item->next = free_pool;
    free_pool = first_item;
    first_item = second;
  }
}
