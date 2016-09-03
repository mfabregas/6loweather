/*
 * Copyright (c) 2016, Marc Fabregas - mfabregas@zolertia.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup weather-station
 * @{
 *
 * \file
 * Weather station application
 *
 * \author
 *         Marc Fabregas <mfabregas@zolertia.com>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "mqtt.h"
#include "ip64-addr.h"
#include "dev/sys-ctrl.h"
#include "sixloweather.h"
#include "dev/leds.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
/*---------------------------------------------------------------------------*/
#define APP_STRING            "Weather station MQTT client"
#define VERSION_STRING        "v.0.1"
/*---------------------------------------------------------------------------*/
static uint8_t state;
#define STATE_INIT            0
#define STATE_CONNECTED       1
#define STATE_READY           2
#define STATE_DISCONNECTED    3
/*---------------------------------------------------------------------------*/
static uint8_t sensor_started = 0;
#if MQTT_MAX_BLOCK_TRIES
static uint8_t mqtt_workaround_if_block = 0;
#endif
/*---------------------------------------------------------------------------*/
static struct mqtt_connection conn;
static struct mqtt_message *msg_ptr = 0;
/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
process_event_t weather_station_config_event;
/*---------------------------------------------------------------------------*/
PROCESS_NAME(weather_station_process);
PROCESS(mqtt_client_process, "MQTT client process");
AUTOSTART_PROCESSES(&mqtt_client_process);
/*---------------------------------------------------------------------------*/
static void
pub_received(const char *topic, uint16_t topic_len, const uint8_t *chunk,
            uint16_t chunk_len)
{
  uint8_t aux[APP_PUB_RECV_SIZE];
  memset(aux, 0, APP_PUB_RECV_SIZE);

  PRINTF("MQTT: topic \"%s\" (%u), payload \"%s\" (%u)\n", topic, topic_len,
                                                           chunk, chunk_len);
  if(chunk_len > APP_PUB_RECV_SIZE) {
    return;
  }

  if(strncmp(topic, SUB_TOPIC, strlen(SUB_TOPIC)) == 0) {
    memcpy(aux, chunk, chunk_len);
    process_post(PROCESS_BROADCAST, weather_station_config_event, aux);
  }
}
/*---------------------------------------------------------------------------*/
static void
mqtt_client_publish(uint8_t *mqtt_string)
{
  mqtt_publish(&conn, NULL, PUB_TOPIC, mqtt_string,
               strlen((const char *)mqtt_string), MQTT_QOS_LEVEL_0,
               MQTT_RETAIN_OFF);
  PRINTF("MQTT: Publish %s (%u) to %s\n", mqtt_string, 
                                          strlen((const char *)mqtt_string),
                                          PUB_TOPIC);
}
/*---------------------------------------------------------------------------*/
static void
mqtt_event(struct mqtt_connection *m, mqtt_event_t event, void *data)
{
  switch(event) {
  case MQTT_EVENT_CONNECTED:
    PRINTF("MQTT: Connected to the Broker\n");
    state = STATE_CONNECTED;
    process_poll(&mqtt_client_process);
    break;

  case MQTT_EVENT_DISCONNECTED:
    PRINTF("MQTT: Disconnected, reattempting to connect again\n");
    state = STATE_DISCONNECTED;
    process_poll(&mqtt_client_process);
    break;

  case MQTT_EVENT_PUBLISH:
    msg_ptr = data;
    if(msg_ptr->first_chunk) {
      msg_ptr->first_chunk = 0;
      PRINTF("MQTT: Publish RX --> '%s'. (%i bytes)\n\n", msg_ptr->topic,
             msg_ptr->payload_length);
    }
    pub_received(msg_ptr->topic, strlen(msg_ptr->topic), msg_ptr->payload_chunk,
                msg_ptr->payload_length);
    break;

  case MQTT_EVENT_SUBACK:
  case MQTT_EVENT_UNSUBACK:
  case MQTT_EVENT_PUBACK:
  case MQTT_EVENT_ERROR:
  case MQTT_EVENT_PROTOCOL_ERROR:
  case MQTT_EVENT_CONNECTION_REFUSED_ERROR:
  case MQTT_EVENT_DNS_ERROR:
  case MQTT_EVENT_NOT_IMPLEMENTED_ERROR:
    PRINTF("MQTT: Event --> %i\n", event);
    break;

  default:
    PRINTF("MQTT: Unrecognized event %i\n", event);
    break;
  }
}
/*---------------------------------------------------------------------------*/
static void
state_machine(void)
{
  switch(state) {

  /* Attempt to connect to the broker
   */
  case STATE_INIT:
    PRINTF("MQTT: Attempting to connect\n");
    PRINTF("MQTT: Broker address %s (%u)\n", MQTT_BROKER_IP_ADDR,
                                             MQTT_BROKER_PORT);
    mqtt_connect(&conn, MQTT_BROKER_IP_ADDR, MQTT_BROKER_PORT,
                 DEFAULT_PUB_INTERVAL * 3);

    /* We wait for the MQTT callback event, either disconnection or a successful
     * connection to the broker
     */
    return;

  /* We wait until the connection is stable to attempt to subscribe to a topic */
  case STATE_CONNECTED:
    /* If the connection is ready subscribe, else wait a bit more */
    if(mqtt_ready(&conn) && conn.out_buffer_sent) {
      state = STATE_READY;

      PRINTF("MQTT: connection ready, attempt to subscribe to %s\n", SUB_TOPIC);
      mqtt_subscribe(&conn, NULL, SUB_TOPIC, MQTT_QOS_LEVEL_0);

      /* Start the weather station process */
      if(!sensor_started) {
        sensor_started = 1;
        process_start(&weather_station_process, NULL);
      }

      return;
    }

  case STATE_READY:
      /* Nothing to do here, we are now connected and ready */
      return;

  case STATE_DISCONNECTED:
    mqtt_disconnect(&conn);
    PRINTF("MQTT: Disconnected\n");
    state = STATE_INIT;
    break;

  default:
    return;
  }

  /* Schedule a timer to poll the MQTT process below and trigger the state
   * machine again
   */
  etimer_set(&et, CLOCK_SECOND);
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      if (state == ADDR_TENTATIVE) {
	    uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mqtt_client_process, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  printf("*--------------------------------------*\n");
  printf("%s %s\n", APP_STRING, VERSION_STRING);
  printf("*--------------------------------------*\n");

#if WITH_RELAYR_PLATFORM
  printf("Relay MQTT broker, ");
#else
  printf("Mosquitto MQTT broker, ");
#endif

#if WITH_LLSEC_ENABLED
  printf("LLSEC enabled\n");
#else
  printf("LLSEC disabled\n");
#endif

  /* Bootstrap until we join the DODAG */
  PRINTF("Connecting to the wireless network ");
  while(1) {
    if(uip_ds6_get_global(ADDR_PREFERRED) != NULL) {
      PRINTF("\n");
      print_local_addresses();
      break;

    } else {
      PRINTF(".");
      etimer_set(&et, CLOCK_SECOND >> 1);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }
  }

  /* Event to be sent upon receiving a new configuration value for the sensors */
  weather_station_config_event = process_alloc_event();

  /* MQTT process starts here, we create a MQTT client session and kick the
   * state machine to connect to the broker.  The maximum TCP segment size is 32
   * bytes
   */
  mqtt_register(&conn, &mqtt_client_process, MQTT_CLIENT_ID, mqtt_event, 32);

#if WITH_RELAYR_PLATFORM
  mqtt_set_username_password(&conn, MQTT_AUTH_USER, MQTT_AUTH_TOKEN);
#endif

  /* Do not auto-reconnect as we will trigger this from a disconnect event */
  conn.auto_reconnect = 0;

  state = STATE_INIT;
  state_machine();

  /* A timer scheduled by the state machine in cases for which we need to make a
   * new attempt to connect, etc
   */
  while(1) {
    PROCESS_YIELD();
    if((ev == PROCESS_EVENT_TIMER && data == &et) ||
       (ev == PROCESS_EVENT_POLL)) {
      state_machine();
    }

    /* A poll from the weather sensors will trigger a new publish */
    if((ev == weather_station_data_event) && (data != NULL)) {
      if((mqtt_ready(&conn) && conn.out_buffer_sent) && (state == STATE_READY)) {
#if MQTT_MAX_BLOCK_TRIES
        mqtt_workaround_if_block = 0;
#endif
        mqtt_client_publish((uint8_t *)data);
      } else {

        /* Avoid to trigger a new message and wait for TCP to either ACK the
         * packet after retries, or to timeout and notify us.
         */
        PRINTF("MQTT: Still publishing (MQTT state=%d, q=%u)\n",
                                                      conn.state,
                                                      conn.out_queue_full);
#if MQTT_MAX_BLOCK_TRIES
        mqtt_workaround_if_block++;
        if((MQTT_MAX_BLOCK_TRIES > 0) &&
          (mqtt_workaround_if_block > MQTT_MAX_BLOCK_TRIES)) {
          sys_ctrl_reset();
        }
#endif
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */
