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
 * \addtogroup 6loweather
 * @{
 *
 * \file
 * 6LoWPAN Weather station application
 *
 * \author
 *         Marc Fabregas <mfabregas@zolertia.com>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/random.h"
#include "sys/etimer.h"
#include "dev/bmpx8x.h"
#include "dev/sht25.h"
#include "dev/weather-meter.h"
#include "sixloweather.h"
#include "mqtt-client.h"
#include "dev/leds.h"
#include "cfs/cfs.h"
#include "cfs/cfs-coffee.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
weather_station_t weather_sensor_values;
process_event_t weather_station_data_event;
weather_station_config_t ws_config;
/*---------------------------------------------------------------------------*/
static char *buf_ptr = NULL;
static char app_buffer[APP_BUFFER_SIZE];
/*---------------------------------------------------------------------------*/
PROCESS(weather_station_process, "6LoWeather process");
/*---------------------------------------------------------------------------*/
static void
pack_weather_into_mqtt_string(void)
{
  buf_ptr = app_buffer;
  snprintf(buf_ptr, APP_BUFFER_SIZE,
                 "["
                 "{\"meaning\":\"%s\",\"value\":%u},"
                 "{\"meaning\":\"%s\",\"value\":%u},"
                 "{\"meaning\":\"%s\",\"value\":%u.%u},"
                 "{\"meaning\":\"%s\",\"value\":%u},"
                 "{\"meaning\":\"%s\",\"value\":%u},"
                 "{\"meaning\":\"%s\",\"value\":%u},"
                 "{\"meaning\":\"%s\",\"value\":%u.%u},"
                 "{\"meaning\":\"%s\",\"value\":%u.%02u},"
                 "{\"meaning\":\"%s\",\"value\":%u.%02u}]", 

                  "counter",     weather_sensor_values.counter,
                  "rain_mm",     weather_sensor_values.rain_mm,
                  "wdir_int",    weather_sensor_values.wind_dir_avg_int / 10,
                                 weather_sensor_values.wind_dir_avg_int % 10,
                  "wspeed_avg",  weather_sensor_values.wind_speed_avg,
                  "wspeed_int",  weather_sensor_values.wind_speed_avg_int,
                  "wspeed_max",  weather_sensor_values.wind_speed_max,
                  "atm_press",   weather_sensor_values.atmospheric_pressure / 10,
                                 weather_sensor_values.atmospheric_pressure % 10,
                  "temperature", weather_sensor_values.temperature / 100,
                                 weather_sensor_values.temperature % 100,
                  "humidity",    weather_sensor_values.humidity / 100,
                                 weather_sensor_values.humidity % 100);

  // PRINTF("  > WS: stringify: %s\n", app_buffer);
}
/*---------------------------------------------------------------------------*/
static void
check_weather_values(void)
{
  /* Check if the I2C-based sensor values are OK */
  if(weather_sensor_values.atmospheric_pressure == BMPx8x_ERROR) {
    PRINTF("  > WS: *** BMPx8x failed, sending zero instead\n");
    weather_sensor_values.atmospheric_pressure = 0;
  }

  if((weather_sensor_values.temperature == SHT25_ERROR) ||
    (weather_sensor_values.humidity == SHT25_ERROR)) {
    PRINTF("  > WS: *** SHT25 failed, sending zero instead\n");
    weather_sensor_values.temperature = 0;
    weather_sensor_values.humidity = 0;
  }
}
/*---------------------------------------------------------------------------*/
static void
print_weather_values(void)
{
  PRINTF("  > WS: Pressure = %u.%u(hPa)\n",
         weather_sensor_values.atmospheric_pressure / 10,
         weather_sensor_values.atmospheric_pressure % 10);
  PRINTF("  > WS: Temperature %02d.%02d ºC, ",
          weather_sensor_values.temperature / 100,
          weather_sensor_values.temperature % 100);
  PRINTF("  > Humidity %02d.%02d RH\n", weather_sensor_values.humidity / 100,
                                    weather_sensor_values.humidity % 100);
  PRINTF("  > WS: Rain (ticks): %u, ", weather_sensor_values.rain_mm);
  PRINTF("  > WS: Wind direction ");
  PRINTF("%u.%01u deg avg\n", weather_sensor_values.wind_dir_avg_int / 10,
                              weather_sensor_values.wind_dir_avg_int % 10);
  PRINTF("  > WS: Wind speed ");
  PRINTF("%u m/h avg, %u m/h 2m avg, %u m/h max\n\n",
         weather_sensor_values.wind_speed_avg,
         weather_sensor_values.wind_speed_avg_int,
         weather_sensor_values.wind_speed_max);
}
/*---------------------------------------------------------------------------*/
static void
poll_sensors(void)
{
  weather_sensor_values.counter++;

  /* Poll the weather meter */
  weather_sensor_values.rain_mm = weather_meter.value(WEATHER_METER_RAIN_GAUGE);
  weather_sensor_values.wind_dir_avg_int = weather_meter.value(WEATHER_METER_WIND_VANE_AVG_X);
  weather_sensor_values.wind_speed_avg = weather_meter.value(WEATHER_METER_ANEMOMETER_AVG);
  weather_sensor_values.wind_speed_avg_int = weather_meter.value(WEATHER_METER_ANEMOMETER_AVG_X);
  weather_sensor_values.wind_speed_max = weather_meter.value(WEATHER_METER_ANEMOMETER_MAX);

  /* Poll the atmospheric pressure sensor */
  SENSORS_ACTIVATE(bmpx8x);
  weather_sensor_values.atmospheric_pressure = bmpx8x.value(BMPx8x_READ_PRESSURE);
  SENSORS_DEACTIVATE(bmpx8x);

  /* Poll the temperature and humidity sensor */
  SENSORS_ACTIVATE(sht25);
  weather_sensor_values.temperature = sht25.value(SHT25_VAL_TEMP);
  weather_sensor_values.humidity = sht25.value(SHT25_VAL_HUM);
  SENSORS_DEACTIVATE(sht25);

  check_weather_values();
  print_weather_values();
  pack_weather_into_mqtt_string();

  /* Post the event */
  process_post(PROCESS_BROADCAST, weather_station_data_event, buf_ptr);
}
/*---------------------------------------------------------------------------*/
static void
interval_read_handler(void)
{
  int fd;
  uint8_t buf[2];
  ws_config.interval = WEATHER_STATION_SENSOR_PERIOD;
  fd = cfs_open("WS_int", CFS_READ | CFS_WRITE);
  if(fd >= 0) {
    if(cfs_read(fd, &buf, 2) > 0) {
      ws_config.interval = (buf[1] << 8) + buf[0];
    }
    cfs_close(fd);
  }
  PRINTF("  > WS: interval %u\n", (uint16_t)(ws_config.interval / CLOCK_SECOND));
}
/*---------------------------------------------------------------------------*/
static void
interval_pub_handler(uint16_t interval)
{
  int fd;
  uint8_t buf[2];

  if((interval * CLOCK_SECOND) == ws_config.interval) {
    PRINTF("  > WS: same interval as before, ommiting\n");
    return;
  }

  PRINTF("  > WS: new interval (ticks): %u\n", interval);
  ws_config.interval = interval * CLOCK_SECOND;
  fd = cfs_open("WS_int", CFS_READ | CFS_WRITE);
  if(fd >= 0) {
    buf[0] = ((uint8_t *)&ws_config.interval)[0];
    buf[1] = ((uint8_t *)&ws_config.interval)[1];
    if(cfs_write(fd, &buf, 2) > 0) {
      PRINTF("  > WS: interval saved in flash\n");
    }
    cfs_close(fd);
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(weather_station_process, ev, data)
{
  static uint16_t rv;
  static struct etimer et;

  PROCESS_BEGIN();
  weather_station_data_event = process_alloc_event();

  /* Activate the sensors */
  SENSORS_ACTIVATE(weather_meter);

  /* Read configuration from flash */
  interval_read_handler();

  /* Start the periodic process */
  etimer_set(&et, ws_config.interval);

  weather_sensor_values.counter = 0;

  while(1) {

    PROCESS_YIELD();

    if(ev == weather_station_config_event) {
      uint8_t *rcv = (uint8_t *)data;
      PRINTF("  > Received over MQTT: %s\n", rcv);

      /* Configuracion options are given by a JSON string as follows:
       * {"name":"interval","value":300}
       * This will allow to extend to further options
       */
      if(strncmp((const char *)&rcv[9], MQTT_RELAYR_CMD_INTERVAL,
                 strlen(MQTT_RELAYR_CMD_INTERVAL)) == 0) {
        rv = atoi((const char *)&rcv[strlen(MQTT_RELAYR_CMD_INTERVAL) + 19]);

        if((rv >= WEATHER_STATION_WS_INTERVAL_MIN) &&
          (rv <= WEATHER_STATION_WS_INTERVAL_MAX)) {
          PRINTF("  > WS: New configuration over MQTT, restarting timer\n");
          etimer_stop(&et);
          interval_pub_handler(rv);
          etimer_set(&et, ws_config.interval);
        }
      } /* else, silently discard */
    }

    if(ev == PROCESS_EVENT_TIMER && data == &et) {
      poll_sensors();
      etimer_set(&et, ws_config.interval);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */
