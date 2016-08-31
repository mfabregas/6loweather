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
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup zoul-examples
 * @{
 *
 * \defgroup 6loweather 6LoWPAN Weather station
 *
 * DIY Weather station based on the Zoul module
 * @{
 *
 * \file
 *  Configuration file for the Weather station
 *
 * \author
 *         Marc Fabregas <mfabregas@zolertia.com>
 */
/*---------------------------------------------------------------------------*/
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_
/*---------------------------------------------------------------------------*/

/* Default is LPM2, which is the lowest power mode the MCU retain control, as
 * PM3 requires an external interrupt.  In PM2 only 16KB RAM are available, if
 * running out of RAM, select as lowest allowed PM mode level 1
 */
#define LPM_CONF_MAX_PM                     2

/* Use fewer router entries to save RAM */
#define NBR_TABLE_CONF_MAX_NEIGHBORS        5
#define UIP_CONF_MAX_ROUTES                 5

/* In case we need to change the default 6LoWPAN prefix context */
#define UIP_CONF_DS6_DEFAULT_PREFIX         0xaaaa

/* Disable duty cycle (increases battery drain, for testing only) */
// #define NETSTACK_CONF_RDC                nullrdc_driver

/* Security related configuration */
#if WITH_LLSEC
#undef LLSEC802154_CONF_ENABLED
#define LLSEC802154_CONF_ENABLED            1

#undef LLSEC802154_CONF_SECURITY
#define LLSEC802154_CONF_SECURITY           1

#define NONCORESEC_CONF_SEC_LVL             7

#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER                noncoresec_framer
#undef NETSTACK_CONF_LLSEC
#define NETSTACK_CONF_LLSEC                 noncoresec_driver

#undef AES_128_CONF
#define AES_128_CONF                        aes_128_driver

#define LLSEC_ANTIREPLAY_ENABLED            0
#define LLSEC_REBOOT_WORKAROUND_ENABLED     1

#define NONCORESEC_CONF_KEY { 0x00 , 0x01 , 0x02 , 0x03 , \
                              0x04 , 0x05 , 0x06 , 0x07 , \
                              0x08 , 0x09 , 0x0A , 0x0B , \
                              0x0C , 0x0D , 0x0E , 0x0F }
#else
#undef NETSTACK_CONF_LLSEC
#define NETSTACK_CONF_LLSEC nullsec_driver
#endif

#define RPL_CONF_WITH_DAO_ACK               1
#define RPL_CONF_RPL_REPAIR_ON_DAO_NACK     0
#define RPL_CONF_DIO_REFRESH_DAO_ROUTES     0

/*---------------------------------------------------------------------------*/
/* Connection pin-out 
 * -------------+-----------------------------
 * Rain gauge   | PC0
 * Anemometer   | PC1
 * Wind vane    | ADC1 (PA5)
 *--------------+-----------------------------
 * I2C sensors connected via the default I2C 5-pin connector
 */
#define WEATHER_METER_CONF_ANEMOMETER_PIN       1
#define WEATHER_METER_CONF_ANEMOMETER_PORT      GPIO_C_NUM
#define WEATHER_METER_CONF_ANEMOMETER_VECTOR    NVIC_INT_GPIO_PORT_C
#define WEATHER_METER_CONF_RAIN_GAUGE_PIN       0
#define WEATHER_METER_CONF_RAIN_GAUGE_PORT      GPIO_C_NUM
#define WEATHER_METER_CONF_RAIN_GAUGE_VECTOR    NVIC_INT_GPIO_PORT_C
#define WEATHER_METER_CONF_RAIN_WIND_VANE_ADC   ZOUL_SENSORS_ADC1
/*---------------------------------------------------------------------------*/
#define MQTT_CLIENT_ID                          "zolertia"

/* A list of MQTT brokers:
 * test.mosquitto.org (37.187.106.16) --> ::ffff:25bb:6a10
 * mqtt.relayr.io (52.48.96.194) --> ::ffff:3430:60c2
 */
#if WITH_RELAYR_PLATFORM
#define MQTT_BROKER_IP_ADDR                     "::ffff:3430:60c2"
#define MQTT_AUTH_USER                          ""
#define MQTT_AUTH_TOKEN                         ""
#else /* Start with Mosquitto broker */
#define MQTT_BROKER_IP_ADDR                     "::ffff:25bb:6a10"
#define MQTT_AUTH_USER                          "my_user"
#endif

#define MQTT_BROKER_PORT                        1883
#define DEFAULT_PUB_INTERVAL                    (30 * CLOCK_SECOND)
#define DEFAULT_KEEP_ALIVE_TIMER                60
#define APP_BUFFER_SIZE                         364
#define APP_PUB_RECV_SIZE                       48

/* Workaround to prevent being blocked, disabled if zero */
#define MQTT_MAX_BLOCK_TRIES                    10

/* Relayr-specific */
#define MQTT_RELAYR_API_VER                     "/v1/"
#define MQTT_RELAYR_DATA_STRING                 "/data"
#define MQTT_RELAYR_CMD_STRING                  "/cmd"
#define MQTT_RELAYR_CMD_INTERVAL                "interval"
/*---------------------------------------------------------------------------*/
#define PUB_TOPIC     MQTT_RELAYR_API_VER MQTT_AUTH_USER MQTT_RELAYR_DATA_STRING
#define SUB_TOPIC     MQTT_RELAYR_API_VER MQTT_AUTH_USER MQTT_RELAYR_CMD_STRING
/*---------------------------------------------------------------------------*/
#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
