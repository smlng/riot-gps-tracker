#ifndef _APP_CONFIG_H
#define _APP_CONFIG_H

#include "xtimer.h"

/* LoRaWAN application configurations */
#define LORAWAN_BUF_SIZE                (64U)
#define APP_LORAWAN_TX_PORT             1
#define APP_LORAWAN_DATARATE            3
#define APP_LORAWAN_JOIN_RETRY_TIME     10

/* we use "Dynamic Sensor Payload for our data */
#define APP_CAYENNE_LPP_GPS_CHANNEL     1

#define GPS_UART_BAUDRATE               (9600)
#define GPS_UART_DEV                    UART_DEV(1)
#define GPS_QUALITY_THRESHOLD           (3U)
#define GPS_MAXWAIT_S                   (5U)
#define GPS_MAXWAIT_US                  (GPS_MAXWAIT_S * US_PER_SEC)

/* we must respect the duty cycle limitations */
#define APP_SLEEP_TIME_S                (20U)
#define APP_TX_PERIOD_US                (APP_SLEEP_TIME_S * US_PER_SEC)

#endif
