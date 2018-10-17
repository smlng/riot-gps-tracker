#ifndef _APP_CONFIG_H
#define _APP_CONFIG_H

#include "xtimer.h"
/* LoRaWAN application configurations */
#define APP_LORAWAN_RX_BUF_SIZE         64
#define APP_LORAWAN_TX_PORT             1
#define APP_LORAWAN_DATARATE            2
#define APP_LORAWAN_JOIN_RETRY_TIME     10

/* we use "Dynamic Sensor Payload for our data */
#define APP_CAYENNE_LPP_GPS_CHANNEL         1



#define APP_GPS_UART_BUFSIZE        (128U)
#define APP_GPS_UART_BAUDRATE       (9600)

#define APP_THREAD_SENDER_PRIO      (THREAD_PRIORITY_MAIN - 1)

/* we must respect the duty cycle limitations */
#define APP_SLEEP_TIME_S            (19U)
#define APP_TX_PERIOD_US            (APP_SLEEP_TIME_S * US_PER_SEC)

/* Amount of measurements before turning GPS off */
#define APP_GPS_FIXED_TH    (2U)
/* Max loop counter before turning of GPS and try again later */
#define APP_MAX_COUNT       (64U)

#endif
