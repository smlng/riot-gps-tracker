#ifndef _APP_CONFIG_H
#define _APP_CONFIG_H

#define BUF_SIZE    64
#define LORA_PORT   1

#define LORAWAN_DATARATE 5

/* we use "Dynamic Sensor Payload for our data */
#define CAYENNE_LPP_CHANNEL 1

/* we must respect the duty cycle limitations */
#define SLEEP_TIME 10

#define UART_BUFSIZE        (128U)
#define PRINTER_PRIO        (THREAD_PRIORITY_MAIN - 1)
#define TX_PERIOD           (10000000U)

/* Amount of measurements before turning GPS off */
#define GPS_FIXED_TH   3

#define BAUDRATE (9600)

#endif
