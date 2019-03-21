#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "byteorder.h"
#include "periph/uart.h"
#include "xtimer.h"
#include "fmt.h"

#include "net/loramac.h"
#include "semtech_loramac.h"

#include "lora-keys.bear.h"
#include "hardware.h"
#include "config.h"
#include "gps.h"

#define ENABLE_DEBUG        (1)
#include "debug.h"


static semtech_loramac_t g_loramac;
static uint8_t buf[LORAWAN_BUF_SIZE];

void lorawan_setup(semtech_loramac_t *loramac)
{
    DEBUG("%s\n", __func__);
    /* init LoRaMAC */
    semtech_loramac_init(loramac);
    /* load device EUI */
    fmt_hex_bytes(buf, LORA_DEVEUI);
    semtech_loramac_set_deveui(loramac, buf);
    /* load application EUI */
    fmt_hex_bytes(buf, LORA_APPEUI);
    semtech_loramac_set_appeui(loramac, buf);
    /* load application key */
    fmt_hex_bytes(buf, LORA_APPKEY);
    semtech_loramac_set_appkey(loramac, buf);
    /* Try to join by Over The Air Activation */
    DEBUG(". join LoRaWAN: ");
    while (semtech_loramac_join(loramac, LORAMAC_JOIN_OTAA) !=
           SEMTECH_LORAMAC_JOIN_SUCCEEDED) {
        DEBUG("failed\n... retry:");
        xtimer_sleep(APP_LORAWAN_JOIN_RETRY_TIME);
    }
    DEBUG("success\n");
}

int create_buf(int32_t lat, int32_t lon, int16_t alt, uint8_t sat,
               uint8_t *buf, size_t maxlen)
{
    size_t len = sizeof(lat) + sizeof(lon) + sizeof(alt) + sizeof(sat);
    if (maxlen < len) {
        return (-1);
    }
    memset(buf, 0, maxlen);
    lat = htonl(lat);
    memcpy(buf, &lat, sizeof(lat));
    buf += sizeof(lat);
    lon = htonl(lon);
    memcpy(buf, &lon, sizeof(lon));
    buf += sizeof(lon);
    alt = htons(alt);
    memcpy(buf, &alt, sizeof(alt));
    buf += sizeof(alt);
    memcpy(buf, &sat, sizeof(sat));
    return len;
}

void lorawan_send(semtech_loramac_t *loramac, uint8_t *buf, uint8_t len)
{
    DEBUG("%s\n", __func__);

    semtech_loramac_set_tx_mode(loramac, LORAMAC_TX_UNCNF);
    semtech_loramac_set_tx_port(loramac, APP_LORAWAN_TX_PORT);
    /* set datarate */
    semtech_loramac_set_dr(loramac, APP_LORAWAN_DATARATE);
    /* try to send data */
    DEBUG(". send: ");
    unsigned ret = semtech_loramac_send(loramac, buf, len);
    switch (ret) {
        case SEMTECH_LORAMAC_TX_OK:
            DEBUG("success\n");
            break;

        case SEMTECH_LORAMAC_NOT_JOINED:
            DEBUG("failed, not joined\n");
            break;

        case SEMTECH_LORAMAC_BUSY:
            DEBUG("failed, MAC busy\n");
            break;

        default:
            DEBUG("failed with %u\n", ret);
    }
    /* try to receive something (mandatory to unblock) */
    DEBUG(". recv: ");
    ret = semtech_loramac_recv(loramac);
    /* check if something was received */
    switch (ret) {
        case SEMTECH_LORAMAC_DATA_RECEIVED:
            loramac->rx_data.payload[loramac->rx_data.payload_len] = 0;
            DEBUG("got data  [%s] on port %d\n",
                  (char *)loramac->rx_data.payload, loramac->rx_data.port);
            break;

        case SEMTECH_LORAMAC_TX_CNF_FAILED:
            DEBUG("confirmable TX failed!\n");
            break;

        case SEMTECH_LORAMAC_TX_DONE:
            DEBUG("TX complete, no data received\n");
            break;

        default:
            DEBUG("failed with %u\n", ret);
    }
}

int main(void)
{
    /* Enable the onboard Step Up regulator */
    EN3V3_ON;
    /* Initialize and enable gps */
    gps_init(GPS_UART_DEV, GPS_UART_BAUDRATE);
    /* Setup LoRa parameters and OTAA join */
    lorawan_setup(&g_loramac);

    unsigned gps_quality = 0;
    bool gps_off = true;

    while (1) {
        if (gps_off) {
            gps_start(GPS_UART_DEV);
            gps_off = false;
        }

        int32_t lat  = 0;
        int32_t lon  = 0;
        int32_t alt  = 0;
        unsigned sat = 0;
        unsigned fix = 0;

        if(gps_read(&lat, &lon, &alt, &sat, &fix) == 0) {
            DEBUG(". got GPS data\n");
            gps_quality += fix;
            if (gps_quality > GPS_QUALITY_THRESHOLD) {
                DEBUG(".. send\n");
                gps_stop(GPS_UART_DEV);
                gps_quality = 0;
                gps_off = true;
                int len = create_buf(lat, lon, alt, sat, &buf[0], LORAWAN_BUF_SIZE);
                if (len > 0) {
                    lorawan_send(&g_loramac, buf, len);
                    xtimer_sleep(APP_SLEEP_TIME_S);
                }
            }
            else {
                DEBUG(".. wait for more\n");
            }
        }
    }

    return 0;
}
