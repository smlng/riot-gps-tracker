#include <stdio.h>

#include <stdlib.h>

#include "board.h"
#include "periph/uart.h"
#include "xtimer.h"
#include "fmt.h"

#include "net/loramac.h"
#include "semtech_loramac.h"

/* we will use Cayenne LPP for displaying our data */
#include "cayenne_lpp.h"

#include "lora-keys.beta.h"
#include "hardware.h"
#include "config.h"
#include "gps.h"

#define ENABLE_DEBUG        (1)
#include "debug.h"

semtech_loramac_t g_loramac;

void lorawan_setup(semtech_loramac_t *loramac)
{
    DEBUG("%s\n", __func__);
    uint8_t buf[LORAWAN_BUF_SIZE];

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

void lorawan_send(semtech_loramac_t *loramac, float lat, float lon, float alt, unsigned sat)
{
    DEBUG("%s\n", __func__);

    cayenne_lpp_t lpp;
    /* reset our cayenne buffer */
    cayenne_lpp_reset(&lpp);
    /* write data into frame */
    cayenne_lpp_add_gps(&lpp, APP_CAYENNE_LPP_GPS_CHANNEL, lat, lon, alt);
    cayenne_lpp_add_digital_input(&lpp, 2, sat);

    semtech_loramac_set_tx_mode(loramac, LORAMAC_TX_UNCNF);
    semtech_loramac_set_tx_port(loramac, APP_LORAWAN_TX_PORT);
    /* set datarate */
    semtech_loramac_set_dr(loramac, APP_LORAWAN_DATARATE);
    /* try to send data */
    DEBUG(". send: ");
    unsigned ret = semtech_loramac_send(loramac, lpp.buffer, lpp.cursor);
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

    float lat;
    float lon;
    float alt;
    unsigned sat;
    unsigned fix;

    while (1) {
        if (gps_off) {
            gps_start(GPS_UART_DEV);
            gps_off = false;
        }
        lat = 0;
        lon = 0;
        alt = 0;
        sat = 0;
        fix = 0;
        if(gps_read_float(&lat, &lon, &alt, &sat, &fix, GPS_MAXWAIT_US) == 0) {
            DEBUG(". got GPS data\n");
            gps_quality += fix;
            if (gps_quality > GPS_QUALITY_THRESHOLD) {
                DEBUG(".. send\n");
                gps_stop(GPS_UART_DEV);
                gps_quality = 0;
                gps_off = true;
                lorawan_send(&g_loramac, lat, lon, alt, sat);
                xtimer_sleep(APP_SLEEP_TIME_S);
            }
            else {
                DEBUG(".. wait for more\n");
                xtimer_sleep(1);
            }
        }
    }

    return 0;
}
