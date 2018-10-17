#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "thread.h"
#include "msg.h"
#include "periph/uart.h"
#include "minmea.h"
#include "xtimer.h"
#include "fmt.h"

#include "net/loramac.h"
#include "semtech_loramac.h"

/* we will use Cayenne LPP for displaying our data */
#include "cayenne_lpp.h"

#include "lora-keys.beta.h"
#include "hardware.h"
#include "app-config.h"

#define ENABLE_DEBUG        (1)
#include "debug.h"

#define MSG_QUEUE_LEN       (4U)

semtech_loramac_t g_loramac;

static char rx_mem[APP_GPS_UART_BUFSIZE];
static kernel_pid_t sender_pid;
static xtimer_t gps_read_timer;
static msg_t msg_queue[MSG_QUEUE_LEN];
static volatile unsigned counter=0;

void lorawan_setup(semtech_loramac_t *loramac)
{
    DEBUG("lorawan_setup: ");
    uint8_t buf[APP_LORAWAN_RX_BUF_SIZE];

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
    while (semtech_loramac_join(loramac, LORAMAC_JOIN_OTAA) !=
           SEMTECH_LORAMAC_JOIN_SUCCEEDED) {
        DEBUG("failed\n");
        xtimer_sleep(APP_LORAWAN_JOIN_RETRY_TIME);
    }
    DEBUG("success\n");
}

void lorawan_send(semtech_loramac_t *loramac, float lat, float lon, float alt, int sat)
{
    DEBUG("lorawan_send: ");

    cayenne_lpp_t lpp;

    cayenne_lpp_add_gps(&lpp, APP_CAYENNE_LPP_GPS_CHANNEL, lat, lon, alt);
    cayenne_lpp_add_digital_input(&lpp, 2, sat);

    semtech_loramac_set_tx_mode(loramac, LORAMAC_TX_UNCNF);
    semtech_loramac_set_tx_port(loramac, APP_LORAWAN_TX_PORT);

    /* set datarate */
    semtech_loramac_set_dr(loramac, APP_LORAWAN_DATARATE);

    /* try to send data */
    switch (semtech_loramac_send(loramac, lpp.buffer, lpp.cursor)) {
        case SEMTECH_LORAMAC_NOT_JOINED:
            DEBUG("failed, not joined\n");
            return;

        case SEMTECH_LORAMAC_BUSY:
            DEBUG("failed, MAC busy\n");
            return;
    }
    DEBUG("success\n");
}

static void uart_rx_cb(void *arg, uint8_t data)
{
    (void) arg;

    rx_mem[counter++] = data;

    /* Check if received a complete frame */
    if (data == '\n') {
        rx_mem[counter++] = 0;
        msg_t m;
        msg_send(&m, sender_pid);
        counter = 0;
    }
}

void gps_start(void *arg)
{
    (void) arg;
    DEBUG("gps_start\n");
    LED0_ON;
    /* Enable UART */
    uart_poweron(UART_DEV(1));
    /* Enable GPS */
    gpio_clear(GPS_EN_PIN);
}

void gps_stop(void)
{
    DEBUG("gps_stop\n");
    /* Disable GPS */
    //gpio_set(GPS_EN_PIN);
    /* Disable UART */
    uart_poweroff(UART_DEV(1));
    LED0_OFF;
    xtimer_set(&gps_read_timer, APP_TX_PERIOD_US);
}

/**
 * @brief Initializes the gps enable pin and the external mux 2 control line,
 *        then enables the gps.
 */
static void gps_init(void)
{
    DEBUG("gps_init\n");
    /* GPS enable pin */
    gpio_init(GPS_EN_PIN, GPIO_OUT);
    /* External IO control 2 line */
    gpio_init(EXT_IO_CTRL2_PIN, GPIO_OUT);
    /* Select GPS enable pin on the external IO 2 mux */
    gpio_set(EXT_IO_CTRL2_PIN);
    /* Disable GPS */
    gpio_set(GPS_EN_PIN);
}

int main(void)
{
    /* Enable the onboard Step Up regulator */
    EN3V3_ON;

    /* Initialize and enable gps */
    gps_init();

    /* Initialize UART */
    uart_init(UART_DEV(1), APP_GPS_UART_BAUDRATE, uart_rx_cb, NULL);

    /* Setup LoRa parameters and OTAA join */
    lorawan_setup(&g_loramac);

    sender_pid = thread_getpid();
    gps_read_timer.callback = gps_start;

    msg_init_queue(msg_queue, MSG_QUEUE_LEN);

    gps_start(NULL);
    while (1) {
        msg_t msg;
        msg_receive(&msg);
        struct minmea_sentence_gga frame;
        if(minmea_parse_gga(&frame, rx_mem)) {
            DEBUG("LAT=%d, LON=%d, ALT=%d, FIX=%d, NUM=%d\n",
                  (int)frame.latitude.value, (int)frame.longitude.value,
                  (int)frame.height.value,
                  (int)frame.fix_quality, (int)frame.satellites_tracked);
            if (frame.fix_quality) {
                float lat = minmea_tocoord(&frame.latitude);
                float lon = minmea_tocoord(&frame.longitude);
                float alt = minmea_tofloat(&frame.height);
                lorawan_send(&g_loramac, lat, lon, alt, frame.satellites_tracked);
                gps_stop();
            }
        }
    }

    return 0;
}
