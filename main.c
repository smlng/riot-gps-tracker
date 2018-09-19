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

#include "lora-keys.h"
#include "hardware.h"
#include "app-config.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

static cayenne_lpp_t lpp;
semtech_loramac_t g_loramac;

static char rx_mem[UART_BUFSIZE];
static kernel_pid_t printer_pid;
static char printer_stack[THREAD_STACKSIZE_MAIN];

static int counter=0;

void setup_lora(semtech_loramac_t *loramac) {
    uint8_t buf[BUF_SIZE];

    /* init LoRaMAC */
    semtech_loramac_init(loramac);

    /* load required keys into LoRaMAC */
    fmt_hex_bytes(buf, LORA_DEVEUI);
    semtech_loramac_set_deveui(loramac, buf);

    fmt_hex_bytes(buf, LORA_APPEUI);
    semtech_loramac_set_appeui(loramac, buf);

    fmt_hex_bytes(buf, LORA_APPKEY);
    semtech_loramac_set_appkey(loramac, buf);

    /* Try to join by Over The Air Activation */
    if (semtech_loramac_join(loramac, LORAMAC_JOIN_OTAA) != SEMTECH_LORAMAC_JOIN_SUCCEEDED) {
		puts("Join failed");
	}
    else {
        puts("Join Success");
    }
}
void send_lora_data(semtech_loramac_t *loramac, float lat, float lon, int link) {

    printf("Sending:");

    /* reset our cayenne buffer */
    cayenne_lpp_reset(&lpp);

    cayenne_lpp_add_gps(&lpp, CAYENNE_LPP_CHANNEL, lat, lon ,0);
    cayenne_lpp_add_digital_input(&lpp, 2, link);

    semtech_loramac_set_tx_mode(loramac, LORAMAC_TX_UNCNF);
    semtech_loramac_set_tx_port(loramac, LORA_PORT);

    /* set datarate */
    semtech_loramac_set_dr(loramac, LORAWAN_DATARATE);


    /* try to send data */
    switch (semtech_loramac_send(loramac, lpp.buffer, lpp.cursor)) {
        case SEMTECH_LORAMAC_NOT_JOINED:
            puts("Failed: not joined");
            return;

        case SEMTECH_LORAMAC_BUSY:
            puts("Failed: mac is busy");
            return;
    }

    /* check if something was received */
    switch (semtech_loramac_recv(loramac)) {
        case SEMTECH_LORAMAC_DATA_RECEIVED:
            loramac->rx_data.payload[loramac->rx_data.payload_len] = 0;
            printf("Data received: %s, port: %d\n",
                   (char *)loramac->rx_data.payload, loramac->rx_data.port);
            break;

        case SEMTECH_LORAMAC_TX_CNF_FAILED:
            puts("Confirmable TX failed");
            break;

        case SEMTECH_LORAMAC_TX_DONE:
            puts("TX complete, no data received");
            break;
    }
}

static void uart_rx_cb(void *arg, uint8_t data)
{
    (void) arg;
    rx_mem[counter++] = data;

    /* Check if received a complete frame */
    if (data == '\n') {
        rx_mem[counter++] = 0;
        counter = 0;
        msg_t msg;
        msg_send(&msg, printer_pid);
    }
}

void start_gps(void *arg)
{
    (void) arg;
    puts("Starting UART and GPS");

    /* Enable UART */
    uart_poweron(UART_DEV(1));

    /* Enable GPS */
    gpio_clear(GPS_EN_PIN);
}

void stop_gps(int gpsFixQuality)
{
    static int gpsQualityCount = 0;

    puts("Stopping UART and GPS");

    /* Save the amount of readings with fixed quality */
    gpsQualityCount = gpsQualityCount * gpsFixQuality + gpsFixQuality;

    /* Disable UART */
    uart_poweroff(UART_DEV(1));

    if (gpsQualityCount >= GPS_FIXED_TH) {
        DEBUG("Enough times fixed, turning GPS off\n");
        /* Disable GPS */
        gpio_set(GPS_EN_PIN);
    }
}

static void *printer(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    //struct minmea_sentence_rmc frame;
    struct minmea_sentence_gga frame;
    xtimer_t gps_read_timer;
    gps_read_timer.callback = start_gps;

    msg_init_queue(msg_queue, 8);

    start_gps(NULL);

    while (1) {
        msg_receive(&msg);

        /* Check if the messages is a valid minmea sentence */
        if(minmea_sentence_id(rx_mem, false) == MINMEA_SENTENCE_GGA) {

            minmea_parse_gga(&frame, rx_mem);

            stop_gps(frame.fix_quality);

            printf("TX: %d, %d. Quality: %d\n", (int)frame.latitude.value, (int)frame.longitude.value, frame.fix_quality);

            send_lora_data(&g_loramac, minmea_tocoord(&frame.latitude),
                           minmea_tocoord(&frame.longitude), frame.fix_quality);

            // minmea_parse_rmc(&frame, rx_mem);

            // printf("TX: %d, %d\n", (int) frame.latitude.value, (int) frame.longitude.value);

            // //Send value
            // send_lora_data(&g_loramac, minmea_tocoord(&frame.latitude),
            //               minmea_tocoord(&frame.longitude));
        }
        xtimer_set(&gps_read_timer, TX_PERIOD);
    }

    /* this should never be reached */
    return NULL;
}

/**
 * @brief Initializes the gps enable pin and the external mux 2 control line,
 *        then enables the gps.
 */
static void init_gps(void)
{
    /* GPS enable pin */
    gpio_init(GPS_EN_PIN, GPIO_OUT);

    /* External IO control 2 line */
    gpio_init(EXT_IO_CTRL2_PIN, GPIO_OUT);

    /* Select GPS enable pin on the external IO 2 mux */
    gpio_set(EXT_IO_CTRL2_PIN);

    /* Enable GPS */
    //gpio_clear(GPS_EN_PIN);
}

int main(void)
{

    /* Initialize and enable gps */
    init_gps();

    /* initialize UART */
    uart_init(UART_DEV(1), BAUDRATE, uart_rx_cb, NULL);


    setup_lora(&g_loramac);

    /* start the printer thread */
    printer_pid = thread_create(printer_stack, sizeof(printer_stack),
                                PRINTER_PRIO, 0, printer, NULL, "printer");

    return 0;
}
