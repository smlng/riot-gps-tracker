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

#include "lora-keys.h"
#include "hardware.h"

/* we will use Cayenne LPP for displaying our data */
#include "cayenne_lpp.h"

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

#define BAUDRATE (9600)

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
void send_lora_data(semtech_loramac_t *loramac, float lat, float lon) {

    printf("Sending:");

    /* reset our cayenne buffer */
    cayenne_lpp_reset(&lpp);

    cayenne_lpp_add_gps(&lpp, CAYENNE_LPP_CHANNEL, lat, lon ,0);

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

static void rx_cb(void *arg, uint8_t data)
{
    (void) arg;
    rx_mem[counter++] = data;

    if (data == '\n') {
        rx_mem[counter++] = 0;
        counter = 0;
        msg_t msg;
        msg_send(&msg, printer_pid);
    }
}

void cb(void *arg)
{
    (void) arg;
    uart_poweron(UART_DEV(1));
}

static void *printer(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);
    struct minmea_sentence_rmc frame;
    xtimer_t uart_timer;
    uart_timer.callback = cb;

    while (1) {
        msg_receive(&msg);
        if(minmea_sentence_id(rx_mem, false) == MINMEA_SENTENCE_RMC) {
            minmea_parse_rmc(&frame, rx_mem);

            printf("TX: %d, %d\n", (int) frame.latitude.value, (int) frame.longitude.value);

            //Send value
            send_lora_data(&g_loramac, minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude));
            uart_poweroff(UART_DEV(1));
            xtimer_set(&uart_timer, TX_PERIOD);
        }

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
    gpio_clear(GPS_EN_PIN);
}

int main(void)
{

    /* Initialize and enable gps */
    init_gps();

    /* Turn LED0 off to save power */
    LED0_OFF;

    /* initialize UART */
    uart_init(UART_DEV(1), BAUDRATE, rx_cb, NULL);


    setup_lora(&g_loramac);

    /* start the printer thread */
    printer_pid = thread_create(printer_stack, sizeof(printer_stack),
                                PRINTER_PRIO, 0, printer, NULL, "printer");

    return 0;
}
