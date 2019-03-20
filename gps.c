#include <stdint.h>
#include <string.h>

#include "isrpipe.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "xtimer.h"

#include "minmea.h"

#include "config.h"
#include "gps.h"
#include "hardware.h"

#define ENABLE_DEBUG        (1)
#include "debug.h"


static char _rx_buf_mem[GPS_UART_BUFSIZE];
isrpipe_t gps_uart_isrpipe = ISRPIPE_INIT(_rx_buf_mem);

static ssize_t _readline(isrpipe_t *isrpipe, char *resp_buf, size_t len, uint32_t timeout)
{
    DEBUG("%s\n", __func__);
    ssize_t res = -1;
    char *resp_pos = resp_buf;

    tsrb_empty(&isrpipe->tsrb);

    memset(resp_buf, 0, len);

    while (len) {
        int read_res;
        if ((read_res = isrpipe_read_timeout(isrpipe, resp_pos, 1, timeout)) == 1) {
            if (*resp_pos == '\r') {
                continue;
            }
            if (*resp_pos == '\n') {
                *resp_pos = '\0';
                res = resp_pos - resp_buf;
                goto out;
            }

            resp_pos += read_res;
            len -= read_res;
        }
        else if (read_res == -ETIMEDOUT) {
            res = -ETIMEDOUT;
            break;
        }
    }

out:
    return res;
}

void gps_start(uart_t dev)
{
    DEBUG("%s\n", __func__);
    LED0_ON;
    /* Enable UART */
    uart_poweron(dev);
    /* Enable GPS */
    gpio_clear(GPS_EN_PIN);
}

void gps_stop(uart_t dev)
{
    DEBUG("%s\n", __func__);
    /* Disable GPS */
    gpio_set(GPS_EN_PIN);
    /* Disable UART */
    uart_poweroff(dev);
    LED0_OFF;
}

#define MINMEA_COORD_UPSCALE    (10000000L)

int_least32_t _minmea_tocoord32(int32_t value, int32_t scale)
{
    if (scale == 0)
        return 0;

    int_least32_t degrees = value / (scale * 100);
    int_least32_t minutes = value % (scale * 100);

    degrees *= MINMEA_COORD_UPSCALE;
    minutes *= (MINMEA_COORD_UPSCALE / scale);
    minutes /= 60;

    return (degrees + minutes);
}

int_least32_t _minmea_tometer(int32_t value, int32_t scale)
{
    if (scale == 0)
        return 0;
    return value / scale;
}

int _gps_parse_gga_int32(const char *buf, int32_t *lat, int32_t *lon, int32_t *alt,
                         unsigned *sat, unsigned *fix)
{
    DEBUG("%s\n", __func__);
    struct minmea_sentence_gga frame;
    if(minmea_parse_gga(&frame, buf)) {
        *lat = _minmea_tocoord32(frame.latitude.value, frame.latitude.scale);
        DEBUG(". MINMEA INT32 LAT (%"PRIi32")\n", *lat);
        *lon = _minmea_tocoord32(frame.longitude.value, frame.longitude.scale);
        DEBUG(". MINMEA INT32 LON (%"PRIi32")\n", *lon);
        *alt = _minmea_tometer(frame.height.value, frame.height.scale);
        DEBUG(". MINMEA INT32 ALT (%"PRIi32")\n", *alt);
        *sat = (unsigned)frame.satellites_tracked;
        *fix = (unsigned)frame.fix_quality;
        return 0;
    }
    return (-1);
}

int gps_read_int32(int32_t *lat, int32_t *lon, int32_t *alt,
                   unsigned *sat, unsigned *fix, uint32_t maxwait)
{
    DEBUG("%s\n", __func__);

    char linebuf[GPS_LINE_BUFSIZE];
    uint64_t before, now = 0;
    if (maxwait) {
        now = xtimer_now_usec64();
    }

    do {
        before = now;
        ssize_t res = _readline(&gps_uart_isrpipe, linebuf, sizeof(linebuf), maxwait);
        if (res >= 6 /* $GPGLL */) {
            DEBUG("gps: %s\n", linebuf);
            if (_gps_parse_gga_int32(linebuf, lat, lon, alt, sat, fix) == 0) {
                return 0;
            }
        }
        if (maxwait) {
            now = xtimer_now_usec64();
        }
    } while ((maxwait == 0) || ((now - before) < maxwait));

    return -ETIMEDOUT;
}

int _gps_parse_gga_float(const char *buf, float *lat, float *lon, float *alt,
                   unsigned *sat, unsigned *fix)
{
    DEBUG("%s\n", __func__);
    struct minmea_sentence_gga frame;
    if(minmea_parse_gga(&frame, buf)) {
        DEBUG(". MINMEA FLOAT LAT (%"PRIi32", %"PRIi32")\n", frame.latitude.value, frame.latitude.scale);
        *lat = minmea_tocoord(&frame.latitude);
        DEBUG(". MINMEA FLOAT LON (%"PRIi32", %"PRIi32")\n", frame.longitude.value, frame.longitude.scale);
        *lon = minmea_tocoord(&frame.longitude);
        DEBUG(". MINMEA FLOAT ALT (%"PRIi32", %"PRIi32")\n", frame.height.value, frame.height.scale);
        *alt = minmea_tofloat(&frame.height);
        *sat = (unsigned)frame.satellites_tracked;
        *fix = (unsigned)frame.fix_quality;
        return 0;
    }
    return (-1);
}

int gps_read_float(float *lat, float *lon, float *alt,
                   unsigned *sat, unsigned *fix, uint32_t maxwait)
{
    DEBUG("%s\n", __func__);

    char linebuf[GPS_LINE_BUFSIZE];
    uint64_t before, now = 0;
    if (maxwait) {
        now = xtimer_now_usec64();
    }

    do {
        before = now;
        ssize_t res = _readline(&gps_uart_isrpipe, linebuf, sizeof(linebuf), maxwait);
        if (res >= 6 /* $GPGLL */) {
            DEBUG("gps: %s\n", linebuf);
            if (_gps_parse_gga_float(linebuf, lat, lon, alt, sat, fix) == 0) {
                return 0;
            }
        }
        if (maxwait) {
            now = xtimer_now_usec64();
        }
    } while ((maxwait == 0) || ((now - before) < maxwait));

    return -ETIMEDOUT;
}
/**
 * @brief Initializes the gps enable pin and the external mux 2 control line,
 *        then enables the gps.
 */
void gps_init(uart_t dev, uint32_t baud)
{
    DEBUG("%s\n", __func__);
    /* Initialize UART */
    uart_init(dev, baud, (uart_rx_cb_t) isrpipe_write_one, &gps_uart_isrpipe);
    /* GPS enable pin */
    gpio_init(GPS_EN_PIN, GPIO_OUT);
    /* External IO control 2 line */
    gpio_init(EXT_IO_CTRL2_PIN, GPIO_OUT);
    /* Select GPS enable pin on the external IO 2 mux */
    gpio_set(EXT_IO_CTRL2_PIN);
    /* Disable GPS */
    gpio_set(GPS_EN_PIN);
}
