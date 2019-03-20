#include <stdint.h>

#include "periph/uart.h"

#define GPS_LINE_BUFSIZE        (128U)
#define GPS_UART_BUFSIZE        (512U)

int gps_read_float(float *lat, float *lon, float *alt,
                   unsigned *sat, unsigned *fix, uint32_t maxwait);

int gps_read_int32(int32_t *lat, int32_t *lon, int32_t *alt,
                   unsigned *sat, unsigned *fix, uint32_t maxwait);

void gps_init(uart_t dev, uint32_t baud);

void gps_start(uart_t dev);

void gps_stop(uart_t dev);
