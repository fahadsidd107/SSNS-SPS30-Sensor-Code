/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>  // printf
#include "sps30.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>  // printf
#include "sps30.h"

#define UART1_NODE DT_NODELABEL(uart1)
static const struct device *uart_dev = DEVICE_DT_GET(UART1_NODE);

static char tx_buf[256];

void send_data_over_uart(const char *data) {
    uart_tx(uart_dev, data, strlen(data), SYS_FOREVER_MS);
}

int main(void) {
    struct sps30_measurement m;
    int16_t ret;

    if (!device_is_ready(uart_dev)) {
        printf("UART device not ready\n");
        return 0;
    }

    /* Initialize I2C bus */
    sensirion_i2c_init();

    /* Busy loop for initialization, because the main loop does not work without
     * a sensor.
     */
    while (sps30_probe() != 0) {
        printf("SPS sensor probing failed\n");
        snprintf(tx_buf, sizeof(tx_buf), "SPS sensor probing failed\n");
        send_data_over_uart(tx_buf);
        sensirion_sleep_usec(1000000); /* wait 1s */
    }
    printf("SPS sensor probing successful\n");
    snprintf(tx_buf, sizeof(tx_buf), "SPS sensor probing successful\n");
    send_data_over_uart(tx_buf);

    uint8_t fw_major;
    uint8_t fw_minor;
    ret = sps30_read_firmware_version(&fw_major, &fw_minor);
    if (ret) {
        printf("error reading firmware version\n");
        snprintf(tx_buf, sizeof(tx_buf), "error reading firmware version\n");
        send_data_over_uart(tx_buf);
    } else {
        printf("FW: %u.%u\n", fw_major, fw_minor);
        snprintf(tx_buf, sizeof(tx_buf), "FW: %u.%u\n", fw_major, fw_minor);
        send_data_over_uart(tx_buf);
    }

    char serial_number[SPS30_MAX_SERIAL_LEN];
    ret = sps30_get_serial(serial_number);
    if (ret) {
        printf("error reading serial number\n");
        snprintf(tx_buf, sizeof(tx_buf), "error reading serial number\n");
        send_data_over_uart(tx_buf);
    } else {
        printf("Serial Number: %s\n", serial_number);
        snprintf(tx_buf, sizeof(tx_buf), "Serial Number: %s\n", serial_number);
        send_data_over_uart(tx_buf);
    }

    ret = sps30_start_measurement();
    if (ret < 0) {
        printf("error starting measurement\n");
        snprintf(tx_buf, sizeof(tx_buf), "error starting measurement\n");
        send_data_over_uart(tx_buf);
    } else {
        printf("measurements started\n");
        snprintf(tx_buf, sizeof(tx_buf), "measurements started\n");
        send_data_over_uart(tx_buf);
    }

    while (1) {
        sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC); /* wait 1s */
        ret = sps30_read_measurement(&m);
        if (ret < 0) {
            printf("error reading measurement\n");
            snprintf(tx_buf, sizeof(tx_buf), "error reading measurement\n");
            send_data_over_uart(tx_buf);
        } else {
            printf("Particulate Matter: %0.2f pm10.0\n",m.mc_10p0);
            snprintf(tx_buf, sizeof(tx_buf),"Particulate Matter: %0.2f pm10.0\n",m.mc_10p0);
            send_data_over_uart(tx_buf);
        }
    }

    return 0;
}
