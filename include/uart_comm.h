#ifndef uart_comm_H
#define uart_comm_H

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/mesh.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Check to ensure the console is correctly configured as CDC ACM UART
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

// Definitions for buffer size and device tree nodes
#define MSG_SIZE 64
//cdc acm uart chosen in overlay file (virtual uart)
#define CDC_UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
//physical uart pins
#define UART_DEVICE_NODE DT_NODELABEL(uart0)


extern volatile uint8_t send_flag;


extern char result[MSG_SIZE];


//device tree devices
extern const struct device *cdc_uart_device;
extern const struct device *uart0_device;
extern int loop_time;


int initialise_uart(void);


void serial_cb(const struct device *dev, void *user_data);


void uart_send(const struct device *dev, const char *msg);

#endif 