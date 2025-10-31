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

// --- 1. Compile-Time Checks and Definitions ---

// Check to ensure the console is correctly configured as CDC ACM UART
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

// Definitions for buffer size and device tree nodes
#define MSG_SIZE 64
//cdc acm uart chosen in overlay file (virtual uart)
#define CDC_UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
//physical uart pins
#define UART_DEVICE_NODE DT_NODELABEL(uart0)


// --- 2. External Global Variables ---

// Flag set by the interrupt callback and checked in the main loop
extern volatile uint8_t send_flag;

// Buffer used to hold the extracted AT command to be sent externally
extern char result[MSG_SIZE];


// --- 3. External Device Pointers ---

// Pointers to the two UART devices (Virtual CDC and External UART0)
// Must be defined globally in a C file (e.g., uart_comm.c)
extern const struct device *cdc_uart_device;
extern const struct device *uart0_device;
extern int loop_time;

// --- 4. Function Prototypes (Public API) ---

/**
 * @brief Initializes both CDC ACM and UART0 devices, setting up interrupts.
 * @return 1 on success, 0 on failure.
 */
int initialise_uart(void);

/**
 * @brief UART callback function (ISR) for handling received data.
 * @param dev The device structure of the UART that triggered the interrupt.
 * @param user_data Unused user data pointer.
 */
void serial_cb(const struct device *dev, void *user_data);

/**
 * @brief Sends a string over the specified UART device, terminating with \r.
 * @param dev The device structure of the UART to use.
 * @param msg The null-terminated string to send.
 */
void uart_send(const struct device *dev, const char *msg);

#endif 