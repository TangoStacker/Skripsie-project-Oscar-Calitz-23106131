/*
 * UART_COMM.C
 * Implementation of the UART initialization and communication logic.
 */

#include "uart_comm.h" // Includes all necessary Zephyr headers and prototypes
#include "sensors.h"
#include "mesh.h"
// --- GLOBAL VARIABLE DEFINITIONS (Matching extern in header) ---
volatile uint8_t send_flag = 0;
char result[MSG_SIZE]; // Array used to hold extracted AT command

// --- DEVICE POINTER DEFINITIONS (Matching extern in header) ---
const struct device *cdc_uart_device = DEVICE_DT_GET(CDC_UART_DEVICE_NODE);
const struct device *uart0_device = DEVICE_DT_GET(UART_DEVICE_NODE);

// --- STATIC (File-Local) Variables ---
// These are only needed by the callback/ISR and should not be global.
static volatile int rx_buf_pos = 0;
static char rx_buf[MSG_SIZE];
int loop_time;

// --- FUNCTION IMPLEMENTATIONS ---

int initialise_uart(void)
{
    // Implementation body remains the same as your input
    uint32_t cdc_dtr = 0;
    if (usb_enable(NULL)) {
            return 0;
        }
    //check is serial port is connected and ready to recieve
    while (!cdc_dtr) {
            uart_line_ctrl_get(cdc_uart_device, UART_LINE_CTRL_DTR, &cdc_dtr);
            // Give CPU resources to low priority threads
            printk("CDC DTR = %u\n", cdc_dtr);
            k_sleep(K_MSEC(1000));
        }
    //initialise interrupt and set callback function for cdc uart (virtual)
    int ret = uart_irq_callback_user_data_set(cdc_uart_device, serial_cb, NULL);
    printk("CDC ready\n");
    //check return for errors
    if (ret < 0) {
        if (ret == -ENOTSUP) {
            printk("Interrupt-driven UART API support not enabled\n");
        } else if (ret == -ENOSYS) {
            printk("UART device does not support interrupt-driven API\n");
        } else {
            printk("Error setting UART callback: %d\n", ret);
        }
        return 0;
    }
    printk("CDC UART callback set\n");
    //interrupt for uart0 device (pins)
    ret = uart_irq_callback_user_data_set(uart0_device, serial_cb, NULL);
    printk("Setting UART0 callback ...\n");
    if (ret < 0) {
        if (ret == -ENOTSUP) {
            printk("Interrupt-driven UART API support not enabled\n");
        } else if (ret == -ENOSYS) {
            printk("UART device does not support interrupt-driven API\n");
        } else {
            printk("Error setting UART callback: %d\n", ret);
        }
        return 0;
    }
    printk("UART0 callback set\n");
    uart_irq_rx_enable(cdc_uart_device); //enable rx interrupt to start receiving
    uart_irq_rx_enable(uart0_device); //enable rx interrupt to start receiving
    printk("enabling uart rx interrupts ...\n");
    k_msleep(3); 
    printk("done\n");
    return 1;
}



void serial_cb(const struct device *dev, void *user_data)
{
    // Implementation body remains the same as your input, using static variables
    uint8_t c;
    if (!uart_irq_update(dev)) {
        return;
    }
    if (!uart_irq_rx_ready(dev)) {
        return;
    }
    while (uart_fifo_read(dev, &c, 1) == 1) {
        if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
            rx_buf[rx_buf_pos] = '\0';
            rx_buf_pos = 0;
            
            if (strstr(rx_buf,"send AT") != NULL)
            {
                const char* start_ptr = strstr(rx_buf, "AT");
                const char* end_ptr = strchr(start_ptr, '\0');
                if (end_ptr == NULL) {
                    printk("no end line char found");
                    return; 
                }
                uint8_t length = end_ptr - start_ptr;
                strncpy(result, start_ptr, length);
                result[length] = '\0';
                send_flag = 1;
                printk("result is:%s and falg is %d",result,send_flag);
                memset(rx_buf,0,MSG_SIZE);
                return;
            }
            else if(strstr(rx_buf,"measure") != NULL)
            {
                hs3003_measurement();
                printk("%.0f%.1f",hs3003_humidity,hs3003_temperature);
                bmi270_measurement();
                printk("%f%f%f%f%f%f",acc[0],acc[1],acc[2],gyr[0],gyr[1],gyr[2]);   
                bmm150_measurement();
                printk("%f%f%f\n",mag[0],mag[1],mag[2]);   
            }
            else if(strstr(rx_buf,"loop:"))
            {
                const char* start_ptr = strstr(rx_buf, ":")+1;
                const char* end_ptr = strchr(start_ptr, '\0');
                if (end_ptr == NULL) {
                    printk("no end line char found");
                    return; 
                }
                uint8_t length = end_ptr - start_ptr;
                strncpy(result, start_ptr, length);
                result[length] = '\0';
                loop_time = 0;
                for(int i = 0; i < length; i++)
                {
                    loop_time += (result[i] -'0')*pow(10,length - i -1);//ascii char to int * powers of 10 dependign on placement and length
                }
                // loop_time = (int)(result)-30;//ascii to int
                // loop_time = atoi(result);
                // printk("time:%d",loop_time);
                memset(rx_buf,0,MSG_SIZE);
                return;
            }
            else if(strstr(rx_buf,"danger"))
            {
                genericOnOffSetUnAck(onoff[1]);
            }
            else if(strstr(rx_buf,"safe"))
            {
                genericOnOffSetUnAck(onoff[0]);
            }
            else 
            {
                printk("recieved: %s\n", rx_buf);
                return;
            }
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
    }
}

void uart_send(const struct device *dev, const char *msg){
    for (uint32_t i = 0; i < strlen(msg); i++) {
        uart_poll_out(dev, msg[i]);
    }
    uart_poll_out(dev, '\r');
}
