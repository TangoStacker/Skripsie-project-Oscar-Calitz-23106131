/*
i2c send                                 --yes
i2c read                                 --yes
i2c hs3003 process                       --yes
i2c hs3003 measurement resolution        --no
*/
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/syscalls/time_units.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart_comm.h"
#include "GPIO.h"
#include "mesh.h"
#include "sensors.h"
#include "initdata.h"

#define TIMER_MS 1000
#define PI 3.14159265359
#define WINDOW_SIZE 40

static struct k_timer gyr_timer;

double acc_roll = 0;
double acc_pitch = 0;
double gyr_roll = 0;
double gyr_pitch = 0;
double gyr_yaw = 0;
double gyr_roll_prev = 0;
double gyr_pitch_prev = 0;
double gyr_yaw_prev = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;
double time_elapsed = 0;
double alpha = 0.4;

void timer_cb();
// #include "HealthConfig.h"
int main(void)
{       
        onoff_tid = 0;
        k_timer_init(&gyr_timer, timer_cb, NULL);

        if (initialise_uart() !=1)
        {
                printk("Error during UART initialisation\n");
                return 0;
        }
        k_sleep(K_SECONDS(1));

        if(initialise_gpio() != 1)
        {
                printk("Error during GPIO initialisation\n");
                return 0;
        }
        k_sleep(K_SECONDS(1));

        if(initialise_mesh() != 1)
        {
                printk("Error in health server configuration\n");
                return 0;
        }
        k_sleep(K_SECONDS(2));

        if(initialise_i2c() != 1)
        {
                printk("Error initialising i2c device \n");
                return 0;
        }
        k_sleep(K_SECONDS(1));
        loop_time = 10;
        //start timer 
        k_timer_start(&gyr_timer,K_MSEC(TIMER_MS),K_MSEC(TIMER_MS));
        while (1) 
        {
                
                k_msleep(loop_time);
                //============================================================================
                //complementary filter
                //====================
                hs3003_measurement();
                bmi270_measurement();
                //time elapsed since last measurement
                time_elapsed = (TIMER_MS - k_timer_remaining_get(&gyr_timer))/1000.0;
                k_timer_start(&gyr_timer,K_MSEC(TIMER_MS),K_NO_WAIT);
                //accelerometer measurements to roll and pitch
                acc_roll = -atan2(acc[0],acc[2]);//outputs radians
                acc_pitch = asinf(fmaxf(-1.0f,fminf(1.0f,acc[1])));//no need to divid by 9.81 since working in g's
                //gyr measurements to roll pitch yaw
                //must conver to radians
                gyr_roll = gyr_roll_prev + (gyr[1]*PI/180.0)*time_elapsed;
                gyr_pitch = gyr_pitch_prev + (gyr[0]*PI/180.0)* time_elapsed;
                gyr_yaw = gyr_yaw_prev + (gyr[2]*PI/180.0)*time_elapsed;
                //current value into old value 
                //complementary filter
                roll = alpha * acc_roll +(1-alpha)*gyr_roll;
                pitch = alpha*acc_pitch + (1-alpha)*gyr_pitch;
                yaw = gyr_yaw;
                gyr_roll_prev = roll;
                gyr_pitch_prev = pitch;
                gyr_yaw_prev = yaw;
                printk("%f %f %f\n",roll,pitch,yaw);
                //up vector dot product with 
                // bmm150_measurement();
                //============================================================================
                //Window filter
                //====================
                if(send_flag == 1){
                        printk("sending AT command: %s\n",result);
                        uart_send(uart0_device, result);
                        send_flag = 0;
                }

        }
        return 0;
}
void timer_cb(struct k_timer *timer)
{
        //start timer again if it ends for any reason
        if(timer == &gyr_timer)
        {
                printk("Timer expired fault occured!\n");
        }
}