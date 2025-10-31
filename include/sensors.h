#ifndef sensors_H
#define sensors_H
/*&i2c1 {
	hts221: hts221@5f {
		compatible = "st,hts221"; --> rev 2 has HS3003
		status = "okay";
		reg = <0x5f>;
	};
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>//device node for sensor hs3003
#include <zephyr/drivers/sensor.h>
#define BMI270_CONFIG_START_REG 0x5E
//device struct, normal i2c device describes as hs3003
extern const struct i2c_dt_spec dev_i2c_hs3003;
extern const struct i2c_dt_spec dev_i2c_bmi270;
extern const struct i2c_dt_spec dev_i2c_bmm150;
extern const struct device *const dev_bmm150;
//global variables containing humidity and temperature readings
extern double hs3003_humidity; //%
extern double hs3003_temperature; //degrees C
extern double acc[3];//xyz
extern double gyr[3];//xyz
extern double mag[3];//xyz
//fucntions
int initialise_i2c(void);
//hs3003 measurement into global variable
void hs3003_measurement(void);
//bmi270 measurement into globa variable
void bmi270_measurement(void);
//bmm150 measurment into global var
void bmm150_measurement(void);
#endif