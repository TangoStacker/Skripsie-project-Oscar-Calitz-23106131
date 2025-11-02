/*
all sensors on &i2c1 (the only i2c device)
humidity and temp sensor --> HS3003 default address 0x44

*/

#include "sensors.h"
#include "initdata.h"
//registers for bmi270
#define ID 0X00
#define PWR_CONFIG 0X7C
#define INIT_CTRL 0X59
#define INIT_DATA 0X5E
#define INIT_ADDR_0 0X5B
#define INIT_ADDR_1 0X5C
#define INTERNAL_STATUS 0X21
#define PWR_CTRL 0X7D
#define ACC_CONF 0X40
#define GYR_CONF 0X42
//4 byte temp and humid data
uint8_t hs3003_raw_data[4];
double acc[3] = {0}; //xyz
double gyr[3] = {0}; //xyz
double mag[3] = {0};//xyz
//HS3003 sensor 
double hs3003_humidity = 0;
double hs3003_temperature = 0;
//BMI270 sensor

//BMI270 settings

//initialise i2c
const struct i2c_dt_spec dev_i2c_hs3003 = I2C_DT_SPEC_GET(DT_NODELABEL(hs3003));
const struct i2c_dt_spec dev_i2c_bmi270 = I2C_DT_SPEC_GET(DT_NODELABEL(bmi270));
const struct i2c_dt_spec dev_i2c_bmm150 = I2C_DT_SPEC_GET(DT_NODELABEL(bmm150));
const struct device *const dev_bmm150 = DEVICE_DT_GET_ONE(bosch_bmm150);
int initialise_i2c(void);
int i2c_reg_byte_write_read(const struct i2c_dt_spec dev,uint8_t reg,uint8_t data);
void bmi270_measurement(void);
void hs3003_measurement(void);
void hs3003_process_data(const uint8_t *raw_data, double *humidity, double *temperature);
void init_data_address_increment(uint16_t iteration);
void i2c_request(const struct i2c_dt_spec dev, uint8_t *store_raw_data );
void i2c_send(const struct i2c_dt_spec dev, const uint8_t *msg);
int i2c_write_byte(const struct i2c_dt_spec dev, uint8_t addr, uint8_t value);
int i2c_read_byte(const struct i2c_dt_spec dev, uint8_t reg,uint8_t *store_target);
int config_bmi270(void);
int config_bmm150(void);
void bmm150_measurement(void);
//HS3003 temperature and humidity functions using I2C
int initialise_i2c(void)
{
    //check that device is ready to operate
    //HS3003
    uint8_t err = 0;
   
    if(!device_is_ready(dev_i2c_hs3003.bus))
    {
        printk("the hs3003 i2c device is not ready \n");
        return 0;
    }
    //BMI270
    if(!device_is_ready(dev_i2c_bmi270.bus)){
        printk("the bmi270 i2c device is not ready \n");
        return 0;
    }
    if(!device_is_ready(dev_i2c_bmm150.bus)){
        printk("the bmm150 i2c device is not ready \n");
        return 0;
    }
    if(device_is_ready(dev_bmm150)){
        printk("the bmm150 is not ready\n");
    }
    //config bmi270
    err = config_bmi270();if(err !=1){return 0;}
    //config bmm150
    err = config_bmm150();if(err !=1){return 0;}
    printk("I2C initialised\n");
    return 1;
}
void i2c_send(const struct i2c_dt_spec dev, const uint8_t *msg)
{
    //send custom message to i2c device
    int ret = i2c_write_dt(&dev,msg , sizeof(msg));
    //if send was unuseccesful
    if(ret != 0)
    {
            printk("failed to send \n");
    }
}
int i2c_write_byte(const struct i2c_dt_spec dev, uint8_t reg, uint8_t value){
    uint8_t err = 0xFF;
    err = i2c_reg_write_byte_dt(&dev,reg,value);
    if(err != 0)
    {
        printk("byte write errorin address 0x%X\n",reg);
        return 0;
    }
    return 1;
}
int i2c_read_byte(const struct i2c_dt_spec dev, uint8_t reg,uint8_t *store_target)
{
    uint8_t err = 0;
    err = i2c_reg_read_byte_dt(&dev,reg,store_target);
    if(err != 0)
    {
        printk("read fail reg 0x%X\n",reg);
        return 0;
    }
    return 1;
}
void i2c_request(const struct i2c_dt_spec dev, uint8_t *store_raw_data )
{
    //read data from i2c device and specify storage array location
    int ret = i2c_read_dt(&dev, store_raw_data , sizeof(store_raw_data));
    //report error if read was unsuccesfull
    if(ret != 0)
    {
            printk("failed to send \n");
    } 
    
}
void init_data_address_increment(uint16_t iteration){
    uint16_t init_val=128*iteration;
    // printk("increment address: %d\n",init_val);
    uint8_t init_data_0_value = (uint8_t)(init_val & 0x0F);
    // printk("init_add_0 = %X \n",init_data_0_value);
    uint8_t init_data_1_value = (uint8_t)((init_val >> 4) & 0xFF);
    // printk("init_add_1 = %X \n",init_data_1_value);
    i2c_reg_write_byte_dt(&dev_i2c_bmi270,INIT_ADDR_0,init_data_0_value);
    i2c_reg_write_byte_dt(&dev_i2c_bmi270,INIT_ADDR_1,init_data_1_value);
}
int i2c_reg_byte_write_read(const struct i2c_dt_spec dev,uint8_t reg,uint8_t data)
{
    //returns 0 if error
    uint8_t temp = 0xFF;
    uint8_t err = 0xFF;
    err = i2c_reg_write_byte_dt(&dev,reg,data);
        if(err != 0)
        {
            printk("byte write fail power save\n");
            return 0;
        }
    err = i2c_reg_read_byte_dt(&dev_i2c_bmi270,reg,&temp);
        if(err != 0)
        {
            printk("write fail\n");
            return 0;
        }
        printk("BMI270 reg 0x%X: 0x%X \n",reg,temp);
    return 1;
}
void hs3003_process_data(const uint8_t *raw_data, double *humidity, double *temperature)
{
    uint16_t hum_word;
    uint16_t temp_word;

    double x;
    
    // 1. Combine Humidity Bytes (raw_data[0] and raw_data[1])
    // The two MSBs of raw_data[0] are status bits and must be masked (0x3F) and moved left by 8 then append the other byte.
    hum_word = (uint16_t)((raw_data[0] & 0x3F) << 8 | raw_data[1]);
    x = hum_word/16383.0*100.0;
    // printk("x hum is %f",x);
    *humidity = x; // Now holds the pure 14-bit value (0 to 16383)

    // 2. Combine Temperature Bytes (raw_data[2] and raw_data[3])
    // The two LSBs of raw_data[3] are undetermined and must be masked (0xFC).
    temp_word = (uint16_t)(raw_data[2] << 8 | (raw_data[3] & 0xFC));

    // The raw 14-bit temperature value is bit-shifted 2 positions left (Temp[15:2] in the datasheet)
    // To get the pure 14-bit value (0 to 16383), we must shift the combined word 2 positions right.
    temp_word = temp_word >>2;
    x = temp_word/16383.0*165.0-40.0;
        // printk("x temp is %f",x);
    *temperature = x; 
    
}
void hs3003_measurement(void)
{
    i2c_send(dev_i2c_hs3003,NULL);
    k_msleep(35);
    i2c_request(dev_i2c_hs3003,hs3003_raw_data);
    k_msleep(10);
    hs3003_process_data(hs3003_raw_data,&hs3003_humidity,&hs3003_temperature);
    return;
}
int config_bmi270(void)
{
    uint8_t err = 0;
    uint8_t read_data = 0;
    //read chip ID
    err = i2c_read_byte(dev_i2c_bmi270,ID,&read_data); if(err !=1){return 0;}
    //disable power save
    err = i2c_write_byte(dev_i2c_bmi270,PWR_CONFIG,0X00);if(err !=1){return 0;}
    k_msleep(10);//sleep for min 450 us
    //start initialisation
    err = i2c_write_byte(dev_i2c_bmi270,INIT_CTRL,0x00);if(err !=1){return 0;}
    k_msleep(10);
    //write config file
    uint8_t data_array[257] = {0};
    // printk("size = %d\n",size);
    for (uint16_t i = 0; i < 32; i++)
    { 
        //increment address (starts with reset on i = 0)
        //write to init_data0 and init_data_1 to increment registers. Write bytes/2
        init_data_address_increment(i);
        // creates 256 byte array with address at start
        data_array[0] = INIT_DATA;
        for (uint16_t x = 0; x < 256; x++) 
        {
            // printk(" x %d\n",x);
            data_array[x + 1] = bmi270_config_file[i*256+x];
        }
        //write that array to that address
        err = i2c_write_dt(&dev_i2c_bmi270,data_array,257);
        if(err != 0)
            {
                printk("write fail\n");
                return 0;
            }
        // i2c_send(dev_i2c_bmi270,data_array); 
    }
    // // printk("internal error:%X 0 is good\n",test);
    //end init
    k_msleep(10);
    err = i2c_write_byte(dev_i2c_bmi270,INIT_CTRL,0x01);if(err !=1){return 0;}
    //wait min 20ms
    k_msleep(100);
    // check initiation status
    err = i2c_read_byte(dev_i2c_bmi270,INTERNAL_STATUS,&read_data);if(err !=1){return 0;}  
    k_msleep(10);
    //config bmi for acc gyro and temp (performance mode)
    err = i2c_write_byte(dev_i2c_bmi270,PWR_CTRL,0x0E);if(err !=1){return 0;}
    k_msleep(10);
    //acc config bandwidth and filter
    err = i2c_write_byte(dev_i2c_bmi270,ACC_CONF,0XA8);if(err !=1){return 0;}
    k_msleep(10);
    //gyr config noise filter, bandwidth 
    err = i2c_write_byte(dev_i2c_bmi270,GYR_CONF,0xE9);if(err !=1){return 0;}          
    k_msleep(10);
    //fast power up config
    err =i2c_write_byte(dev_i2c_bmi270,PWR_CONFIG,0x02);if(err !=1){return 0;}
    k_msleep(10);
    // i2c_reg_byte_write_read(dev_i2c_bmi270,0x41,0x03);
    return 1;
}
void bmi270_measurement(void)
{
    //default accell range is 16g g= 9.81m/s
    uint8_t bmi270_raw_data[12] = {0};
    uint8_t err = 0;
    uint8_t gyr_factor = 0;
    uint8_t acc_range = 0;
    uint16_t gyr_range = 0;
    int16_t val =0;
    i2c_read_byte(dev_i2c_bmi270,0x3c,&gyr_factor);
    gyr_factor = gyr_factor & 0x3F;
    i2c_read_byte(dev_i2c_bmi270,0x41,&err);
    switch (err &0x03)
    {
    case 0x00:
        acc_range = 2;
        break;
    case 0x01 :
        acc_range = 4;
        break;
    case 0x02:
        acc_range = 8;
        break;
    case 0x03:
        acc_range = 16;
        break;
    default:
        printk("acc range error");
        break;
    }
    i2c_read_byte(dev_i2c_bmi270,0x43,&err);
    switch (err&0x07)
    {
    case 0:
        gyr_range = 2000;
        break;
    case 1 :
        gyr_range = 1000;
        break;
    case 2:
        gyr_range = 500;
        break;
    case 3:
        gyr_range = 250;
        break;
    case 4:
        gyr_range = 125;
        break;
    default:
        printk("gyr range error");
        break;
    }
    // printk("acc range: %d gyr range %d",acc_range,gyr_range);
    err = i2c_burst_read_dt(&dev_i2c_bmi270,0x0C,bmi270_raw_data,12);
    if(err!=0){
        printk("burst read error in bmi270\n");
    }
    //RAW DATA IS TWO'S COMPLIMENT
    // printk("raw data : %X %X %X %X %X %X %X %X %X %X %X %X  \n",bmi270_raw_data[0],bmi270_raw_data[1],bmi270_raw_data[2],bmi270_raw_data[3],bmi270_raw_data[4],bmi270_raw_data[5],bmi270_raw_data[6],bmi270_raw_data[7],bmi270_raw_data[8],bmi270_raw_data[9],bmi270_raw_data[10],bmi270_raw_data[11]);
    val = (int16_t)(bmi270_raw_data[1] << 8| bmi270_raw_data[0]);
    acc[0] = (double)(val)/(32768.0)*(double)(acc_range);
    val = (int16_t)(bmi270_raw_data[3] << 8| bmi270_raw_data[2]);
    acc[1] = (double)(val)/32768.0*(double)(acc_range);
    val = (int16_t)(bmi270_raw_data[5] << 8| bmi270_raw_data[4]);
    acc[2] = (double)(val)/32768.0*(double)(acc_range);
    
    val = (int16_t)(bmi270_raw_data[7] << 8| bmi270_raw_data[6]);
    gyr[0] =  (double)val; //g1 contains unconverted g1 val
    val = (int16_t)(bmi270_raw_data[11] << 8| bmi270_raw_data[10]);//now ive got unconverted g2 in val
    gyr[2] =  (double)(val)/32768.0*(double)(gyr_range);
    //ug1-factor*(ug2)/2^9 (cross coupling z and x)
    gyr[0] = (gyr[0]-(double)(gyr_factor)*(double)val/512)/32768.0*(double)(gyr_range);
    val = (int16_t)(bmi270_raw_data[9] << 8| bmi270_raw_data[8]);
    gyr[1] =  (double)(val)/32768.0*(double)(gyr_range);
    //accuray in g's gyroscope in degrees/s
    return;
}
int config_bmm150(void)
{
    uint8_t err = 0;
    //put to sleep mode from suspend mode
    err =i2c_write_byte(dev_i2c_bmm150,0x4C,0x02);if(err !=1){return 0;}
    k_msleep(500);
    //put into normal mode to constantly measure
    err = i2c_write_byte(dev_i2c_bmm150,0x4C,0x00);if(err !=1){return 0;}
    return 1;
}

void bmm150_measurement(void)
{
    uint8_t err = 0;
    uint8_t data[6] = {0};
    int16_t val = 0;
    //x and y axis is 13 bits, stored in 2's compliment z axis is 15bits
    err = i2c_burst_read_dt(&dev_i2c_bmm150,0x42,data,sizeof(data));
    if(err!=0)
    {
            printk("burst read error in bmm150\n");
    }
    //x
    val = (int16_t)((data[1]) << 8 | (data[0] & 0x1F)<<3);
    mag[0] = (double)(val)/8191.0/8.0*1300;//to fill sign from the 13th sign bit then divide by 8 to get the original number
    //y
    val = (int16_t)((data[3]) << 8 | (data[2] & 0x1F)<<3);
    mag[1]= (double)((val))/8191.0/8.0*1300;
    //z 15 bits
    val = (int16_t)((data[5]) << 8 | (data[4] & 0x1F)<<1);
    mag[2]= (double)((val))/32767/2.0*2500;
}

// void bmm150_measurement(void)
// {
//     int err = 0;
//     struct sensor_value x, y, z;
//     err = sensor_sample_fetch(dev_bmm150);
//     if(err)
//     {
//         printk("sample fetch fail");
//     }
//     err = sensor_channel_get(dev_bmm150,SENSOR_CHAN_MAGN_X,&x);
//     err = sensor_channel_get(dev_bmm150,SENSOR_CHAN_MAGN_Y,&y);
//     err = sensor_channel_get(dev_bmm150,SENSOR_CHAN_MAGN_Z,&z);
//     printk("( x y z ) = ( %f  %f  %f )\n", sensor_value_to_double(&x),sensor_value_to_double(&y), sensor_value_to_double(&z));//in uT *100 for GAUS
// }
