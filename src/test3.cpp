#include <bcm2835.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <ros/ros.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "raspberry_reading");

    bcm2835_init();
    bcm2835_i2c_begin();

    char addr = 0x68;
    //I found this address somewhere in the internet...
    char buf[1];
    char regaddr[2];
    int x = 0;
    int ret;

    bcm2835_i2c_setSlaveAddress(addr);

    //disable sleep mode!!!!!
    regaddr[0] = 107;
    regaddr[1] = 0;
    //This is the basic operation to write to an register
    //regaddr[0] is the register address
    //regaddr[1] is the value
    bcm2835_i2c_write(regaddr, 2);
   
    regaddr[0] = 59;
    ret = BCM2835_I2C_REASON_ERROR_DATA;
    while(ret != BCM2835_I2C_REASON_OK)
    {
        //This is the basic operation to read an register
        //regaddr[0] is the register address
        //buf[0] is the value
        bcm2835_i2c_write(regaddr, 1);
        ret = bcm2835_i2c_read(buf, 1);
    }
    x = buf[0]<<8;

    regaddr[0] = 60;
    ret = BCM2835_I2C_REASON_ERROR_DATA;
    while(buf[0] == 99)
    {
        bcm2835_i2c_write(regaddr, 1);
        ret = bcm2835_i2c_read(buf, 1);
    }
    x += buf[0];

    //because of the sign, we have here 32-bit integers,
    //the value is 16-bit signed.

    if (x & 1<<15)
    {
        x -= 1<<16;
    }

    double x_val = x;
    x_val = x_val / 16384;
    //This is only valid if the accel-mode is +- 2g
    //The range can be controlled via the 
    //GYRO_CONFIG and ACCEL_CONFIG registers

    printf("accel: %g\n", x_val);

    bcm2835_i2c_end();
    
    ros::spin();

}