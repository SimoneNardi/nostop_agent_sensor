#include <ros/ros.h>
#include <Raspberry_sensor_reader.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <bcm2835/src/bcm2835.h>


using namespace std;
using namespace Robotics;
using namespace GameTheory;


Raspberry_sensor_reader::Raspberry_sensor_reader():
m_value(0)
{
  bcm2835_init();
  bcm2835_i2c_begin();

  bcm2835_i2c_setSlaveAddress(m_address);
  m_addr[0] = 107;
  m_addr[1] = 0;
  bcm2835_i2c_write(m_addr, 2);
  m_ret = BCM2835_I2C_REASON_ERROR_DATA;
}


Raspberry_sensor_reader::~Raspberry_sensor_reader()
{    bcm2835_i2c_end();}


void Raspberry_sensor_reader::run()//TODO
{
    while(m_ret != BCM2835_I2C_REASON_OK)
    {
      bcm2835_i2c_write(m_addr, 1);
      m_ret = bcm2835_i2c_read(m_buf, 1);
    }
    m_value = m_buf[0]<<8;
    m_addr[0] = 60;
    m_ret = BCM2835_I2C_REASON_ERROR_DATA;
    while(m_buf[0] == 99)
    {
      bcm2835_i2c_write(m_addr, 1);
      m_ret = bcm2835_i2c_read(m_buf, 1);
    }
    m_value += m_buf[0];
    //because of the sign, we have here 32-bit integers,
    //the value is 16-bit signed.
    if (m_value & 1<<15)
    {
      m_value -= 1<<16;
    }
    double final_val = m_value;
    final_val = final_val / 16384;
    printf("accel: %g\n", final_val);
}
