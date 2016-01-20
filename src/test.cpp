#include "I2Cdev/I2Cdev.h"
#include "ros/ros.h"
#include "Math.h"

uint8_t power_mgmt_1 = 0x6b;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "raspberry_test");
  uint8_t address = 0x68;
  int8_t a;
  uint8_t *data;
  uint16_t timeout = I2Cdev::readTimeout;
  a = I2Cdev::readByte(address,power_mgmt_1,data,timeout);
  
//   bus.write_byte_data(address,power_mgmt_1,0);// wake up the 6050
  float accx;
  while(true)
  {

  }
}