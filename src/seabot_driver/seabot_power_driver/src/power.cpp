#include "power.h"
#include <sys/ioctl.h>

Power::Power(){

}

Power::~Power(){
  set_flash_enable(false);
  close(m_file);
}

int Power::i2c_open(){
  m_file = open(m_i2c_periph,O_RDWR);
  if (m_file < 0) {
    ROS_WARN("[Piston_driver] Failed to open the I2C bus (%s) - %s", m_i2c_periph, strerror(m_file));
    exit(1);
  }

  int rslt = ioctl(m_file,I2C_SLAVE,m_i2c_addr);
  if (rslt < 0) {
    ROS_WARN("[Piston_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE, strerror(rslt));
    exit(1);
  }
  usleep(100000);
  return 0;
}

void Power::set_sleep_mode_countdown(const unsigned char &hours, const unsigned char &min, const unsigned char &sec, const unsigned char &sec_to_stop) const{
  uint8_t buff[4];
  buff[0] = hours;
  buff[1] = min;
  buff[2] = sec;
  buff[3] = sec_to_stop;

  //if(i2c_smbus_write_i2c_block_data(m_file, 0x03, 4,buff)<0)
  //  ROS_WARN("[Power_driver] I2C Bus Failure - Set Sleep mode countdown");
}

void Power::set_nb_flash(const unsigned char &nb) const{
  //if(i2c_smbus_write_byte_data(m_file, 0x02, nb)<0)
  //  ROS_WARN("[Power_driver] I2C Bus Failure - Set Flash Led Delay");
}

void Power::set_flash_enable(const bool &val) const{
  //if(i2c_smbus_write_byte_data(m_file, 0x01, val?0x01:0x00)<0)
  //  ROS_WARN("[Power_driver] I2C Bus Failure - Set Flash Led");
}

//void Power::set_flash_enable_with_delay(const unsigned char &dt) const{
//    __u16 val = (0x01 | (dt<<8));
//    if(i2c_smbus_write_word_data(m_file, 0x01, val))
//      ROS_WARN("[Power_driver] I2C Bus Failure - Set Flash and Period Led");
//}

void Power::set_sleep_mode() const{
  //if(i2c_smbus_write_byte_data(m_file, 0x00, 0x02)<0)
  //  ROS_WARN("[Power_driver] I2C Bus Failure - Set Sleep Mode");
}

void Power::stop_sleep_mode() const{
  //if(i2c_smbus_write_byte_data(m_file, 0x00, 0x01)<0)
  //  ROS_WARN("[Power_driver] I2C Bus Failure - Stop Sleep Mode");
}

void Power::get_batteries(){
  uint8_t buff[2];
  if(i2c_smbus_read_i2c_block_data(m_file, 0xB0, 2,buff) != 2)
    ROS_WARN("[Power_driver] I2C Bus Failure - Get Batteries");

  m_level_battery[0] = (buff[0] | buff[1] << 8) * ADC_BATTERY_LEVEL_CONV;
  m_level_battery[1] = (buff[0] | buff[1] << 8) * ADC_BATTERY_LEVEL_CONV;
  m_level_battery[2] = (buff[0] | buff[1] << 8) * ADC_BATTERY_LEVEL_CONV;
  m_level_battery[3] = (buff[0] | buff[1] << 8) * ADC_BATTERY_LEVEL_CONV;
}

uint8_t& Power::get_version(){
  //m_version = i2c_smbus_read_byte_data(m_file, 0xC0);
  //usleep(100);
  return m_version;
}




