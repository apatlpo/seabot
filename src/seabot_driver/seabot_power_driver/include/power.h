#ifndef POWER_H
#define POWER_H

#include <sys/types.h>
#include <iostream>
#include <fstream>

extern "C" {
    #include <linux/i2c-dev.h>
}
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,8,0)
extern "C" {
    #include <i2c/smbus.h>
}
#endif

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>

#include <ros/ros.h>

// 3.5*5/1024
// 5V = 1024
// pont diviseur de tension rapport 3.5
#define ADC_BATTERY_LEVEL_CONV 0.017090

class Power
{
public:
  Power();
  ~Power();

  /**
   * @brief i2c_open
   * @return
   */
  int i2c_open();

  /**
   * @brief enable the flash
   * @param val
   */
  void set_flash_enable(const bool &val) const;

  /**
   * @brief set_nb_flash
   * @param nb of flash
   */
  void set_nb_flash(const unsigned char &nb) const;

//  /**
//   * @brief set_flash_enable_with_delay
//   * @param dt
//   */
//  void set_flash_enable_with_delay(const unsigned char &dt) const;

  /**
   * @brief set_sleep_mode
   */
  void set_sleep_mode() const;

  /**
   * @brief stop_sleep_mode
   */
  void stop_sleep_mode() const;

  /**
   * @brief set_sleep_mode_countdown
   * @param hours
   * @param min
   * @param sec
   * @param sec_to_stop
   */
  void set_sleep_mode_countdown(const unsigned char &hours, const unsigned char &min, const unsigned char &sec, const unsigned char &sec_to_stop=60) const;

  /**
   * @brief measure_battery
   */
  void get_batteries();

  /**
   * @brief get_level_battery
   * @param id
   * @return
   */
  const float &get_level_battery(size_t id) const;

  /**
   * @brief get_version
   */
  uint8_t& get_version();

private:
  int m_file;
  const int m_i2c_addr = 0x38;
  const char* m_i2c_periph = "/dev/i2c-1";

  float m_level_battery[4] =  {0.0, 0.0, 0.0, 0.0};

  uint8_t m_version=0;
};

inline const float& Power::get_level_battery(size_t id) const{
  return m_level_battery[id];
}

#endif // POWER_H
