#ifndef IRIDIUM_H
#define IRIDIUM_H

#include <string>
#include <vector>
#include "logtdt.h"

extern "C"{
    #include "tis.h"
}

class Iridium
{
public:
    /**
   * @brief Iridium
   */
    Iridium();

    /**
     * @brief send_and_receive_data
     * @param files
     * @param files_count
     */
    bool send_and_receive_data();

    /**
     * @brief serialize_log_TDT1
     * @return
     */
    bool serialize_log_TDT1();

    /**
     * @brief iridium_power
     * @param enable
     * @return
     */
    bool iridium_power(const bool &enable);

    /**
     * @brief enable_com
     * @param val
     */
    void enable_com(bool val);

    /**
     * @brief uart_init
     * @param fd
     * @return
     */
    int32_t uart_init();

    /**
     * @brief get_new_tdt_file
     * @return
     */
    const std::string get_new_tdt_file();

    /**
     * @brief is_demo_mode
     * @return
     */
    bool is_demo_mode() const;

    /**
     * @brief Iridium::get_new_log_files
     */
    void get_new_log_files();

    /**
     * @brief process_cmd_file
     * @param file_name
     */
    void process_cmd_file(const string &file_name);

public:
    bool m_iridium_power_state = false;

    LogTDT logTDT;
    std::vector<LogTDT> m_cmd_list;

private:
    uint64_t m_imei = 300234065392110;
    std::string m_path_received = "iridium/received";
    std::string m_path_received_full = "";
    std::string m_path_send = "iridium/send";
    unsigned int m_gpio_power = 5;

    int m_transmission_number_attempt = 10;
    int m_transmission_sleep_time = 30;

    std::vector<std::string> m_files_to_send;

    TIS_properties m_tis;
    int m_uart_fd = 0;

    bool m_enable_iridium = false;

    bool m_demo_mode = false;
};

int32_t uart_init(int &fd);
int32_t uart_send_data(void *serial_struct, uint8_t *data, int32_t count);
int32_t uart_receive_data(void *serial_struct, uint8_t *data, int32_t count);
int32_t uart_wait_data(void *serial_struct, uint32_t timeout);
int32_t uart_flush_TX(void *serial_struct);
int32_t uart_flush_RX(void *serial_struct);
int32_t uart_release(void *serial_struct);

inline void Iridium::enable_com(bool val){
    m_enable_iridium = val;
}

inline bool Iridium::is_demo_mode()const{
  return m_demo_mode;
}

#endif // IRIDIUM_H
