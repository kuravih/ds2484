#ifndef __DS2484_H__
#define __DS2484_H__

#pragma once
#include <stdint.h>
#include <cstring>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <sys/ioctl.h>
#include <fcntl.h>

extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h> // from i2clib-dev
}

#include "kato/function.hpp"
#include "kato/log.hpp"

// ====================================================================================================================
struct _sample
{
    uint64_t rom = 0;
    float temperature = 0;
    char timestamp[12] = {0};
    _sample() = default;
    _sample(const uint64_t _rom, const float _temperature, std::string _timestamp = kato::function::TimeStampString(3, "%d%H%M%S", "")) : rom(_rom), temperature(_temperature)
    {
        std::memcpy(timestamp, _timestamp.c_str(), 12);
    }
    void set_timestamp(std::string _timestamp = kato::function::TimeStampString(3, "%d%H%M%S", ""))
    {
        std::memcpy(timestamp, _timestamp.c_str(), 12);
    }
};
typedef struct _sample sample_t;

inline std::ostream &operator<<(std::ostream &_outs, const sample_t &_sample)
{
    return _outs << kato::function::StringPrintf("0x%016llX", _sample.rom) << "," << kato::function::StringPrintf("%+09.4f", _sample.temperature) << "," << _sample.timestamp;
}

inline const std::string to_string(const sample_t &_sample)
{
    std::ostringstream oss;
    oss << _sample;
    return oss.str();
}
// ====================================================================================================================

// #define DS2484_DEBUG

#define DS2484_ADDRESS 0x18
#define DS2484_I2C_BUS_PREFIX "/dev/i2c-"

/*
 * The DS2482 registers - there are 3 registers that are addressed by a read
 * pointer. The read pointer is set by the last command executed.
 *
 * To read the data, issue a register read for any address
 */
#define DS2484_CMD_RESET 0xF0            /* No param */
#define DS2484_CMD_SET_READ_PTR 0xE1     /* Param: DS2484_PTR_CODE_xxx */
#define DS2484_CMD_WRITE_CONFIG 0xD2     /* Param: Config byte */
#define DS2484_CMD_1WIRE_RESET 0xB4      /* Param: None */
#define DS2484_CMD_1WIRE_SINGLE_BIT 0x87 /* Param: Bit byte (bit7) */
#define DS2484_CMD_1WIRE_WRITE_BYTE 0xA5 /* Param: Data byte */
#define DS2484_CMD_1WIRE_READ_BYTE 0x96  /* Param: None */
#define DS2484_CMD_1WIRE_TRIPLET 0x78    /* Param: Dir byte (bit7) */
#define DS2484_CMD_COMMAND_SKIP 0xCC

/* Values for DS2482_CMD_SET_READ_PTR */
#define DS2484_PTR_CODE_DEV_CONFIG 0xC3
#define DS2484_PTR_CODE_STATUS 0xF0
#define DS2484_PTR_CODE_DATA 0xE1
#define DS2484_PTR_CODE_PORT_CONFIG 0xB4

/*
 * Configure Register bit definitions
 * The top 4 bits always read 0.
 * To write, the top nibble must be the 1's compl. of the low nibble.
 */
#define DS2484_REG_CFG_1WS 0x08 /* 1-wire speed */
#define DS2484_REG_CFG_SPU 0x04 /* strong pull-up */
#define DS2484_REG_CFG_APU 0x01 /* active pull-up */

/*
 * Status Register bit definitions (read only)
 */
#define DS2484_REG_STS_SBR 0x20
#define DS2484_REG_STS_RST 0x10
#define DS2484_REG_STS_LL 0x08
#define DS2484_REG_STS_SD 0x04  /* Short Detected */
#define DS2484_REG_STS_PPD 0x02 /* Presence-Pulse Detect */
#define DS2484_REG_STS_1WB 0x01 /* 1-Wire Busy */

#define DS28EA00_FAMILY_CODE 0x42
#define DS28EA00_CMD_CONVERT_T 0x44
#define DS28EA00_CMD_MATCH_ROM 0x55
#define DS28EA00_CMD_READ_SCRATCHPAD 0xBE
#define DS28EA00_PIO 0xA5

// ====================================================================================================================
class DS2484
{
public:
    DS2484(const char *_dev, bool scan = true, const uint8_t address = DS2484_ADDRESS) : dev(_dev), m_address(address)
    {
        begin();
        read_device_config();
        read_status();
        read_port_config();
        reset();
        if (onewire_reset() < 0)
            return;
        // write_device_config(DS2484_REG_CFG_1WS);
        if (scan)
            scan_for_sensors();
    }
    // ----------------------------------------------------------------------------------------------------------------
    ~DS2484() = default;
    // ----------------------------------------------------------------------------------------------------------------
    void led(const uint64_t _rom, const bool &_on_off)
    {
        // Select the DS18B20 device
        onewire_reset();
        onewire_write_byte(DS28EA00_CMD_MATCH_ROM); // Match ROM command
        uint8_t address[8];
        std::memcpy(address, &_rom, 8);
        for (int i = 0; i < 8; i++)
            onewire_write_byte(address[i]);

        onewire_write_byte(DS28EA00_PIO);
        if (_on_off)
        {
            onewire_write_byte(0b00000010);
            onewire_write_byte(0b11111101);
        }
        else
        {
            onewire_write_byte(0b11111101);
            onewire_write_byte(0b00000010);
        }
    }
    // ----------------------------------------------------------------------------------------------------------------
    void start_conversion(bool reset = true)
    {
        if (reset)
            onewire_reset();

        onewire_write_byte(DS2484_CMD_COMMAND_SKIP);

        onewire_write_byte(DS28EA00_CMD_CONVERT_T); // Convert T command

        std::this_thread::sleep_for(std::chrono::milliseconds(750)); // Sleep for a short duration
    }
    // ----------------------------------------------------------------------------------------------------------------
    void start_conversion(const uint64_t _rom, bool reset = true)
    {
        if (reset)
            onewire_reset();

        onewire_write_byte(DS28EA00_CMD_MATCH_ROM); // Match ROM command
        uint8_t address[8] = {0};
        memcpy(address, &_rom, 8);
        for (int i = 0; i < 8; i++)
            onewire_write_byte(address[i]);

        onewire_write_byte(DS28EA00_CMD_CONVERT_T); // Convert T command

        std::this_thread::sleep_for(std::chrono::milliseconds(750)); // Sleep for a short duration
    }
    // ----------------------------------------------------------------------------------------------------------------
    void read_temperature(const uint64_t _rom, int16_t &_temperature_c, bool convert = true, bool reset = true)
    {
        if (convert)
            start_conversion(_rom, reset);

        // Read scratchpad
        onewire_reset();
        onewire_write_byte(DS28EA00_CMD_MATCH_ROM); // Match ROM command
        uint8_t address[8] = {0};
        memcpy(address, &_rom, 8);
        for (int i = 0; i < 8; i++)
            onewire_write_byte(address[i]);

        onewire_write_byte(DS28EA00_CMD_READ_SCRATCHPAD); // Read Scratchpad command
        uint8_t data[9] = {0};
        for (int i = 0; i < 9; i++)
            onewire_read_byte(data[i]);

        _temperature_c = (data[1] << 8) | data[0];
    }
    // ----------------------------------------------------------------------------------------------------------------
    void read_temperature(const uint64_t _rom, float &_temperature_c, bool convert = true, bool reset = true)
    {
        int16_t _temperature_c_int16;
        read_temperature(_rom, _temperature_c_int16, convert, reset);
        _temperature_c = (float)_temperature_c_int16 / 16.0; // Calculate temperature
    }
    // ----------------------------------------------------------------------------------------------------------------
    void scan_for_sensors()
    {
        samples.clear();
        start_conversion(true);
        while (true)
        {
            uint64_t rom = 0;
            if (!onewire_search(rom))
                break;
            led(rom, true);
            float temperature = 0;
            read_temperature(rom, temperature, false, false);
            led(rom, false);
            sample_t sample(rom, temperature);
            kato::log::cout << KATO_BLUE << "DS2484::" << "scan_for_sensors() on " << dev << " sample " << sample << KATO_RESET << std::endl;
            samples.push_back(sample);
        }
    }
    // ----------------------------------------------------------------------------------------------------------------
    std::vector<sample_t> samples;
    std::string dev;
    // ----------------------------------------------------------------------------------------------------------------
private:
    int m_fd;
    uint8_t m_address;
    uint8_t m_read_prt;
    uint8_t m_dev_config, m_status, m_port_config;
    // global search state
    uint8_t ROM[8] = {0};
    uint8_t _last_discrepancy, _last_family_discrepancy;
    bool _last_dev_flag;
    // ----------------------------------------------------------------------------------------------------------------
    inline bool is_onewire_busy()
    {
        return m_status & 0x01; // Check the 1-Wire busy bit
    }
    // ----------------------------------------------------------------------------------------------------------------
    inline bool is_presence_pulse_detect()
    {
        return m_status & 0x02; // Check the presence pulse detected bit
    }
    // ----------------------------------------------------------------------------------------------------------------
    inline bool is_short_detected()
    {
        return m_status & 0x04; // Check the short detected bit
    }
    // ----------------------------------------------------------------------------------------------------------------
    inline bool is_logic_level()
    {
        return m_status & 0x08; // Check the logic level bit
    }
    // ----------------------------------------------------------------------------------------------------------------
    inline bool is_reset()
    {
        return m_status & 0x10; // Check if reset
    }
    // ----------------------------------------------------------------------------------------------------------------
    inline bool is_single_bit_result()
    {
        return m_status & 0x20; // Check the single bit result bit
    }
    // ----------------------------------------------------------------------------------------------------------------
    inline bool is_triplet_second_bit()
    {
        return m_status & 0x40; // Check the triplet second bit
    }
    // ----------------------------------------------------------------------------------------------------------------
    inline bool is_branch_direction_taken()
    {
        return m_status & 0x80; // Check the branch direction taken bit
    }
    // ----------------------------------------------------------------------------------------------------------------
    int begin()
    {
        m_fd = open(dev.c_str(), O_RDWR);
        if (m_fd < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "begin() " << "open(" << dev << ") failed" << KATO_RESET << std::endl;
#endif
            return -1;
        }
        if (ioctl(m_fd, I2C_SLAVE, m_address) < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "begin() " << "ioctl() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int reset()
    {
        if (i2c_smbus_write_byte(m_fd, DS2484_CMD_RESET) < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "reset() " << "i2c_smbus_write_byte() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (read_status() < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "reset() " << "read_status() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (!is_reset())
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "reset() " << "is_reset() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int set_read_pointer(const uint8_t _read_ptr)
    {
        if (m_read_prt != _read_ptr)
        {
            if (i2c_smbus_write_byte_data(m_fd, DS2484_CMD_SET_READ_PTR, _read_ptr) < 0)
            {
#ifdef DS2484_DEBUG
                kato::log::cout << KATO_RED << "DS2484::" << "set_read_pointer() " << "i2c_smbus_write_byte_data() failed" << KATO_RESET << std::endl;
#endif
                close(m_fd);
                return -1;
            }
            m_read_prt = _read_ptr;
        }
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int read_device_config()
    {
        if (set_read_pointer(DS2484_PTR_CODE_DEV_CONFIG) < 0)
        {
            // #ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "read_device_config() " << "set_read_pointer() failed" << KATO_RESET << std::endl;
            // #endif
            close(m_fd);
            return -1;
        }
        int dev_config = i2c_smbus_read_byte(m_fd);
        if (dev_config < 0)
        {
            // #ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "read_device_config() " << "i2c_smbus_read_byte() failed" << KATO_RESET << std::endl;
            // #endif
            close(m_fd);
            return -1;
        }
        m_dev_config = dev_config;
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int write_device_config(const uint8_t _config)
    {
        m_dev_config |= _config;
        uint8_t dev_config = (m_dev_config & 0x0F) | ((~m_dev_config & 0x0F) << 4);
        if (i2c_smbus_write_byte_data(m_fd, DS2484_CMD_WRITE_CONFIG, dev_config) < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "write_device_config() " << "i2c_smbus_write_byte_data() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int read_status()
    {
        if (set_read_pointer(DS2484_PTR_CODE_STATUS) < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "read_status() " << "set_read_pointer() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        int status = i2c_smbus_read_byte(m_fd);
        if (status < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "read_status() " << "i2c_smbus_read_byte() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        m_status = status;
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int read_port_config()
    {
        if (set_read_pointer(DS2484_PTR_CODE_PORT_CONFIG) < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "read_port_config() " << "set_read_pointer() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        int port_config = i2c_smbus_read_byte(m_fd);
        if (port_config < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "read_port_config() " << "i2c_smbus_read_byte() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        m_port_config = port_config;
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int onewire_reset()
    {
        if (onewire_busy_wait() < 0) // Wait for the bus to be free
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_reset() " << "onewire_busy_wait() failed on bus " << dev << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (i2c_smbus_write_byte(m_fd, DS2484_CMD_1WIRE_RESET) < 0) // Send the 1-Wire reset command
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_reset() " << "i2c_smbus_write_byte() failed on bus " << dev << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (onewire_busy_wait() < 0) // Wait for the bus to be free
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_reset() " << "onewire_busy_wait() failed on bus " << dev << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (read_status() < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_reset() " << "read_status() failed on bus " << dev << KATO_RESET << std::endl;
#endif
        }
        if (is_short_detected())
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_reset() " << "is_short_detected() failed on bus " << dev << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (!is_presence_pulse_detect())
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_reset() " << "is_presence_pulse_detect() failed on bus " << dev << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int onewire_write_bit(const bool _bit)
    {
        if (onewire_busy_wait() < 0) // Wait for the bus to be free
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "read_port_config() " << "i2c_smbus_read_byte() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (i2c_smbus_write_byte_data(m_fd, DS2484_CMD_1WIRE_SINGLE_BIT, (_bit ? 0xFF : 0x00)) < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "read_port_config() " << "i2c_smbus_write_byte_data() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int onewire_read_bit(bool &_bit)
    {
        if (onewire_write_bit(true) < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_read_bit() " << "onewire_write_bit() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (onewire_busy_wait() < 0) // Wait for the bus to be free
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_read_bit() " << "onewire_busy_wait() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (read_status() < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_read_bit() " << "read_status() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        _bit = is_single_bit_result();
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int onewire_write_byte(const uint8_t _byte)
    {
        if (onewire_busy_wait() < 0) // Wait for the bus to be free
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_write_byte() " << "onewire_busy_wait() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (i2c_smbus_write_byte_data(m_fd, DS2484_CMD_1WIRE_WRITE_BYTE, _byte) < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_write_byte() " << "i2c_smbus_write_byte_data() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        // Wait for the write operation to complete
        return onewire_busy_wait(); // Return false if the bus is busy after the timeout
    }
    // ----------------------------------------------------------------------------------------------------------------
    int onewire_read_byte(uint8_t &_byte)
    {
        if (onewire_busy_wait() < 0) // Wait for the bus to be free
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_read_byte() " << "onewire_busy_wait() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (i2c_smbus_write_byte(m_fd, DS2484_CMD_1WIRE_READ_BYTE) < 0) // Send the 1-Wire read byte command
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_read_byte() " << "i2c_smbus_write_byte() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (onewire_busy_wait() < 0) // Wait for the bus to be free
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_read_byte() " << "onewire_busy_wait() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        if (set_read_pointer(DS2484_PTR_CODE_DATA) < 0) // Read the byte from the read data register
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_read_byte() " << "set_read_pointer() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        int byte = i2c_smbus_read_byte(m_fd);
        if (byte < 0)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_read_byte() " << "i2c_smbus_read_byte() failed" << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        _byte = byte;
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------
    int onewire_busy_wait(uint16_t timeout_ms = 100)
    {
        int status = 0, retries = 0;
        if (!set_read_pointer(DS2484_PTR_CODE_STATUS))
        {
            do
            {
                status = i2c_smbus_read_byte(m_fd);
                if (status < 0)
                {
#ifdef DS2484_DEBUG
                    kato::log::cout << KATO_RED << "DS2484::" << "onewire_busy_wait() " << "i2c_smbus_read_byte() failed on bus " << dev << KATO_RESET << std::endl;
#endif
                    close(m_fd);
                    return -1;
                }
                m_status = status;
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Sleep for a short duration
            } while (is_onewire_busy() && (++retries < timeout_ms));
        }
        if (retries >= timeout_ms)
        {
#ifdef DS2484_DEBUG
            kato::log::cout << KATO_RED << "DS2484::" << "onewire_busy_wait() " << "set_read_pointer() timed out on bus " << dev << KATO_RESET << std::endl;
#endif
            close(m_fd);
            return -1;
        }
        return 0;
    }
    // ----------------------------------------------------------------------------------------------------------------

public:
    bool onewire_search(uint64_t &_rom)
    {
        uint8_t id_bit_number = 1;
        uint8_t last_zero = 0, rom_byte_number = 0, rom_byte_mask = 1;
        bool search_result = false, search_direction, id_bit, cmp_id_bit;

        if (!_last_dev_flag) // if the last call was not the last one
        {
            if (onewire_reset() < 0) // 1-Wire reset
            {
#ifdef DS2484_DEBUG
                kato::log::cout << KATO_RED << "DS2484::" << "onewire_search() " << "onewire_reset() failed" << KATO_RESET << std::endl;
#endif
                // reset the search
                _last_discrepancy = 0;
                _last_dev_flag = false;
                _last_family_discrepancy = 0;
                return false;
            }

            if (onewire_write_byte(0xF0) < 0) // issue the search command
            {

#ifdef DS2484_DEBUG
                kato::log::cout << KATO_RED << "DS2484::" << "onewire_search() " << "onewire_write_byte() failed" << KATO_RESET << std::endl;
#endif
                return false;
            }

            do // loop to do the search
            {
                if ((onewire_read_bit(id_bit) < 0) || (onewire_read_bit(cmp_id_bit) < 0)) // read a bit and its complement
                {

#ifdef DS2484_DEBUG
                    kato::log::cout << KATO_RED << "DS2484::" << "onewire_search() " << "onewire_read_bit() failed" << KATO_RESET << std::endl;
#endif
                    return false;
                }

#ifdef DS2484_DEBUG
// kato::log::cout << KATO_BLUE << "DS2484::" << "onewire_search() " << "id_bit = " << id_bit << ", cmp_id_bit = " << cmp_id_bit << KATO_RESET << std::endl;
#endif
                if (id_bit && cmp_id_bit) // check for no devices on 1-wire
                    break;                // No devices participating
                else
                {
                    if (id_bit != cmp_id_bit)      // all devices coupled have 0 or 1
                        search_direction = id_bit; // bit write value for search
                    else
                    {

#ifdef DS2484_DEBUG
// kato::log::cout << KATO_BLUE << "DS2484::" << "onewire_search() " << "id_bit_number = " + std::to_string(id_bit_number) + " _last_discrepancy = " + std::to_string(_last_discrepancy) << KATO_RESET << std::endl;
#endif
                        if (id_bit_number < _last_discrepancy) // if this discrepancy if before the Last Discrepancy on a previous next then pick the same as last time
                            search_direction = ((ROM[rom_byte_number] & rom_byte_mask) > 0);
                        else
                            search_direction = (id_bit_number == _last_discrepancy); // if equal to last pick 1, if not then pick 0

                        if (search_direction == 0) // if 0 was picked then record its position in LastZero
                        {
                            last_zero = id_bit_number;
                            if (last_zero < 9) // check for Last discrepancy in family
                                _last_family_discrepancy = last_zero;
                        }
                    }

                    if (search_direction == 1) // set or clear the bit in the ROM byte rom_byte_number with mask rom_byte_mask
                        ROM[rom_byte_number] |= rom_byte_mask;
                    else
                        ROM[rom_byte_number] &= ~rom_byte_mask;

                    if (onewire_write_bit(search_direction) < 0) // serial number search direction write bit
                    {

#ifdef DS2484_DEBUG
                        kato::log::cout << KATO_RED << "DS2484::" << "onewire_search() " << "onewire_write_bit() failed" << KATO_RESET << std::endl;
#endif
                        return false;
                    }
                    // increment the byte counter id_bit_number and shift the mask rom_byte_mask
                    id_bit_number++;
                    rom_byte_mask <<= 1;

                    if (rom_byte_mask == 0) // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                    {
                        rom_byte_number++;
                        rom_byte_mask = 1;
                    }
                }

            } while (rom_byte_number < 8); // loop until through all ROM bytes 0-7

            if (id_bit_number >= 65) // if the search was successful then
            {
                _last_discrepancy = last_zero; // search successful so set _last_discrepancy,_last_dev_flag,search_result
                if (_last_discrepancy == 0)    // check for last device
                    _last_dev_flag = true;
                search_result = true;
            }
        }

        if (!search_result || !ROM[0]) // if no device found then reset counters so next 'search' will be like a first
        {
            _last_discrepancy = 0;
            _last_dev_flag = false;
            _last_family_discrepancy = 0;
            search_result = false;
        }

        std::memcpy(&_rom, ROM, 8);

        return search_result;
    }
};
// ====================================================================================================================

int log_channel(const uint);

#endif // __DS2484_H__
