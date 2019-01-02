/**
 ******************************************************************************
 * @file    nt3h_driver.h
 * @author  Fred.Li@Arm.com
 * @brief   This file provides a set of functions to interface with the NT3H
 *          device.
 */

#ifndef NT3H_H
#define NT3H_H

#include <stdint.h>
#include <mbed.h>
#include "I2C.h"
#include "NFCEEPROMDriver.h"
#include "EventQueue.h"

#include "nt3h_defines.h"


#define NT3H_I2C_SDA_PIN     NC
#define NT3H_I2C_SCL_PIN     NC
#define NT3H_FD_PIN         NC


namespace mbed
{
namespace nfc
{
namespace vendor
{
namespace NXP
{


#define MAX_USER_MEM_SIZE         0x374  /*I2C Block 01  ~ I2C Block 55 + 4 bytes in I2C Block 56 **/


class NT3HDriver : public NFCEEPROMDriver
{
public:

    /** Create the driver, default pin names will be used appropriate for the board.
     *  @param i2c_data_pin I2C data pin name.
     *  @param i2c_clock_pin I2C clock pin name.
     *  @param FD_pin field detection pin name.
     */
    NT3HDriver(PinName i2c_data_pin = NT3H_I2C_SDA_PIN, PinName i2c_clock_pin = NT3H_I2C_SCL_PIN,
               PinName fd_pin = NT3H_FD_PIN);

    virtual ~NT3HDriver() { }


    /* ------------------------------------------------------------------------
    * Implementation of NFCEEPROMDriver
    */

    /** @see NFCEEPROMDriver::reset
     */
    virtual void reset();

    /** @see NFCEEPROMDriver::get_max_size
     */
    virtual size_t read_max_size()
    {
        return MAX_USER_MEM_SIZE;
    }

    virtual void start_session(bool force);

    virtual void end_session();

    virtual void read_bytes(uint32_t address, uint8_t *bytes, size_t count);

    virtual void write_bytes(uint32_t address, const uint8_t *bytes, size_t count);

    virtual void read_size();

    virtual void write_size(size_t count);

    virtual void erase_bytes(uint32_t address, size_t count);

private:
    void factory_reset_Tag(void);

    int io_poll_i2c(void);



private:
    /** Default password used to change the write/read permission */
    static const uint8_t default_password[16];

    I2C _i2c_channel;

    /** Interrupt object fired when the FD status changes */
    InterruptIn _fdpin_state;
    NTAG_DEVICE _ntag_device;
    NTAG_DEVICE* _ntag_handle;

    int _current_eeprom_size;
    int _user_mem_offset;
    uint8_t _ndef_header[3];
    uint8_t _ndef_tail[1];
    bool _session_opened;


};

} //ST
} //vendor
} //nfc
} //mbed

#endif // NT3H_H

