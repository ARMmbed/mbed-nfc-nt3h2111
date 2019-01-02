/* mbed Microcontroller Library
 * Copyright (c) 2018-2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <nt3h_driver.h>
#include "mbed.h"
#include "nt3h_defines.h"


namespace mbed
{
namespace nfc
{
namespace vendor
{
namespace NXP
{

/***********************************************************************/
/* GLOBAL PRIVATE FUNCTIONS                                            */
/***********************************************************************/

static bool NTAG_ReadRegister(NTAG_HANDLE_T ntag, uint8_t reg, uint8_t *val);
static bool NTAG_WriteBytes(NTAG_HANDLE_T ntag, uint16_t address, const uint8_t *bytes, uint16_t len);
static bool NTAG_ReadBytes(NTAG_HANDLE_T ntag, uint16_t address, uint8_t *bytes, uint16_t len);


bool NTAG_ReadBlock(NTAG_HANDLE_T ntag, uint8_t block, uint8_t *bytes, uint8_t len)
{
    size_t i = 0;

    ntag->tx_buffer[TX_START] = block;

    /* send block number */
    ntag->i2cbus->write(ntag->waddr, (const char*)ntag->tx_buffer, 1);

    /*if (HAL_I2C_OK != HAL_I2C_SendBytes(ntag->i2cbus, ntag->address, ntag->tx_buffer, 1))
    {
        ntag->status = NTAG_ERROR_TX_FAILED;
        return TRUE;
    }*/

    /* receive bytes */
    ntag->i2cbus->read(ntag->raddr, (char*)ntag->rx_buffer, NTAG_I2C_BLOCK_SIZE);

    /*if (HAL_I2C_OK != HAL_I2C_RecvBytes(ntag->i2cbus, ntag->address, ntag->rx_buffer, NTAG_I2C_BLOCK_SIZE))
    {
        ntag->status = NTAG_ERROR_RX_FAILED;
        return TRUE;
    }
    */

    len = MIN(len, NTAG_I2C_BLOCK_SIZE);

    /* write to bytes buffer */
    for (i = 0; i < len; i++)
        bytes[i] = ntag->rx_buffer[RX_START + i];

    return false;
}

bool NTAG_WriteBlock(NTAG_HANDLE_T ntag, uint8_t block, const uint8_t *bytes, uint8_t len)
{
    uint8_t ns_reg = 0;
    uint32_t timeout = NTAG_MAX_WRITE_DELAY_MS / 5 + 1;
    size_t i = 0;

    ntag->tx_buffer[TX_START] = block;

    len = MIN(len, NTAG_I2C_BLOCK_SIZE);

    /* copy len bytes */
    for (i = 0; i < len; i++)
        ntag->tx_buffer[TX_START + i + 1] = bytes[i];

    /* zero rest of the buffer */
    for (i = len; i < NTAG_I2C_BLOCK_SIZE; i++)
        ntag->tx_buffer[TX_START + i + 1] = 0;

    /* send block number */
    ntag->i2cbus->write(ntag->waddr, (const char*)ntag->tx_buffer, NTAG_I2C_BLOCK_SIZE + 1);

    /*
    if (HAL_I2C_OK != HAL_I2C_SendBytes(ntag->i2cbus, ntag->address, ntag->tx_buffer, NTAG_I2C_BLOCK_SIZE + 1))
    {
        ntag->status = NTAG_ERROR_TX_FAILED;
        return TRUE;
    }*/

    /* do not wait for completion when writing SRAM */
    if (block >= NTAG_MEM_BLOCK_START_SRAM && block < NTAG_MEM_BLOCK_START_SRAM + NTAG_MEM_SRAM_BLOCKS)
        return ntag->status;

    /* wait for completion */
    do {
        wait_ms(5);
        if (NTAG_ReadRegister(ntag, NTAG_MEM_OFFSET_NS_REG, &ns_reg))
            break;
        timeout--;
    } while (timeout && ns_reg & NTAG_NS_REG_MASK_EEPROM_WR_BUSY);

    if (0 == timeout)
        ntag->status = NTAG_ERROR_WRITE_TIMEOUT;

    return ntag->status;
}


static bool NTAG_ReadBytes(NTAG_HANDLE_T ntag, uint16_t address, uint8_t *bytes, uint16_t len)
{
    uint16_t bytes_read = 0;

    if (ntag->status == NTAG_CLOSED)
        return true;

    ntag->status = NTAG_OK;

    while (bytes_read < len) {
        uint8_t current_block = (address + bytes_read) / NTAG_I2C_BLOCK_SIZE;
        uint8_t begin = (address + bytes_read) % NTAG_I2C_BLOCK_SIZE;
        uint8_t current_len = MIN(len - bytes_read, NTAG_I2C_BLOCK_SIZE - begin);

        if (current_len < NTAG_I2C_BLOCK_SIZE) {
            size_t i = 0;

            /* read block into ntag->rx_buffer only */
            if (NTAG_ReadBlock(ntag, current_block, NULL, 0))
                break;

            /* modify rx_buffer */
            for (i = 0; i < current_len; i++)
                bytes[bytes_read + i] = ntag->rx_buffer[RX_START + begin + i];
        } else {
            /* full block read */
            if (NTAG_ReadBlock(ntag, current_block, bytes + bytes_read, NTAG_I2C_BLOCK_SIZE))
                break;
        }

        bytes_read += current_len;
    }
    return ntag->status;
}

static bool NTAG_WriteBytes(NTAG_HANDLE_T ntag, uint16_t address, const uint8_t *bytes, uint16_t len)
{
    uint16_t bytes_written = 0;

    if (ntag->status == NTAG_CLOSED)
        return true;

    ntag->status = NTAG_OK;

    while (bytes_written < len) {
        uint8_t current_block = (address + bytes_written) / NTAG_I2C_BLOCK_SIZE;
        uint8_t begin = (address + bytes_written) % NTAG_I2C_BLOCK_SIZE;
        uint8_t current_len = MIN(len - bytes_written, NTAG_I2C_BLOCK_SIZE - begin);

        if (current_len < NTAG_I2C_BLOCK_SIZE) {
            size_t i = 0;

            /* read block into ntag->rx_buffer only */
            if (NTAG_ReadBlock(ntag, current_block, NULL, 0))
                break;

            /* check if it is the first Block(0x00) and not the I2C Addr */
            /* be careful with writing of first byte in management block */
            /* the byte contains part of the serial number on read but   */
            /* on write the I2C address of the device can be modified    */
            if (0x00 == current_block && NTAG_MEM_ADRR_I2C_ADDRESS < begin)
                ntag->rx_buffer[RX_START + 0] = ntag->waddr;

            /* modify rx_buffer */
            for (i = 0; i < current_len; i++)
                ntag->rx_buffer[RX_START + begin + i] = bytes[bytes_written + i];

            /* writeback modified buffer */
            if (NTAG_WriteBlock(ntag, current_block, ntag->rx_buffer + RX_START, NTAG_I2C_BLOCK_SIZE))
                break;
        } else {
            /* full block write */
            if (NTAG_WriteBlock(ntag, current_block, bytes + bytes_written, NTAG_I2C_BLOCK_SIZE))
                break;
        }

        bytes_written += current_len;
    }

    return ntag->status;
}


static bool NTAG_ReadRegister(NTAG_HANDLE_T ntag, uint8_t reg, uint8_t *val)
{
    ntag->tx_buffer[TX_START + 0] = NTAG_MEM_BLOCK_SESSION_REGS;
    ntag->tx_buffer[TX_START + 1] = reg;

    // send block number
    ntag->i2cbus->write(ntag->waddr, (const char*)ntag->tx_buffer, 2);

    /*if (HAL_I2C_OK != HAL_I2C_SendBytes(ntag->i2cbus, ntag->address, ntag->tx_buffer, 2))
    {
        ntag->status = NTAG_ERROR_TX_FAILED;
        return TRUE;
    }*/

    // receive bytes
    ntag->i2cbus->read(ntag->raddr, (char*)ntag->rx_buffer, 1);

    /*
    if (HAL_I2C_OK != HAL_I2C_RecvBytes(ntag->i2cbus, ntag->address, ntag->rx_buffer, 1))
    {
        ntag->status = NTAG_ERROR_RX_FAILED;
        return TRUE;
    }*/

    *val = ntag->rx_buffer[RX_START + 0];
    return false;
}



static bool NTAG_WriteRegister(NTAG_HANDLE_T ntag, uint8_t reg, uint8_t mask, uint8_t val)
{
    ntag->tx_buffer[TX_START + 0] = NTAG_MEM_BLOCK_SESSION_REGS;
    ntag->tx_buffer[TX_START + 1] = reg;
    ntag->tx_buffer[TX_START + 2] = mask;
    ntag->tx_buffer[TX_START + 3] = val;

    ntag->i2cbus->write(ntag->waddr,(const char*) ntag->tx_buffer, 4);

    /*
        if (HAL_I2C_OK != HAL_I2C_SendBytes(ntag->i2cbus, ntag->address, ntag->tx_buffer, 4))
        {
            ntag->status = NTAG_ERROR_TX_FAILED;
            return TRUE;
        }
        */

    return false;
}


static bool NTAG_ReadConfiguration(NTAG_HANDLE_T ntag, uint8_t reg, uint8_t *val)
{
#ifdef NTAG_2k
    uint8_t config = NTAG_MEM_BLOCK_CONFIGURATION_2k;
#elif NTAG_1k
    uint8_t config = NTAG_MEM_BLOCK_CONFIGURATION_1k;
#endif

    uint8_t I2C_Buf[NTAG_I2C_BLOCK_SIZE];
    if (NTAG_ReadBlock(ntag, config, I2C_Buf, NTAG_I2C_BLOCK_SIZE))
        return NTAG_ERR_COMMUNICATION;

    *val = I2C_Buf[reg];
    return NTAG_ERR_OK;
}

static bool NTAG_WriteConfiguration(NTAG_HANDLE_T ntag, uint8_t reg, uint8_t mask, uint8_t val)
{
#ifdef NTAG_2k
    uint8_t config = NTAG_MEM_BLOCK_CONFIGURATION_2k;
#elif NTAG_1k
    uint8_t config = NTAG_MEM_BLOCK_CONFIGURATION_1k;
#endif

    uint8_t I2C_Buf[NTAG_I2C_BLOCK_SIZE];
    if (NTAG_ReadBlock(ntag, config, I2C_Buf, NTAG_I2C_BLOCK_SIZE))
        return NTAG_ERR_COMMUNICATION;

    // Clear all other bits of the val
    val = val & mask;

    // Clear specific bit in the Buffer
    I2C_Buf[reg] = I2C_Buf[reg] & ~mask;

    // write bits in the Buffer
    I2C_Buf[reg] = I2C_Buf[reg] | val;

    if (NTAG_WriteBlock(ntag, config, I2C_Buf, NTAG_I2C_BLOCK_SIZE))
        return NTAG_ERR_COMMUNICATION;

    return NTAG_ERR_OK;
}

NTAG_STATUS_T NTAG_GetLastError(NTAG_HANDLE_T ntag)
{
    return ntag->status;
}







void NT3HDriver:: factory_reset_Tag(void)
{
    printf("\r\nFactory Reset of Tag memory");
    //SwitchLEDs(REDLED);
    //HAL_Timer_delay_ms(100);
    wait_ms(100);

    /* reset default eeprom memory values (smart poster) */
    NTAG_WriteBytes(_ntag_handle, NTAG_MEM_ADRR_I2C_ADDRESS, Default_BeginingOfMemory, Default_BeginingOfMemory_length);

    /* reset pages from 1 to 56 */
    uint8_t page = NTAG_MEM_BLOCK_START_USER_MEMORY;
    while (page < 56) {
        NTAG_WriteBlock(_ntag_handle, page, Null_Block, NTAG_I2C_BLOCK_SIZE);
        page++;
    }
    /* reset pages 56,57,58 */
    NTAG_WriteBlock(_ntag_handle, 56, Default_Page_56, NTAG_I2C_BLOCK_SIZE);
    NTAG_WriteBlock(_ntag_handle, 57, Default_Page_57, NTAG_I2C_BLOCK_SIZE);
    NTAG_WriteBlock(_ntag_handle, 58, Default_Page_58, NTAG_I2C_BLOCK_SIZE);

    //SwitchLEDs(GREENLED);
    //HAL_Timer_delay_ms(100);
    wait_ms(100);
}


int NT3HDriver::io_poll_i2c()
{
    int status = NTAG_ERR_COMMUNICATION;
    while (status != NTAG_ERR_OK) {
        /* send the device address and wait to receive an ack bit */
        status = _i2c_channel.write(NTAG_MEM_ADRR_I2C_ADDRESS, NULL, 0);
    }
    return status;
}


NT3HDriver::NT3HDriver(PinName i2c_data_pin, PinName i2c_clock_pin, PinName fd_pin)
    :_i2c_channel(i2c_data_pin, i2c_clock_pin), _fdpin_state(fd_pin), _current_eeprom_size(MAX_USER_MEM_SIZE)
{
    //Fred_Todo: double confirm the I2C freq.
    _i2c_channel.frequency(100000);
    //NFC_InitDevice();
    _ntag_device.i2cbus = &_i2c_channel;
    _ntag_device.status = NTAG_OK;
    _ntag_device.waddr = NT3H_I2C_WRITE_ADDR;
	_ntag_device.raddr= NT3H_I2C_READ_ADDR;
    _ntag_handle = &_ntag_device;

	//For NXP I2C Tag.  Page 4 start with Header 0x03 followed by NDEF Message size.

	_ndef_header[0]=NT3H_NDEF_HEADER;

	//For NXP I2C Tag.  There always be 0xfe append to the end of the ndef message as tail.
	_ndef_tail[0] = NT3H_NDEF_TAIL;

	_user_mem_offset =  NTAG_MEM_ADDR_START_USER_MEMORY;

}


void NT3HDriver::reset()
{
    factory_reset_Tag();
	_user_mem_offset =  NTAG_MEM_ADDR_START_USER_MEMORY;
}


void NT3HDriver::start_session(bool force)
{
    //polling the i2c, make sure I2C lock the communication now.
    if(io_poll_i2c()) {
        delegate()->on_session_started(false);
    } else {
        _session_opened = true;
        delegate()->on_session_started(true);
    }
}

void NT3HDriver::end_session()
{
    _session_opened = false;
    delegate()->on_session_ended(true);

}

void NT3HDriver::read_bytes(uint32_t address, uint8_t *bytes, size_t count)
{
    bool status;

    if (address >= _current_eeprom_size) {
        delegate()->on_bytes_read(0);
        return;
    }

    count = (address + count > MAX_USER_MEM_SIZE) ? (MAX_USER_MEM_SIZE - address) : count;


    status = NTAG_WriteBlock(_ntag_handle, address + _user_mem_offset, bytes, count);

    if(status) {
        printf("Error while Read bytes! Status:%d\n", NTAG_GetLastError(_ntag_handle));
        delegate()->on_bytes_written(0);
    }


    delegate()->on_bytes_read(count);
}

void NT3HDriver::write_bytes(uint32_t address, const uint8_t *bytes, size_t count)
{
    bool status;
    if (address >= _current_eeprom_size || address + count > _current_eeprom_size ) {
        delegate()->on_bytes_written(0);
        return;
    }

	/*printf("%s==>enter:",__FUNCTION__);
	for(int i=0;i<count;i++)
	{
		printf("0x%02x,",bytes[i]);
	}*/

    //count = (address + count > MAX_USER_MEM_SIZE) ? (MAX_USER_MEM_SIZE - address) : count;

    status = NTAG_WriteBytes(_ntag_handle, address + _user_mem_offset, bytes, count);

    if(status) {
        printf("Error while write bytes! Status:%d\n", NTAG_GetLastError(_ntag_handle));
        delegate()->on_bytes_written(0);
    }

	//always append the tail 0xfe.
	if(address + count == _current_eeprom_size)
	{
		NTAG_WriteBytes(_ntag_handle, _current_eeprom_size + _user_mem_offset, _ndef_tail, 1);
	}

    delegate()->on_bytes_written(count);
}

void NT3HDriver::read_size()
{
    delegate()->on_size_read(true, _current_eeprom_size);
}

void NT3HDriver::write_size(size_t count)
{
	//printf("%s==>count:%d",__FUNCTION__,count);
    if (count > MAX_USER_MEM_SIZE) {
        delegate()->on_size_written(false);
    } else {
        _current_eeprom_size = count;
		uint8_t header_size = 0;

		if(count<0xff)
		{
			_ndef_header[1] = count;
			header_size = 2;
		}
		else
		{
			uint16_t _ndef_size = (uint16_t)count;
			uint8_t* bytes = (uint8_t*)&_ndef_size;
			_ndef_header[1] = bytes[0];
			_ndef_header[2] = bytes[1];
			header_size = 3;
		}

		if(NTAG_WriteBytes(_ntag_handle, NTAG_MEM_ADDR_START_USER_MEMORY, _ndef_header, header_size)) {
	        printf("Error while write bytes! Status:%d\n", NTAG_GetLastError(_ntag_handle));
	        delegate()->on_size_written(false);
	    }
		_user_mem_offset = NTAG_MEM_ADDR_START_USER_MEMORY + header_size;

        delegate()->on_size_written(true);
    }
}

void NT3HDriver::erase_bytes(uint32_t address, size_t count)
{
    bool status;
    uint16_t bytes_written = 0;

    if (address >= _current_eeprom_size) {
        delegate()->on_bytes_erased(0);
        return;
    }

    address+= _user_mem_offset; //Ndef message real started from I2C block1; Block0 was config data.

    while (bytes_written < count) {
        uint8_t current_block = (address + bytes_written) / NTAG_I2C_BLOCK_SIZE;
        uint8_t begin = (address + bytes_written) % NTAG_I2C_BLOCK_SIZE;
        uint8_t current_len = MIN(count - bytes_written, NTAG_I2C_BLOCK_SIZE - begin);

        if (current_len < NTAG_I2C_BLOCK_SIZE) {
            size_t i = 0;

            /* read block into ntag->rx_buffer only */
            if (NTAG_ReadBlock(_ntag_handle, current_block, NULL, 0))
                break;

            /* check if it is the first Block(0x00) and not the I2C Addr */
            /* be careful with writing of first byte in management block */
            /* the byte contains part of the serial number on read but   */
            /* on write the I2C address of the device can be modified    */
            if (0x00 == current_block && NTAG_MEM_ADRR_I2C_ADDRESS < begin)
                _ntag_handle->rx_buffer[RX_START + 0] = _ntag_handle->waddr;

            /* modify rx_buffer */
            for (i = 0; i < current_len; i++)
                _ntag_handle->rx_buffer[RX_START + begin + i] = 0;

            /* writeback modified buffer */
            if (NTAG_WriteBlock(_ntag_handle, current_block, _ntag_handle->rx_buffer + RX_START, NTAG_I2C_BLOCK_SIZE))
                break;
        } else {
            /* full block write */
            if (NTAG_WriteBlock(_ntag_handle, current_block, Null_Block, NTAG_I2C_BLOCK_SIZE))
                break;
        }

        bytes_written += current_len;
    }


    if(NTAG_GetLastError(_ntag_handle)) {
        printf("Error while erase bytes! Status:%d\n", NTAG_GetLastError(_ntag_handle));
        delegate()->on_bytes_erased(bytes_written);
    }

    delegate()->on_bytes_erased(count);
}


}  // namespace nxp
}  // namespace vendor
}// namespace nfc
}// namespace mbed
