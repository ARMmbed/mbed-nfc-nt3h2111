
#ifndef NT3H_MESSAGE_H
#define NT3H_MESSAGE_H


#include "ntag_defines.h"

#define TX_START 0
#define RX_START 0
#define NTAG_ADDRESS_SIZE 1

#define NT3H_I2C_WRITE_ADDR   0xAA
#define NT3H_I2C_READ_ADDR   0xAB

#define NT3H_NDEF_HEADER 0x03
#define NT3H_NDEF_TAIL 0xFE


//Fred_Todo:  1k or 2k?
#define NTAG_1k 1


/***********************************************************************/
/* TYPES                                                               */
/***********************************************************************/

typedef enum {
    NTAG_OK,
    NTAG_ERROR_TX_FAILED,
    NTAG_ERROR_RX_FAILED,
    NTAG_ERROR_WRITE_TIMEOUT,
    NTAG_ERROR_INVALID_PARAM,
    NTAG_CLOSED,
    NTAG_STATUS_MAX_NUMBER
} NTAG_STATUS_T;


typedef enum {
    RF_SWITCHED_OFF_00b = (0x0 << 4),
    HALT_OR_RF_SWITCHED_OFF_01b = (0x1 << 4),
    LAST_NDEF_BLOCK_READ_OR_RF_SWITCHED_OFF_10b = (0x2 << 4),
    I2C_LAST_DATA_READ_OR_WRITTEN_OR_RF_SWITCHED_OFF_11b = (0x3 << 4)
} NTAG_FD_OFF_FUNCTIONS_T;

typedef enum {
    RF_SWITCHED_ON_00b = (0x0 << 2),
    FIRST_VALID_SoF_01b = (0x1 << 2),
    SELECTION_OF_TAG_10b = (0x2 << 2),
    DATA_READY_BY_I2C_OR_DATA_READ_BY_RF_11b = (0x3 << 2)
} NTAG_FD_ON_FUNCTIONS_T;

typedef enum {
    RF_TO_I2C = NTAG_NC_REG_MASK_TRANSFER_DIR,
    I2C_TO_RF = 0
} NTAG_TRANSFER_DIR_T;

typedef enum {
#ifdef HAVE_NTAG_INTERRUPT
    NTAG_EVENT_FD_PIN_HI_INTERRUPT,
    NTAG_EVENT_FD_PIN_LO_INTERRUPT,
    NTAG_EVENT_RF_FIELD_PRESENT_INTERRUPT,
    NTAG_EVENT_RF_FIELD_NOT_PRESENT_INTERRUPT,
    NTAG_EVENT_HALT_STATE,
    NTAG_EVENT_NDEF_DATA_READ_INTERRUPT,
    NTAG_EVENT_SoF,
    NTAG_EVENT_SELECT,
    NTAG_EVENT_RF_WROTE_SRAM_INTERRUPT,
    NTAG_EVENT_RF_READ_SRAM_INTERRUPT,
#endif
    NTAG_EVENT_NDEF_DATA_READ_POLLED,
    NTAG_EVENT_RF_FIELD_PRESENT_POLLED,
    NTAG_EVENT_RF_FIELD_NOT_PRESENT_POLLED,
    NTAG_EVENT_RF_WROTE_SRAM_POLLED,
    NTAG_EVENT_RF_READ_SRAM_POLLED
} NTAG_EVENT_T;


struct NTAG_DEVICE {
    NTAG_STATUS_T status;
    I2C* i2cbus;
    uint8_t waddr;
    uint8_t raddr;
#ifdef HAVE_NTAG_INTERRUPT
    ISR_SOURCE_T isr;
#endif
    uint8_t tx_buffer[TX_START + NTAG_I2C_BLOCK_SIZE + NTAG_ADDRESS_SIZE];
    uint8_t rx_buffer[RX_START + NTAG_I2C_BLOCK_SIZE];
};


typedef struct NTAG_DEVICE* NTAG_HANDLE_T;



//#define HAVE_NTAG_INTERRUPT

typedef enum {
#ifdef HAVE_NTAG_INTERRUPT
    NTAG_FD_PIN_STATE_HI,
    NTAG_FD_PIN_STATE_LO,
#endif

} NTAG_FD_STATE_T;


static const uint8_t Default_BeginingOfMemory[] = {
    0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xE1, 0x10, 0x6D, 0x00
};

static const uint8_t Null_Block[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                    };

static const uint8_t Default_Page_56[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF
                                         };

static const uint8_t Default_Page_57[] = {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                         };

static const uint8_t Default_Page_58[] = {0x01, 0x00, 0xF8, 0x48, 0x08, 0x01, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                         };

static const uint16_t Default_BeginingOfMemory_length = sizeof(Default_BeginingOfMemory);
static const uint16_t Null_Block_length = sizeof(Null_Block);
static const uint16_t Default_Page_56_length = sizeof(Default_Page_56);
static const uint16_t Default_Page_57_length = sizeof(Default_Page_57);
static const uint16_t Default_Page_58_length = sizeof(Default_Page_58);

#endif /* NT3H_MESSAGE_H */
