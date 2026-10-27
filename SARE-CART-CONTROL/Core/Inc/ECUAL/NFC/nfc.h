/******************* Section 0 : File Guard *******************/

#ifndef __NFC_H
#define __NFC_H

/******************* Section 1 : Includes *******************/

#include "stm32f4xx_hal.h"
#include "../../MCAL/mcal_std_types.h"
#include <stddef.h>

/******************* Section 2 : Macros Declarations *******************/

/* Pin id = (port << 4) | pin number. Usage: NFC_PIN(A, 7) */
#define NFC_PIN(port, num)          ((uint8)(((uint8)NFC_PORT_##port << 4U) | ((uint8)(num) & 0x0FU)))

/* Set to 1U to allow writing sector trailers / block 0 (dangerous) */
#ifndef NFC_ALLOW_TRAILER_WRITE
#define NFC_ALLOW_TRAILER_WRITE     (0U)
#endif

/* PN532 SPI prefix bytes */
#define NFC_SPI_DATA_WRITE          (0x01U)
#define NFC_SPI_STATUS_READ         (0x02U)
#define NFC_SPI_DATA_READ           (0x03U)
#define NFC_SPI_STATUS_READY        (0x01U)

/* Frame constants */
#define NFC_FRAME_PREAMBLE          (0x00U)
#define NFC_FRAME_START1            (0x00U)
#define NFC_FRAME_START2            (0xFFU)
#define NFC_FRAME_POSTAMBLE         (0x00U)
#define NFC_TFI_HOST_TO_PN532       (0xD4U)
#define NFC_TFI_PN532_TO_HOST       (0xD5U)

/* PN532 commands */
#define NFC_CMD_GETFIRMWAREVERSION  (0x02U)
#define NFC_CMD_SAMCONFIGURATION    (0x14U)
#define NFC_CMD_RFCONFIGURATION     (0x32U)
#define NFC_CMD_INDATAEXCHANGE      (0x40U)
#define NFC_CMD_INLISTPASSIVETARGET (0x4AU)

#define NFC_RFCFG_MAX_RETRIES       (0x05U)
#define NFC_BAUD_106K_TYPE_A        (0x00U)

/* Firmware version word: [IC][Ver][Rev][Support] */
#define NFC_IC_PN532                (0x32U)
#define NFC_FW_IC(fw)               ((uint8)(((fw) >> 24U) & 0xFFU))

/* MIFARE Classic */
#define NFC_MIFARE_READ             (0x30U)
#define NFC_MIFARE_WRITE            (0xA0U)
#define NFC_MIFARE_KEY_SIZE         (6U)
#define NFC_MIFARE_BLOCK_SIZE       (16U)

#define NFC_UID_MAX_LEN             (10U)

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

typedef enum {
    NFC_PORT_A = 0,
    NFC_PORT_B,
    NFC_PORT_C,
    NFC_PORT_D,
    NFC_PORT_E,
    NFC_PORT_H = 7
} nfc_port_t;

typedef enum {
    NFC_SPI_1 = 0,
    NFC_SPI_2,
    NFC_SPI_3
} nfc_spi_id_t;

/* Values are already positioned in SPI_CR1 BR[5:3] */
typedef enum {
    NFC_SPI_BAUD_DIV_2   = 0x00U,
    NFC_SPI_BAUD_DIV_4   = 0x08U,
    NFC_SPI_BAUD_DIV_8   = 0x10U,
    NFC_SPI_BAUD_DIV_16  = 0x18U,
    NFC_SPI_BAUD_DIV_32  = 0x20U,
    NFC_SPI_BAUD_DIV_64  = 0x28U,
    NFC_SPI_BAUD_DIV_128 = 0x30U,
    NFC_SPI_BAUD_DIV_256 = 0x38U
} nfc_spi_baud_t;

typedef enum {
    NFC_ERR_NONE = 0,
    NFC_ERR_BUS,
    NFC_ERR_TIMEOUT,
    NFC_ERR_NO_ACK,
    NFC_ERR_FRAME,
    NFC_ERR_PARAM,
    NFC_ERR_NO_CARD,
    NFC_ERR_AUTH,
    NFC_ERR_CARD
} nfc_error_t;

typedef enum {
    NFC_KEY_A = 0x60U,
    NFC_KEY_B = 0x61U
} nfc_key_type_t;

typedef struct {
    nfc_spi_id_t   spi_id;
    uint8          mosi_pin;
    uint8          miso_pin;
    uint8          sck_pin;
    uint8          cs_pin;
    nfc_spi_baud_t baud_prescaler;
} nfc_cfg_t;

typedef struct {
    bool    card_present;
    uint8   uid_len;
    uint8   uid[NFC_UID_MAX_LEN];
    uint16  atqa;
    uint8   sak;
    uint32  last_seen_ms;
} nfc_card_t;

typedef struct {
    SPI_TypeDef  *spi;
    GPIO_TypeDef *cs_port;
    uint16        cs_pin_mask;
    bool          initialized;
    nfc_error_t   last_error;
    uint32        firmware_ver;
    uint32        last_poll_tick;
    nfc_card_t    latest_card;
} nfc_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

Std_ReturnType nfc_init(nfc_t *h, const nfc_cfg_t *cfg);

/* Non-blocking-ish poll (every 250 ms). Call from main loop. */
void nfc_task(nfc_t *h);

bool nfc_get_card(const nfc_t *h, nfc_card_t *out_card);
bool nfc_is_card_present(const nfc_t *h);

Std_ReturnType nfc_get_firmware_version(nfc_t *h, uint32 *fw);
Std_ReturnType nfc_sam_config(nfc_t *h);
Std_ReturnType nfc_set_retries(nfc_t *h, uint8 retries);

Std_ReturnType nfc_read_passive_target_id(nfc_t *h, uint8 *uid, uint8 uid_max,
                                          uint8 *uid_len, uint32 timeout_ms);

Std_ReturnType nfc_mifare_auth(nfc_t *h, const uint8 *uid, uint8 uid_len,
                               uint8 block, nfc_key_type_t key_type,
                               const uint8 key[NFC_MIFARE_KEY_SIZE]);
Std_ReturnType nfc_mifare_read_block(nfc_t *h, uint8 block,
                                     uint8 data[NFC_MIFARE_BLOCK_SIZE]);
Std_ReturnType nfc_mifare_write_block(nfc_t *h, uint8 block,
                                      const uint8 data[NFC_MIFARE_BLOCK_SIZE]);

#endif /* __NFC_H */
