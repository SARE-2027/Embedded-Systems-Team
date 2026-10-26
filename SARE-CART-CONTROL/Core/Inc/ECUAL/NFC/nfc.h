/******************* Section 0 : File Guard *******************/

#ifndef __NFC_H
#define __NFC_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- Driver Version --- */
#define NFC_VERSION_MAJOR            (1U)
#define NFC_VERSION_MINOR            (0U)
#define NFC_VERSION_PATCH            (0U)

/* --- Generic Pin Selection ---
 *   Any GPIO can be used. Pick it with NFC_PIN(<port letter>, <pin number>):
 *       NFC_PIN(A, 7) -> PA7 (MOSI)
 *       NFC_PIN(A, 6) -> PA6 (MISO)
 *       NFC_PIN(A, 5) -> PA5 (SCK)
 *       NFC_PIN(A, 4) -> PA4 (CS)
 *   Pin id layout: bits[7:4] = port (A=0 .. E=4, H=7), bits[3:0] = pin number (0..15).
 */
#define NFC_PORT_A                   (0U)
#define NFC_PORT_B                   (1U)
#define NFC_PORT_C                   (2U)
#define NFC_PORT_D                   (3U)
#define NFC_PORT_E                   (4U)
#define NFC_PORT_H                   (7U)
#define NFC_PIN(port_letter, num)    ((uint8)((NFC_PORT_##port_letter << 4U) | ((uint8)(num) & 0x0FU)))

/* --- PN532 SPI Data-Direction Prefix Bytes (LSB-First bus) --- */
#define NFC_SPI_DATA_WRITE           (0x01U)
#define NFC_SPI_STATUS_READ          (0x02U)
#define NFC_SPI_DATA_READ            (0x03U)
#define NFC_SPI_STATUS_READY         (0x01U)   /* Bit 0 is set when reply is ready */

/* --- PN532 Frame Bytes --- */
#define NFC_FRAME_PREAMBLE           (0x00U)
#define NFC_FRAME_START1             (0x00U)
#define NFC_FRAME_START2             (0xFFU)
#define NFC_FRAME_POSTAMBLE          (0x00U)
#define NFC_TFI_HOST_TO_PN532        (0xD4U)
#define NFC_TFI_PN532_TO_HOST        (0xD5U)

/* --- PN532 Commands --- */
#define NFC_CMD_GETFIRMWAREVERSION   (0x02U)
#define NFC_CMD_SAMCONFIGURATION     (0x14U)
#define NFC_CMD_RFCONFIGURATION      (0x32U)
#define NFC_CMD_INDATAEXCHANGE       (0x40U)
#define NFC_CMD_INLISTPASSIVETARGET  (0x4AU)

/* --- RFConfiguration Item Numbers --- */
#define NFC_RFCFG_MAX_RETRIES        (0x05U)

/* --- Card Baud-Rate / Modulation for InListPassiveTarget (ISO14443A) --- */
#define NFC_BAUD_106K_TYPE_A         (0x00U)

/* --- MIFARE Classic Commands (sent inside InDataExchange) --- */
#define NFC_MIFARE_AUTH_A            (0x60U)
#define NFC_MIFARE_AUTH_B            (0x61U)
#define NFC_MIFARE_READ              (0x30U)
#define NFC_MIFARE_WRITE             (0xA0U)

/* --- Sizes & Constants --- */
#define NFC_MIFARE_BLOCK_SIZE        (16U)
#define NFC_MIFARE_KEY_SIZE          (6U)
#define NFC_UID_MAX_LEN              (10U)     /* ISO14443A UIDs are 4, 7 or 10 bytes */
#define NFC_IC_PN532                 (0x32U)   /* Genuine PN532 IC identification */

/* Safety: writing block 0 or a sector trailer can brick a card */
#ifndef NFC_ALLOW_TRAILER_WRITE
#define NFC_ALLOW_TRAILER_WRITE      (0U)
#endif

/******************* Section 3 : Macros Functions Declarations **************/

/* Extract fields from packed 32-bit firmware word */
#define NFC_FW_IC(fw)                ((uint8)(((fw) >> 24) & 0xFFU))
#define NFC_FW_VER(fw)               ((uint8)(((fw) >> 16) & 0xFFU))
#define NFC_FW_REV(fw)               ((uint8)(((fw) >>  8) & 0xFFU))
#define NFC_FW_SUPPORT(fw)           ((uint8)( (fw)        & 0xFFU))

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

/* --- SPI Channel Selector --- */
typedef enum {
    NFC_SPI_1 = 0,
    NFC_SPI_2 = 1,
    NFC_SPI_3 = 2
} nfc_spi_id_t;

/* --- SPI Baud Rate Prescaler (Bits 5:3 of SPI_CR1) --- */
typedef enum {
    NFC_SPI_BAUD_DIV_2   = 0x00000000U,
    NFC_SPI_BAUD_DIV_4   = (0x01UL << 3U),
    NFC_SPI_BAUD_DIV_8   = (0x02UL << 3U),
    NFC_SPI_BAUD_DIV_16  = (0x03UL << 3U),
    NFC_SPI_BAUD_DIV_32  = (0x04UL << 3U),
    NFC_SPI_BAUD_DIV_64  = (0x05UL << 3U),
    NFC_SPI_BAUD_DIV_128 = (0x06UL << 3U),
    NFC_SPI_BAUD_DIV_256 = (0x07UL << 3U)
} nfc_spi_baud_t;

/* --- Error States --- */
typedef enum {
    NFC_ERR_NONE = 0,    /* Last operation succeeded */
    NFC_ERR_BUS,         /* SPI / GPIO hardware error */
    NFC_ERR_TIMEOUT,     /* PN532 did not assert Ready */
    NFC_ERR_NO_ACK,      /* Missing or bad ACK frame */
    NFC_ERR_FRAME,       /* Framing / checksum error */
    NFC_ERR_PARAM,       /* Invalid function argument */
    NFC_ERR_NO_CARD,     /* No card in the field */
    NFC_ERR_AUTH,        /* MIFARE authentication failed */
    NFC_ERR_CARD         /* Card returned error */
} nfc_error_t;

/* --- MIFARE Classic Key Selector --- */
typedef enum {
    NFC_KEY_A = NFC_MIFARE_AUTH_A,
    NFC_KEY_B = NFC_MIFARE_AUTH_B
} nfc_key_type_t;

/* --- Card Information Structure --- */
typedef struct {
    bool     card_present;
    uint8    uid[NFC_UID_MAX_LEN];
    uint8    uid_len;
    uint8    sak;          /* Select Acknowledge (SEL_RES): Card type indicator */
    uint16   atqa;         /* Answer to Request A (SENS_RES) */
    uint32   last_seen_ms;
} nfc_card_t;

/*
 * NFC Configuration Structure (Customizable hardware mapping).
 *
 *   spi_id         : NFC_SPI_1, NFC_SPI_2, or NFC_SPI_3
 *   mosi_pin       : MOSI pin id, built with NFC_PIN(port, pin)
 *   miso_pin       : MISO pin id, built with NFC_PIN(port, pin)
 *   sck_pin        : SCK pin id,  built with NFC_PIN(port, pin)
 *   cs_pin         : CS pin id,   built with NFC_PIN(port, pin)
 *   baud_prescaler : SPI clock divider, e.g. NFC_SPI_BAUD_DIV_16
 */
typedef struct {
    nfc_spi_id_t    spi_id;
    uint8           mosi_pin;
    uint8           miso_pin;
    uint8           sck_pin;
    uint8           cs_pin;
    nfc_spi_baud_t  baud_prescaler;
} nfc_cfg_t;

/* --- NFC Instance Handle --- */
typedef struct {
    SPI_TypeDef     *spi;
    GPIO_TypeDef    *cs_port;
    uint16           cs_pin_mask;
    nfc_error_t      last_error;
    uint32           firmware_ver;
    nfc_card_t       latest_card;
    uint32           last_poll_tick;
    bool             initialized;
} nfc_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/* Initialise SPI, GPIOs and PN532. Returns E_OK on success. */
Std_ReturnType nfc_init(nfc_t *h, const nfc_cfg_t *cfg);

/* Cooperative, non-blocking service task: polls for cards every ~250 ms. */
void           nfc_task(nfc_t *h);

/* Return latest detected card info. Returns true if card is present. */
bool           nfc_get_card(const nfc_t *h, nfc_card_t *out_card);
bool           nfc_is_card_present(const nfc_t *h);

/* Low-level and diagnostic functions */
Std_ReturnType nfc_get_firmware_version(nfc_t *h, uint32 *fw);
Std_ReturnType nfc_sam_config(nfc_t *h);
Std_ReturnType nfc_set_retries(nfc_t *h, uint8 retries);
Std_ReturnType nfc_read_passive_target_id(nfc_t *h, uint8 *uid, uint8 uid_max,
                                          uint8 *uid_len, uint32 timeout_ms);

/* MIFARE Classic APIs */
Std_ReturnType nfc_mifare_auth(nfc_t *h, const uint8 *uid, uint8 uid_len,
                               uint8 block, nfc_key_type_t key_type,
                               const uint8 key[NFC_MIFARE_KEY_SIZE]);
Std_ReturnType nfc_mifare_read_block(nfc_t *h, uint8 block,
                                     uint8 data[NFC_MIFARE_BLOCK_SIZE]);
Std_ReturnType nfc_mifare_write_block(nfc_t *h, uint8 block,
                                      const uint8 data[NFC_MIFARE_BLOCK_SIZE]);

#endif /* __NFC_H */
