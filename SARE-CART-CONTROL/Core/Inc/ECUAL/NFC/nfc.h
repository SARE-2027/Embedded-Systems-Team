/******************* Section 0 : File Guard *******************/

#ifndef NFC_H
#define NFC_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"
#include "../../MCAL/mcal_spi.h"
#include "../../MCAL/mcal_gpio.h"
#include "../../MCAL/mcal_delay.h"

/******************* Section 2 : Macros Declarations *******************/

/* Safety: writing block 0 or a sector trailer (keys + access bits) can brick a
 * card. Such writes are rejected with NFC_ERR_PARAM unless this is set to 1U. */
#ifndef NFC_ALLOW_TRAILER_WRITE
#define NFC_ALLOW_TRAILER_WRITE      0U
#endif

/* --- PN532 SPI data-direction prefix bytes (LSB-first bus) --- */
#define NFC_SPI_DATA_WRITE           0x01U
#define NFC_SPI_STATUS_READ          0x02U
#define NFC_SPI_DATA_READ            0x03U
#define NFC_SPI_STATUS_READY         0x01U   /* status bit set when a reply is ready */

/* --- PN532 frame bytes --- */
#define NFC_FRAME_PREAMBLE           0x00U
#define NFC_FRAME_START1             0x00U
#define NFC_FRAME_START2             0xFFU
#define NFC_FRAME_POSTAMBLE          0x00U
#define NFC_TFI_HOST_TO_PN532        0xD4U
#define NFC_TFI_PN532_TO_HOST        0xD5U

/* --- PN532 commands --- */
#define NFC_CMD_GETFIRMWAREVERSION   0x02U
#define NFC_CMD_SAMCONFIGURATION     0x14U
#define NFC_CMD_RFCONFIGURATION      0x32U
#define NFC_CMD_INDATAEXCHANGE       0x40U
#define NFC_CMD_INLISTPASSIVETARGET  0x4AU

/* --- RFConfiguration item numbers --- */
#define NFC_RFCFG_MAX_RETRIES        0x05U

/* --- Card baud-rate / modulation for InListPassiveTarget (ISO14443A) --- */
#define NFC_BAUD_106K_TYPE_A         0x00U

/* --- MIFARE Classic commands (sent inside InDataExchange) --- */
#define NFC_MIFARE_AUTH_A            0x60U
#define NFC_MIFARE_AUTH_B            0x61U
#define NFC_MIFARE_READ              0x30U
#define NFC_MIFARE_WRITE             0xA0U

/* --- Sizes --- */
#define NFC_MIFARE_BLOCK_SIZE        16U
#define NFC_MIFARE_KEY_SIZE          6U
#define NFC_UID_MAX_SIZE             10U     /* ISO14443A UIDs are 4, 7 or 10 bytes */

/* --- Firmware-version "IC" byte expected from a genuine PN532 --- */
#define NFC_IC_PN532                 0x32U

/************** Section 3 : Macros Functions Declarations **************/

/* Extract fields from the packed word returned by ECUAL_NFC_GetFirmwareVersion() */
#define NFC_FW_IC(fw)        ((uint8_t)(((fw) >> 24) & 0xFFU))
#define NFC_FW_VER(fw)       ((uint8_t)(((fw) >> 16) & 0xFFU))
#define NFC_FW_REV(fw)       ((uint8_t)(((fw) >>  8) & 0xFFU))
#define NFC_FW_SUPPORT(fw)   ((uint8_t)( (fw)        & 0xFFU))

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

/* Error states (stored inside the handle) */
typedef enum {
    NFC_ERR_NONE = 0,    /* last call succeeded                                   */
    NFC_ERR_BUS,         /* SPI / GPIO (MCAL) error                               */
    NFC_ERR_TIMEOUT,     /* chip never became ready                               */
    NFC_ERR_NO_ACK,      /* chip did not ACK our command frame                    */
    NFC_ERR_FRAME,       /* bad preamble / checksum / wrong reply                 */
    NFC_ERR_PARAM,       /* bad argument (NULL, out of range, buffer too small)   */
    NFC_ERR_NO_CARD,     /* no tag in the field                                   */
    NFC_ERR_AUTH,        /* MIFARE authentication failed                          */
    NFC_ERR_CARD         /* card returned an error status                         */
} NFC_ErrorState_t;

/* MIFARE Classic key selector (values match the on-air command bytes) */
typedef enum {
    NFC_KEY_A = NFC_MIFARE_AUTH_A,
    NFC_KEY_B = NFC_MIFARE_AUTH_B
} NFC_KeyType_t;

typedef struct {
    Mcal_SpiId_t       spi;       /* MCAL SPI instance                       */
    Mcal_GpioId_t      cs;        /* MCAL GPIO id of the chip select         */
    NFC_ErrorState_t   error;     /* error state of the LAST function call   */
} NFC_Handle_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/**
 * All functions return E_OK on success. On failure they return E_NOT_OK and
 * store the reason in h->error (when h is non-NULL). Always compare the
 * result explicitly:  if (ECUAL_NFC_xxx(...) == E_OK)
 */

/**
 * Bind the handle to an SPI instance + CS pin, release CS (HIGH), wake the
 * PN532, verify it is a PN532, run SAMConfig and set the activation retries.
 * Requires the SPI peripheral and the CS pin to be initialised beforehand
 * (MX_GPIO_Init / MX_SPI1_Init).
 */
Std_ReturnType ECUAL_NFC_Init(NFC_Handle_t *h, Mcal_SpiId_t spi, Mcal_GpioId_t cs);

/**
 * Read PN532 firmware version.
 * *fw is packed as: IC<<24 | Ver<<16 | Rev<<8 | Support
 * Use NFC_FW_IC()/NFC_FW_VER()/... to unpack. IC must equal NFC_IC_PN532 (0x32).
 */
Std_ReturnType ECUAL_NFC_GetFirmwareVersion(NFC_Handle_t *h, uint32_t *fw);

/** Configure the SAM in Normal mode (required once before reading cards). */
Std_ReturnType ECUAL_NFC_SAMConfig(NFC_Handle_t *h);

/**
 * Set how many times the PN532 retries card detection per poll.
 * 0x00 = single try, 0xFF = retry forever (blocks) - avoid 0xFF in polling loops.
 */
Std_ReturnType ECUAL_NFC_SetPassiveActivationRetries(NFC_Handle_t *h, uint8_t retries);

/**
 * Detect one ISO14443A card and return its UID (4, 7 or 10 bytes).
 *  - uid_max must be >= the card's UID length, otherwise NFC_ERR_PARAM.
 *  - timeout_ms is the maximum time to wait for the PN532 reply. It must be
 *    longer than the PN532's own retry time (about 100..500 ms is typical);
 *    0 means "do not wait" and will almost always give NFC_ERR_TIMEOUT.
 *  - If nothing is in the field the call returns E_NOT_OK with
 *    h->error == NFC_ERR_NO_CARD.
 */
Std_ReturnType ECUAL_NFC_ReadPassiveTargetID(NFC_Handle_t *h, uint8_t *uid,
                                             uint8_t uid_max, uint8_t *uid_len,
                                             uint32_t timeout_ms);

/**
 * Authenticate one MIFARE Classic block (card target number is fixed at 1).
 *  - uid / uid_len: as returned by ReadPassiveTargetID (4..10 bytes). The PN532
 *    uses the LAST 4 bytes of the UID.
 *  - block: 0..63 (Classic 1K) or 0..255 (Classic 4K).
 *  - key_type: NFC_KEY_A or NFC_KEY_B, anything else gives NFC_ERR_PARAM.
 * After a failed authentication the card must be re-selected with
 * ECUAL_NFC_ReadPassiveTargetID() before trying again.
 */
Std_ReturnType ECUAL_NFC_MifareAuth(NFC_Handle_t *h, const uint8_t *uid, uint8_t uid_len,
                                    uint8_t block, NFC_KeyType_t key_type,
                                    const uint8_t key[NFC_MIFARE_KEY_SIZE]);

/** Read 16 bytes from an already-authenticated block. */
Std_ReturnType ECUAL_NFC_MifareReadBlock(NFC_Handle_t *h, uint8_t block,
                                         uint8_t data[NFC_MIFARE_BLOCK_SIZE]);

/**
 * Write 16 bytes to an already-authenticated block.
 * Block 0 and sector trailers are rejected (NFC_ERR_PARAM) unless
 * NFC_ALLOW_TRAILER_WRITE is defined as 1U.
 */
Std_ReturnType ECUAL_NFC_MifareWriteBlock(NFC_Handle_t *h, uint8_t block,
                                          const uint8_t data[NFC_MIFARE_BLOCK_SIZE]);

#endif /* NFC_H */
