/******************* Section 0 : Includes *******************/

#include "../../../Inc/ECUAL/NFC/nfc.h"
#include <string.h>

/******************* Section 1 : Private Macros & Variables *******************/

#define TX_BUF_SIZE         48U
#define RX_BUF_SIZE         48U

#define SPI_TIMEOUT_MS      20U     /* per SPI transfer                    */
#define CS_WAKE_MS          2U      /* PN532 wake-up time after CS falls   */
#define ACK_TIMEOUT_MS      100U    /* wait for the ACK frame              */
#define CMD_TIMEOUT_MS      100U    /* wait for a normal command reply     */
#define AUTH_TIMEOUT_MS     200U
#define READ_TIMEOUT_MS     200U
#define WRITE_TIMEOUT_MS    300U
#define INIT_BOOT_MS        100U    /* PN532 power-on reset time           */
#define INIT_RETRY_MS       20U
#define INIT_ATTEMPTS       3U
#define INIT_PASSIVE_RETRY  0x05U

static const uint8_t ack_frame[6] = { 0x00U, 0x00U, 0xFFU, 0x00U, 0xFFU, 0x00U };

/************** Section 2 : Helper Functions Declarations **************/

static Std_ReturnType finish(NFC_Handle_t *h, NFC_ErrorState_t err);
static void cs_low(NFC_Handle_t *h);
static void cs_high(NFC_Handle_t *h);
static NFC_ErrorState_t spi_write(NFC_Handle_t *h, const uint8_t *frame, uint8_t len);
static NFC_ErrorState_t spi_read(NFC_Handle_t *h, uint8_t *out, uint8_t len);
static NFC_ErrorState_t wait_ready(NFC_Handle_t *h, uint32_t timeout_ms);
static NFC_ErrorState_t write_frame(NFC_Handle_t *h, const uint8_t *payload, uint8_t len);
static NFC_ErrorState_t read_ack(NFC_Handle_t *h, uint32_t timeout_ms);
static NFC_ErrorState_t read_response(NFC_Handle_t *h, uint8_t cmd,
                                      uint8_t *out, uint8_t out_max, uint8_t *out_len,
                                      uint32_t timeout_ms);
static NFC_ErrorState_t send_command(NFC_Handle_t *h, const uint8_t *cmd, uint8_t cmd_len,
                                     uint8_t *resp, uint8_t resp_max, uint8_t *resp_len,
                                     uint32_t timeout_ms);
static NFC_ErrorState_t get_fw(NFC_Handle_t *h, uint32_t *fw);
static NFC_ErrorState_t sam_config(NFC_Handle_t *h);
static NFC_ErrorState_t set_retries(NFC_Handle_t *h, uint8_t retries);
static bool is_protected_block(uint8_t block);

/*************** Section 3 : Software Interfaces Definitions (APIs) ****************/

Std_ReturnType ECUAL_NFC_Init(NFC_Handle_t *h, Mcal_SpiId_t spi, Mcal_GpioId_t cs)
{
    if (h == NULL) return E_NOT_OK;

    h->spi   = spi;
    h->cs    = cs;
    h->error = NFC_ERR_NONE;

    /* Generate SPI wake-up pulse: CS Low -> wait -> CS High */
    cs_low(h);
    Mcal_Delay_Ms(2);
    cs_high(h);
    Mcal_Delay_Ms(10);

    /* Fetch firmware version to confirm communication with a genuine PN532 */
    uint32_t fw = 0;
    NFC_ErrorState_t e = NFC_ERR_TIMEOUT;
    for (int i = 0; i < 3 && e != NFC_ERR_NONE; i++) {
        e = get_fw(h, &fw);
        if (e != NFC_ERR_NONE) {
            Mcal_Delay_Ms(20);
        }
    }
    if (e != NFC_ERR_NONE)   return finish(h, e);
    if ((fw >> 24) != 0x32U) return finish(h, NFC_ERR_FRAME);

    e = sam_config(h);
    if (e != NFC_ERR_NONE) return finish(h, e);

    return finish(h, set_retries(h, INIT_PASSIVE_RETRY));
}

Std_ReturnType ECUAL_NFC_GetFirmwareVersion(NFC_Handle_t *h, uint32_t *fw)
{
    if (h == NULL) return E_NOT_OK;
    if (fw == NULL) return finish(h, NFC_ERR_PARAM);
    return finish(h, get_fw(h, fw));
}

Std_ReturnType ECUAL_NFC_SAMConfig(NFC_Handle_t *h)
{
    if (h == NULL) return E_NOT_OK;
    return finish(h, sam_config(h));
}

Std_ReturnType ECUAL_NFC_SetPassiveActivationRetries(NFC_Handle_t *h, uint8_t retries)
{
    if (h == NULL) return E_NOT_OK;
    return finish(h, set_retries(h, retries));
}

Std_ReturnType ECUAL_NFC_ReadPassiveTargetID(NFC_Handle_t *h, uint8_t *uid,
                                             uint8_t uid_max, uint8_t *uid_len,
                                             uint32_t timeout_ms)
{
    if (h == NULL) return E_NOT_OK;
    if (uid == NULL || uid_len == NULL) return finish(h, NFC_ERR_PARAM);

    /* MaxTg = 1 card, ISO14443A @106 kbps */
    uint8_t cmd[3] = { NFC_CMD_INLISTPASSIVETARGET, 0x01U, NFC_BAUD_106K_TYPE_A };
    uint8_t resp[24];
    uint8_t rlen = 0U;

    NFC_ErrorState_t e = send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, timeout_ms);
    if (e != NFC_ERR_NONE) return finish(h, e);

    /* resp: NbTg, Tg, SENS_RES(2), SEL_RES, NFCIDLength, NFCID... */
    if (rlen < 1U)      return finish(h, NFC_ERR_FRAME);
    if (resp[0] == 0U)  return finish(h, NFC_ERR_NO_CARD);

    if (rlen < 6U) return finish(h, NFC_ERR_FRAME);
    uint8_t n = resp[5];
    if (n == 0U || n > NFC_UID_MAX_SIZE)  return finish(h, NFC_ERR_FRAME);
    if (rlen < (uint8_t)(6U + n))         return finish(h, NFC_ERR_FRAME);
    if (n > uid_max)                      return finish(h, NFC_ERR_PARAM);

    memcpy(uid, &resp[6], n);
    *uid_len = n;
    return finish(h, NFC_ERR_NONE);
}

Std_ReturnType ECUAL_NFC_MifareAuth(NFC_Handle_t *h, const uint8_t *uid, uint8_t uid_len,
                                    uint8_t block, NFC_KeyType_t key_type,
                                    const uint8_t key[NFC_MIFARE_KEY_SIZE])
{
    if (h == NULL) return E_NOT_OK;
    if (uid == NULL || key == NULL)                      return finish(h, NFC_ERR_PARAM);
    if (uid_len < 4U || uid_len > NFC_UID_MAX_SIZE)      return finish(h, NFC_ERR_PARAM);
    if (key_type != NFC_KEY_A && key_type != NFC_KEY_B)  return finish(h, NFC_ERR_PARAM);

    uint8_t cmd[14];
    cmd[0] = NFC_CMD_INDATAEXCHANGE;
    cmd[1] = 0x01U;                                   /* target number */
    cmd[2] = (uint8_t)key_type;
    cmd[3] = block;
    memcpy(&cmd[4],  key, NFC_MIFARE_KEY_SIZE);
    memcpy(&cmd[10], &uid[uid_len - 4U], 4U);         /* last 4 UID bytes */

    uint8_t resp[2];
    uint8_t rlen = 0U;
    NFC_ErrorState_t e = send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, AUTH_TIMEOUT_MS);
    if (e != NFC_ERR_NONE) return finish(h, e);
    if (rlen < 1U)         return finish(h, NFC_ERR_FRAME);
    return finish(h, (resp[0] == 0x00U) ? NFC_ERR_NONE : NFC_ERR_AUTH);
}

Std_ReturnType ECUAL_NFC_MifareReadBlock(NFC_Handle_t *h, uint8_t block,
                                         uint8_t data[NFC_MIFARE_BLOCK_SIZE])
{
    if (h == NULL) return E_NOT_OK;
    if (data == NULL) return finish(h, NFC_ERR_PARAM);

    uint8_t cmd[4] = { NFC_CMD_INDATAEXCHANGE, 0x01U, NFC_MIFARE_READ, block };
    uint8_t resp[1U + NFC_MIFARE_BLOCK_SIZE];
    uint8_t rlen = 0U;

    NFC_ErrorState_t e = send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, READ_TIMEOUT_MS);
    if (e != NFC_ERR_NONE) return finish(h, e);
    if (rlen < 1U)         return finish(h, NFC_ERR_FRAME);
    if (resp[0] != 0x00U)  return finish(h, NFC_ERR_CARD);
    if (rlen != (1U + NFC_MIFARE_BLOCK_SIZE)) return finish(h, NFC_ERR_FRAME);

    memcpy(data, &resp[1], NFC_MIFARE_BLOCK_SIZE);
    return finish(h, NFC_ERR_NONE);
}

Std_ReturnType ECUAL_NFC_MifareWriteBlock(NFC_Handle_t *h, uint8_t block,
                                          const uint8_t data[NFC_MIFARE_BLOCK_SIZE])
{
    if (h == NULL) return E_NOT_OK;
    if (data == NULL) return finish(h, NFC_ERR_PARAM);

#if (NFC_ALLOW_TRAILER_WRITE == 0U)
    if (is_protected_block(block)) return finish(h, NFC_ERR_PARAM);
#endif

    uint8_t cmd[4U + NFC_MIFARE_BLOCK_SIZE];
    cmd[0] = NFC_CMD_INDATAEXCHANGE;
    cmd[1] = 0x01U;
    cmd[2] = NFC_MIFARE_WRITE;
    cmd[3] = block;
    memcpy(&cmd[4], data, NFC_MIFARE_BLOCK_SIZE);

    uint8_t resp[2];
    uint8_t rlen = 0U;
    NFC_ErrorState_t e = send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, WRITE_TIMEOUT_MS);
    if (e != NFC_ERR_NONE) return finish(h, e);
    if (rlen < 1U)         return finish(h, NFC_ERR_FRAME);
    return finish(h, (resp[0] == 0x00U) ? NFC_ERR_NONE : NFC_ERR_CARD);
}

/************** Section 4 : Helper Functions Definitions **************/

static Std_ReturnType finish(NFC_Handle_t *h, NFC_ErrorState_t err)
{
    if (h != NULL) h->error = err;
    return (err == NFC_ERR_NONE) ? E_OK : E_NOT_OK;
}

/* ---------- Layer 1: SPI transport (through MCAL only) ---------- */

static void cs_low(NFC_Handle_t *h)
{
    (void)Mcal_Gpio_Write(h->cs, MCAL_GPIO_LOW);
}

static void cs_high(NFC_Handle_t *h)
{
    (void)Mcal_Gpio_Write(h->cs, MCAL_GPIO_HIGH);
}

static NFC_ErrorState_t spi_write(NFC_Handle_t *h, const uint8_t *frame, uint8_t len)
{
    uint8_t buf[TX_BUF_SIZE];
    if (((uint16_t)len + 1U) > TX_BUF_SIZE) return NFC_ERR_PARAM;

    buf[0] = NFC_SPI_DATA_WRITE;
    memcpy(&buf[1], frame, len);

    cs_low(h);
    Mcal_Delay_Ms(CS_WAKE_MS);                  /* wake-up / setup time (writes only) */
    Std_ReturnType r = Mcal_Spi_Transmit(h->spi, buf, (uint16_t)(len + 1U), SPI_TIMEOUT_MS);
    cs_high(h);
    return (r == E_OK) ? NFC_ERR_NONE : NFC_ERR_BUS;
}

static NFC_ErrorState_t spi_read(NFC_Handle_t *h, uint8_t *out, uint8_t len)
{
    uint8_t cmd = NFC_SPI_DATA_READ;

    cs_low(h);
    Std_ReturnType r = Mcal_Spi_Transmit(h->spi, &cmd, 1U, SPI_TIMEOUT_MS);
    if (r == E_OK) r = Mcal_Spi_Receive(h->spi, out, len, SPI_TIMEOUT_MS);
    cs_high(h);
    return (r == E_OK) ? NFC_ERR_NONE : NFC_ERR_BUS;
}

static NFC_ErrorState_t wait_ready(NFC_Handle_t *h, uint32_t timeout_ms)
{
    uint32_t start = Mcal_GetTick_Ms();

    while ((Mcal_GetTick_Ms() - start) < timeout_ms) {
        uint8_t cmd = NFC_SPI_STATUS_READ;
        uint8_t st  = 0;

        cs_low(h);
        Std_ReturnType r = Mcal_Spi_Transmit(h->spi, &cmd, 1, SPI_TIMEOUT_MS);
        if (r == E_OK) {
            r = Mcal_Spi_Receive(h->spi, &st, 1, SPI_TIMEOUT_MS);
        }
        cs_high(h);

        /* Ready bit is bit 0 */
        if ((r == E_OK) && (st & 0x01U)) {
            return NFC_ERR_NONE;
        }

        Mcal_Delay_Ms(1); /* Allow SPI recovery between polls */
    }

    return NFC_ERR_TIMEOUT;
}

/* ---------- Layer 2: framing ---------- */

static NFC_ErrorState_t write_frame(NFC_Handle_t *h, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[TX_BUF_SIZE];

    if (payload == NULL || len == 0U || ((uint16_t)len + 9U) > TX_BUF_SIZE) return NFC_ERR_PARAM;

    uint8_t n   = (uint8_t)(len + 1U);                  /* TFI + payload */
    uint8_t sum = NFC_TFI_HOST_TO_PN532;

    buf[0] = NFC_FRAME_PREAMBLE;
    buf[1] = NFC_FRAME_START1;
    buf[2] = NFC_FRAME_START2;
    buf[3] = n;
    buf[4] = (uint8_t)(~n + 1U);                        /* LCS */
    buf[5] = NFC_TFI_HOST_TO_PN532;
    for (uint8_t i = 0U; i < len; i++) {
        buf[6U + i] = payload[i];
        sum = (uint8_t)(sum + payload[i]);
    }
    buf[6U + len] = (uint8_t)(~sum + 1U);               /* DCS */
    buf[7U + len] = NFC_FRAME_POSTAMBLE;

    return spi_write(h, buf, (uint8_t)(len + 8U));
}

static NFC_ErrorState_t read_ack(NFC_Handle_t *h, uint32_t timeout_ms)
{
    uint8_t buf[6];

    NFC_ErrorState_t e = wait_ready(h, timeout_ms);
    if (e != NFC_ERR_NONE) return e;

    e = spi_read(h, buf, sizeof(buf));
    if (e != NFC_ERR_NONE) return e;

    return (memcmp(buf, ack_frame, sizeof(ack_frame)) == 0) ? NFC_ERR_NONE : NFC_ERR_NO_ACK;
}

static NFC_ErrorState_t read_response(NFC_Handle_t *h, uint8_t cmd,
                                      uint8_t *out, uint8_t out_max, uint8_t *out_len,
                                      uint32_t timeout_ms)
{
    uint8_t header[6]; /* 00 00 FF LEN LCS TFI */

    NFC_ErrorState_t e = wait_ready(h, timeout_ms);
    if (e != NFC_ERR_NONE) return e;

    /* Read preamble through to TFI */
    uint8_t cmd_read = SPI_DATA_READ;
    cs_low(h);
    Std_ReturnType r = Mcal_Spi_Transmit(h->spi, &cmd_read, 1, SPI_TIMEOUT_MS);
    if (r == E_OK) {
        r = Mcal_Spi_Receive(h->spi, header, 6, SPI_TIMEOUT_MS);
    }

    if (r != E_OK) {
        cs_high(h);
        return NFC_ERR_BUS;
    }

    if (header[0] != 0x00 || header[1] != 0x00 || header[2] != 0xFF) {
        cs_high(h);
        return NFC_ERR_FRAME;
    }

    uint8_t len = header[3];
    uint8_t lcs = header[4];
    if ((uint8_t)(len + lcs) != 0x00 || len < 2U || len > (RX_BUF_SIZE - 7U)) {
        cs_high(h);
        return NFC_ERR_FRAME;
    }

    /* Read exact remaining frame: (len - 1) payload bytes + DCS + Postamble (0x00) */
    uint8_t remaining = (uint8_t)(len + 1U);
    uint8_t rest[RX_BUF_SIZE];
    r = Mcal_Spi_Receive(h->spi, rest, remaining, SPI_TIMEOUT_MS);
    cs_high(h);

    if (r != E_OK) return NFC_ERR_BUS;

    /* Verify response TFI and command opcode echo */
    if (header[5] != TFI_PN532_TO_HOST || rest[0] != (uint8_t)(cmd + 1U)) {
        return NFC_ERR_FRAME;
    }

    /* Verify data checksum */
    uint8_t sum = header[5];
    for (uint8_t i = 0; i < (len - 1U); i++) {
        sum = (uint8_t)(sum + rest[i]);
    }
    sum = (uint8_t)(sum + rest[len - 1U]); /* DCS byte */
    if (sum != 0x00) return NFC_ERR_FRAME;

    uint8_t dlen = (uint8_t)(len - 2U);
    if (dlen > out_max) return NFC_ERR_PARAM;

    if (dlen > 0U && out != NULL) {
        memcpy(out, &rest[1], dlen);
    }
    if (out_len != NULL) {
        *out_len = dlen;
    }

    return NFC_ERR_NONE;
}

static NFC_ErrorState_t send_command(NFC_Handle_t *h, const uint8_t *cmd, uint8_t cmd_len,
                                     uint8_t *resp, uint8_t resp_max, uint8_t *resp_len,
                                     uint32_t timeout_ms)
{
    NFC_ErrorState_t e = write_frame(h, cmd, cmd_len);
    if (e != NFC_ERR_NONE) return e;

    e = read_ack(h, ACK_TIMEOUT_MS);
    if (e != NFC_ERR_NONE) return e;

    e = read_response(h, cmd[0], resp, resp_max, resp_len, timeout_ms);
    if (e == NFC_ERR_TIMEOUT) {
        /* Cancel the still-running command so a late reply cannot be mistaken
         * for the ACK of the next command (host ACK frame aborts the PN532). */
        (void)spi_write(h, ack_frame, (uint8_t)sizeof(ack_frame));
    }
    return e;
}

/* ---------- Layer 3: commands ---------- */

static NFC_ErrorState_t get_fw(NFC_Handle_t *h, uint32_t *fw)
{
    uint8_t cmd[1] = { NFC_CMD_GETFIRMWAREVERSION };
    uint8_t resp[4];
    uint8_t rlen = 0U;

    NFC_ErrorState_t e = send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, CMD_TIMEOUT_MS);
    if (e != NFC_ERR_NONE) return e;
    if (rlen != 4U)        return NFC_ERR_FRAME;

    if (fw != NULL) {
        *fw = ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16) |
              ((uint32_t)resp[2] << 8)  |  (uint32_t)resp[3];
    }
    return NFC_ERR_NONE;
}

static NFC_ErrorState_t sam_config(NFC_Handle_t *h)
{
    /* Normal mode, timeout 20 x 50 ms (only used in virtual-card mode), use IRQ pin */
    uint8_t cmd[4] = { NFC_CMD_SAMCONFIGURATION, 0x01U, 0x14U, 0x01U };
    uint8_t rlen = 0U;
    return send_command(h, cmd, sizeof(cmd), NULL, 0U, &rlen, CMD_TIMEOUT_MS);
}

static NFC_ErrorState_t set_retries(NFC_Handle_t *h, uint8_t retries)
{
    /* MxRtyATR = 0xFF, MxRtyPSL = 0x01, MxRtyPassiveActivation = retries */
    uint8_t cmd[5] = { NFC_CMD_RFCONFIGURATION, NFC_RFCFG_MAX_RETRIES, 0xFFU, 0x01U, retries };
    uint8_t rlen = 0U;
    return send_command(h, cmd, sizeof(cmd), NULL, 0U, &rlen, CMD_TIMEOUT_MS);
}

/* Block 0 (manufacturer data) and every sector trailer (keys + access bits). */
static bool is_protected_block(uint8_t block)
{
    if (block == 0U)    return true;
    if (block < 128U)   return ((block & 0x03U) == 0x03U);   /* 4-block sectors  */
    return ((block & 0x0FU) == 0x0FU);                       /* 16-block sectors */
}
