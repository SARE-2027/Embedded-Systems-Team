/******************* Section 0 : Includes *******************/

#include "../../../Inc/ECUAL/NFC/nfc.h"
#include <string.h>

/******************* Section 1 : Variables Definitions *******************/

#define TX_BUF_SIZE         (48U)
#define RX_BUF_SIZE         (48U)

#define SPI_TRANSFER_TIMEOUT_MS  (10U)
#define ACK_TIMEOUT_MS           (50U)
#define CMD_TIMEOUT_MS           (50U)
#define AUTH_TIMEOUT_MS          (150U)
#define READ_TIMEOUT_MS          (150U)
#define WRITE_TIMEOUT_MS         (200U)

static const uint8 ack_frame[6] = { 0x00U, 0x00U, 0xFFU, 0x00U, 0xFFU, 0x00U };

/************** Section 2 : Helper Functions Declarations **************/

static GPIO_TypeDef   *m_port_base(uint8 pin_id);
static uint16          m_pin_mask(uint8 pin_id);
static void            m_enable_gpio_clock(uint8 pin_id);
static void            m_enable_spi_clock(nfc_spi_id_t spi_id);
static SPI_TypeDef    *m_get_spi_instance(nfc_spi_id_t spi_id);
static uint8           m_get_spi_af(nfc_spi_id_t spi_id);
static Std_ReturnType  m_spi_hw_init(SPI_TypeDef *spi, nfc_spi_baud_t prescaler);

static inline void     m_cs_low(const nfc_t *h);
static inline void     m_cs_high(const nfc_t *h);

static Std_ReturnType  m_spi_transfer_byte(SPI_TypeDef *spi, uint8 tx_data, uint8 *rx_data, uint32 timeout_ms);
static nfc_error_t     m_spi_write(nfc_t *h, const uint8 *frame, uint8 len);
static nfc_error_t     m_spi_read(nfc_t *h, uint8 *out, uint8 len);
static nfc_error_t     m_wait_ready(nfc_t *h, uint32 timeout_ms);
static nfc_error_t     m_write_frame(nfc_t *h, const uint8 *payload, uint8 len);
static nfc_error_t     m_read_ack(nfc_t *h, uint32 timeout_ms);
static nfc_error_t     m_read_response(nfc_t *h, uint8 cmd, uint8 *out, uint8 out_max, uint8 *out_len, uint32 timeout_ms);
static nfc_error_t     m_send_command(nfc_t *h, const uint8 *cmd, uint8 cmd_len,
                                      uint8 *resp, uint8 resp_max, uint8 *resp_len, uint32 timeout_ms);

static nfc_error_t     m_get_fw(nfc_t *h, uint32 *fw);
static nfc_error_t     m_sam_config(nfc_t *h);
static nfc_error_t     m_set_retries(nfc_t *h, uint8 retries);
static bool            m_is_protected_block(uint8 block);

/*************** Section 3 : Software Interfaces Definitions (APIs) ****************/

Std_ReturnType nfc_init(nfc_t *h, const nfc_cfg_t *cfg)
{
    if ((NULL == h) || (NULL == cfg)) {
        return E_NOT_OK;
    }

    memset(h, 0, sizeof(nfc_t));
    h->last_error = NFC_ERR_NONE;

    /* Resolve SPI hardware instance */
    h->spi = m_get_spi_instance(cfg->spi_id);
    if (NULL == h->spi) {
        h->last_error = NFC_ERR_PARAM;
        return E_NOT_OK;
    }

    /* Enable peripheral clocks */
    m_enable_spi_clock(cfg->spi_id);
    m_enable_gpio_clock(cfg->mosi_pin);
    m_enable_gpio_clock(cfg->miso_pin);
    m_enable_gpio_clock(cfg->sck_pin);
    m_enable_gpio_clock(cfg->cs_pin);

    uint8 af = m_get_spi_af(cfg->spi_id);

    /* Configure SCK pin (Alternate Function Push-Pull) */
    GPIO_InitTypeDef gpio_sck = {0};
    gpio_sck.Pin       = m_pin_mask(cfg->sck_pin);
    gpio_sck.Mode      = GPIO_MODE_AF_PP;
    gpio_sck.Pull      = GPIO_NOPULL;
    gpio_sck.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_sck.Alternate = af;
    HAL_GPIO_Init(m_port_base(cfg->sck_pin), &gpio_sck);

    /* Configure MISO pin (Alternate Function with Pull-up) */
    GPIO_InitTypeDef gpio_miso = {0};
    gpio_miso.Pin       = m_pin_mask(cfg->miso_pin);
    gpio_miso.Mode      = GPIO_MODE_AF_PP;
    gpio_miso.Pull      = GPIO_PULLUP;
    gpio_miso.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_miso.Alternate = af;
    HAL_GPIO_Init(m_port_base(cfg->miso_pin), &gpio_miso);

    /* Configure MOSI pin (Alternate Function Push-Pull) */
    GPIO_InitTypeDef gpio_mosi = {0};
    gpio_mosi.Pin       = m_pin_mask(cfg->mosi_pin);
    gpio_mosi.Mode      = GPIO_MODE_AF_PP;
    gpio_mosi.Pull      = GPIO_NOPULL;
    gpio_mosi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_mosi.Alternate = af;
    HAL_GPIO_Init(m_port_base(cfg->mosi_pin), &gpio_mosi);

    /* Configure CS pin (Software NSS Output Push-Pull, initial HIGH) */
    h->cs_port     = m_port_base(cfg->cs_pin);
    h->cs_pin_mask = m_pin_mask(cfg->cs_pin);
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin_mask, GPIO_PIN_SET);

    GPIO_InitTypeDef gpio_cs = {0};
    gpio_cs.Pin   = h->cs_pin_mask;
    gpio_cs.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_cs.Pull  = GPIO_PULLUP;
    gpio_cs.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(h->cs_port, &gpio_cs);

    /* Initialize SPI hardware: Master, Mode 0 (CPOL=0, CPHA=0), LSB First */
    if (E_OK != m_spi_hw_init(h->spi, cfg->baud_prescaler)) {
        h->last_error = NFC_ERR_BUS;
        return E_NOT_OK;
    }

    /* Wake-up pulse sequence: CS Low -> 2 ms -> CS High -> 10 ms */
    m_cs_low(h);
    HAL_Delay(2);
    m_cs_high(h);
    HAL_Delay(10);

    /* Read firmware version to confirm communication with genuine PN532 */
    uint32 fw = 0;
    nfc_error_t err = NFC_ERR_TIMEOUT;
    for (int retry = 0; (retry < 3) && (err != NFC_ERR_NONE); retry++) {
        err = m_get_fw(h, &fw);
        if (err != NFC_ERR_NONE) {
            HAL_Delay(20);
        }
    }

    if (err != NFC_ERR_NONE) {
        h->last_error = err;
        return E_NOT_OK;
    }

    if (NFC_FW_IC(fw) != NFC_IC_PN532) {
        h->last_error = NFC_ERR_FRAME;
        return E_NOT_OK;
    }
    h->firmware_ver = fw;

    /* Configure SAM (Security Access Module) in Normal mode */
    err = m_sam_config(h);
    if (err != NFC_ERR_NONE) {
        h->last_error = err;
        return E_NOT_OK;
    }

    /* Set passive activation retries to 1 (enables quick non-blocking polling) */
    err = m_set_retries(h, 0x01U);
    if (err != NFC_ERR_NONE) {
        h->last_error = err;
        return E_NOT_OK;
    }

    h->initialized = true;
    h->last_poll_tick = HAL_GetTick();
    return E_OK;
}

void nfc_task(nfc_t *h)
{
    if ((NULL == h) || !h->initialized) {
        return;
    }

    uint32 current_tick = HAL_GetTick();
    if ((current_tick - h->last_poll_tick) < 250U) {
        return;
    }
    h->last_poll_tick = current_tick;

    /* Poll for ISO14443A card (106 kbps) with single retry (~10-15 ms max) */
    uint8 cmd[3] = { NFC_CMD_INLISTPASSIVETARGET, 0x01U, NFC_BAUD_106K_TYPE_A };
    uint8 resp[24] = {0};
    uint8 rlen = 0U;

    nfc_error_t err = m_send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, 30U);
    if ((NFC_ERR_NONE == err) && (rlen >= 1U)) {
        if ((resp[0] > 0U) && (rlen >= 6U)) {
            /* Card found! Extract ISO14443A Target properties */
            uint8 uid_len = resp[5];
            if ((uid_len > 0U) && (uid_len <= NFC_UID_MAX_LEN) && (rlen >= (6U + uid_len))) {
                h->latest_card.card_present = true;
                h->latest_card.uid_len = uid_len;
                memcpy(h->latest_card.uid, &resp[6], uid_len);
                h->latest_card.atqa = (uint16)((uint16)resp[2] | ((uint16)resp[3] << 8U));
                h->latest_card.sak = resp[4];
                h->latest_card.last_seen_ms = current_tick;
                h->last_error = NFC_ERR_NONE;
                return;
            }
        }
    }

    /* If no card detected in this cycle, clear present flag after 1000 ms absence */
    if (h->latest_card.card_present) {
        if ((current_tick - h->latest_card.last_seen_ms) > 1000U) {
            h->latest_card.card_present = false;
        }
    }
}

bool nfc_get_card(const nfc_t *h, nfc_card_t *out_card)
{
    if ((NULL == h) || !h->initialized || (NULL == out_card)) {
        return false;
    }

    *out_card = h->latest_card;
    return h->latest_card.card_present;
}

bool nfc_is_card_present(const nfc_t *h)
{
    if ((NULL == h) || !h->initialized) {
        return false;
    }
    return h->latest_card.card_present;
}

Std_ReturnType nfc_get_firmware_version(nfc_t *h, uint32 *fw)
{
    if ((NULL == h) || (NULL == fw)) {
        return E_NOT_OK;
    }
    nfc_error_t err = m_get_fw(h, fw);
    h->last_error = err;
    return (err == NFC_ERR_NONE) ? E_OK : E_NOT_OK;
}

Std_ReturnType nfc_sam_config(nfc_t *h)
{
    if (NULL == h) {
        return E_NOT_OK;
    }
    nfc_error_t err = m_sam_config(h);
    h->last_error = err;
    return (err == NFC_ERR_NONE) ? E_OK : E_NOT_OK;
}

Std_ReturnType nfc_set_retries(nfc_t *h, uint8 retries)
{
    if (NULL == h) {
        return E_NOT_OK;
    }
    nfc_error_t err = m_set_retries(h, retries);
    h->last_error = err;
    return (err == NFC_ERR_NONE) ? E_OK : E_NOT_OK;
}

Std_ReturnType nfc_read_passive_target_id(nfc_t *h, uint8 *uid, uint8 uid_max,
                                          uint8 *uid_len, uint32 timeout_ms)
{
    if ((NULL == h) || (NULL == uid) || (NULL == uid_len)) {
        return E_NOT_OK;
    }

    uint8 cmd[3] = { NFC_CMD_INLISTPASSIVETARGET, 0x01U, NFC_BAUD_106K_TYPE_A };
    uint8 resp[24];
    uint8 rlen = 0U;

    nfc_error_t err = m_send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, timeout_ms);
    h->last_error = err;
    if (err != NFC_ERR_NONE) return E_NOT_OK;

    if (rlen < 1U)      { h->last_error = NFC_ERR_FRAME;   return E_NOT_OK; }
    if (resp[0] == 0U)  { h->last_error = NFC_ERR_NO_CARD; return E_NOT_OK; }
    if (rlen < 6U)      { h->last_error = NFC_ERR_FRAME;   return E_NOT_OK; }

    uint8 n = resp[5];
    if ((n == 0U) || (n > NFC_UID_MAX_LEN) || (rlen < (uint8)(6U + n))) {
        h->last_error = NFC_ERR_FRAME;
        return E_NOT_OK;
    }
    if (n > uid_max) {
        h->last_error = NFC_ERR_PARAM;
        return E_NOT_OK;
    }

    memcpy(uid, &resp[6], n);
    *uid_len = n;
    return E_OK;
}

Std_ReturnType nfc_mifare_auth(nfc_t *h, const uint8 *uid, uint8 uid_len,
                               uint8 block, nfc_key_type_t key_type,
                               const uint8 key[NFC_MIFARE_KEY_SIZE])
{
    if ((NULL == h) || (NULL == uid) || (NULL == key)) return E_NOT_OK;
    if ((uid_len < 4U) || (uid_len > NFC_UID_MAX_LEN))  return E_NOT_OK;
    if ((key_type != NFC_KEY_A) && (key_type != NFC_KEY_B)) return E_NOT_OK;

    uint8 cmd[14];
    cmd[0] = NFC_CMD_INDATAEXCHANGE;
    cmd[1] = 0x01U; /* Target number */
    cmd[2] = (uint8)key_type;
    cmd[3] = block;
    memcpy(&cmd[4], key, NFC_MIFARE_KEY_SIZE);
    memcpy(&cmd[10], &uid[uid_len - 4U], 4U); /* Last 4 bytes of UID */

    uint8 resp[2];
    uint8 rlen = 0U;
    nfc_error_t err = m_send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, AUTH_TIMEOUT_MS);
    h->last_error = err;
    if (err != NFC_ERR_NONE) return E_NOT_OK;
    if (rlen < 1U) { h->last_error = NFC_ERR_FRAME; return E_NOT_OK; }

    if (resp[0] != 0x00U) {
        h->last_error = NFC_ERR_AUTH;
        return E_NOT_OK;
    }
    return E_OK;
}

Std_ReturnType nfc_mifare_read_block(nfc_t *h, uint8 block,
                                     uint8 data[NFC_MIFARE_BLOCK_SIZE])
{
    if ((NULL == h) || (NULL == data)) return E_NOT_OK;

    uint8 cmd[4] = { NFC_CMD_INDATAEXCHANGE, 0x01U, NFC_MIFARE_READ, block };
    uint8 resp[1U + NFC_MIFARE_BLOCK_SIZE];
    uint8 rlen = 0U;

    nfc_error_t err = m_send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, READ_TIMEOUT_MS);
    h->last_error = err;
    if (err != NFC_ERR_NONE) return E_NOT_OK;
    if (rlen < 1U) { h->last_error = NFC_ERR_FRAME; return E_NOT_OK; }
    if (resp[0] != 0x00U) { h->last_error = NFC_ERR_CARD; return E_NOT_OK; }
    if (rlen != (1U + NFC_MIFARE_BLOCK_SIZE)) { h->last_error = NFC_ERR_FRAME; return E_NOT_OK; }

    memcpy(data, &resp[1], NFC_MIFARE_BLOCK_SIZE);
    return E_OK;
}

Std_ReturnType nfc_mifare_write_block(nfc_t *h, uint8 block,
                                      const uint8 data[NFC_MIFARE_BLOCK_SIZE])
{
    if ((NULL == h) || (NULL == data)) return E_NOT_OK;

#if (NFC_ALLOW_TRAILER_WRITE == 0U)
    if (m_is_protected_block(block)) {
        h->last_error = NFC_ERR_PARAM;
        return E_NOT_OK;
    }
#endif

    uint8 cmd[4U + NFC_MIFARE_BLOCK_SIZE];
    cmd[0] = NFC_CMD_INDATAEXCHANGE;
    cmd[1] = 0x01U;
    cmd[2] = NFC_MIFARE_WRITE;
    cmd[3] = block;
    memcpy(&cmd[4], data, NFC_MIFARE_BLOCK_SIZE);

    uint8 resp[2];
    uint8 rlen = 0U;
    nfc_error_t err = m_send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, WRITE_TIMEOUT_MS);
    h->last_error = err;
    if (err != NFC_ERR_NONE) return E_NOT_OK;
    if (rlen < 1U) { h->last_error = NFC_ERR_FRAME; return E_NOT_OK; }
    if (resp[0] != 0x00U) { h->last_error = NFC_ERR_CARD; return E_NOT_OK; }

    return E_OK;
}

/************** Section 4 : Helper Functions Definitions **************/

static GPIO_TypeDef *m_port_base(uint8 pin_id)
{
    switch (pin_id >> 4U) {
        case NFC_PORT_A: return GPIOA;
        case NFC_PORT_B: return GPIOB;
        case NFC_PORT_C: return GPIOC;
        case NFC_PORT_D: return GPIOD;
        case NFC_PORT_E: return GPIOE;
        case NFC_PORT_H: return GPIOH;
        default:         return NULL;
    }
}

static uint16 m_pin_mask(uint8 pin_id)
{
    return (uint16)(1U << (pin_id & 0x0FU));
}

static void m_enable_gpio_clock(uint8 pin_id)
{
    switch (pin_id >> 4U) {
        case NFC_PORT_A: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
        case NFC_PORT_B: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
        case NFC_PORT_C: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
        case NFC_PORT_D: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
        case NFC_PORT_E: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
        case NFC_PORT_H: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
        default: break;
    }
}

static void m_enable_spi_clock(nfc_spi_id_t spi_id)
{
    switch (spi_id) {
        case NFC_SPI_1: __HAL_RCC_SPI1_CLK_ENABLE(); break;
        case NFC_SPI_2: __HAL_RCC_SPI2_CLK_ENABLE(); break;
        case NFC_SPI_3: __HAL_RCC_SPI3_CLK_ENABLE(); break;
        default: break;
    }
}

static SPI_TypeDef *m_get_spi_instance(nfc_spi_id_t spi_id)
{
    switch (spi_id) {
        case NFC_SPI_1: return SPI1;
        case NFC_SPI_2: return SPI2;
        case NFC_SPI_3: return SPI3;
        default:        return NULL;
    }
}

static uint8 m_get_spi_af(nfc_spi_id_t spi_id)
{
    switch (spi_id) {
        case NFC_SPI_1: return GPIO_AF5_SPI1;
        case NFC_SPI_2: return GPIO_AF5_SPI2;
        case NFC_SPI_3: return GPIO_AF6_SPI3;
        default:        return GPIO_AF5_SPI1;
    }
}

static Std_ReturnType m_spi_hw_init(SPI_TypeDef *spi, nfc_spi_baud_t prescaler)
{
    if (NULL == spi) {
        return E_NOT_OK;
    }

    /* Disable SPI before configuration */
    spi->CR1 &= ~SPI_CR1_SPE;

    /*
     * SPI CR1 Configuration:
     * - Master mode: SPI_CR1_MSTR
     * - Software Slave Management: SPI_CR1_SSM | SPI_CR1_SSI
     * - Mode 0: CPOL = 0, CPHA = 0 (bits cleared)
     * - LSB First: SPI_CR1_LSBFIRST (required by PN532 SPI protocol)
     * - 8-bit data format: DFF = 0 (bit cleared)
     * - Prescaler bits: bits 5:3
     */
    uint32 cr1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_LSBFIRST;
    cr1 |= ((uint32)prescaler & SPI_CR1_BR_Msk);
    spi->CR1 = cr1;

    /* CR2 default (SSOE = 0 since SSM = 1) */
    spi->CR2 = 0;

    /* Enable SPI peripheral */
    spi->CR1 |= SPI_CR1_SPE;

    return E_OK;
}

static inline void m_cs_low(const nfc_t *h)
{
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin_mask, GPIO_PIN_RESET);
}

static inline void m_cs_high(const nfc_t *h)
{
    HAL_GPIO_WritePin(h->cs_port, h->cs_pin_mask, GPIO_PIN_SET);
}

static Std_ReturnType m_spi_transfer_byte(SPI_TypeDef *spi, uint8 tx_data, uint8 *rx_data, uint32 timeout_ms)
{
    uint32 start = HAL_GetTick();

    /* Wait until TXE (Transmit buffer empty) */
    while (!(spi->SR & SPI_SR_TXE)) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return E_NOT_OK;
        }
    }

    /* Write 8-bit byte into Data Register */
    *(__IO uint8_t *)&spi->DR = tx_data;

    /* Wait until RXNE (Receive buffer not empty) */
    while (!(spi->SR & SPI_SR_RXNE)) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return E_NOT_OK;
        }
    }

    /* Read received 8-bit byte */
    uint8 val = *(__IO uint8_t *)&spi->DR;
    if (rx_data != NULL) {
        *rx_data = val;
    }

    return E_OK;
}

static nfc_error_t m_spi_write(nfc_t *h, const uint8 *frame, uint8 len)
{
    if ((NULL == frame) || (len == 0U)) {
        return NFC_ERR_PARAM;
    }

    m_cs_low(h);
    HAL_Delay(2); /* PN532 wake-up / setup time: t_SSEL_TO_CLK >= 1 ms */

    /* Send Data Write prefix byte */
    if (E_OK != m_spi_transfer_byte(h->spi, NFC_SPI_DATA_WRITE, NULL, SPI_TRANSFER_TIMEOUT_MS)) {
        m_cs_high(h);
        return NFC_ERR_BUS;
    }

    /* Send frame bytes */
    for (uint8 i = 0; i < len; i++) {
        if (E_OK != m_spi_transfer_byte(h->spi, frame[i], NULL, SPI_TRANSFER_TIMEOUT_MS)) {
            m_cs_high(h);
            return NFC_ERR_BUS;
        }
    }

    /* Wait for SPI transmission to complete before raising CS */
    uint32 start = HAL_GetTick();
    while (h->spi->SR & SPI_SR_BSY) {
        if ((HAL_GetTick() - start) > SPI_TRANSFER_TIMEOUT_MS) {
            break;
        }
    }

    m_cs_high(h);
    return NFC_ERR_NONE;
}

static nfc_error_t m_spi_read(nfc_t *h, uint8 *out, uint8 len)
{
    if ((NULL == out) || (len == 0U)) {
        return NFC_ERR_PARAM;
    }

    m_cs_low(h);

    /* Send Data Read prefix byte */
    if (E_OK != m_spi_transfer_byte(h->spi, NFC_SPI_DATA_READ, NULL, SPI_TRANSFER_TIMEOUT_MS)) {
        m_cs_high(h);
        return NFC_ERR_BUS;
    }

    /* Receive len bytes */
    for (uint8 i = 0; i < len; i++) {
        if (E_OK != m_spi_transfer_byte(h->spi, 0x00, &out[i], SPI_TRANSFER_TIMEOUT_MS)) {
            m_cs_high(h);
            return NFC_ERR_BUS;
        }
    }

    m_cs_high(h);
    return NFC_ERR_NONE;
}

static nfc_error_t m_wait_ready(nfc_t *h, uint32 timeout_ms)
{
    uint32 start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8 cmd = NFC_SPI_STATUS_READ;
        uint8 st = 0;

        m_cs_low(h);
        if (E_OK == m_spi_transfer_byte(h->spi, cmd, NULL, SPI_TRANSFER_TIMEOUT_MS)) {
            (void)m_spi_transfer_byte(h->spi, 0x00, &st, SPI_TRANSFER_TIMEOUT_MS);
        }
        m_cs_high(h);

        /* Ready bit is bit 0 */
        if (st & NFC_SPI_STATUS_READY) {
            return NFC_ERR_NONE;
        }

        HAL_Delay(1);
    }

    return NFC_ERR_TIMEOUT;
}

static nfc_error_t m_write_frame(nfc_t *h, const uint8 *payload, uint8 len)
{
    uint8 buf[TX_BUF_SIZE];

    if ((NULL == payload) || (len == 0U) || (((uint16)len + 9U) > TX_BUF_SIZE)) {
        return NFC_ERR_PARAM;
    }

    uint8 n   = (uint8)(len + 1U); /* TFI + payload */
    uint8 sum = NFC_TFI_HOST_TO_PN532;

    buf[0] = NFC_FRAME_PREAMBLE;
    buf[1] = NFC_FRAME_START1;
    buf[2] = NFC_FRAME_START2;
    buf[3] = n;
    buf[4] = (uint8)(~n + 1U);     /* LCS: Length checksum */
    buf[5] = NFC_TFI_HOST_TO_PN532;
    for (uint8 i = 0U; i < len; i++) {
        buf[6U + i] = payload[i];
        sum = (uint8)(sum + payload[i]);
    }
    buf[6U + len] = (uint8)(~sum + 1U); /* DCS: Data checksum */
    buf[7U + len] = NFC_FRAME_POSTAMBLE;

    return m_spi_write(h, buf, (uint8)(len + 8U));
}

static nfc_error_t m_read_ack(nfc_t *h, uint32 timeout_ms)
{
    uint8 buf[6];

    nfc_error_t err = m_wait_ready(h, timeout_ms);
    if (NFC_ERR_NONE != err) {
        return err;
    }

    err = m_spi_read(h, buf, sizeof(buf));
    if (NFC_ERR_NONE != err) {
        return err;
    }

    return (memcmp(buf, ack_frame, sizeof(ack_frame)) == 0) ? NFC_ERR_NONE : NFC_ERR_NO_ACK;
}

static nfc_error_t m_read_response(nfc_t *h, uint8 cmd,
                                   uint8 *out, uint8 out_max, uint8 *out_len,
                                   uint32 timeout_ms)
{
    uint8 header[6]; /* 00 00 FF LEN LCS TFI */

    nfc_error_t err = m_wait_ready(h, timeout_ms);
    if (NFC_ERR_NONE != err) {
        return err;
    }

    m_cs_low(h);

    /* Send Data Read prefix */
    if (E_OK != m_spi_transfer_byte(h->spi, NFC_SPI_DATA_READ, NULL, SPI_TRANSFER_TIMEOUT_MS)) {
        m_cs_high(h);
        return NFC_ERR_BUS;
    }

    /* Read preamble through to TFI (6 bytes) */
    for (uint8 i = 0; i < 6; i++) {
        if (E_OK != m_spi_transfer_byte(h->spi, 0x00, &header[i], SPI_TRANSFER_TIMEOUT_MS)) {
            m_cs_high(h);
            return NFC_ERR_BUS;
        }
    }

    /* Validate preamble and start codes */
    if ((header[0] != NFC_FRAME_PREAMBLE) || 
        (header[1] != NFC_FRAME_START1) || 
        (header[2] != NFC_FRAME_START2)) {
        m_cs_high(h);
        return NFC_ERR_FRAME;
    }

    uint8 len = header[3];
    uint8 lcs = header[4];

    /* Validate length checksum and buffer bounds */
    if (((uint8)(len + lcs) != 0x00U) || (len < 2U) || (len > (RX_BUF_SIZE - 7U))) {
        m_cs_high(h);
        return NFC_ERR_FRAME;
    }

    /* Read remaining frame: (len - 1) payload bytes + DCS + Postamble */
    uint8 remaining = (uint8)(len + 1U);
    uint8 rest[RX_BUF_SIZE];
    for (uint8 i = 0; i < remaining; i++) {
        if (E_OK != m_spi_transfer_byte(h->spi, 0x00, &rest[i], SPI_TRANSFER_TIMEOUT_MS)) {
            m_cs_high(h);
            return NFC_ERR_BUS;
        }
    }

    m_cs_high(h);

    /* Verify response TFI (0xD5) and command opcode echo (cmd + 1) */
    if ((header[5] != NFC_TFI_PN532_TO_HOST) || (rest[0] != (uint8)(cmd + 1U))) {
        return NFC_ERR_FRAME;
    }

    /* Verify data checksum */
    uint8 sum = header[5];
    for (uint8 i = 0; i < (len - 1U); i++) {
        sum = (uint8)(sum + rest[i]);
    }
    sum = (uint8)(sum + rest[len - 1U]); /* DCS byte */
    if (sum != 0x00U) {
        return NFC_ERR_FRAME;
    }

    uint8 dlen = (uint8)(len - 2U);
    if ((dlen > 0U) && (dlen > out_max)) {
        return NFC_ERR_PARAM;
    }

    if ((dlen > 0U) && (out != NULL)) {
        memcpy(out, &rest[1], dlen);
    }
    if (out_len != NULL) {
        *out_len = dlen;
    }

    return NFC_ERR_NONE;
}

static nfc_error_t m_send_command(nfc_t *h, const uint8 *cmd, uint8 cmd_len,
                                  uint8 *resp, uint8 resp_max, uint8 *resp_len,
                                  uint32 timeout_ms)
{
    nfc_error_t err = m_write_frame(h, cmd, cmd_len);
    if (NFC_ERR_NONE != err) {
        return err;
    }

    err = m_read_ack(h, ACK_TIMEOUT_MS);
    if (NFC_ERR_NONE != err) {
        return err;
    }

    err = m_read_response(h, cmd[0], resp, resp_max, resp_len, timeout_ms);
    if (NFC_ERR_TIMEOUT == err) {
        /* Abort ongoing command on PN532 to prevent late responses colliding with next command */
        (void)m_spi_write(h, ack_frame, (uint8)sizeof(ack_frame));
    }
    return err;
}

static nfc_error_t m_get_fw(nfc_t *h, uint32 *fw)
{
    uint8 cmd[1] = { NFC_CMD_GETFIRMWAREVERSION };
    uint8 resp[4] = {0};
    uint8 rlen = 0U;

    nfc_error_t err = m_send_command(h, cmd, sizeof(cmd), resp, sizeof(resp), &rlen, CMD_TIMEOUT_MS);
    if (err != NFC_ERR_NONE) return err;
    if (rlen != 4U) return NFC_ERR_FRAME;

    if (fw != NULL) {
        *fw = ((uint32)resp[0] << 24U) | ((uint32)resp[1] << 16U) |
              ((uint32)resp[2] << 8U)  |  (uint32)resp[3];
    }
    return NFC_ERR_NONE;
}

static nfc_error_t m_sam_config(nfc_t *h)
{
    /* Normal mode (0x01), timeout 20 x 50 ms (0x14), use IRQ pin (0x01) */
    uint8 cmd[4] = { NFC_CMD_SAMCONFIGURATION, 0x01U, 0x14U, 0x01U };
    uint8 rlen = 0U;
    return m_send_command(h, cmd, sizeof(cmd), NULL, 0U, &rlen, CMD_TIMEOUT_MS);
}

static nfc_error_t m_set_retries(nfc_t *h, uint8 retries)
{
    /* MxRtyATR = 0xFF, MxRtyPSL = 0x01, MxRtyPassiveActivation = retries */
    uint8 cmd[5] = { NFC_CMD_RFCONFIGURATION, NFC_RFCFG_MAX_RETRIES, 0xFFU, 0x01U, retries };
    uint8 rlen = 0U;
    return m_send_command(h, cmd, sizeof(cmd), NULL, 0U, &rlen, CMD_TIMEOUT_MS);
}

static bool m_is_protected_block(uint8 block)
{
    if (block == 0U)    return true;
    if (block < 128U)   return ((block & 0x03U) == 0x03U); /* 4-block sectors */
    return ((block & 0x0FU) == 0x0FU);                     /* 16-block sectors */
}
