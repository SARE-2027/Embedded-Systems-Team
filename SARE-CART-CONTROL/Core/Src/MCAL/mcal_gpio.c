/******************* Section 0 : Includes *******************/

/* NOTE: the port/pin table must match the CubeMX (.ioc) configuration.
 * main.h is NOT included on purpose (it includes ECUAL -> upward include). */
#include "../../Inc/MCAL/mcal_gpio.h"

/******************* Section 1 : Variables Definitions *******************/

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} GpioMap_t;

static const GpioMap_t gpio_map[MCAL_GPIO_COUNT] = {
    [MCAL_GPIO_NFC_CS] = { GPIOA, GPIO_PIN_0 },
};

/*************** Section 3 : Software Interfaces Definitions (APIs) ****************/

Std_ReturnType Mcal_Gpio_Write(Mcal_GpioId_t id, Mcal_GpioLevel_t level)
{
    if ((unsigned)id >= (unsigned)MCAL_GPIO_COUNT) return E_NOT_OK;

    HAL_GPIO_WritePin(gpio_map[id].port, gpio_map[id].pin,
                      (level == MCAL_GPIO_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return E_OK;
}
