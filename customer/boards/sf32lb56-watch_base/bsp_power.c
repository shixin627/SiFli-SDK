#include "bsp_board.h"

#ifdef CUSTOMER_BOARD
    #define GSENSOR_POWER_EN WATCH_GSENSOR_POWER_EN // GPIOB
#else
    #define GSENSOR_POWER_EN (21) // GPIOB_21
#endif

// NO SD
// #define SD1_RESET_PIN       (11)
// #define SD1_EN_PIN          (2)

// NO MPI Power pins
// #define MPI1_POWER_PIN  (33)
// #define MPI2_POWER_PIN  (26)
// #define MPI3_POWER_PIN  (43)

void BSP_GPIO_Set(int pin, int val, int is_porta)
{
    GPIO_TypeDef *gpio = (is_porta) ? hwp_gpio1 : hwp_gpio2;
    GPIO_InitTypeDef GPIO_InitStruct;

    // set sensor pin to output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    // set sensor pin to high == power on sensor board
    HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)val);
}

static void BSP_PBR_GPIO_Set(int pin, uint8_t state)
{
    HAL_PBR_ConfigMode(pin, true);
    HAL_PBR_WritePin(pin, state);
}

void BSP_Power_Up(bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU
    /*
        Power on g-sensor to prevent it pull down PB23 which using for TWI_DAT
       for Europa.
    */
    BSP_GPIO_Set(GSENSOR_POWER_EN, 1, 0);
    if (is_deep_sleep)
    {

        if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
        {
        }

        // Replace with API that is OS-independent.
        // rt_psram_exit_low_power("psram0");
    }
#elif defined(SOC_BF0_LCPU)
    {
        ;
    }
#endif
}

void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU
    if (coreid == CORE_ID_HCPU)
    {
        // Replace with API that is OS-independent.
        // if (is_deep_sleep)
        // rt_psram_enter_low_power("psram0");
    }
#else
    {
        ;
    }
#endif
}

void BSP_SD_PowerUp(void)
{
#ifdef PMIC_CTRL_ENABLE
    BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_5, 1, 1); // LCD_1V8 power
    BSP_PMIC_Control(PMIC_OUT_LDO33_VOUT, 1, 1);    // LCD_3V3 power
#endif                                              /* PMIC_CTRL_ENABLE */

#ifdef SD1_RESET_PIN
    BSP_GPIO_Set(SD1_EN_PIN, 1, 1);
#endif
#ifdef SD1_RESET_PIN
    BSP_GPIO_Set(SD1_RESET_PIN, 1, 1);
#endif
}

void BSP_SD_PowerDown(void)
{
#ifdef SD1_RESET_PIN
    BSP_GPIO_Set(SD1_EN_PIN, 0, 1);
#endif
#ifdef SD1_RESET_PIN
    BSP_GPIO_Set(SD1_RESET_PIN, 0, 1);
#endif
}

void BSP_Motor_PowerSwitch(bool is_power_on)
{
#if defined(MOTOR_POWER_VCC_EN_PIN)
    #if (MOTOR_POWER_VCC_EN_PIN >= 160)
    {
        BSP_PBR_GPIO_Set(MOTOR_POWER_VCC_EN_PIN - 160, is_power_on);
    }
    #elif (MOTOR_POWER_VCC_EN_PIN >= 96)
    {
        BSP_GPIO_Set(MOTOR_POWER_VCC_EN_PIN - 96, is_power_on, 0);
    }
    #else
    {
        BSP_GPIO_Set(MOTOR_POWER_VCC_EN_PIN, is_power_on, 1);
    }
    #endif
#endif
}