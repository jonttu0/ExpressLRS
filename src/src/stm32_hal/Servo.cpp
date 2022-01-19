#include "Servo.h"
#include "internal.h"
#include "irq.h"
#include "priorities.h"
#include "hal_inc.h"
#include "helpers.h" // ARRAY_SIZE
#include <Arduino.h>

#if defined(TARGET_R9M_RX)

//#include <stm32f1xx_ll_tim.h>
//#include <stm32l0xx_ll_tim.h>

#define TIMx            TIM1
#define TIMx_IRQn       TIM1_IRQn

uint16_t TIM1_pin_to_ch[] = {PA8, PA9, PA10, PA11};


static TIM_HandleTypeDef htim;


void servo_timer_init_debug(void)
{
    /* Peripheral clock enable */
    enable_pclock((uint32_t)TIMx);

#if 1

#if 0
    gpio_peripheral(PA8,  GPIO_FUNCTION(2), 0);
    gpio_peripheral(PA9,  GPIO_FUNCTION(2), 0);
    gpio_peripheral(PA10, GPIO_FUNCTION(2), 0);
    gpio_peripheral(PA11, GPIO_FUNCTION(2), 0);

    TIMx->CR1 = 0;
    //TIMx->DIER = 0;
    //TIMx->SR &= ~(TIM_SR_UIF);

    // Set clock prescaler to 1us or 2us
    // Note: PSC == 1 clock is APB1 x1 (36MHz) else x2 (72MHz)
    TIMx->PSC = (2 * get_pclock_frequency((uint32_t)TIMx) / (1000000)) - 1;
    TIMx->ARR = 20000 - 1; // 20ms
    TIMx->CNT = 0;
    //TIMx->RCR = 0;
    TIMx->EGR = TIM_EGR_UG /*| TIM_EGR_TG*/;
    TIMx->DIER = 0; //TIM_DIER_UIE;

    //TIMx->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
    //TIMx->CR2 = (TIM_CR2_OIS1 | TIM_CR2_OIS2 | TIM_CR2_OIS3 | TIM_CR2_OIS4);

    TIMx->CCER = 0;
    TIMx->CCR1 = 1000;
    TIMx->CCR2 = 1500;
    TIMx->CCR3 = 2000;
    TIMx->CCR4 = 3000;
    TIMx->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1CE;

    TIMx->CCER = (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    // TIM1 reguires:
    TIMx->BDTR |= TIM_BDTR_MOE;

    TIMx->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;

#else
    //TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    gpio_peripheral(PA8,  GPIO_FUNCTION(2), 0);
    gpio_peripheral(PA9,  GPIO_FUNCTION(2), 0);
    gpio_peripheral(PA10, GPIO_FUNCTION(2), 0);
    gpio_peripheral(PA11, GPIO_FUNCTION(2), 0);

    __HAL_RCC_USART1_CLK_DISABLE();
    USART1->CR1 = 0;

    htim.Instance = TIM1;
    htim.Init.Prescaler = 72 - 1;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = 20000 - 1;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.RepetitionCounter = 0;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim);

    //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    //sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    //HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 2000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCNIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_3, 4000);
#endif
#else

    LL_TIM_InitTypeDef TIM_InitStruct;
    // microsecond timer
    TIM_InitStruct.Prescaler = (2 * get_pclock_frequency((uint32_t)TIMx) / 1000000) - 1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 20000; // 20ms
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIMx, &TIM_InitStruct);
    //LL_TIM_DisableARRPreload(TIMx);
    //LL_TIM_SetClockSource(TIMx, LL_TIM_CLOCKSOURCE_INTERNAL);
    //LL_TIM_DisableMasterSlaveMode(TIMx);
    //LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
#endif

    while(1) {}
}



static uint8_t timer_initialized;
static void servo_timer_init(void)
{
    if (timer_initialized) return;

    /* Disable uart to avoid problems */
    __HAL_RCC_USART1_CLK_DISABLE();
    USART1->CR1 = 0;

    /* Peripheral clock enable */
    enable_pclock((uint32_t)TIMx);

    htim.Instance = TIMx;
    htim.Init.Prescaler = (2 * get_pclock_frequency((uint32_t)TIMx) / 1000000) - 1;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = 20000 - 1;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim.Init.RepetitionCounter = 0;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim);

    timer_initialized = 1;
}


static void TIMx_Channel_Init(uint32_t channel, uint16_t val=1500)
{

    servo_timer_init();

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = val;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCNIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel);
    HAL_TIM_PWM_Start(&htim, channel);
}


void Servo::attach(int pin, uint16_t min, uint16_t max)
{
    uint8_t ch;
    for (ch = 0; ch < ARRAY_SIZE(TIM1_pin_to_ch); ch++) {
        if (TIM1_pin_to_ch[ch] == pin)
            break;
    }
    if (ARRAY_SIZE(TIM1_pin_to_ch) <= ch)
        return;

    _min = min;
    _max = max;

    _channel = ch << 2;

    gpio_peripheral(pin, GPIO_FUNCTION(2), 0);

    min = (min + max) / 2; // resuse mid for mid point
    TIMx_Channel_Init(_channel, min);
}

void Servo::writeMicroseconds(uint32_t us)
{
    /* limit input to range */
    us = (us < _min) ? _min : (_max < us) ? _max : us;
    // set channel...
    __HAL_TIM_SET_COMPARE(&htim, _channel, us);
}

#endif /* TARGET_R9M_RX */
