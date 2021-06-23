#include "soft_serial_send.h"
#include "stm32f4xx_hal.h"
#include "bitband.h"

//定时器频率
#define TIMER_FREQ 84000000
//串口波特率
#define BAUD_RATE 115200

//发送波特率定时器句柄
TIM_HandleTypeDef htim11;
//要发送的字节
uint8_t send_byte;
//GPIO_ODR寄存器的位带地址
volatile uint32_t *GPIO_ODR;
//发送状态机
uint8_t state;

/**********************************************************************************************************
*函 数 名: soft_erial_send_init
*功能说明: 软件串口发送初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void soft_erial_send_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //初始化GPIO
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_1, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //初始化发送波特率定时器
    __HAL_RCC_TIM11_CLK_ENABLE();
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 0;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = TIMER_FREQ / BAUD_RATE;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim11);
    
    //初始化中断控制器
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

/**********************************************************************************************************
*函 数 名: soft_erial_send
*功能说明: 软件串口发送
*形    参: 串口号 要发送的数据
*返 回 值: 无
**********************************************************************************************************/
void soft_erial_send(uint8_t serial_num, uint8_t data)
{
    if (serial_num == 3) {
        while (state);
        //初始化GPIO_ODR寄存器的位带地址
        GPIO_ODR = (volatile uint32_t *)&PAout(5);
        //初始化要发送的数据
        send_byte = data;
        //清除定时器中断标志
        __HAL_TIM_CLEAR_IT(&htim11, TIM_IT_UPDATE);
        //计数值归零
        __HAL_TIM_SET_COUNTER(&htim11, 0);
        //启动定时器
        HAL_TIM_Base_Start_IT(&htim11);
        //起始位
        *GPIO_ODR = 0;
    } else if (serial_num == 4) {
        while (state);
        //初始化GPIO_ODR寄存器的位带地址
        GPIO_ODR = (volatile uint32_t *)&PBout(7);
        //初始化要发送的数据
        send_byte = data;
        //清除定时器中断标志
        __HAL_TIM_CLEAR_IT(&htim11, TIM_IT_UPDATE);
        //计数值归零
        __HAL_TIM_SET_COUNTER(&htim11, 0);
        //启动定时器
        HAL_TIM_Base_Start_IT(&htim11);
        //起始位
        *GPIO_ODR = 0;
    } else if (serial_num == 5) {
        while (state);
        //初始化GPIO_ODR寄存器的位带地址
        GPIO_ODR = (volatile uint32_t *)&PAout(1);
        //初始化要发送的数据
        send_byte = data;
        //清除定时器中断标志
        __HAL_TIM_CLEAR_IT(&htim11, TIM_IT_UPDATE);
        //计数值归零
        __HAL_TIM_SET_COUNTER(&htim11, 0);
        //启动定时器
        HAL_TIM_Base_Start_IT(&htim11);
        //起始位
        *GPIO_ODR = 0;
    }
}

/**********************************************************************************************************
*函 数 名: TIM1_TRG_COM_TIM11_IRQHandler
*功能说明: 发送波特率定时器中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    switch (state) {
        //第0位
        case 0:
            if (send_byte & 1)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 1;
            break;
        //第1位
        case 1:
            if (send_byte & 2)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 2;
            break;
        //第2位
        case 2:
            if (send_byte & 4)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 3;
            break;
        //第3位
        case 3:
            if (send_byte & 8)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 4;
            break;
        //第4位
        case 4:
            if (send_byte & 0x10)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 5;
            break;
        //第5位
        case 5:
            if (send_byte & 0x20)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 6;
            break;
        //第6位
        case 6:
            if (send_byte & 0x40)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 7;
            break;
        //第7位
        case 7:
            if (send_byte & 0x80)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 8;
            break;
        //停止位
        case 8:
            *GPIO_ODR = 1;
            state = 0;
            //关闭定时器
            HAL_TIM_Base_Stop_IT(&htim11);
            break;
    }
    //清除定时器中断标志
    __HAL_TIM_CLEAR_IT(&htim11, TIM_IT_UPDATE);
}
