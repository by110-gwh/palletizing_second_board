#include "steering_pwm.h"
#include "stm32f4xx_hal.h"
#include "bitband.h"
#include "steering_task.h"

//通道IO输出高电平，用户需重写
//如某通道不用，则将后面清空，如#define CH16_H
#define CH1_H PAout(7) = 1
#define CH2_H PBout(0) = 1
#define CH3_H PBout(1) = 1
#define CH4_H PBout(12) = 1
#define CH5_H PBout(13) = 1
#define CH6_H PBout(14) = 1
#define CH7_H PBout(15) = 1
#define CH8_H PAout(8) = 1
#define CH9_H
#define CH10_H
#define CH11_H
#define CH12_H
#define CH13_H
#define CH14_H
#define CH15_H
#define CH16_H

//通道IO输出低电平，用户需重写
//如某通道不用，则将后面清空，如#define CH16_L
#define CH1_L PAout(7) = 0
#define CH2_L PBout(0) = 0
#define CH3_L PBout(1) = 0
#define CH4_L PBout(12) = 0
#define CH5_L PBout(13) = 0
#define CH6_L PBout(14) = 0
#define CH7_L PBout(15) = 0
#define CH8_L PAout(8) = 0
#define CH9_L
#define CH10_L
#define CH11_L
#define CH12_L
#define CH13_L
#define CH14_L
#define CH15_L
#define CH16_L

//各个通道高电平时间，单位us,范围0 - 4500
volatile uint16_t steering_pulse_ch[16];
//定时器2句柄
static TIM_HandleTypeDef htim2;

/**********************************************************************************************************
*函 数 名: steering_gpio_init
*功能说明: 舵机驱动IO初始化，用户需重写此函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void steering_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	//使能GPIO时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	//初始化IO引脚
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOB, 0xF003, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = 0xF003;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
}

/**********************************************************************************************************
*函 数 名: steering_init
*功能说明: 舵机驱动初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void steering_pwm_init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	
	steering_gpio_init();
	
	//使能定时器时钟
	__HAL_RCC_TIM2_CLK_ENABLE();
	
	//定时器2计数器初始化
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 84 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 5000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim2);

	//定时器2选择内部时钟
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
	
	//定时器2选择输出通道初始化
	HAL_TIM_OC_Init(&htim2);
	
	//关闭定时器2级联功能输出
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
	
	//初始化输出比较通道
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
	
	//使能定时器2中断
	HAL_NVIC_SetPriority(TIM2_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
	//计数器使能
	HAL_TIM_Base_Start_IT(&htim2);
	//输出比较通道使能
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
}

/**********************************************************************************************************
*函 数 名: TIM2_IRQHandler
*功能说明: 定时器2中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TIM2_IRQHandler(void)
{
	static uint8_t next_ch;
	//定时器溢出中断
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
		//判断输出哪几号通道
		if (next_ch == 0) {
			//四个通道输出高电平
			if (steering_pulse_ch[4])
				CH5_H;
			if (steering_pulse_ch[5])
				CH6_H;
			if (steering_pulse_ch[6])
				CH7_H;
			if (steering_pulse_ch[7])
				CH8_H;
			//设置比较器
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, steering_pulse_ch[4]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, steering_pulse_ch[5]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, steering_pulse_ch[6]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, steering_pulse_ch[7]);
			next_ch = 4;
		} else if (next_ch == 4) {
			//四个通道输出高电平
			if (steering_pulse_ch[8])
				CH9_H;
			if (steering_pulse_ch[9])
				CH10_H;
			if (steering_pulse_ch[10])
				CH11_H;
			if (steering_pulse_ch[11])
				CH12_H;
			//设置比较器
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, steering_pulse_ch[8]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, steering_pulse_ch[9]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, steering_pulse_ch[10]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, steering_pulse_ch[11]);
			next_ch = 8;
		} else if (next_ch == 8) {
			//四个通道输出高电平
			if (steering_pulse_ch[12])
				CH13_H;
			if (steering_pulse_ch[13])
				CH14_H;
			if (steering_pulse_ch[14])
				CH15_H;
			if (steering_pulse_ch[15])
				CH16_H;
			//设置比较器
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, steering_pulse_ch[12]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, steering_pulse_ch[13]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, steering_pulse_ch[14]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, steering_pulse_ch[15]);
			next_ch = 12;
		} else if (next_ch == 12) {
			//四个通道输出高电平
			if (steering_pulse_ch[0])
				CH1_H;
			if (steering_pulse_ch[1])
				CH2_H;
			if (steering_pulse_ch[2])
				CH3_H;
			if (steering_pulse_ch[3])
				CH4_H;
			//设置比较器
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, steering_pulse_ch[0]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, steering_pulse_ch[1]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, steering_pulse_ch[2]);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, steering_pulse_ch[3]);
			next_ch = 0;
		}
			
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC1);
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC2);
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC3);
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC4);
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_UPDATE);
	//定时器输出通道1比较中断
	} else if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) != RESET) {
		if (next_ch == 0) {
			//相应通道输出低电平
			CH1_L;
		} else if (next_ch == 4) {
			//相应通道输出低电平
			CH5_L;
		} else if (next_ch == 8) {
			//相应通道输出低电平
			CH9_L;
		} else if (next_ch == 12) {
			//相应通道输出低电平
			CH13_L;
		}
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC1);
	//定时器输出通道2比较中断
	} else if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) != RESET) {
		if (next_ch == 0) {
			//相应通道输出低电平
			CH2_L;
		} else if (next_ch == 4) {
			//相应通道输出低电平
			CH6_L;
		} else if (next_ch == 8) {
			//相应通道输出低电平
			CH10_L;
		} else if (next_ch == 12) {
			//相应通道输出低电平
			CH14_L;
		}
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC2);
	//定时器输出通道3比较中断
	} else if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC3) != RESET) {
		if (next_ch == 0) {
			//相应通道输出低电平
			CH3_L;
		} else if (next_ch == 4) {
			//相应通道输出低电平
			CH7_L;
		} else if (next_ch == 8) {
			//相应通道输出低电平
			CH11_L;
		} else if (next_ch == 12) {
			//相应通道输出低电平
			CH15_L;
		}
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC3);
	//定时器输出通道4比较中断
	} else if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4) != RESET) {
		if (next_ch == 0) {
			//相应通道输出低电平
			CH4_L;
		} else if (next_ch == 4) {
			//相应通道输出低电平
			CH8_L;
		} else if (next_ch == 8) {
			//相应通道输出低电平
			CH12_L;
		} else if (next_ch == 12) {
			//相应通道输出低电平
			CH16_L;
		}
		__HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC4);
	}
}
