#include "soft_serial_tim3.h"
#include "stm32f4xx_hal.h"
#include "main_serial.h"

//定时器频率
#define TIMER_FREQ 84000000
//串口波特率
#define BAUD_RATE 115200

//定时器句柄
TIM_HandleTypeDef htim3;

//定时器更新计数
static uint8_t timer_cnt;

//上次电平跳变时间戳
static uint16_t last_timestamp_3;
//本次电平跳变时间戳
static uint16_t timestamp_3;
//已经收到的位
static uint8_t rec_bit_3;
//记录的定时器更新计数
static uint8_t timer_note_3;
//接受数据
static uint8_t rec_data_3;
//下一个是上升沿
static uint8_t is_raise;

/**********************************************************************************************************
*函 数 名: soft_serial_tim3_init
*功能说明: 软件串口初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void soft_serial_tim3_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};
    
    //配置PA0为软件串口接受端口
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    //配置定时器
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = TIMER_FREQ / BAUD_RATE / 10;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 400 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim3);
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
    
    HAL_TIM_IC_Init(&htim3);
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
    
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);
    
    //启动定时器
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim3);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    
    //使能定时器中断
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    
}

/**********************************************************************************************************
*函 数 名: soft_serial_cb_3
*功能说明: 3号软件串口接收回调
*形    参: 接收到的字节
*返 回 值: 无
**********************************************************************************************************/
void soft_serial_cb_3(uint8_t data)
{
    //把数据传给主串口
    circ_buf_push_two(&main_serial_cicr_buf, 3, data);
    //使能主串口发送中断
    main_serial_send();
}

void TIM3_IRQHandler(void)
{
    //上升沿
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_IT_CC1)) {
        if (is_raise == 1) {
            //串口时钟间隔
            uint8_t clock;
            
            //获取当前定时器计数
            timestamp_3 = __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_1);
            //计算相邻两次中断的串口位数
            if (timestamp_3 > last_timestamp_3)
                clock = (timestamp_3 - last_timestamp_3 + 5) / 10;
            else
                clock = (timestamp_3 + 400 - 1 - last_timestamp_3 + 5) / 10;
            //更新上次电平跳变时间戳
            last_timestamp_3 = timestamp_3;
            
            //更新已接收位个数
            rec_bit_3 += clock;
            //记录定时器更新计数，便于定时器识别是否超时
            timer_note_3 = timer_cnt;
            //起始位+8个数据位接受完毕
            if (rec_bit_3 == 9) {
                //复位已接收位个数
                rec_bit_3 = 0;
                //接收完毕回调
                soft_serial_cb_3(rec_data_3);
                //接收数据清零
                rec_data_3 = 0;
            }
            is_raise = 0;
        }
        //清除定时器中断标志
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
    }
    //下降沿
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_IT_CC2)) {
        if (is_raise == 0) {
            //串口时钟间隔
            uint8_t clock;
            
            //获取当前定时器计数
            timestamp_3 = __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_2);
            //计算相邻两次中断的串口位数
            if (timestamp_3 > last_timestamp_3)
                clock = (timestamp_3 - last_timestamp_3 + 5) / 10;
            else
                clock = (timestamp_3 + 400 - 1 - last_timestamp_3 + 5) / 10;
            //更新上次电平跳变时间戳
            last_timestamp_3 = timestamp_3;
            //由于上一次的最后几个数据位为1，无法产新的终端，只能通过下一次数据的起始位或者定时器
            //更新终端来完成上一次数据的接收。
            //上一次数据需要完成接收
            if (rec_bit_3 && rec_bit_3 + clock > 9) {
                //上一次数据最后几个数据位置1
                rec_data_3 |= ((1 << (9 - rec_bit_3)) - 1) << (rec_bit_3 - 1);
                //复位已接收位个数
                rec_bit_3 = 0;
                //接收完毕回调
                soft_serial_cb_3(rec_data_3);
                //接收数据清零
                rec_data_3 = 0;
            } else if (rec_bit_3) {
                //根据高电平的时间和已接收位个数将数据的相应位置1
                rec_data_3 |= ((1 << clock) - 1) << (rec_bit_3 - 1);
                //更新已接收位个数
                rec_bit_3 += clock;
            }
            is_raise = 1;
        }
        //清除定时器中断标志
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
    }
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_IT_UPDATE)) {
        //是否有未接受完成的数据超时
        if (timer_note_3 < timer_cnt && rec_bit_3) {
            //上一次数据最后几个数据位置1
            rec_data_3 |= ((1 << (9 - rec_bit_3)) - 1) << (rec_bit_3 - 1);
            //复位已接收位个数
            rec_bit_3 = 0;
            //接收完毕回调
            soft_serial_cb_3(rec_data_3);
            //接收数据清零
            rec_data_3 = 0;
        }
        //更新定时器更新计数
        timer_cnt++;
        //清除定时器中断标志
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    }
}
