#include "soft_serial_tim5.h"
#include "stm32f4xx_hal.h"
#include "main_serial.h"

//定时器频率
#define TIMER_FREQ 84000000
//串口波特率
#define BAUD_RATE 115200

//定时器句柄
TIM_HandleTypeDef htim5;

//定时器更新计数
//static uint8_t timer_cnt;
static uint8_t timer_cnt;

//上次电平跳变时间戳
static uint16_t last_timestamp_5;
//本次电平跳变时间戳
static uint16_t timestamp_5;
//已经收到的位
static uint8_t rec_bit_5;
//记录的定时器更新计数
static uint8_t timer_note_5;
//接受数据
static uint8_t rec_data_5;
//下一个是上升沿
static uint8_t is_raise;

/**********************************************************************************************************
*函 数 名: soft_serial_tim4_init
*功能说明: 软件串口初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void soft_serial_tim5_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};
    
    //配置PA0为软件串口接受端口
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    //配置定时器
    __HAL_RCC_TIM5_CLK_ENABLE();
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = TIMER_FREQ / BAUD_RATE / 10;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 400 - 1;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim5);
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);
    
    HAL_TIM_IC_Init(&htim5);
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);
    
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1);
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2);
    
    //启动定时器
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim5);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC2);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    
    //使能定时器中断
    HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    
    //定时器时序复位
    HAL_Delay(100);
    is_raise = 1;
    HAL_Delay(100);
    is_raise = 0;
}

/**********************************************************************************************************
*函 数 名: soft_serial_cb_5
*功能说明: 5号软件串口接收回调
*形    参: 接收到的字节
*返 回 值: 无
**********************************************************************************************************/
void soft_serial_cb_5(uint8_t data)
{
    //把数据传给主串口
    circ_buf_push(&main_serial_cicr_buf, 5);
    circ_buf_push(&main_serial_cicr_buf, data);
    //使能主串口发送中断
    main_serial_send();
}

void TIM5_IRQHandler(void)
{
    //上升沿
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_CC1) && is_raise == 1) {
        //串口时钟间隔
        uint8_t clock;
        
        //获取当前定时器计数
        timestamp_5 = __HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_1);
        //计算相邻两次中断的串口位数
        if (timestamp_5 > last_timestamp_5)
            clock = (timestamp_5 - last_timestamp_5 + 5) / 10;
        else
            clock = (timestamp_5 + 400 - 1 - last_timestamp_5 + 5) / 10;
        //更新上次电平跳变时间戳
        last_timestamp_5 = timestamp_5;
        
        //更新已接收位个数
        rec_bit_5 += clock;
        //记录定时器更新计数，便于定时器识别是否超时
        if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_UPDATE) && timestamp_5 < 200) {
            timer_note_5 = timer_cnt + 1;
        } else {
            timer_note_5 = timer_cnt;
        }
        //起始位+8个数据位接受完毕
        if (rec_bit_5 == 9) {
            //复位已接收位个数
            rec_bit_5 = 0;
            //接收完毕回调
            soft_serial_cb_5(rec_data_5);
            //接收数据清零
            rec_data_5 = 0;
        }
        is_raise = 0;
        //清除定时器中断标志
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
    //下降沿
    } else if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_CC2) && is_raise == 0) {
        //串口时钟间隔
        uint8_t clock;
        
        //获取当前定时器计数
        timestamp_5 = __HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_2);
        //计算相邻两次中断的串口位数
        if (timer_note_5 != timer_cnt) {
            clock = (timestamp_5 + 400 - 1 - last_timestamp_5 + 5) / 10;
        } else if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_UPDATE) && timestamp_5 < 200) {
            clock = (timestamp_5 + 400 - 1 - last_timestamp_5 + 5) / 10;
        } else if (timestamp_5 > last_timestamp_5) {
            clock = (timestamp_5 - last_timestamp_5 + 5) / 10;
        } else {
            clock = (timestamp_5 + 400 - 1 - last_timestamp_5 + 5) / 10;
        }
        //更新上次电平跳变时间戳
        last_timestamp_5 = timestamp_5;
        //由于上一次的最后几个数据位为1，无法产新的终端，只能通过下一次数据的起始位或者定时器
        //更新终端来完成上一次数据的接收。
        //上一次数据需要完成接收
        if (rec_bit_5 && rec_bit_5 + clock > 9) {
            //上一次数据最后几个数据位置1
            rec_data_5 |= ((1 << (9 - rec_bit_5)) - 1) << (rec_bit_5 - 1);
            //复位已接收位个数
            rec_bit_5 = 0;
            //接收完毕回调
            soft_serial_cb_5(rec_data_5);
            //接收数据清零
            rec_data_5 = 0;
        } else if (rec_bit_5) {
            //根据高电平的时间和已接收位个数将数据的相应位置1
            rec_data_5 |= ((1 << clock) - 1) << (rec_bit_5 - 1);
            //更新已接收位个数
            rec_bit_5 += clock;
        }
        is_raise = 1;
        //清除定时器中断标志
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC2);
    } else {
        is_raise = 0;
        rec_bit_5 = 0;
        rec_data_5 = 0;
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1 | TIM_IT_CC2);
    }
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_UPDATE)) {
        //是否有未接受完成的数据超时
        if (timer_note_5 - timer_cnt > 0x80U && rec_bit_5) {
            //上一次数据最后几个数据位置1
            rec_data_5 |= ((1 << (9 - rec_bit_5)) - 1) << (rec_bit_5 - 1);
            //复位已接收位个数
            rec_bit_5 = 0;
            //接收完毕回调
            soft_serial_cb_5(rec_data_5);
            //接收数据清零
            rec_data_5 = 0;
        }
        //更新定时器更新计数
        timer_cnt++;
        //清除定时器中断标志
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
    }
}
