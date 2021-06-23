#include "second_serial.h"
#include "stm32f4xx_hal.h"
#include "main_serial.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//串口2句柄
UART_HandleTypeDef huart2;

/**********************************************************************************************************
*函 数 名: second_serial_init
*功能说明: 2号串口初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void second_serial_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //配置串口引脚
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //配置2号串口
    __HAL_RCC_USART2_CLK_ENABLE();
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    HAL_NVIC_SetPriority(USART2_IRQn, 11, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    
    //使能接收中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/**********************************************************************************************************
*函 数 名: second_serial_send
*功能说明: 2号串口发送数据
*形    参: 要发送的数据
*返 回 值: 无
**********************************************************************************************************/
void second_serial_send(uint8_t data)
{
    //__HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
    USART2->DR = data;
}

/**********************************************************************************************************
*函 数 名: USART2_IRQHandler
*功能说明: 2号串口中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void USART2_IRQHandler(void)
{
	uint8_t Res;
	//接收中断
	if(__HAL_UART_GET_FLAG(&huart2, USART_SR_RXNE)) {
        //读取串口数据
		Res = huart2.Instance->DR;
		
        //把数据传给主串口
        circ_buf_push(&main_serial_cicr_buf, 2);
        circ_buf_push(&main_serial_cicr_buf, Res);
        //使能主串口发送中断
        main_serial_send();
	}
}
