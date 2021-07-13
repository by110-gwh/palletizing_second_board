#include "main_serial.h"
#include "stm32f4xx_hal.h"
#include "steering_task.h"
#include "second_serial.h"
#include "soft_serial_send.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

UART_HandleTypeDef huart1;

circ_buf_t main_serial_cicr_buf;
uint8_t main_serial_buf[1024];

/**********************************************************************************************************
*函 数 名: main_serial_init
*功能说明: 主串口初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void main_serial_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //初始化主串口缓冲区
    circ_buf_init(&main_serial_cicr_buf, main_serial_buf, sizeof(main_serial_buf));
    
    //配置串口引脚
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //配置2号串口
    __HAL_RCC_USART1_CLK_ENABLE();
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 11, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    
    //使能接收中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

/**********************************************************************************************************
*函 数 名: main_serial_send
*功能说明: 主串口发送中断使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void main_serial_send(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
}

/**********************************************************************************************************
*函 数 名: USART1_IRQHandler
*功能说明: 主串口中断函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void USART1_IRQHandler(void)
{
    //串口号
    static uint8_t rec_id;
    //舵机控制数据已接收个数
    static uint8_t steering_rec_cnt;
    //舵机控制数据缓冲
    static uint8_t steering_rec_buf[5];
    //接收的数据
	uint8_t Res;
	//接收中断
	if(__HAL_UART_GET_FLAG(&huart1, USART_SR_RXNE)) {
        //读取串口数据
		Res = huart1.Instance->DR;
		if (rec_id) {
            switch (rec_id) {
                //数据发送给舵机控制
                case 1:
                    if (steering_rec_cnt == 0) {
                        if (Res == 0xFF) {
                            steering_rec_buf[0] = Res;
                            steering_rec_cnt = 1;
                        }
                    } else if (steering_rec_cnt == 4) {
                        BaseType_t xHigherPriorityTaskWoken;
                        steering_rec_buf[4] = Res;
                        xHigherPriorityTaskWoken = pdFALSE;
                        xQueueSendFromISR(rec_data_queue, steering_rec_buf, &xHigherPriorityTaskWoken);
                        if(xHigherPriorityTaskWoken) {
                            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                        }
                        steering_rec_cnt = 0;
                    } else {
                        steering_rec_buf[steering_rec_cnt] = Res;
                        steering_rec_cnt++;
                    }
                    break;
                //数据发送给副串口
                case 2:
                    second_serial_send(Res);
                    break;
                //数据发送给软件串口
                default:
                    soft_erial_send(rec_id, Res);
                    break;
            }
            rec_id = 0;
        } else {
            //读取第一个字节用来指定数据目的地
            if (Res > 0 && Res < 6)
                rec_id = Res;
        }
		
	} else if(__HAL_UART_GET_FLAG(&huart1, USART_SR_TXE)) {
        __disable_irq();
		//发送队列中有数据需要发送
		if (circ_buf_count_used(&main_serial_cicr_buf))
			huart1.Instance->DR = circ_buf_pop(&main_serial_cicr_buf);
		else
			//无数据发送就关闭发送中断
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
        __enable_irq();
	}
}
