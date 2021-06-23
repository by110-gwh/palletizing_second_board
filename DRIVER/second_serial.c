#include "second_serial.h"
#include "stm32f4xx_hal.h"
#include "main_serial.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//����2���
UART_HandleTypeDef huart2;

/**********************************************************************************************************
*�� �� ��: second_serial_init
*����˵��: 2�Ŵ��ڳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void second_serial_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //���ô�������
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //����2�Ŵ���
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
    
    //ʹ�ܽ����ж�
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

/**********************************************************************************************************
*�� �� ��: second_serial_send
*����˵��: 2�Ŵ��ڷ�������
*��    ��: Ҫ���͵�����
*�� �� ֵ: ��
**********************************************************************************************************/
void second_serial_send(uint8_t data)
{
    //__HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
    USART2->DR = data;
}

/**********************************************************************************************************
*�� �� ��: USART2_IRQHandler
*����˵��: 2�Ŵ����жϺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void USART2_IRQHandler(void)
{
	uint8_t Res;
	//�����ж�
	if(__HAL_UART_GET_FLAG(&huart2, USART_SR_RXNE)) {
        //��ȡ��������
		Res = huart2.Instance->DR;
		
        //�����ݴ���������
        circ_buf_push(&main_serial_cicr_buf, 2);
        circ_buf_push(&main_serial_cicr_buf, Res);
        //ʹ�������ڷ����ж�
        main_serial_send();
	}
}
