#include "soft_serial_send.h"
#include "stm32f4xx_hal.h"
#include "bitband.h"

//��ʱ��Ƶ��
#define TIMER_FREQ 84000000
//���ڲ�����
#define BAUD_RATE 115200

//���Ͳ����ʶ�ʱ�����
TIM_HandleTypeDef htim11;
//Ҫ���͵��ֽ�
uint8_t send_byte;
//GPIO_ODR�Ĵ�����λ����ַ
volatile uint32_t *GPIO_ODR;
//����״̬��
uint8_t state;

/**********************************************************************************************************
*�� �� ��: soft_erial_send_init
*����˵��: ������ڷ��ͳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void soft_erial_send_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //��ʼ��GPIO
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

    //��ʼ�����Ͳ����ʶ�ʱ��
    __HAL_RCC_TIM11_CLK_ENABLE();
    htim11.Instance = TIM11;
    htim11.Init.Prescaler = 0;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = TIMER_FREQ / BAUD_RATE;
    htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim11);
    
    //��ʼ���жϿ�����
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

/**********************************************************************************************************
*�� �� ��: soft_erial_send
*����˵��: ������ڷ���
*��    ��: ���ں� Ҫ���͵�����
*�� �� ֵ: ��
**********************************************************************************************************/
void soft_erial_send(uint8_t serial_num, uint8_t data)
{
    if (serial_num == 3) {
        while (state);
        //��ʼ��GPIO_ODR�Ĵ�����λ����ַ
        GPIO_ODR = (volatile uint32_t *)&PAout(5);
        //��ʼ��Ҫ���͵�����
        send_byte = data;
        //�����ʱ���жϱ�־
        __HAL_TIM_CLEAR_IT(&htim11, TIM_IT_UPDATE);
        //����ֵ����
        __HAL_TIM_SET_COUNTER(&htim11, 0);
        //������ʱ��
        HAL_TIM_Base_Start_IT(&htim11);
        //��ʼλ
        *GPIO_ODR = 0;
    } else if (serial_num == 4) {
        while (state);
        //��ʼ��GPIO_ODR�Ĵ�����λ����ַ
        GPIO_ODR = (volatile uint32_t *)&PBout(7);
        //��ʼ��Ҫ���͵�����
        send_byte = data;
        //�����ʱ���жϱ�־
        __HAL_TIM_CLEAR_IT(&htim11, TIM_IT_UPDATE);
        //����ֵ����
        __HAL_TIM_SET_COUNTER(&htim11, 0);
        //������ʱ��
        HAL_TIM_Base_Start_IT(&htim11);
        //��ʼλ
        *GPIO_ODR = 0;
    } else if (serial_num == 5) {
        while (state);
        //��ʼ��GPIO_ODR�Ĵ�����λ����ַ
        GPIO_ODR = (volatile uint32_t *)&PAout(1);
        //��ʼ��Ҫ���͵�����
        send_byte = data;
        //�����ʱ���жϱ�־
        __HAL_TIM_CLEAR_IT(&htim11, TIM_IT_UPDATE);
        //����ֵ����
        __HAL_TIM_SET_COUNTER(&htim11, 0);
        //������ʱ��
        HAL_TIM_Base_Start_IT(&htim11);
        //��ʼλ
        *GPIO_ODR = 0;
    }
}

/**********************************************************************************************************
*�� �� ��: TIM1_TRG_COM_TIM11_IRQHandler
*����˵��: ���Ͳ����ʶ�ʱ���жϺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    switch (state) {
        //��0λ
        case 0:
            if (send_byte & 1)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 1;
            break;
        //��1λ
        case 1:
            if (send_byte & 2)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 2;
            break;
        //��2λ
        case 2:
            if (send_byte & 4)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 3;
            break;
        //��3λ
        case 3:
            if (send_byte & 8)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 4;
            break;
        //��4λ
        case 4:
            if (send_byte & 0x10)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 5;
            break;
        //��5λ
        case 5:
            if (send_byte & 0x20)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 6;
            break;
        //��6λ
        case 6:
            if (send_byte & 0x40)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 7;
            break;
        //��7λ
        case 7:
            if (send_byte & 0x80)
                *GPIO_ODR = 1;
            else
                *GPIO_ODR = 0;
            state = 8;
            break;
        //ֹͣλ
        case 8:
            *GPIO_ODR = 1;
            state = 0;
            //�رն�ʱ��
            HAL_TIM_Base_Stop_IT(&htim11);
            break;
    }
    //�����ʱ���жϱ�־
    __HAL_TIM_CLEAR_IT(&htim11, TIM_IT_UPDATE);
}
