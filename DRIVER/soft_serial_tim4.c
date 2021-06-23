#include "soft_serial_tim4.h"
#include "stm32f4xx_hal.h"
#include "main_serial.h"

//��ʱ��Ƶ��
#define TIMER_FREQ 84000000
//���ڲ�����
#define BAUD_RATE 115200

//��ʱ�����
TIM_HandleTypeDef htim4;

//��ʱ�����¼���
static uint8_t timer_cnt;

//�ϴε�ƽ����ʱ���
static uint16_t last_timestamp_4;
//���ε�ƽ����ʱ���
static uint16_t timestamp_4;
//�Ѿ��յ���λ
static uint8_t rec_bit_4;
//��¼�Ķ�ʱ�����¼���
static uint8_t timer_note_4;
//��������
static uint8_t rec_data_4;
//��һ����������
static uint8_t is_raise;

/**********************************************************************************************************
*�� �� ��: soft_serial_tim4_init
*����˵��: �������ڳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void soft_serial_tim4_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};
    
    //����PA0Ϊ�������ڽ��ܶ˿�
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    //���ö�ʱ��
    __HAL_RCC_TIM4_CLK_ENABLE();
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = TIMER_FREQ / BAUD_RATE / 10;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 400 - 1;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim4);
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
    
    HAL_TIM_IC_Init(&htim4);
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
    
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);
    
    //������ʱ��
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim4);
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC1);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC2);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
    
    //ʹ�ܶ�ʱ���ж�
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/**********************************************************************************************************
*�� �� ��: soft_serial_cb_4
*����˵��: 4���������ڽ��ջص�
*��    ��: ���յ����ֽ�
*�� �� ֵ: ��
**********************************************************************************************************/
void soft_serial_cb_4(uint8_t data)
{
    //�����ݴ���������
    circ_buf_push_two(&main_serial_cicr_buf, 4, data);
    //ʹ�������ڷ����ж�
    main_serial_send();
}

void TIM4_IRQHandler(void)
{
    //������
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_IT_CC1)) {
        if (is_raise == 1) {
            //����ʱ�Ӽ��
            uint8_t clock;
            
            //��ȡ��ǰ��ʱ������
            timestamp_4 = __HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_1);
            //�������������жϵĴ���λ��
            if (timestamp_4 > last_timestamp_4)
                clock = (timestamp_4 - last_timestamp_4 + 5) / 10;
            else
                clock = (timestamp_4 + 400 - 1 - last_timestamp_4 + 5) / 10;
            //�����ϴε�ƽ����ʱ���
            last_timestamp_4 = timestamp_4;
            
            //�����ѽ���λ����
            rec_bit_4 += clock;
            //��¼��ʱ�����¼��������ڶ�ʱ��ʶ���Ƿ�ʱ
            timer_note_4 = timer_cnt;
            //��ʼλ+8������λ�������
            if (rec_bit_4 == 9) {
                //��λ�ѽ���λ����
                rec_bit_4 = 0;
                //������ϻص�
                soft_serial_cb_4(rec_data_4);
                //������������
                rec_data_4 = 0;
            }
            is_raise = 0;
        }
        //�����ʱ���жϱ�־
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC1);
    }
    //�½���
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_IT_CC2)) {
        if (is_raise == 0) {
            //����ʱ�Ӽ��
            uint8_t clock;
            
            //��ȡ��ǰ��ʱ������
            timestamp_4 = __HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_2);
            //�������������жϵĴ���λ��
            if (timestamp_4 > last_timestamp_4)
                clock = (timestamp_4 - last_timestamp_4 + 5) / 10;
            else
                clock = (timestamp_4 + 400 - 1 - last_timestamp_4 + 5) / 10;
            //�����ϴε�ƽ����ʱ���
            last_timestamp_4 = timestamp_4;
            //������һ�ε���󼸸�����λΪ1���޷����µ��նˣ�ֻ��ͨ����һ�����ݵ���ʼλ���߶�ʱ��
            //�����ն��������һ�����ݵĽ��ա�
            //��һ��������Ҫ��ɽ���
            if (rec_bit_4 && rec_bit_4 + clock > 9) {
                //��һ��������󼸸�����λ��1
                rec_data_4 |= ((1 << (9 - rec_bit_4)) - 1) << (rec_bit_4 - 1);
                //��λ�ѽ���λ����
                rec_bit_4 = 0;
                //������ϻص�
                soft_serial_cb_4(rec_data_4);
                //������������
                rec_data_4 = 0;
            } else if (rec_bit_4) {
                //���ݸߵ�ƽ��ʱ����ѽ���λ���������ݵ���Ӧλ��1
                rec_data_4 |= ((1 << clock) - 1) << (rec_bit_4 - 1);
                //�����ѽ���λ����
                rec_bit_4 += clock;
            }
            is_raise = 1;
        }
        //�����ʱ���жϱ�־
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC2);
    }
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_IT_UPDATE)) {
        //�Ƿ���δ������ɵ����ݳ�ʱ
        if (timer_note_4 < timer_cnt && rec_bit_4) {
            //��һ��������󼸸�����λ��1
            rec_data_4 |= ((1 << (9 - rec_bit_4)) - 1) << (rec_bit_4 - 1);
            //��λ�ѽ���λ����
            rec_bit_4 = 0;
            //������ϻص�
            soft_serial_cb_4(rec_data_4);
            //������������
            rec_data_4 = 0;
        }
        //���¶�ʱ�����¼���
        timer_cnt++;
        //�����ʱ���жϱ�־
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    }
}