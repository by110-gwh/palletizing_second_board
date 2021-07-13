#include "soft_serial_tim5.h"
#include "stm32f4xx_hal.h"
#include "main_serial.h"

//��ʱ��Ƶ��
#define TIMER_FREQ 84000000
//���ڲ�����
#define BAUD_RATE 115200

//��ʱ�����
TIM_HandleTypeDef htim5;

//��ʱ�����¼���
//static uint8_t timer_cnt;
static uint8_t timer_cnt;

//�ϴε�ƽ����ʱ���
static uint16_t last_timestamp_5;
//���ε�ƽ����ʱ���
static uint16_t timestamp_5;
//�Ѿ��յ���λ
static uint8_t rec_bit_5;
//��¼�Ķ�ʱ�����¼���
static uint8_t timer_note_5;
//��������
static uint8_t rec_data_5;
//��һ����������
static uint8_t is_raise;

/**********************************************************************************************************
*�� �� ��: soft_serial_tim4_init
*����˵��: ������ڳ�ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void soft_serial_tim5_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};
    
    //����PA0Ϊ������ڽ��ܶ˿�
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    //���ö�ʱ��
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
    
    //������ʱ��
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim5);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC2);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    
    //ʹ�ܶ�ʱ���ж�
    HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    
    //��ʱ��ʱ��λ
    HAL_Delay(100);
    is_raise = 1;
    HAL_Delay(100);
    is_raise = 0;
}

/**********************************************************************************************************
*�� �� ��: soft_serial_cb_5
*����˵��: 5��������ڽ��ջص�
*��    ��: ���յ����ֽ�
*�� �� ֵ: ��
**********************************************************************************************************/
void soft_serial_cb_5(uint8_t data)
{
    //�����ݴ���������
    circ_buf_push(&main_serial_cicr_buf, 5);
    circ_buf_push(&main_serial_cicr_buf, data);
    //ʹ�������ڷ����ж�
    main_serial_send();
}

void TIM5_IRQHandler(void)
{
    //������
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_CC1) && is_raise == 1) {
        //����ʱ�Ӽ��
        uint8_t clock;
        
        //��ȡ��ǰ��ʱ������
        timestamp_5 = __HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_1);
        //�������������жϵĴ���λ��
        if (timestamp_5 > last_timestamp_5)
            clock = (timestamp_5 - last_timestamp_5 + 5) / 10;
        else
            clock = (timestamp_5 + 400 - 1 - last_timestamp_5 + 5) / 10;
        //�����ϴε�ƽ����ʱ���
        last_timestamp_5 = timestamp_5;
        
        //�����ѽ���λ����
        rec_bit_5 += clock;
        //��¼��ʱ�����¼��������ڶ�ʱ��ʶ���Ƿ�ʱ
        if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_UPDATE) && timestamp_5 < 200) {
            timer_note_5 = timer_cnt + 1;
        } else {
            timer_note_5 = timer_cnt;
        }
        //��ʼλ+8������λ�������
        if (rec_bit_5 == 9) {
            //��λ�ѽ���λ����
            rec_bit_5 = 0;
            //������ϻص�
            soft_serial_cb_5(rec_data_5);
            //������������
            rec_data_5 = 0;
        }
        is_raise = 0;
        //�����ʱ���жϱ�־
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
    //�½���
    } else if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_CC2) && is_raise == 0) {
        //����ʱ�Ӽ��
        uint8_t clock;
        
        //��ȡ��ǰ��ʱ������
        timestamp_5 = __HAL_TIM_GetCompare(&htim5, TIM_CHANNEL_2);
        //�������������жϵĴ���λ��
        if (timer_note_5 != timer_cnt) {
            clock = (timestamp_5 + 400 - 1 - last_timestamp_5 + 5) / 10;
        } else if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_UPDATE) && timestamp_5 < 200) {
            clock = (timestamp_5 + 400 - 1 - last_timestamp_5 + 5) / 10;
        } else if (timestamp_5 > last_timestamp_5) {
            clock = (timestamp_5 - last_timestamp_5 + 5) / 10;
        } else {
            clock = (timestamp_5 + 400 - 1 - last_timestamp_5 + 5) / 10;
        }
        //�����ϴε�ƽ����ʱ���
        last_timestamp_5 = timestamp_5;
        //������һ�ε���󼸸�����λΪ1���޷����µ��նˣ�ֻ��ͨ����һ�����ݵ���ʼλ���߶�ʱ��
        //�����ն��������һ�����ݵĽ��ա�
        //��һ��������Ҫ��ɽ���
        if (rec_bit_5 && rec_bit_5 + clock > 9) {
            //��һ��������󼸸�����λ��1
            rec_data_5 |= ((1 << (9 - rec_bit_5)) - 1) << (rec_bit_5 - 1);
            //��λ�ѽ���λ����
            rec_bit_5 = 0;
            //������ϻص�
            soft_serial_cb_5(rec_data_5);
            //������������
            rec_data_5 = 0;
        } else if (rec_bit_5) {
            //���ݸߵ�ƽ��ʱ����ѽ���λ���������ݵ���Ӧλ��1
            rec_data_5 |= ((1 << clock) - 1) << (rec_bit_5 - 1);
            //�����ѽ���λ����
            rec_bit_5 += clock;
        }
        is_raise = 1;
        //�����ʱ���жϱ�־
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC2);
    } else {
        is_raise = 0;
        rec_bit_5 = 0;
        rec_data_5 = 0;
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1 | TIM_IT_CC2);
    }
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_UPDATE)) {
        //�Ƿ���δ������ɵ����ݳ�ʱ
        if (timer_note_5 - timer_cnt > 0x80U && rec_bit_5) {
            //��һ��������󼸸�����λ��1
            rec_data_5 |= ((1 << (9 - rec_bit_5)) - 1) << (rec_bit_5 - 1);
            //��λ�ѽ���λ����
            rec_bit_5 = 0;
            //������ϻص�
            soft_serial_cb_5(rec_data_5);
            //������������
            rec_data_5 = 0;
        }
        //���¶�ʱ�����¼���
        timer_cnt++;
        //�����ʱ���жϱ�־
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
    }
}
