/*********************************************************************************************************/

/**************************************         TIMA2          *******************************************/

#include "tim_A2.h"

#define CAP_TIMA_SELECTION      TIMER_A2_BASE                         //����Ķ�ʱ��
#define CAP_REGISTER_SELECTION  TIMER_A_CAPTURECOMPARE_REGISTER_1     //����Ķ�ʱ��ͨ��
#define CAP_CCR_NUM             1                                     //;
#define CAP_PORT_PIN            GPIO_PORT_P5, GPIO_PIN6               //����ĸ�������

/*  ����ģʽ  */
void TimA2_Cap_Init(void)
{
    // 1.��������
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(CAP_PORT_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    /* 2.��ʱ�����ò���*/
    Timer_A_ContinuousModeConfig continuousModeConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,      // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_48, // SMCLK/48 = 1MHz
        TIMER_A_TAIE_INTERRUPT_ENABLE,  // ������ʱ������жϣ����ʱTAIFG��λ������TAx_N_IRQHander��
        TIMER_A_DO_CLEAR                // Clear Counter
    };
    // 3.����ʱ����ʼ��Ϊ��������ģʽ
    MAP_Timer_A_configureContinuousMode(CAP_TIMA_SELECTION, &continuousModeConfig);

    // 4.���ò�׽ģʽ�ṹ�� */
    const Timer_A_CaptureModeConfig captureModeConfig_TA2 = {
        CAP_REGISTER_SELECTION,                      //���ţ�ͨ����
        TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE, //�����½��ز���
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,           // CCIxA:�ⲿ��������  ��CCIxB:���ڲ�ACLK����(�ֲ�)
        TIMER_A_CAPTURE_SYNCHRONOUS,                 //ͬ������
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,     //����CCRN�����ж�
        TIMER_A_OUTPUTMODE_OUTBITVALUE               //���λֵ
    };
    // 5.��ʼ����ʱ���Ĳ���ģʽ
    MAP_Timer_A_initCapture(CAP_TIMA_SELECTION, &captureModeConfig_TA2);

    // 6.ѡ������ģʽ������ʼ����
    MAP_Timer_A_startCounter(CAP_TIMA_SELECTION, TIMER_A_CONTINUOUS_MODE);

    // 7.����жϱ�־λ
    MAP_Timer_A_clearInterruptFlag(CAP_TIMA_SELECTION);                                   //�����ʱ������жϱ�־λ
    MAP_Timer_A_clearCaptureCompareInterrupt(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION); //��� CCR1 �����жϱ�־λ

    // 8.������ʱ���˿��ж�
    MAP_Interrupt_enableInterrupt(INT_TA2_N); //������ʱ��A2�˿��ж�
}
// 10.��дTIMA ISR ��������

/*
 *  TIMA2 �жϷ�����
 *
 *
 */
void TA2_0_IRQHandler(void)
{
    /* ����жϱ�־λ */
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /* ��ʼ����û����� */

    /* ��������û����� */
}

// TIMA2_CAP_STA ����״̬
// [7]:����ߵ�ƽ���״̬
// [6]:0��ʾδ���������أ�1��ʾ�����������
// [5:0]:�������
uint8_t TIMA2_CAP_STA = 0;
uint16_t TIMA2_CAP_VAL = 0;
void TA2_N_IRQHandler(void)
{
    
    if ((TIMA2_CAP_STA & 0X80) == 0) //��δ�ɹ�����
    {
        /*  ����жϺͲ����жϹ���TAx_N_IRQHander�ж�   */
        /*  ����жϺͲ����жϿ���ͬʱ������������������if�жϣ������ģ���Ҫ������жϷ��ڲ���֮ǰ���Խ�����ʱ�պò�������������  */

        /*  ���������ж�  */
        if (Timer_A_getEnabledInterruptStatus(CAP_TIMA_SELECTION))
        {
            // printf("int1\r\n");
            MAP_Timer_A_clearInterruptFlag(CAP_TIMA_SELECTION); //�����ʱ������жϱ�־λ

            /* �� �����λCOV �� */
            /* ����UP���ǽ��ˣ������δ����ж�λֵʱ������һ���жϣ�COV����λ����Ҫ�����λ������û�йٷ��⺯����������Բο������ֲ�(slau356h.pdf) P790 */
            BITBAND_PERI(TIMER_A_CMSIS(CAP_TIMA_SELECTION)->CCTL[CAP_CCR_NUM], TIMER_A_CCTLN_COV_OFS) = 0;

            if (TIMA2_CAP_STA & 0X40) //�Ѿ����񵽸ߵ�ƽ�� 40H = 0x01000000
            {
                if ((TIMA2_CAP_STA & 0X3F) == 0X3F) //�ߵ�ƽ̫����
                {
                    TIMA2_CAP_STA |= 0X80;  //ǿ�Ʊ�ǳɹ�������ߵ�ƽ 80H = 0x10000000
                    TIMA2_CAP_VAL = 0XFFFF; //������ֵ
                }
                else
                    TIMA2_CAP_STA++; //���������1
            }
        }

        /*  ��������Բ���ͨ���Ĳ����ж�  */
        if (Timer_A_getCaptureCompareEnabledInterruptStatus(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION))
        {
            // printf("int2\r\n");
            MAP_Timer_A_clearCaptureCompareInterrupt(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION); //��� CCR1 �����жϱ�־λ

            //�ж��Ƿ񲶻��½���
            if (TIMA2_CAP_STA & 0X40 && (MAP_Timer_A_getSynchronizedCaptureCompareInput(CAP_TIMA_SELECTION,
                                                                                        CAP_REGISTER_SELECTION,
                                                                                        TIMER_A_READ_CAPTURE_COMPARE_INPUT) == TIMER_A_CAPTURECOMPARE_INPUT_LOW))
            {
                TIMA2_CAP_STA |= 0X80; //��ǳɹ�������ߵ�ƽ
                TIMA2_CAP_VAL = Timer_A_getCaptureCompareCount(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION);

                /*����������������������������SR04 ���롪��������������������������*/
                static uint32_t temp;
                temp = (TIMA2_CAP_STA & 0X3F);
                temp *= 65536;                 //���ʱ���ܺ�
                temp += TIMA2_CAP_VAL;         //�õ��ܵĸߵ�ƽʱ��
                sr04_distance = (float)temp * 17.0 / 1000.0;
                // printf("HIGH:%dus\r\n", temp); //��ӡ�ܵĸߵ�ƽʱ��
                // printf("dsitance: %.2f\r\n", (float)temp * 17.0 / 1000.0);
                /*����������������������������SR04 ���롪��������������������������*/
            }
            else //��δ��ʼ,��һ�β���������
            {
                TIMA2_CAP_STA = 0;
                TIMA2_CAP_VAL = 0;
                MAP_Timer_A_clearTimer(CAP_TIMA_SELECTION); //��ն�ʱ�� ���´�0����
                TIMA2_CAP_STA |= 0X40;                      //��ǲ�����������
            }
        }
    }
}
