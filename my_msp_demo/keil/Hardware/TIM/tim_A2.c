/*********************************************************************************************************/

/**************************************         TIMA2          *******************************************/

#include "tim_A2.h"

#define CAP_TIMA_SELECTION      TIMER_A2_BASE                         //这里改定时器
#define CAP_REGISTER_SELECTION  TIMER_A_CAPTURECOMPARE_REGISTER_1     //这里改定时器通道
#define CAP_CCR_NUM             1                                     //;
#define CAP_PORT_PIN            GPIO_PORT_P5, GPIO_PIN6               //这里改复用引脚

/*  捕获模式  */
void TimA2_Cap_Init(void)
{
    // 1.复用输入
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(CAP_PORT_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    /* 2.定时器配置参数*/
    Timer_A_ContinuousModeConfig continuousModeConfig = {
        TIMER_A_CLOCKSOURCE_SMCLK,      // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_48, // SMCLK/48 = 1MHz
        TIMER_A_TAIE_INTERRUPT_ENABLE,  // 开启定时器溢出中断（溢出时TAIFG置位，进入TAx_N_IRQHander）
        TIMER_A_DO_CLEAR                // Clear Counter
    };
    // 3.将定时器初始化为连续计数模式
    MAP_Timer_A_configureContinuousMode(CAP_TIMA_SELECTION, &continuousModeConfig);

    // 4.配置捕捉模式结构体 */
    const Timer_A_CaptureModeConfig captureModeConfig_TA2 = {
        CAP_REGISTER_SELECTION,                      //引脚（通道）
        TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE, //上升下降沿捕获
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,           // CCIxA:外部引脚输入  （CCIxB:与内部ACLK连接(手册)
        TIMER_A_CAPTURE_SYNCHRONOUS,                 //同步捕获
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,     //开启CCRN捕获中断
        TIMER_A_OUTPUTMODE_OUTBITVALUE               //输出位值
    };
    // 5.初始化定时器的捕获模式
    MAP_Timer_A_initCapture(CAP_TIMA_SELECTION, &captureModeConfig_TA2);

    // 6.选择连续模式计数开始计数
    MAP_Timer_A_startCounter(CAP_TIMA_SELECTION, TIMER_A_CONTINUOUS_MODE);

    // 7.清除中断标志位
    MAP_Timer_A_clearInterruptFlag(CAP_TIMA_SELECTION);                                   //清除定时器溢出中断标志位
    MAP_Timer_A_clearCaptureCompareInterrupt(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION); //清除 CCR1 更新中断标志位

    // 8.开启定时器端口中断
    MAP_Interrupt_enableInterrupt(INT_TA2_N); //开启定时器A2端口中断
}
// 10.编写TIMA ISR ↓↓↓↓

/*
 *  TIMA2 中断服务函数
 *
 *
 */
void TA2_0_IRQHandler(void)
{
    /* 清除中断标志位 */
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    /* 开始填充用户代码 */

    /* 结束填充用户代码 */
}

// TIMA2_CAP_STA 捕获状态
// [7]:捕获高电平完成状态
// [6]:0表示未捕获到上升沿，1表示捕获过上升沿
// [5:0]:溢出次数
uint8_t TIMA2_CAP_STA = 0;
uint16_t TIMA2_CAP_VAL = 0;
void TA2_N_IRQHandler(void)
{
    
    if ((TIMA2_CAP_STA & 0X80) == 0) //还未成功捕获
    {
        /*  溢出中断和捕获中断共用TAx_N_IRQHander中断   */
        /*  溢出中断和捕获中断可以同时产生，所以两个都用if判断，独立的，但要把溢出判断放在捕获之前，以解决溢出时刚好产生捕获的情况。  */

        /*  如果是溢出中断  */
        if (Timer_A_getEnabledInterruptStatus(CAP_TIMA_SELECTION))
        {
            // printf("int1\r\n");
            MAP_Timer_A_clearInterruptFlag(CAP_TIMA_SELECTION); //清除定时器溢出中断标志位

            /* ★ 软件复位COV ★ */
            /* 这里UP忘记讲了，如果在未清除中断位值时，来了一次中断，COV会置位，需要软件复位，这里没有官方库函数。具体可以参考技术手册(slau356h.pdf) P790 */
            BITBAND_PERI(TIMER_A_CMSIS(CAP_TIMA_SELECTION)->CCTL[CAP_CCR_NUM], TIMER_A_CCTLN_COV_OFS) = 0;

            if (TIMA2_CAP_STA & 0X40) //已经捕获到高电平了 40H = 0x01000000
            {
                if ((TIMA2_CAP_STA & 0X3F) == 0X3F) //高电平太长了
                {
                    TIMA2_CAP_STA |= 0X80;  //强制标记成功捕获完高电平 80H = 0x10000000
                    TIMA2_CAP_VAL = 0XFFFF; //最大计数值
                }
                else
                    TIMA2_CAP_STA++; //溢出次数加1
            }
        }

        /*  如果是来自捕获通道的捕获中断  */
        if (Timer_A_getCaptureCompareEnabledInterruptStatus(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION))
        {
            // printf("int2\r\n");
            MAP_Timer_A_clearCaptureCompareInterrupt(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION); //清除 CCR1 更新中断标志位

            //判断是否捕获到下降沿
            if (TIMA2_CAP_STA & 0X40 && (MAP_Timer_A_getSynchronizedCaptureCompareInput(CAP_TIMA_SELECTION,
                                                                                        CAP_REGISTER_SELECTION,
                                                                                        TIMER_A_READ_CAPTURE_COMPARE_INPUT) == TIMER_A_CAPTURECOMPARE_INPUT_LOW))
            {
                TIMA2_CAP_STA |= 0X80; //标记成功捕获完高电平
                TIMA2_CAP_VAL = Timer_A_getCaptureCompareCount(CAP_TIMA_SELECTION, CAP_REGISTER_SELECTION);

                /*——————————————SR04 距离——————————————*/
                static uint32_t temp;
                temp = (TIMA2_CAP_STA & 0X3F);
                temp *= 65536;                 //溢出时间总和
                temp += TIMA2_CAP_VAL;         //得到总的高电平时间
                sr04_distance = (float)temp * 17.0 / 1000.0;
                // printf("HIGH:%dus\r\n", temp); //打印总的高点平时间
                // printf("dsitance: %.2f\r\n", (float)temp * 17.0 / 1000.0);
                /*——————————————SR04 距离——————————————*/
            }
            else //还未开始,第一次捕获上升沿
            {
                TIMA2_CAP_STA = 0;
                TIMA2_CAP_VAL = 0;
                MAP_Timer_A_clearTimer(CAP_TIMA_SELECTION); //清空定时器 重新从0计数
                TIMA2_CAP_STA |= 0X40;                      //标记捕获到了上升沿
            }
        }
    }
}
