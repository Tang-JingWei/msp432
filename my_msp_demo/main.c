#include "sysinit.h"

short temp = 0, gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
float roll = 0, yaw = 0, pitch = 0;
char str[50];

// void MPU_bsp_init();

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    SysInit();    //时钟
    LED_Init();   // LED
    KEY_Init(0);  //按键
    delay_init(); // systick延时设置
    // Systick_Init(10000000);
    // MPU6050_Init();             // mpu6050初始化 注意，mpu6050初始化优先级要放高一点，因为iic有时序要求，防止被某些中断打断
    uart_0_init(115200);        //上位机
    uart_1_init(9600);          //蓝牙
    uart_3_init(115200);        //串口屏
    OLED_Init();                // OLED
    UltrasonicWave_TimerInit(); //超声波  TimerA2_1
    Motors_IO_Init();
    Motors_Timer_Init();
    Encoder_Configuration();

    // Motors_Control(MOTOR_FWD,500,MOTOR_FWD,500);
    /*
       电机任务 Timer32_0
       psc: 1
       arr: 1920000 - 1
       actual_hz: 48000000 / 1920000 = 25hz 25hz-->40ms
    */
    // Tim32_0_Int_Init(1920000 - 1, 1);

    /*  开启全局中断  */
    Interrupt_enableMaster();
    UltrasonicWave_Measure();
    while (1)
    {
        if (TIMA2_CAP_STA & 0X80) //成功捕获到了一次上升沿
        {
            printf("dsitance: %.2f\r\n", sr04_distance);
            /*
                !!:
                TIMA2_CAP_STA归零放在U0ltrasonicWave_Measure()之后，因为超声波很快，可能会出现
                捕获到了但是还没有归零
            */
            TIMA2_CAP_STA = 0;   //清除状态变量，开启下一次捕获
            UltrasonicWave_Measure();
        }
        // delay_ms(100);
    }
}
