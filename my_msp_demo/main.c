#include "sysinit.h"

short temp = 0, gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
float roll = 0, yaw = 0, pitch = 0;
char str[50];

// void MPU_bsp_init();

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    SysInit();    //ʱ��
    LED_Init();   // LED
    KEY_Init(0);  //����
    delay_init(); // systick��ʱ����
    // Systick_Init(10000000);
    // MPU6050_Init();             // mpu6050��ʼ�� ע�⣬mpu6050��ʼ�����ȼ�Ҫ�Ÿ�һ�㣬��Ϊiic��ʱ��Ҫ�󣬷�ֹ��ĳЩ�жϴ��
    uart_0_init(115200);        //��λ��
    uart_1_init(9600);          //����
    uart_3_init(115200);        //������
    OLED_Init();                // OLED
    UltrasonicWave_TimerInit(); //������  TimerA2_1
    Motors_IO_Init();
    Motors_Timer_Init();
    Encoder_Configuration();

    // Motors_Control(MOTOR_FWD,500,MOTOR_FWD,500);
    /*
       ������� Timer32_0
       psc: 1
       arr: 1920000 - 1
       actual_hz: 48000000 / 1920000 = 25hz 25hz-->40ms
    */
    // Tim32_0_Int_Init(1920000 - 1, 1);

    /*  ����ȫ���ж�  */
    Interrupt_enableMaster();
    UltrasonicWave_Measure();
    while (1)
    {
        if (TIMA2_CAP_STA & 0X80) //�ɹ�������һ��������
        {
            printf("dsitance: %.2f\r\n", sr04_distance);
            /*
                !!:
                TIMA2_CAP_STA�������U0ltrasonicWave_Measure()֮����Ϊ�������ܿ죬���ܻ����
                �����˵��ǻ�û�й���
            */
            TIMA2_CAP_STA = 0;   //���״̬������������һ�β���
            UltrasonicWave_Measure();
        }
        // delay_ms(100);
    }
}
