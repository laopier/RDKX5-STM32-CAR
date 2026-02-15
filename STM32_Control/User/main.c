#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*
 * 项目：STM32 视觉循迹小车控制
 * 作者：[廖宏商、蔡锐潜]
 * 硬件连接：
 * - 舵机：PA0, PA1 (TIM2 PWM)
 * - 电机A：PB5/PB12 (方向), PA6 (PWM)
 * - 电机B：PB7/PB6 (方向), PA7 (PWM)
 * - 串口：PA9 (TX), PA10 (RX)
 */

// --- 宏定义，方便以后改引脚 ---
#define MOTORA_DIR1_PIN     GPIO_Pin_5   
#define MOTORA_DIR2_PIN     GPIO_Pin_12  
#define MOTORA_GPIO_PORT    GPIOB

#define MOTORB_DIR1_PIN     GPIO_Pin_7   
#define MOTORB_DIR2_PIN     GPIO_Pin_6   
#define MOTORB_GPIO_PORT    GPIOB

// --- 全局变量 ---
#define RX_BUFFER_SIZE 64
char RxBuffer[RX_BUFFER_SIZE];
volatile uint8_t RxIndex = 0;
volatile uint8_t RxFlag = 0; // 收到完整数据包标志位

// ================= 硬件初始化部分 =================

void RCC_Configuration(void)
{
    // 开启所有用到的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
}

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 舵机PWM输出 (PA0, PA1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 2. 电机PWM输出 (PA6, PA7)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 电机方向控制 (PB5, PB12, PB6, PB7)
    GPIO_InitStructure.GPIO_Pin = MOTORA_DIR1_PIN | MOTORA_DIR2_PIN | MOTORB_DIR1_PIN | MOTORB_DIR2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 4. 串口 TX (PA9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 5. 串口 RX (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 配置 TIM2 控制舵机 (50Hz)
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    // 舵机需要 20ms 周期 (50Hz)
    // 系统频率 72MHz，分频 72 -> 1MHz (1us 计数一次)
    // 计数 20000 次 -> 20ms
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure); 
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); 

    TIM_Cmd(TIM2, ENABLE);
}

// 配置 TIM3 控制直流电机
void TIM3_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    
   
    TIM_TimeBaseStructure.TIM_Period = 7200 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure); 
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); 

    TIM_Cmd(TIM3, ENABLE);
}

void USART_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;


    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

// ================= 控制函数 =================

// 设置舵机角度 (0-180度)
void Set_Servo(uint8_t id, int angle)
{
   
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // 映射关系：0度->0.5ms(500), 180度->2.5ms(2500)
    uint16_t pwm_val = 500 + (angle * 2000 / 180);

    if (id == 1) TIM_SetCompare1(TIM2, pwm_val);
    else if (id == 2) TIM_SetCompare2(TIM2, pwm_val);
}

// 设置电机速度 (-100 到 100)
void Set_Motor(uint8_t id, int speed)
{
    // 限幅
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    // 计算PWM值 (最大7200)
    uint16_t pwm_val = abs(speed) * 72;

    GPIO_TypeDef* port = (id == 1) ? MOTORA_GPIO_PORT : MOTORB_GPIO_PORT;
    uint16_t pin1 = (id == 1) ? MOTORA_DIR1_PIN : MOTORB_DIR1_PIN;
    uint16_t pin2 = (id == 1) ? MOTORA_DIR2_PIN : MOTORB_DIR2_PIN;

    // 设置方向
    if (speed > 0) {
        GPIO_SetBits(port, pin1);
        GPIO_ResetBits(port, pin2);
    } else if (speed < 0) {
        GPIO_ResetBits(port, pin1);
        GPIO_SetBits(port, pin2);
    } else {
        GPIO_ResetBits(port, pin1 | pin2); // 刹车
    }

    // 设置PWM占空比
    if (id == 1) TIM_SetCompare1(TIM3, pwm_val);
    else TIM_SetCompare2(TIM3, pwm_val);
}

// ================= 中断服务函数 =================

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(USART1);

        // 简单的协议：以换行符结尾
        if (data == '\n' || data == '\r')
        {
            if (RxIndex > 0)
            {
                RxBuffer[RxIndex] = '\0'; // 添加字符串结束符
                RxFlag = 1;               // 告诉主函数数据好了
                RxIndex = 0;
            }
        }
        else
        {
            if (RxIndex < RX_BUFFER_SIZE - 1)
            {
                RxBuffer[RxIndex++] = data;
            }
            else
            {
                RxIndex = 0; // 防止缓冲区溢出（之前这里卡死过）
            }
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

// ================= 主函数 =================

int main(void)
{
    // 初始化
    RCC_Configuration();
    GPIO_Configuration();
    TIM2_Configuration(); 
    TIM3_Configuration(); 
    USART_Configuration();

    // 上电默认停止
    Set_Motor(1, 0);
    Set_Motor(2, 0);
    Set_Servo(1, 0);
    Set_Servo(2, 45);

    int id = 0, val = 0;
    char cmd_type;

    while (1)
    {
        // 处理串口命令
        // 格式例如: "M1:50" (电机1速度50)
        if (RxFlag == 1)
        {
            cmd_type = RxBuffer[0];
            
            if (sscanf(RxBuffer + 1, "%d:%d", &id, &val) == 2)
            {
                if (cmd_type == 'M' || cmd_type == 'm')
                {
                    Set_Motor(id, val);
                }
                else if (cmd_type == 'S' || cmd_type == 's')
                {
                    Set_Servo(id, val);
                }
            }
            RxFlag = 0; // 清除标志位
        }
        
       
    }
}