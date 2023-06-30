#include "stepper_init.h"

TIM_HandleTypeDef TIM_TimeBaseStructure;

// 配置TIM复用输出PWM时用到的I/O
static void Stepper_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	// 开启Motor相关的GPIO外设时钟
	MOTOR_DIR_GPIO_CLK_ENABLE();
	MOTOR_PUL_GPIO_CLK_ENABLE();
	MOTOR_EN_GPIO_CLK_ENABLE();

	// 选择要控制的GPIO引脚															   
	GPIO_InitStruct.Pin = MOTOR_DIR_PIN;	
	// 设置引脚的输出类型为推挽输出
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	// 设置引脚速率为高速   
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	// Motor 方向引脚 初始化
	HAL_GPIO_Init(MOTOR_DIR_GPIO_PORT, &GPIO_InitStruct);	

	// Motor 使能引脚 初始化
	GPIO_InitStruct.Pin = MOTOR_EN_PIN;	
	HAL_GPIO_Init(MOTOR_EN_GPIO_PORT, &GPIO_InitStruct);	


	// 定时器通道1功能引脚IO初始化 
	// 设置输出类型
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	// 设置引脚速率 
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	// 设置复用
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	// 选择要控制的GPIO引脚
	GPIO_InitStruct.Pin = MOTOR_PUL_PIN;
	// Motor 脉冲引脚 初始化
	HAL_GPIO_Init(MOTOR_PUL_PORT, &GPIO_InitStruct);			
}

// 中断优先级配置
static void TIMx_NVIC_Configuration(void)
{
	HAL_NVIC_SetPriority(MOTOR_PUL_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR_PUL_IRQn);
}

void TIM_PWMOUTPUT_Config(void)
{
	TIM_OC_InitTypeDef  TIM_OCInitStructure;  	
	// 使能定时器
	MOTOR_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure.Instance = MOTOR_PUL_TIM;    

	TIM_TimeBaseStructure.Init.Period = TIM_PERIOD; 

	TIM_TimeBaseStructure.Init.Prescaler = TIM_PRESCALER;                

	// 计数方式
	TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;            
	// 采样时钟分频	
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   
	TIM_TimeBaseStructure.Init.RepetitionCounter = 0 ;  		
	// 初始化定时器
	HAL_TIM_OC_Init(&TIM_TimeBaseStructure);

	// PWM模式配置--这里配置为输出比较模式*
	TIM_OCInitStructure.OCMode = TIM_OCMODE_TOGGLE; 
	// 比较输出的计数值
	TIM_OCInitStructure.Pulse = 0;                   
	// 当定时器计数值小于CCR1_Val时为高电平
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;          
	// 设置互补通道输出的极性
	TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_LOW; 
	// 快速模式设置
	TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;   
	// 空闲电平
	TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;  
	// 互补通道设置
	TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET; 
	HAL_TIM_OC_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, MOTOR_PUL_CHANNEL_x);
}


// 引脚初始化
void stepper_Init()
{
	// 电机IO配置
	Stepper_GPIO_Config();
	// 定时器PWM输出配置
	TIM_PWMOUTPUT_Config();
	// 中断配置
	TIMx_NVIC_Configuration();
}
