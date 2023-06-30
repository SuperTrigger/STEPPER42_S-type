#include "stepper_init.h"

TIM_HandleTypeDef TIM_TimeBaseStructure;

// ����TIM�������PWMʱ�õ���I/O
static void Stepper_GPIO_Config(void) 
{
	GPIO_InitTypeDef GPIO_InitStruct;
	// ����Motor��ص�GPIO����ʱ��
	MOTOR_DIR_GPIO_CLK_ENABLE();
	MOTOR_PUL_GPIO_CLK_ENABLE();
	MOTOR_EN_GPIO_CLK_ENABLE();

	// ѡ��Ҫ���Ƶ�GPIO����															   
	GPIO_InitStruct.Pin = MOTOR_DIR_PIN;	
	// �������ŵ��������Ϊ�������
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	// ������������Ϊ����   
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	// Motor �������� ��ʼ��
	HAL_GPIO_Init(MOTOR_DIR_GPIO_PORT, &GPIO_InitStruct);	

	// Motor ʹ������ ��ʼ��
	GPIO_InitStruct.Pin = MOTOR_EN_PIN;	
	HAL_GPIO_Init(MOTOR_EN_GPIO_PORT, &GPIO_InitStruct);	


	// ��ʱ��ͨ��1��������IO��ʼ�� 
	// �����������
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	// ������������ 
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	// ���ø���
	GPIO_InitStruct.Pull =GPIO_PULLUP;
	// ѡ��Ҫ���Ƶ�GPIO����
	GPIO_InitStruct.Pin = MOTOR_PUL_PIN;
	// Motor �������� ��ʼ��
	HAL_GPIO_Init(MOTOR_PUL_PORT, &GPIO_InitStruct);			
}

// �ж����ȼ�����
static void TIMx_NVIC_Configuration(void)
{
	HAL_NVIC_SetPriority(MOTOR_PUL_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MOTOR_PUL_IRQn);
}

void TIM_PWMOUTPUT_Config(void)
{
	TIM_OC_InitTypeDef  TIM_OCInitStructure;  	
	// ʹ�ܶ�ʱ��
	MOTOR_PUL_CLK_ENABLE();

	TIM_TimeBaseStructure.Instance = MOTOR_PUL_TIM;    

	TIM_TimeBaseStructure.Init.Period = TIM_PERIOD; 

	TIM_TimeBaseStructure.Init.Prescaler = TIM_PRESCALER;                

	// ������ʽ
	TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;            
	// ����ʱ�ӷ�Ƶ	
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   
	TIM_TimeBaseStructure.Init.RepetitionCounter = 0 ;  		
	// ��ʼ����ʱ��
	HAL_TIM_OC_Init(&TIM_TimeBaseStructure);

	// PWMģʽ����--��������Ϊ����Ƚ�ģʽ*
	TIM_OCInitStructure.OCMode = TIM_OCMODE_TOGGLE; 
	// �Ƚ�����ļ���ֵ
	TIM_OCInitStructure.Pulse = 0;                   
	// ����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
	TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;          
	// ���û���ͨ������ļ���
	TIM_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_LOW; 
	// ����ģʽ����
	TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;   
	// ���е�ƽ
	TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;  
	// ����ͨ������
	TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET; 
	HAL_TIM_OC_ConfigChannel(&TIM_TimeBaseStructure, &TIM_OCInitStructure, MOTOR_PUL_CHANNEL_x);
}


// ���ų�ʼ��
void stepper_Init()
{
	// ���IO����
	Stepper_GPIO_Config();
	// ��ʱ��PWM�������
	TIM_PWMOUTPUT_Config();
	// �ж�����
	TIMx_NVIC_Configuration();
}
