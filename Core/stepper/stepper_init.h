#ifndef __STEPPER_INIT_H
#define	__STEPPER_INIT_H

#include "main.h"

// 宏定义
//Motor 方向 
#define MOTOR_DIR_PIN                  	GPIO_PIN_0
#define MOTOR_DIR_GPIO_PORT            	GPIOG                 
#define MOTOR_DIR_GPIO_CLK_ENABLE()   	__HAL_RCC_GPIOG_CLK_ENABLE()

//Motor 使能 
#define MOTOR_EN_PIN                  	GPIO_PIN_1
#define MOTOR_EN_GPIO_PORT            	GPIOG               
#define MOTOR_EN_GPIO_CLK_ENABLE()    	__HAL_RCC_GPIOG_CLK_ENABLE()
	
//Motor 脉冲
#define MOTOR_PUL_IRQn                  TIM8_CC_IRQn
#define MOTOR_PUL_IRQHandler            TIM8_CC_IRQHandler

#define MOTOR_PUL_TIM                   TIM8
#define MOTOR_PUL_CLK_ENABLE()  		__HAL_RCC_TIM8_CLK_ENABLE()

#define MOTOR_PUL_PORT       		    GPIOC
#define MOTOR_PUL_PIN             	    GPIO_PIN_6
#define MOTOR_PUL_GPIO_CLK_ENABLE()	    __HAL_RCC_GPIOC_CLK_ENABLE()
#define MOTOR_PUL_CHANNEL_x             TIM_CHANNEL_1

#define MOTOR_TIM_IT_CCx                TIM_IT_CC1
#define MOTOR_TIM_FLAG_CCx              TIM_FLAG_CC1

/****************************************************************/

#define TIM_PRESCALER               (25 - 1) //25--35
// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define TIM_PERIOD                   0xFFFF

/****************************************************************/

#define HIGH    GPIO_PIN_SET	    //高电平
#define LOW     GPIO_PIN_RESET	    //低电平

#define ON      HIGH	            //开
#define OFF     LOW	                //关

#define CW 	    LOW		            //顺时针
#define CCW     HIGH      	        //逆时针

//控制使能引脚
#define MOTOR_EN(x)					HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN,x)
#define MOTOR_PUL(x)				HAL_GPIO_WritePin(MOTOR_PUL_GPIO_PORT,MOTOR_PUL_PIN,x)
#define MOTOR_DIR(x)				HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT,MOTOR_DIR_PIN,x)

#define MOTOR_EN_TOGGLE             HAL_GPIO_TogglePin(MOTOR_EN_GPIO_PORT,MOTOR_EN_PIN)
#define MOTOR_PUL_TOGGLE            HAL_GPIO_TogglePin(MOTOR_PUL_PORT,MOTOR_PUL_PIN)


extern TIM_HandleTypeDef TIM_TimeBaseStructure;

void stepper_Init(void);

#endif
