#include "servo.h"
#include "tim.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_tim.h"

void pwm_servo_init(void) {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void position(uint8_t ID ,uint16_t degrees) { 
	uint16_t pulse;
	switch(ID)
	{
		case 1:
		{
			degrees = LIMIT(degrees,5,115);
			pulse = (uint16_t)(900 + degrees * 10); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);
			// TIM_SetCompare1(TIM2, pulse);      
			break;
		}
		case 2:
		{
			degrees = LIMIT(degrees,5,175);
			pulse = (uint16_t)(500 + degrees * 100/9.0); 

			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);
			break;
		}
		case 3:
		{
			degrees = LIMIT(degrees,5,175);
			pulse = (uint16_t)(500 + degrees * 100/9.0); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);
			break;
		}
		
		case 4:
		{
			degrees = LIMIT(degrees,5,115);
			pulse = (uint16_t)(900 + degrees * 10); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);
			break;
		}
		case 5:
		{
			degrees = LIMIT(degrees,5,175);
			pulse = (uint16_t)(500 + degrees * 100/9.0); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse); 
			break;
		}
		case 6:
		{
			degrees = LIMIT(degrees,5,175);
			pulse = (uint16_t)(500 + degrees * 100/9.0); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);  
			break;
		}
		
		case 7:
		{
			degrees = LIMIT(degrees,5,115);
			pulse = (uint16_t)(900 + degrees * 10); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);   
			break;
		}
		case 8:
		{
			degrees = LIMIT(degrees,5,175);
			pulse = (uint16_t)(500 + degrees * 100/9.0); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);    
			break;
		}
		case 9:
		{
			degrees = LIMIT(degrees,5,175);
			pulse = (uint16_t)(500 + degrees * 100/9.0); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);   
			break;
		}
		
		case 10:
		{
			degrees = LIMIT(degrees,5,115);
			pulse = (uint16_t)(900 + degrees * 10); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);
			break;
		}
		case 11:
		{
			degrees = LIMIT(degrees,5,175);
			pulse = (uint16_t)(500 + degrees * 100/9.0); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse);  
			break;
		}
		case 12:
		{
			degrees = LIMIT(degrees,5,175);
			pulse = (uint16_t)(500 + degrees * 100/9.0); 
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse); 
			break;
		}
		default: ;
	}
	
}
 



