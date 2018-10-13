#include "GPIO.h"

void GPIOInit(int block, int pin, int mode, int speed) {
	
	GPIO_InitTypeDef gpio;
		
	gpio.GPIO_Pin = pin;
	gpio.GPIO_Mode = mode;
	gpio.GPIO_Speed = speed;
	
	
	switch(block) {
		
		case A:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_Init(GPIOA, &gpio);
			break;
		
		case B:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_Init(GPIOB, &gpio);
			break;
		
		case C:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			GPIO_Init(GPIOC, &gpio);
			break;
		
		case D:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
			GPIO_Init(GPIOD, &gpio);
			break;
		
		case E:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
			GPIO_Init(GPIOE, &gpio);
			break;
		
		case F:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
			GPIO_Init(GPIOF, &gpio);
			break;
		
		case G:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
			GPIO_Init(GPIOG, &gpio);
			break;
		
//		case I:
//			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOI, ENABLE);
//			GPIO_Init(GPIOI, &gpio);
//			break;                               if the GPIO_I is exist
		
	}
}
