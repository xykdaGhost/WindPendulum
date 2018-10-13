#ifndef _GPIO_H
#define _GPIO_H

#include "stm32f10x_gpio.h"
#include "public.h"

void GPIOInit(int block, int pin, int mode, int speed);

#define A 12340000
#define B 12340001
#define C 12340002
#define D 12340003
#define E 12340004
#define F 12340005
#define G 12340006
#define H 12340007
#define I 12340008

#define pin1 GPIO_Pin_1
#define pin2 GPIO_Pin_2
#define pin3 GPIO_Pin_3
#define pin4 GPIO_Pin_4
#define pin5 GPIO_Pin_5
#define pin6 GPIO_Pin_6
#define pin7 GPIO_Pin_7
#define pin8 GPIO_Pin_8
#define pin9 GPIO_Pin_9
#define pin10 GPIO_Pin_10
#define pin11 GPIO_Pin_11
#define pin12 GPIO_Pin_12
#define pin13 GPIO_Pin_13
#define pin14 GPIO_Pin_14
#define pin15 GPIO_Pin_15
#define pin0 GPIO_Pin_0

#define ain GPIO_Mode_AIN
#define floatingin GPIO_Mode_IN_FLOATING#define
#define ipd GPIO_Mode_IPD
#define ipu GPIO_Mode_IPU
#define outod GPIO_Mode_Out_OD
#define outpp GPIO_Mode_Out_PP
#define afod GPIO_Mode_AF_OD
#define afpp GPIO_Mode_AF_PP 

#define speed10 GPIO_Speed_10MHz
#define speed2 GPIO_Speed_2MHz
#define speed50 GPIO_Speed_50MHz

#endif
