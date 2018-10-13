#include "Encoder.h"

void TIM4_Mode_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
		NVIC_InitTypeDef nvic;	

    //PB6 ch1  A,PB7 ch2 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//??TIM4??  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//??GPIOA??

    GPIO_StructInit(&GPIO_InitStructure);//?GPIO_InitStruct??????????
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//PA6 PA7????  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);                           

		nvic.NVIC_IRQChannel = TIM4_IRQn;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);

    TIM_DeInit(TIM4);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 359*4;  //????????   TIMx_ARR = 359*4
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM3??????
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//?????? T_dts = T_ck_int    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM???? 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);//???????3,???????
    TIM_ICStructInit(&TIM_ICInitStructure);//????????????
    TIM_ICInitStructure.TIM_ICFilter = 6;  //????????? 
    TIM_ICInit(TIM4, &TIM_ICInitStructure);//?TIM_ICInitStructure?????????TIM3

//  TIM_ARRPreloadConfig(TIM4, ENABLE);//?????
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);//??TIM3??????
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//??????
    //Reset counter
    TIM4->CNT = 0;//

    TIM_Cmd(TIM4, ENABLE);   //??TIM4???

}
/*  
void TIM3_Mode_Config(void)
{
    ///TIM3 clock source enable 
    RCC->APB1ENR|=1<<1;       //TIM3????
    // Enable 1GPIOA, clock 
    RCC->APB2ENR|=1<<2;    //??PORTA??

    // Configure PA.06,07 as encoder input 
    GPIOA->CRL&=0XF0FFFFFF;//PA6
    GPIOA->CRL|=0X04000000;//????
    GPIOA->CRL&=0X0FFFFFFF;//PA7
    GPIOA->CRL|=0X40000000;//????

    // Enable the TIM3 Update Interrupt 
    //?????????????????
    TIM3->DIER|=1<<0;   //??????                
    TIM3->DIER|=1<<6;   //??????

    TIM3_NVIC_Config();


    //Timer configuration in Encoder mode 
    TIM3->PSC = 0x0;//????
    TIM3->ARR = 15-1;//?????????? 
    TIM3->CR1 &=~(3<<8);// ??????:???
    TIM3->CR1 &=~(3<<5);// ??????:??????

    TIM3->CCMR1 |= 1<<0; //CC1S='01' IC1FP1???TI1
    TIM3->CCMR1 |= 1<<8; //CC2S='01' IC2FP2???TI2
    TIM3->CCER &= ~(1<<1);  //CC1P='0'  IC1FP1???,IC1FP1=TI1
    TIM3->CCER &= ~(1<<5);  //CC2P='0'  IC2FP2???,IC2FP2=TI2
    TIM3->CCMR1 |= 3<<4; // IC1F='1000' ????1???
    TIM3->SMCR |= 3<<0;  //SMS='011' ????????????????
    TIM3->CNT = 0;
    TIM3->CR1 |= 0x01;    //CEN=1,?????

}*/

void TIM4_Init(void)
{
  TIM4_Mode_Config();
}