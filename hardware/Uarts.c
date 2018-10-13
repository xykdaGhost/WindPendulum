#include "Uarts.h"

unsigned char txBuffer[256];
unsigned char txCnt = 0;
unsigned char cnt = 0;

void NvicConfiguration(int uartsId) {
	
	NVIC_InitTypeDef nvic;
	
	switch(uartsId) {
		case 1:
			nvic.NVIC_IRQChannel = USART1_IRQn;
			nvic.NVIC_IRQChannelSubPriority = 4;
			break;
		case 2:
			nvic.NVIC_IRQChannel = USART2_IRQn;
			nvic.NVIC_IRQChannelSubPriority = 5;
			break;
		case 3:
			nvic.NVIC_IRQChannel = USART3_IRQn;
			nvic.NVIC_IRQChannelSubPriority = 6;
			break;
		default:
			break;
	}
	
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

}
	
void UartsInit(int baudrate, int uartsId) {
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;
	
	switch(uartsId) {
		
		case 3:
			RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB2Periph_GPIOB, ENABLE);
		
			gpio.GPIO_Pin = GPIO_Pin_10;
			gpio.GPIO_Speed = GPIO_Speed_50MHz;
			gpio.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_Init(GPIOB, &gpio);
		
			gpio.GPIO_Pin = GPIO_Pin_11;
			gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOB, &gpio);
		
			usart.USART_BaudRate = baudrate;
			usart.USART_WordLength = USART_WordLength_8b;
			usart.USART_StopBits = USART_StopBits_1;
			usart.USART_Parity = USART_Parity_No;
			usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			
			USART_Init(USART3, &usart);
		
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
			USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
			USART_ClearFlag(USART3, USART_FLAG_TC);
			
			USART_Cmd(USART3, ENABLE);
			NvicConfiguration(uartsId);
		}
}

void UartsPutChar(char sendData, int uartsId) {
	txBuffer[cnt++] = sendData;
	switch(uartsId) {
		case 1: USART_ITConfig(USART1, USART_IT_TXE, ENABLE); break;
		case 2: USART_ITConfig(USART2, USART_IT_TXE, ENABLE); break;
		case 3: USART_ITConfig(USART3, USART_IT_TXE, ENABLE); break;
	}
}

void UartsPutString(char * str, int uartsId) {
	
	switch(uartsId) {
		
		case 1:
			while (*str) {
				if (*str == '\r') {
					UartsPutChar(0x0d, 1);
				} else if (*str=='\n') {
					UartsPutChar(0x0a, 1);
				}	else {
					UartsPutChar(*str, 1);
				}
				str++;
			}	
			
		case 2:	
			while (*str) {
				if (*str == '\r') {
					UartsPutChar(0x0d, 2);
				} else if (*str=='\n') {
					UartsPutChar(0x0a, 2);
				}	else {
					UartsPutChar(*str, 2);
				}
				str++;
			}	
			
		case 3:
			while (*str) {
				if (*str == '\r') {
					UartsPutChar(0x0d, 3);
				} else if (*str=='\n') {
					UartsPutChar(0x0a, 3);
				}	else {
					UartsPutChar(*str, 3);
				}
				str++;
			}
	}				
}
			
unsigned char charTemp[250];		

void SerialEnventListener();

void USART3_IRQHandler(void) {
	if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {   
		USART_SendData(USART3, txBuffer[txCnt++]); 
		if (txCnt == cnt) {
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);// 全部发送完成
		}
    USART_ClearITPendingBit(USART3, USART_IT_TXE); 
  } else if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		SerialEnventListener((unsigned char)USART3->DR);//处理数据
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
	USART_ClearITPendingBit(USART3, USART_IT_ORE);
}

//uart reicer flag
#define B_UART_HEAD  0x80
#define B_RX_OVER    0x40

// USART Receiver buffer
#define RX_BUFFER_SIZE 100

