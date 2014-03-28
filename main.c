#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_adc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4_discovery.h"
#include <math.h>
#include "platform.h"
#include "sensors.h"
volatile char StringLoop[] = "The quick brown fox jumps over the lazy dog\r\n";

/*
typedef struct SumoPlatform {
   uint8_t _state; //Private
   uint8_t _speed; //Private
   unit8_t flags;  //Public
} SumoPlatform;
*/



/**************************************************************************************/

void RCC_Configuration(void) {
  /* --------------------------- System Clocks Configuration -----------------*/
  /* USART3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);



}

/**************************************************************************************/

void GPIO_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
}

/**************************************************************************************/

void USART3_Configuration(void) {
    USART_InitTypeDef USART_InitStructure;

  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 38400 baud
        - Word Length = 8 Bits
        - Two Stop Bit
        - Odd parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 38400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);

  USART_Cmd(USART3, ENABLE);

  //USART_ITConfig(USART3, USART_IT_RXNE | USART_IT_TXE, ENABLE);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/**************************************************************************************/

void NVIC_Configuration(void) {
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  /* Enable the USART3 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**************************************************************************************/

void USART3_IRQHandler(void) {
  static int tx_index = 0;
  static int rx_index = 0;
  char rcv;
  /*
  if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) // Transmit the string in a loop
  {
    USART_SendData(USART3, StringLoop[tx_index++]);

    if (tx_index >= (sizeof(StringLoop) - 1))
      tx_index = 0;
  }
	*/
  if (USART_GetITStatus(USART3, USART_IT_RXNE)) // Received characters modify string
  {

	rcv = USART3->DR;

	 USART_SendData(USART3, rcv);
	/*
    StringLoop[rx_index++] = USART_ReceiveData(USART3);

    if (rx_index >= (sizeof(StringLoop) - 1))
      rx_index = 0;
      */
  }

  if(rcv != '\n') {
	  if(rcv == 'w') {
		  GPIO_ResetBits(GPIOE, GPIO_Pin_11 | GPIO_Pin_14);
		  GPIO_SetBits(GPIOE, GPIO_Pin_12 | GPIO_Pin_13);
	  } else if (rcv == 's') {
		  GPIO_SetBits(GPIOE, GPIO_Pin_11 | GPIO_Pin_14);
		  GPIO_ResetBits(GPIOE, GPIO_Pin_12 | GPIO_Pin_13);
	  } else if (rcv == 'a') {
		  GPIO_ResetBits(GPIOE, GPIO_Pin_14 | GPIO_Pin_13);
		  GPIO_SetBits(GPIOE, GPIO_Pin_11 | GPIO_Pin_12);
	  } else if (rcv == 'd') {
		  GPIO_SetBits(GPIOE, GPIO_Pin_14 | GPIO_Pin_13);
		  GPIO_ResetBits(GPIOE, GPIO_Pin_11 | GPIO_Pin_12);
	  } else {
		  GPIO_SetBits(GPIOE, GPIO_Pin_11 | GPIO_Pin_14 | GPIO_Pin_12 | GPIO_Pin_13);
	  }
  }


}

/**************************************************************************************/

void Delay(__IO uint32_t nCount) {
  while(nCount--){}
}

int main(void) {
  //RCC_Configuration();

  //GPIO_Configuration();

  //NVIC_Configuration();

  //USART3_Configuration();
  //setupMotor();
  //SumoPlatform* MotorCtrl = initMotorCtrl();

  init_platform();
  init_sensors2();
  uint32_t i = 0;
  while(1) {
	  i++;

	  /*
	  set_forward();
	  process_platform();
	  set_backward();
	  process_platform();
	  set_left();
	  process_platform();
	  set_right();
	  process_platform();
	  set_stop();
	  process_platform();
	  */
  }

}

/**************************************************************************************/
