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

void setupMotor(void) {
		GPIO_InitTypeDef  GPIO_InitStruct;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		//GPIO_InitStruct.GPIO_Pin configures the pins that will be used.
	    //In this case we will use the LED's off of the discovery board which are on
	    //PortD pins 12, 13, 14 and 15
	    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;

	    //PIO_InitStruct.GPIO_Mode configures the pin mode the options are as follows
	    // GPIO_Mode_IN (Input Mode)
	    // GPIO_Mode_OUT (Output Mode)
	    // GPIO_Mode_AF (Alternate Function)
	    // GPIO_Mode_AN (Analog Mode)
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;

	    //GPIO_InitStruct.GPIO_Speed configures the clock speed, options are as follows
	    // GPIO_Speed_2MHz
	    // GPIO_Speed_25MHz
	    // GPIO_Speed_50MHz
	    // GPIO_Speed_100MHz
	    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	    //GPIO_InitStruct.GPIO_OType configures the pin type, options are as follows
	    // GPIO_OType_PP (Push/Pull)
	    // GPIO_OType_OD (Open Drain)
	    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

	    //Configures pullup / pulldown resistors on pin, options are as follows
	    // GPIO_PuPd_NOPULL (Disables internal pullup and pulldown resistors)
	    // GPIO_PuPd_UP (Enables internal pullup resistors) (~40K (30-50k))
	    // GPIO_PuPd_DOWN (Enables internal pulldown resistors) (~40K (30-50k))
	    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;

	    //This finally passes all the values to the GPIO_Init function
	    //which takes care of setting the corresponding bits.
	    GPIO_Init(GPIOE, &GPIO_InitStruct);

	    GPIO_ResetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
}

int ConvertedValue = 0; //Converted value readed from ADC


void adc_configure(){
 ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
 GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin

 //Clock configuration
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
 //RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN,ENABLE);//Clock for the ADC port!! Do not forget about this one ;)
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//The ADC1 is connected the APB2 peripheral bus thus we will use its clock source

 //Analog pin configuration
 GPIO_initStructre.GPIO_Pin = GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3;//The channel 10 is connected to PC0
 GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
 GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
 GPIO_Init(GPIOA, &GPIO_initStructre);//Affecting the port with the initialization structure configuration

 //ADC structure configuration
 ADC_DeInit();
 ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
 ADC_init_structure.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit number giving a maximum value of 4096
 ADC_init_structure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
 ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;// conversion is synchronous with TIM1 and CC1 (actually I'm not sure about this one :/)
 ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
 ADC_init_structure.ADC_NbrOfConversion = 1;//I think this one is clear :p
 ADC_init_structure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
 ADC_Init(ADC1,&ADC_init_structure);//Initialize ADC with the previous configuration

 //Select the channel to be read from
 ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_144Cycles);
 //Enable ADC conversion
 ADC_Cmd(ADC1,ENABLE);


 //Calibrate
 /*
 while(ADC_GetResetCalibrationStatus(ADC1));
 ADC_StartCalibration(ADC1);
 while(ADC_GetCalibrationStatus(ADC1));
 */
}
int adc_convert(){
 ADC_SoftwareStartConv(ADC1);//Start the conversion
 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
 return ADC_GetConversionValue(ADC1); //Return the converted data
}




/*
SumoPlatform* initMotorCtrl() {
	SumoPlatform* MotorCtrl = (SumoPlatform*)malloc(sizeof(SumoPlatform));
	MotorCtrl->state = 0;
	MotorCtrl->speed = 0;
	return MotorCtrl;
}
*/
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

  /*
  adc_configure();//Start configuration
  double readings[100];
  double mean = 0;
  uint32_t i = 0;
  double v_mean = 0;
  double range = 0;

  while(1){//loop while the board is working
   //ConvertedValue = adc_convert();//Read the ADC converted value
   mean 	= 0;
   v_mean 	= 0;
   for(i = 0; i < 10; i++) {
	   readings[i] = adc_convert();
	   mean += readings[i]/10;
   }
   v_mean = (0.000732421875)*mean; // 3/4096

   range = 27.86*pow(v_mean, (double)(-1.15));

   if(range < (20)){
	   //w
	   //GPIO_ResetBits(GPIOE, GPIO_Pin_11 | GPIO_Pin_14);
	   GPIO_SetBits(GPIOE, GPIO_Pin_11 | GPIO_Pin_10);
   }
   else if(range > 60){
	   //stop
	   GPIO_ResetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
   }





  //27.86*(2.5)^-1.15

  }
*/

  //GPIO_ResetBits(GPIOE, GPIO_Pin_13 | GPIO_Pin_14);

  //while(1); // Don't want to exit
}

/**************************************************************************************/
