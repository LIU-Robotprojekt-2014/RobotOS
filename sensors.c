#include <stm32f4xx.h>
#include <misc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_dma.h>
#include <math.h>
#include "sensors.h"
#include <stm32f4xx_exti.h>
#include <stm32f4xx_syscfg.h>


Sensors S;
volatile uint16_t ADCConvertedValue[4];
		float range[4];




void init_sensors2(void) {
	 GPIO_InitTypeDef      GPIO_InitStructure;
	  ADC_InitTypeDef       ADC_InitStructure;
	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
	  DMA_InitTypeDef       DMA_InitStructure;

	  GPIO_StructInit(&GPIO_InitStructure);
	  ADC_StructInit(&ADC_InitStructure);
	  ADC_CommonStructInit(&ADC_CommonInitStructure);
	  DMA_StructInit(&DMA_InitStructure);

	  /**
	    Set up the clocks are needed for the ADC
	  */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

	  /**
	    Initialization of the GPIO Pins [OK]
	  */

	  /* Analog channel configuration : PC.01, 02*/
	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /**
	    Configure the DMA
	  */
	  //==Configure DMA2 - Stream 4
	  DMA_DeInit(DMA2_Stream4);  //Set DMA registers to default values
	  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //Source address
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValue[0]; //Destination address
	  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	  DMA_InitStructure.DMA_BufferSize = 4; //Buffer size
	  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source size - 16bit
	  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // destination size = 16b
	  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	  DMA_Init(DMA2_Stream4, &DMA_InitStructure); //Initialize the DMA
	  DMA_Cmd(DMA2_Stream4, ENABLE); //Enable the DMA2 - Stream 4

	   /**
	     Config the ADC1
	   */
	   ADC_DeInit();
	   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //continuous conversion
	   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	   ADC_InitStructure.ADC_NbrOfConversion = 4;
	   ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1=scan more that one channel in group
	   ADC_Init(ADC1,&ADC_InitStructure);

	   ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	   ADC_CommonInit(&ADC_CommonInitStructure);

	   ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_480Cycles);
	   ADC_RegularChannelConfig(ADC1,ADC_Channel_11,2,ADC_SampleTime_480Cycles);
	   ADC_RegularChannelConfig(ADC1,ADC_Channel_12,3,ADC_SampleTime_480Cycles);
	   ADC_RegularChannelConfig(ADC1,ADC_Channel_13,4,ADC_SampleTime_480Cycles);

	   ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	   ADC_DMACmd(ADC1, ENABLE); //Enable ADC1 DMA

	   ADC_Cmd(ADC1, ENABLE);   // Enable ADC1

	   ADC_SoftwareStartConv(ADC1); // Start ADC1 conversion
}



void init_rotary(void){
	/**
	  * @brief  configures specified GPIO pin as output.
	  * @param  GPIOx: where x can be (A..I) to select the GPIO peripheral.
	  * @param  GPIO_Pin: specifies the port bit to be configured in output mode.
	  *          This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
	  * @param  GPIO_Mode: Specify GPIO Configuration i.e. input/output/ADC/AF
	  *         This parameter can be a value of @ref GPIOMode_TypeDef
	  * @retval None
	  */

	 /*
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //used for digital output and PWM output
	               //this setting does not matter for ADC and digital read
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);*/

	 /* Enable SYSCFG clock */

		i=0;
		GPIO_InitTypeDef  GPIO_InitStructure;
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	   GPIO_Init(GPIOB, &GPIO_InitStructure);
	   //InitGPIO(GPIOB, GPIO_Pin_0, GPIO_Mode_IN); //initialize PB1 is input

	 //-------------------------

	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
	  NVIC_InitTypeDef   NVIC_InitStructure;
	  EXTI_InitTypeDef   EXTI_InitStructure;

	  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	  //-------------------

	 //attach interrupt to GPIO
	 //Attach_GPIO_Interrupt(EXTI_PortSourceGPIOB, EXTI_PinSource1, EXTI_Line1 ,EXTI_Trigger_Rising_Falling,1 );





}

void process_sensors(void) {

	int i;
	float derp;
	for(i=0; i<4; i++){

		range[i]=0;
		derp=ADCConvertedValue[i];
		//range[i] = 27.86*pow(derp*3/4096, (double)(-1.15));
		range[i] = 17.77*pow(derp*3/4096, (double)(-0.9524+ 0.1258)) ;
	}
}

void EXTI1_IRQHandler(void) //EXTI1 ISR
	{

	 if(EXTI_GetITStatus(EXTI_Line1) != RESET) //check if EXTI line is asserted
	 {
		 EXTI_ClearFlag(EXTI_Line1); //clear interrupt
		 i++;
		 GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
	 }
	}


