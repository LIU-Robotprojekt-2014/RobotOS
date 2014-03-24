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

Sensors S;
volatile uint16_t ADCConvertedValue[2];
volatile uint32_t ADCConvertedValues[4];


void init_sensors(void) {
	/* Init IR sensors*/
	int i = 0;
	for(i = 0; i < IR_SENSORS; i++) {
		S.IR[i]._latest_reading 	= 0.0;
		S.IR[i]._calibration 		= 1.0;
	}

	GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin
	//ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;

	//Clock configuration
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//The ADC1 is connected the APB2 peripheral bus thus we will use its clock source

	//Analog pin configuration
	GPIO_initStructre.GPIO_Pin = GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3; //Pins for ADC
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
	GPIO_Init(GPIOA, &GPIO_initStructre);//Affecting the port with the initialization structure configuration

	/* DMA2_Stream0 channel0 configuration **************************************/
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValues[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);




	//ADC structure configuration
	/*
	ADC_DeInit();
	ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
	ADC_init_structure.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit number giving a maximum value of 4096
	ADC_init_structure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
	ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;// conversion is synchronous with TIM1 and CC1 (actually I'm not sure about this one :/)
	ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
	ADC_init_structure.ADC_NbrOfConversion = 1;//I think this one is clear :p
	ADC_init_structure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
	ADC_Init(ADC1,&ADC_init_structure);//Initialize ADC with the previous configuration
	*/
	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 4;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel configuration ******************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles); // PA0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_480Cycles); // PA1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_480Cycles); // PA2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_480Cycles); // PA3

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);


    /* Enable ADC1 **************************************************************/
	ADC_Cmd(ADC1, ENABLE);

    /* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv(ADC1);

	/*
	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_144Cycles);
	//Enable ADC conversion
	ADC_Cmd(ADC1,ENABLE);
	*/

	/* Init Rotary encoder*/
}


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
	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
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
	  DMA_InitStructure.DMA_BufferSize = 2; //Buffer size
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
	   ADC_InitStructure.ADC_NbrOfConversion = 2;
	   ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 1=scan more that one channel in group
	   ADC_Init(ADC1,&ADC_InitStructure);

	   ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	   ADC_CommonInit(&ADC_CommonInitStructure);

	   ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_480Cycles);
	   ADC_RegularChannelConfig(ADC1,ADC_Channel_11,2,ADC_SampleTime_480Cycles);

	   ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	   ADC_DMACmd(ADC1, ENABLE); //Enable ADC1 DMA

	   ADC_Cmd(ADC1, ENABLE);   // Enable ADC1

	   ADC_SoftwareStartConv(ADC1); // Start ADC1 conversion
}

void process_sensors(void) {

}

void getVal(uint32_t arr[]) {
	if (ADCConvertedValues[0] != 0xFFFF && ADCConvertedValues[1] != 0xFFFF && ADCConvertedValues[2] != 0xFFFF && ADCConvertedValues[3] != 0xFFFF) {
		arr[0] = ADCConvertedValues[0];
		arr[1] = ADCConvertedValues[1];
		arr[2] = ADCConvertedValues[2];
		arr[3] = ADCConvertedValues[3];
		ADCConvertedValues[0] = 0xFFFF;
		ADCConvertedValues[1] = 0xFFFF;
		ADCConvertedValues[2] = 0xFFFF;
		ADCConvertedValues[3] = 0xFFFF;
	}
}
