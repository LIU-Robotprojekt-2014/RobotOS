#include <stm32f4xx.h>
#include <misc.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include "stm32f4_discovery.h"
#include "bluetooth.h"
#include "platform.h"

Bluetoothunit BT;

void init_bluetooth(void) {
	BT._baudrate = 38400;
	BT._tx_pin	 = GPIO_Pin_8;
	BT._rx_pin	 = GPIO_Pin_9;
	BT._tx_counter = 0;
	BT._rx_counter = 0;
	BT._tx_flags   = 0;
	BT._chars_to_send = 0;
	BT._flags	   = 0;
	BT._sol_position = 0xFF;
	BT._eol_position = 0xFF;
	_clean_buffer();

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* --------------------------- System Clocks Configuration -----------------*/
	/* USART3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* GPIOD clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = BT._tx_pin | BT._rx_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect USART pins to AF */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	/* USARTx configuration ------------------------------------------------------*/
	/* USARTx configured as follow:
		- BaudRate = 38400 baud
		- Word Length = 8 Bits
		- Two Stop Bit
		- Odd parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = BT._baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);

	USART_Cmd(USART3, ENABLE);

	//USART_ITConfig(USART3, USART_IT_RXNE | USART_IT_TXE, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);


	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	/* Enable the USART3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void USART3_IRQHandler(void) {
  char rcv;
  // Received characters modify string
  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
	rcv = USART3->DR;
	_append_to_buffer(rcv);
	//USART_SendData(USART3, rcv);
  }
  if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {

      /* Write one byte to the transmit data register */
	  if(BT._send_buffer[BT._tx_counter] != 0x00) {
		  USART_SendData(USART3, BT._send_buffer[BT._tx_counter++]);
	  } else {
		  USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		  BT._tx_flags |= BT_TX_DONE;
	  }

      if(BT._tx_counter >= BT._chars_to_send) {
    	  /* Disable the USART3 Transmit interrupt */
    	  USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    	  BT._tx_flags |= BT_TX_DONE;
      }
    }

}

void process_bluetooth(void) {
	if((BT._flags&BT_SOL_FLAG) && (BT._flags&BT_EOL_FLAG)) {
		uint8_t lenght = _clonePackageInBufferToPackage();
		BT._flags = BT_RESET;
		//_send_package("Got it!\n", 8);
		if(lenght >= 4) {
			parse_package();
		}
		_clean_package();
	}
}

void parse_package(void) {
	if(BT._package[0] == 'm') {
		if(BT._package[1] == '0') {
			if(BT._package[2] == '1'){

				if(strncmp(&(BT._package[4]), "forward", 7) == 0) {
					set_forward();
					_send_package("Moving forward, Sire!\n", 22);
				} else if (strncmp(&(BT._package[4]), "backward", 8) == 0) {
					set_backward();
					_send_package("Moving backward, Sire!\n", 23);
				} else if (strncmp(&(BT._package[4]), "left", 4) == 0) {
					set_left();
					_send_package("Turning left, Sire!\n", 20);
				} else if (strncmp(&(BT._package[4]), "right", 5) == 0) {
					set_right();
					_send_package("Turning right, Sire!\n", 21);
				} else {
					set_stop();
					_send_package("Stopping, Sire!\n", 16);
				}
			} else if (BT._package[2] == '2'){
				_send_package("Got M02 command!\n", 17);
			}
		}
	} else if(BT._package[0] == 'c') {
		if(BT._package[1] == '0') {
			if(BT._package[2] == '1'){
				_send_package("Got C01 command!\n", 17);
			} else if (BT._package[2] == '2'){
				_send_package("Got C02 command!\n", 17);
			}
		}
	} else if(BT._package[0] == 'a') {
		if(BT._package[1] == '0') {
			if(BT._package[2] == '1'){
				_send_package("Got A01 command!\n", 17);
			} else if (BT._package[2] == '2'){
				_send_package("Got A02 command!\n", 17);
			}
		}
	}

}
int8_t _send_package(char* arr, uint8_t lenght) {
	if(BT._tx_flags&BT_TX_DONE) {
		_clean_send_buffer();
		BT._tx_flags = 0;
	}
	if(!BT._tx_flags&BT_TX_BUSY) {
		uint8_t i;
		BT._chars_to_send = 0;
		for(i = 0; i < lenght; i++) {
			if(arr[i] == 0x00) {
				break;
			}

			BT._send_buffer[BT._chars_to_send++] = arr[i];

			if(arr[i] == 0x0A) {
				break;
			}
		}
		BT._tx_flags |= BT_TX_BUSY;
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
		return 0;
	}
	return BT_TX_BUSY;
}

void _clean_buffer(void) {
	uint8_t i;
	for(i = 0; i < BLUETOOTH_BUFFER_SIZE; i++) {
		BT._buffer[i] = 0x00;
	}
	BT._rx_counter 	 = 0;
	BT._sol_position = 0xFF;
	BT._eol_position = 0xFF;
}
void _clean_package(void) {
	uint8_t i;
	for(i = 0; i < BLUETOOTH_BUFFER_SIZE; i++) {
		BT._package[i] = 0x00;
	}
}
void _clean_send_buffer(void) {
	uint8_t i;
	for(i = 0; i < BLUETOOTH_BUFFER_SIZE; i++) {
		BT._send_buffer[i] = 0x00;
	}
	BT._tx_counter = 0;
}

/**
 * Appends byte to buffer
 */
void _append_to_buffer(char c) {

	//Clean flags and buffer
	if(BT._flags&BT_RESET) {
		BT._flags = 0;
		_clean_buffer();
	}

	if(c == BT_SOL) {
		if(BT._rx_counter > 0) {
			_clean_buffer();
		}
		BT._flags |= BT_SOL_FLAG;
		BT._sol_position = BT._rx_counter;
	} else if (c == BT_EOL) {
		if(BT._flags&BT_SOL_FLAG) {
			BT._flags |= BT_EOL_FLAG;
			BT._eol_position = BT._rx_counter;
		} else {
			BT._flags |= BT_RESET;
		}
	}
	if(BT._rx_counter >= BLUETOOTH_BUFFER_SIZE) {
		_clean_buffer();
	}

	BT._buffer[BT._rx_counter] = c;
	BT._rx_counter++;

}

uint8_t _buffer_count(void) {
	uint8_t i;
	for(i = 0; i < BLUETOOTH_BUFFER_SIZE; i++) {
		if(BT._buffer[i] == 0x00) {
			return i;
		}
	}
	return 0;
}

uint8_t _clonePackageInBufferToPackage(void) {
	uint8_t length = 0;
	int8_t size;
	uint8_t i;
	if((BT._buffer[BT._sol_position] == BT_SOL) && (BT._buffer[BT._eol_position] == BT_EOL)) {
		size = BT._eol_position - BT._sol_position;
		if(size > 0) {
			for(i = BT._sol_position+1; i < BT._eol_position; i++) {
				BT._package[length] = BT._buffer[i];
				length++;
			}
		}
	}
	return length;
}
