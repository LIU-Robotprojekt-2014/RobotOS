#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#define BLUETOOTH_BUFFER_SIZE 60

#define BT_SOL	0x01
#define BT_EOL	0x0A

//#define BT_SOL	122
//#define BT_EOL	120

#define BT_SOL_FLAG 0x01
#define BT_EOL_FLAG 0x02
#define BT_RESET 0x80

#define BT_TX_BUSY 0x01
#define BT_TX_DONE 0x02

typedef struct BluetoothUnit {
   uint32_t _baudrate;
   uint16_t _tx_pin;
   uint16_t _rx_pin;

   volatile char _buffer[BLUETOOTH_BUFFER_SIZE];
   char _package[BLUETOOTH_BUFFER_SIZE];
   volatile uint8_t _rx_counter;
   volatile uint8_t _flags; //Bitwise, Bit0: sof rec., Bit1: linefeed recv. Bit7: clean buffer and flags
   volatile uint8_t _sol_position;
   volatile uint8_t _eol_position;
   //Send variables
   char _send_buffer[BLUETOOTH_BUFFER_SIZE];
   volatile uint8_t _tx_counter;
   volatile uint8_t _chars_to_send;
   volatile uint8_t _tx_flags;

} Bluetoothunit;

void init_bluetooth(void);
void USART3_IRQHandler(void);
void process_bluetooth(void);
void parse_package(void);
int8_t _send_package(char* arr, uint8_t lenght);
void _clean_buffer(void);
void _clean_package(void);
void _clean_send_buffer(void);
void _append_to_buffer(char c);
uint8_t _buffer_count(void);
uint8_t _clonePackageInBufferToPackage(void);

#endif
