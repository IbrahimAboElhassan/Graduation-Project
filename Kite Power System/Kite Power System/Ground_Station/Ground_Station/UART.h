


/*#ifndef UART_H_
#define UART_H_
//Include Macros
#include "UART_CONFIG.h"
//Functions prototypes
void UART_Init(void);
//Transmitting
void UART_Tx(Uint8t data);
UART_STATUS UART_Transmit(Uint8t data);
//Receiving
Uint8t UART_Rx(void);
void UART_Receive(Uint8t* data);
UART_STATUS UART_Receive_Data(Uint8t* data);
//Negawed 3shn khater asmaa
void UART_Tx_STR(Uint8t* str);
void UART_Rx_STR(Uint8t* str);
#endif /* UART_H_ */

#ifndef UART_H_
#define UART_H_

#include "UART_CONFIG.h"

void UART_Init(void);
void UART_Tx(uint8_t data);
UART_STATUS UART_Transmit(uint8_t data);
uint8_t UART_Rx(void);
void UART_Receive(uint8_t* data);
UART_STATUS UART_Receive_Data(uint8_t* data);
void UART_Tx_STR(const char* str);
void UART_Rx_STR(char* str);

#endif /* UART_H_ */
