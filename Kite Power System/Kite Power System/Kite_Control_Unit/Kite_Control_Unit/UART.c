


/*#include "UART.h"
//Functions prototypes
void UART_Init(void)
{
	Uint16t UBRR_Value = 0;
/*	
	CLR_BIT(DDRD, PD0);
	SET_BIT(DDRD, PD1);

	CLR_BIT(UCSRB, TXCIE);
	CLR_BIT(UCSRB, RXCIE);
	CLR_BIT(UCSRB, UDRIE);
	SET_BIT(UCSRB, TXEN);
	SET_BIT(UCSRB, RXEN);
	SET_BIT(UCSRC, URSEL);
	CLR_BIT(UCSRC, UCSZ2);SET_BIT(UCSRC, UCSZ1);SET_BIT(UCSRC, UCSZ0);
	CLR_BIT(UCSRC, UPM1);CLR_BIT(UCSRC, UPM0);
	CLR_BIT(UCSRC, USBS);	
	*/
/*
	//Enable Transmit and Receive
	UCSRB = (1 << RXEN) | (1 << TXEN);
	//Full configuration for UART
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
	//Baud Rate Calculation
	UBRR_Value = (((FRQ) / (16 * BAUDRATE)) - 1);
	//UBRRL = UBRR_Value;
	UBRRL = UBRR_Value;
	
	UBRRH = UBRR_Value >> 8;
}
//Transmitting
void UART_Tx(Uint8t data)
{
	UDR = data;
	while (GET_BIT(UCSRA, TXC) != 1);
}
UART_STATUS UART_Transmit(Uint8t data)
{
	UDR = data;
	while (GET_BIT(UCSRA, TXC) != 1);
	return UART_OK;
}
//Receiving
Uint8t UART_Rx(void)
{
	Uint8t data;
	while (GET_BIT(UCSRA, RXC) != 1);
	data = UDR;
	return data;
}
void UART_Receive(Uint8t* data)
{
	while (GET_BIT(UCSRA, RXC) != 1);
	*data = UDR;
}
UART_STATUS UART_Receive_Data(Uint8t* data)
{
	while (GET_BIT(UCSRA, RXC) != 1);
	*data = UDR;
	return UART_OK;
}
//Tagwiid
void UART_Tx_STR(Uint8t* str)
{
	while (*str != '\0')
	{
		UART_Tx(*str);
		_delay_ms(1);
		str++;
	}
}
void UART_Rx_STR(Uint8t* str)
{
	Uint8t receive = 0;
	*str = "";
	while (receive != '\0')
	{
		UART_Receive(&receive);
		_delay_ms(1);
		*str += receive;
	}
}*/

#include "UART.h"

/*void UART_Init(void) {
	uint16_t UBRR_Value = (((FRQ) / (16 * BAUDRATE)) - 1);
	UCSRB = (1 << RXEN) | (1 << TXEN);
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
	UBRRL = UBRR_Value;
	UBRRH = UBRR_Value >> 8;
}*/

void UART_Init(void) {
	uint16_t UBRR_Value = (F_CPU / 16 / BAUDRATE) - 1;
	UCSRB = (1 << RXEN) | (1 << TXEN);
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
	UBRRL = UBRR_Value & 0xFF;
	UBRRH = (UBRR_Value >> 8) & 0xFF;
}

/*void UART_Tx(uint8_t data) {
	UDR = data;
	while (!(UCSRA & (1 << TXC)));
}*/

void UART_Tx(uint8_t data) {
	while (!(UCSRA & (1 << UDRE))); 
	UDR = data;
}

UART_STATUS UART_Transmit(uint8_t data) {
	UDR = data;
	while (!(UCSRA & (1 << TXC)));
	return UART_OK;
}

uint8_t UART_Rx(void) {
	while (!(UCSRA & (1 << RXC)));
	return UDR;
}

void UART_Receive(uint8_t* data) {
	while (!(UCSRA & (1 << RXC)));
	*data = UDR;
}

UART_STATUS UART_Receive_Data(uint8_t* data) {
	while (!(UCSRA & (1 << RXC)));
	*data = UDR;
	return UART_OK;
}

void UART_Tx_STR(const char* str) {
	while (*str) {
		UART_Tx(*str++);
	}
}

void UART_Rx_STR(char* str) {
	uint8_t receive = 0;
	while (receive != '\0') {
		UART_Receive(&receive);
		*str++ = receive;
	}
	*str = '\0';
}
