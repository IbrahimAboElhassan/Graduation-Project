
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "UART.h"
#include "LCD.h"
#include "SM.h"

#define STEP_DELAY_MS 10  

uint16_t period = 1;
uint16_t steps_for_90_degrees = 256;  

void rotate_motor(int direction) {
	for (uint16_t i = 0; i < steps_for_90_degrees; i++) {
		if (direction > 0) {
			StepperMotor_RotateClockwise(period, 1);
			} else {
			StepperMotor_RotateAnticlockwise(period, 1);
		}
		_delay_ms(STEP_DELAY_MS);
	}
}

int main(void)
{
	UART_Init();
	StepperMotor_Init();
	LCD_Init();
	
	LCD_Write_String((Uint8t*)"Receiver Ready");
	_delay_ms(1000);
	
	while (1) {
		Uint8t receivedChar;
		if (UART_Receive_Data(&receivedChar) == UART_OK) {
			LCD_Clear();
			_delay_ms(1000);
			
			if (receivedChar == 'i') {
				LCD_Clear();
				LCD_Write_String((Uint8t*)"Altitude up");
				LCD_Write_Command(0xC0);
				LCD_Write_String((Uint8t*)"Rotating CW");
				
				rotate_motor(1);  // Rotate clockwise
			}
			else if (receivedChar == 'd') {
				LCD_Clear();
				LCD_Write_String((Uint8t*)"Altitude down");
				LCD_Write_Command(0xC0);
				LCD_Write_String((Uint8t*)"Rotating CCW");
				
				rotate_motor(-1);  // Rotate counterclockwise
			}
			
			_delay_ms(1000);  
		}
	}
	
	return 0;
}