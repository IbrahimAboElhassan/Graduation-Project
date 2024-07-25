/*#include "SM.h"

void StepperMotor_Init(void)
{
	DIO_SetPort_Direction(STEPPER_PORT, 0b01111000); // Set 4 bits of PORTC as output
}

void StepperMotor_RotateClockwise(uint16_t period, uint8_t steps)
{
	for (uint8_t i = 0; i < steps; i++)
	{
		DIO_SetPort_Value(STEPPER_PORT, 0b00001000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b00011000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b00010000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b00110000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b00100000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b01100000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b01000000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b01001000);
		delay_ms(period);
	}
}

void StepperMotor_RotateAnticlockwise(uint16_t period, uint8_t steps)
{
	for (uint8_t i = 0; i < steps; i++)
	{
		DIO_SetPort_Value(STEPPER_PORT, 0b01001000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b01000000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b01100000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b00100000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b00110000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b00010000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b00011000);
		delay_ms(period);
		DIO_SetPort_Value(STEPPER_PORT, 0b00001000);
		delay_ms(period);
	}
}*/

#include "SM.h"

void StepperMotor_Init(void)
{
	DIO_SetPin_Direction(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_OUTPUT);
	DIO_SetPin_Direction(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_OUTPUT);
	DIO_SetPin_Direction(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_OUTPUT);
	DIO_SetPin_Direction(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_OUTPUT);
}

void StepperMotor_RotateClockwise(uint16_t period, uint16_t steps)
{
	for (uint16_t i = 0; i < steps; i++)
	{
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_HIGH);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_HIGH);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_HIGH);
		delay_ms(period);
	}
}

void StepperMotor_RotateAnticlockwise(uint16_t period, uint16_t steps)
{
	for (uint16_t i = 0; i < steps; i++)
	{
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_HIGH);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_HIGH);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_HIGH);
		delay_ms(period);
		
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);
		
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);

		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN1, DIO_PIN_HIGH);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN2, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN3, DIO_PIN_LOW);
		DIO_SetPin_Value(STEPPER_PORT, STEPPER_PIN4, DIO_PIN_LOW);
		delay_ms(period);
	}
}



void delay_ms(uint16_t ms)
{
	for (uint16_t i = 0; i < ms; i++)
	{
		_delay_ms(1);
	}
}