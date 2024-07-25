/*#ifndef SM_H_
#define SM_H_

#include "DIO.h"
#include <util/delay.h>

#define STEPPER_PORT DIO_PORTC

void StepperMotor_Init(void);
void StepperMotor_RotateClockwise(uint16_t period, uint8_t steps);
void StepperMotor_RotateAnticlockwise(uint16_t period, uint8_t steps);
void delay_ms(uint16_t ms);

#endif /* SM_H_ */


#ifndef SM_H_
#define SM_H_

#include "DIO.h"
#include <util/delay.h>

#define STEPPER_PORT DIO_PORTC
#define STEPPER_PIN1 DIO_PIN3
#define STEPPER_PIN2 DIO_PIN4
#define STEPPER_PIN3 DIO_PIN5
#define STEPPER_PIN4 DIO_PIN6

void StepperMotor_Init(void);
void StepperMotor_RotateClockwise(uint16_t period, uint16_t steps);
void StepperMotor_RotateAnticlockwise(uint16_t period, uint16_t steps);

#endif /* SM_H_ */
