#include "ws.h"
#include <avr/io.h>
#include <util/delay.h>
#include "LCD.h"

void WindSensor_Init(void)
{
    // Initialize ADC for wind sensor reading
    ADC_Init();
}

float WindSensor_GetSpeed(void)
{
    Uint16t adc_reading = ADC_Read();
    
    // Convert ADC reading to voltage
    float voltage = (float)adc_reading * (5.0f / 1023.0f);  
    
    // Convert voltage to wind speed (m/s)
    float windSpeed = (voltage - WIND_SENSOR_OFFSET) / WIND_SENSOR_SLOPE;
    
    // Ensure non-negative wind speed
    if (windSpeed < 0) windSpeed = 0;
    
    return windSpeed;
}

/*
#include "ws.h"
#include <avr/io.h>
#include <util/delay.h>
#include "LCD.h"

void WindSensor_Init(void)
{
	ADC_Init();
}

float WindSensor_GetSpeed(void)
{
	Uint16t adc_reading = ADC_Read();
	
	// Debug: Print ADC reading to LCD
	LCD_Clear();
	LCD_Write_String((Uint8t*)"ADC Reading:");
	LCD_Write_Command(0xC0);  // Move to second line
	LCD_Write_Number(adc_reading);
	_delay_ms(1000);
	
	// Convert ADC reading to voltage
	float voltage = (float)adc_reading * (5.0f / 1023.0f);  // Assuming 5V reference and 10-bit ADC
	
	// Debug: Print Voltage to LCD
	LCD_Clear();
	LCD_Write_String((Uint8t*)"Voltage:");
	LCD_Write_Command(0xC0);  // Move to second line
	LCD_Write_Number((Uint32t)(voltage * 100));  // Multiply by 100 to display two decimal places
	_delay_ms(1000);
	
	// Convert voltage to wind speed (m/s)
	float windSpeed = (voltage - WIND_SENSOR_OFFSET) / WIND_SENSOR_SLOPE;
	
	// Ensure non-negative wind speed
	if (windSpeed < 0) windSpeed = 0;
	
	return windSpeed;
}*/

/*
#include "ws.h"
#include <avr/io.h>
#include <util/delay.h>
#include "LCD.h"

void WindSensor_Init(void)
{

	ADC_Init();
}

float WindSensor_GetSpeed(void)
{
	Uint16t adc_reading = ADC_Read();
	
	// Convert ADC reading to voltage
	float voltage = (float)adc_reading * (5.0f / 1023.0f);  // Assuming 5V reference and 10-bit ADC
	
	// Debug: Print Voltage to LCD
	/*LCD_Clear();
	LCD_Write_String((Uint8t*)"Volt:");
	LCD_Write_Command(0xC0);  // Move to second line
	LCD_Write_Number((Uint32t)(voltage * 100));  // Multiply by 100 to display two decimal places
	_delay_ms(1000);*/
	
	// Convert voltage to wind speed (m/s)
	//float windSpeed = (voltage - WIND_SENSOR_OFFSET) / WIND_SENSOR_SLOPE;
	
	// Ensure non-negative wind speed
	//if (windSpeed < 0) windSpeed = 0;
	
	// Debug: Print Wind Speed to LCD
	/*LCD_Clear();
	LCD_Write_String((Uint8t*)"Speed:");
	LCD_Write_Command(0xC0);  // Move to second line
	LCD_Write_Number((Uint32t)(windSpeed * 100));  // Multiply by 100 to display two decimal places
	_delay_ms(1000);*/
	
//	return windSpeed;
//}*/

