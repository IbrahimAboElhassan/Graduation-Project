
#include "CPU_CONFIGURATION.h"
#include "SM.h"
#include "ws.h"
#include "LCD.h"
#include "BMP180.h"
#include "mpu6050.h"
#include "UART.h"
#include <util/delay.h>
#include <stdbool.h>
#include <math.h>

#define SOME_THRESHOLD 2   

#define MAX_ADJUSTMENT 45  
static int pitch_previous_measurement = 0;      // Threshold for minimum wind speed to start figure-eight maneuver

// PID constants for pitch control
#define Kp_pitch 2.0
#define Ki_pitch 0.1
#define Kd_pitch 0.5
// PID constants for roll control
#define Kp_roll 1.5
#define Ki_roll 0.1
#define Kd_roll 0.4

// Check wind speed every 1000 ms
#define WIND_CHECK_INTERVAL 1000  

// Variables for PID control of pitch
float pitch_integral = 0;          // Integral term for pitch PID
float pitch_previous_error = 0;    // Previous error for pitch PID derivative term

// Variables for PID control of roll
float roll_integral = 0;           // Integral term for roll PID
float roll_previous_error = 0;     // Previous error for roll PID derivative term

// Timestamp of the last PID calculation
unsigned long last_time = 0;


volatile unsigned long milliseconds = 0;  // Counter for milliseconds since program start


unsigned long figureEightStartTime = 0;  // Timestamp for the start of figure-eight maneuver
bool startTimeInitialized = false;       // Flag to indicate if figure-eight start time has been initialized



#define FIGURE_8_PERIOD 10000           // Duration of one complete figure-eight cycle in milliseconds


// Desired angles for figure-eight 
#define desiredPitch1 10     // for loop 1
#define desiredRoll1 20
#define desiredPitch2 -10    // for loop 2
#define desiredRoll2 -20

uint16_t period = 1;                   // Period between steps in milliseconds
uint16_t steps_for_180_degrees = 256;  // Number of steps for 180 degree rotation


void controlKite();
void controlFigureEight();
int determineFigureEightPhase();
void adjustPitch(int desiredPitch);
void adjustRoll(int desiredRoll);
void applyPitchAdjustment(float adjustment);
void applyRollAdjustment(float adjustment);
unsigned long millis();
void timer1_init();
void displayData(int temperature, int altitude, int phase);
void updatePhaseOnLCD(int phase);
int altitudeHasIncreased(int currentAltitude, int* previousAltitude);
int altitudeHasDecreased(int currentAltitude, int* previousAltitude);
void displayMillis();
void display_error(Uint8t error_code) ;
float constrain(float value, float min, float max) ;


/**
 * ================================================================
 * @Fn             - main
 * @brief          - Main function for the kite control system
 * @param [in]     - None
 * @retval         - int
 * Note            - Initializes all components and enters main control loop
 * ================================================================
 */

int main() {

	uint16_t temperature;
	char buffer[16];
	
	UART_Init();  // Initialize UART for HC-12 communication
	StepperMotor_Init();
	WindSensor_Init();
	LCD_Init();

	int8_t init_result = BMP180_Init(BMP180_STANDARD);
	_delay_ms(1000);
	LCD_Clear();

	float referencePressure = BMP180_ReadPressure();
	BMP180_SetReferencePressure(referencePressure);

	float pitch, roll;
	Uint8t error_code;

	I2C_Init(100000);
	error_code = MPU6050_Init();
	if (error_code != 0) {
	display_error(error_code);
	while(1);
}

	MPU6050_Calibrate();

	LCD_Write_String((uint8_t*)"   Kite Power ");
	LCD_Write_Command(0xC0);
	LCD_Write_String((uint8_t*)"     System   ");
	_delay_ms(1000);
	

	// Initialize Timer1 for millis()
	timer1_init();

	// Enable global interrupts
	sei();
	
	last_time = millis();


	while (1) {
		controlKite();
		_delay_ms(100);
	}		
}




/**
 * ================================================================
 * @Fn             - controlKite
 * @brief          - Main control function for kite operation
 * @param [in]     - None
 * @retval         - None
 * Note            - Reads sensor data, displays it, and initiates figure-eight maneuver if conditions are met
 * ================================================================
 */

void controlKite()
{
	static int previousAltitude = 0;
	
	int currentPitch = MPU6050_Get_Pitch();
	LCD_Clear();
	LCD_Write_String((uint8_t*)"Pitch: ");
	LCD_Write_Number(currentPitch);
	LCD_Write_String((uint8_t*)" deg");
	_delay_ms(1000);

	int currentRoll = MPU6050_Get_Roll();
	LCD_Clear();
	LCD_Write_String((uint8_t*)"Roll: ");
	LCD_Write_Number(currentRoll);
	LCD_Write_String((uint8_t*)" deg");
	_delay_ms(1000);
	
	float temp = BMP180_ReadTemperature();
	LCD_Clear();
	LCD_Write_String((uint8_t*)"Temp: ");
	LCD_Write_Number(temp);
	LCD_Write_String((uint8_t*)" c");
	_delay_ms(1000);
	
	float pressure_hPa = BMP180_ReadPressure();   // Read pressure to can read altitude from this equ:
	                                              // alt = 44330 * [1-( pressure / pressure at sea level)^1/5.255] 
	
	int currentAltitude = BMP180_CalculateRelativeAltitude(pressure_hPa);
	LCD_Clear();
	LCD_Write_String((uint8_t*)"Alt: ");
	LCD_Write_Number(currentAltitude);
	LCD_Write_String((uint8_t*)" m");
	_delay_ms(1000);
	
	if (altitudeHasIncreased(currentAltitude, &previousAltitude))
	 {
		// Send data via HC-12
		UART_Tx_STR("i\r\n");
		
		LCD_Clear();
		LCD_Write_String((uint8_t*)"Data sent");
		_delay_ms(1000);
		
	}else if (altitudeHasDecreased(currentAltitude, &previousAltitude)) 
	{
	// Send "data via HC-12
	UART_Tx_STR("d\r\n");
	
	LCD_Clear();
	LCD_Write_String((uint8_t*)"Data sent");
	_delay_ms(1000);
    }
	
	int currentWindSpeed = WindSensor_GetSpeed();
	LCD_Clear();
	LCD_Write_String((uint8_t*)"Wind: ");
	LCD_Write_Number(currentWindSpeed);
	LCD_Write_String((uint8_t*)" m/s");
	_delay_ms(1000);
	

	if (currentWindSpeed > SOME_THRESHOLD)
	{
		LCD_Clear();
		LCD_Write_String((uint8_t*)"Wind OK");
		LCD_Write_Command(0xC0);  
		LCD_Write_String((uint8_t*)"Starting Fig_8");
		_delay_ms(2000);
		
		if (!startTimeInitialized) {
			figureEightStartTime = millis();
			startTimeInitialized = true;
		}
		
		controlFigureEight();
	}
	else
	{
		LCD_Clear();
		LCD_Write_String((uint8_t*)"Wind too low");
		LCD_Write_Command(0xC0);  
		LCD_Write_String((uint8_t*)"Waiting...");
		_delay_ms(1000);
	}

}


/**
 * ================================================================
 * @Fn             - controlFigureEight
 * @brief          - Controls the kite in a figure-eight pattern
 * @param [in]     - None
 * @retval         - None
 * Note            - Executes a two-phase figure-eight maneuver, adjusting pitch and roll
 * ================================================================
 */

void controlFigureEight()
{
	static int previousAltitude = 0;
	unsigned long phaseStartTime;
	unsigned long lastWindCheckTime = 0;
	int currentPhase = 1;
	
	while (1) {
		// Display phase number
		LCD_Clear();
		LCD_Write_String((uint8_t*)"Phase ");
		LCD_Write_Number(currentPhase);
		_delay_ms(2000);
		
		phaseStartTime = millis();
		
		// Execute the current phase for 5 seconds
		while (millis() - phaseStartTime < 5000) {
			float t = (float)(millis() - phaseStartTime) / 5000.0;
			
			float desiredPitch, desiredRoll;
			if (currentPhase == 1) {
				desiredPitch = 10 * sin(2 * M_PI * t);
				desiredRoll = 20 * sin(4 * M_PI * t);
				} else {
				desiredPitch = -10 * sin(2 * M_PI * t);
				desiredRoll = -20 * sin(4 * M_PI * t);
			}
			
			// Adjust pitch and roll
			adjustPitch((int)desiredPitch);
			adjustRoll((int)desiredRoll);
			
			// Check wind speed at regular intervals
			if (millis() - lastWindCheckTime >= WIND_CHECK_INTERVAL) {
				int currentWindSpeed = WindSensor_GetSpeed();
				lastWindCheckTime = millis();
				
				if (currentWindSpeed <= SOME_THRESHOLD) {
					// Wind speed too low, abort figure-eight maneuver
					LCD_Clear();
					LCD_Write_String((uint8_t*)"Wind too low");
					LCD_Write_Command(0xC0);
					LCD_Write_String((uint8_t*)"Aborting Fig8");
					_delay_ms(2000);
					return;  // Exit the function, ending the figure-eight maneuver
				}
			}
			
			// Check altitude and send data if increased
			float pressure_hPa = BMP180_ReadPressure();
			int currentAltitude = BMP180_CalculateRelativeAltitude(pressure_hPa);
			
			if (altitudeHasIncreased(currentAltitude, &previousAltitude))
			{
				// Send data via HC-12
				UART_Tx_STR("i\r\n");
				
				LCD_Clear();
				LCD_Write_String((uint8_t*)"Alt increased");
				_delay_ms(1000);
				
			}else if (altitudeHasDecreased(currentAltitude, &previousAltitude))
			{
				// Send "decrease" via HC-12
				UART_Tx_STR("d\r\n");
				
				LCD_Clear();
				LCD_Write_String((uint8_t*)"Alt decreased");
				_delay_ms(1000);
			}
			
			// Display sensor data
			displaySensorData();
			
			_delay_ms(100);
		}
		
		// Switch phases
		currentPhase = (currentPhase == 1) ? 2 : 1;
		
		// Display completion message if finishing phase 2
		if (currentPhase == 1) {
			LCD_Clear();
			LCD_Write_String((uint8_t*)"Figure 8 Complete");
			_delay_ms(2000);
		}
	}
}


/**
 * ================================================================
 * @Fn             - displaySensorData
 * @brief          - Displays current sensor data on the LCD
 * @param [in]     - None
 * @retval         - None
 * Note            - Reads and displays pitch, roll, wind speed, temperature, and altitude
 * ================================================================
 */

void displaySensorData() {
    int currentPitch = MPU6050_Get_Pitch();
    int currentRoll = MPU6050_Get_Roll();
    int currentWindSpeed = WindSensor_GetSpeed();
    int currentTemperature = BMP180_ReadTemperature();
    float pressure_hPa = BMP180_ReadPressure();
    int currentAltitude = BMP180_CalculateRelativeAltitude(pressure_hPa);
    
    LCD_Clear();
    LCD_Write_String((uint8_t*)"P:");
    LCD_Write_Number(currentPitch);

    LCD_Write_String((uint8_t*)" R:");
    LCD_Write_Number(currentRoll);
    
    LCD_Write_Command(0xC0);  
	
    LCD_Write_String((uint8_t*)"W:");
    LCD_Write_Number(currentWindSpeed);

    LCD_Write_String((uint8_t*)" T:");
    LCD_Write_Number(currentTemperature);
    
    _delay_ms(2000); 
    
    LCD_Clear();
    LCD_Write_String((uint8_t*)"Alt:");
    LCD_Write_Number(currentAltitude);
    LCD_Write_String((uint8_t*)"m");
    
    _delay_ms(2000);  
}


/**
 * ================================================================
 * @Fn             - displayFigure8Data
 * @brief          - Displays data specific to figure-eight maneuver
 * @param [in]     - pitch: float value of current pitch
 * @param [in]     - roll: float value of current roll
 * @param [in]     - phase: int value of current figure-eight phase
 * @retval         - None
 * Note            - Displays the current phase of the figure-eight maneuver
 * ================================================================
 */

void displayFigure8Data(float pitch, float roll, int phase) 
{
	LCD_Clear();
	LCD_Write_String((uint8_t*)"Phase: ");
	LCD_Write_Number(phase);
	_delay_ms(1000);
}


/**
 * ================================================================
 * @Fn             - updatePhaseOnLCD
 * @brief          - Updates the LCD with the current figure-eight phase
 * @param [in]     - phase: int value of the current phase
 * @retval         - None
 * Note            - Displays the phase number and whether it's the upper or lower loop
 * ================================================================
 **/

void updatePhaseOnLCD(int phase) {
	static int lastPhase = -1;
	if (phase != lastPhase) {
		LCD_Clear();
		LCD_Write_String((uint8_t*)"Fig8 Phase: ");
		LCD_Write_Number(phase);
		LCD_Write_Command(0xC0);  
		if (phase == 1) {
			LCD_Write_String((uint8_t*)"Upper Loop");
			} else {
			LCD_Write_String((uint8_t*)"Lower Loop");
		}
		_delay_ms(1000);
		lastPhase = phase;
	}
}


/**
 * ================================================================
 * @Fn             - adjustPitch
 * @brief          - Adjusts the pitch of the kite using PID control
 * @param [in]     - desiredPitch: int value of the target pitch
 * @retval         - None
 * Note            - Calculates PID terms and applies pitch adjustment
 * ================================================================
 */
	
void adjustPitch(int desiredPitch)
{
	unsigned long current_time = millis();
	float dt = (current_time - last_time) / 1000.0;  // Convert interval to seconds
	last_time = current_time;

	int currentPitch = MPU6050_Get_Pitch();
	float error = desiredPitch - currentPitch;
	
	pitch_integral += error * dt;
	float derivative = (error - pitch_previous_error) / dt;
	
	float pitchAdjustment = Kp_pitch * error + Ki_pitch * pitch_integral + Kd_pitch * derivative;
	
	pitch_previous_error = error;
	
	applyPitchAdjustment((int)pitchAdjustment);
}


/**
 * ================================================================
 * @Fn             - adjustRoll
 * @brief          - Adjusts the roll of the kite using PID control
 * @param [in]     - desiredRoll: int value of the target roll
 * @retval         - None
 * Note            - Calculates PID terms and applies roll adjustment
 * ================================================================
 */

void adjustRoll(int desiredRoll)
{
	unsigned long current_time = millis();
	float dt = (current_time - last_time) / 1000.0;  // Convert interval to seconds
	last_time = current_time;

	int currentRoll = MPU6050_Get_Roll();
	float error = desiredRoll - currentRoll;
	
	roll_integral += error * dt;
	float derivative = (error - roll_previous_error) / dt;
	
	float rollAdjustment = Kp_roll * error + Ki_roll * roll_integral + Kd_roll * derivative;
	
	roll_previous_error = error;
	
	applyRollAdjustment((int)rollAdjustment);
}


/**
 * ================================================================
 * @Fn             - applyPitchAdjustment
 * @brief          - Applies the calculated pitch adjustment to the stepper motor
 * @param [in]     - adjustment: float value of the pitch adjustment
 * @retval         - None
 * Note            - Rotates the stepper motor clockwise or counterclockwise based on the adjustment
 * ================================================================
 */

void applyPitchAdjustment(float adjustment)
{
	LCD_Clear();
	
	if (adjustment > 0)
	 {
		LCD_Write_String((uint8_t*)"Pitch CW");
		LCD_Write_Command(0xC0); 
		LCD_Write_String((uint8_t*)"N_Pitch ");
		LCD_Write_Number((int)adjustment);
		LCD_Write_String((uint8_t*)" deg");
		StepperMotor_RotateClockwise(period, (uint16_t)(adjustment * steps_for_180_degrees / 180.0));
	 } else if (adjustment < 0) 
	 {
		LCD_Write_String((uint8_t*)"Pitch CCW");
		LCD_Write_Command(0xC0); 
		LCD_Write_String((uint8_t*)"N_Pitch ");
		LCD_Write_Number((int)-adjustment);
		LCD_Write_String((uint8_t*)" deg");
		StepperMotor_RotateAnticlockwise(period, (uint16_t)(-adjustment * steps_for_180_degrees / 180.0));
	 }
	 
	_delay_ms(1000);
}


/**
 * ================================================================
 * @Fn             - applyRollAdjustment
 * @brief          - Applies the calculated roll adjustment to the stepper motor
 * @param [in]     - adjustment: float value of the roll adjustment
 * @retval         - None
 * Note            - Rotates the stepper motor clockwise or counterclockwise based on the adjustment
 * ================================================================
 */

void applyRollAdjustment(float adjustment)
{
	LCD_Clear();
	
	if (adjustment > 0)
	 {
		LCD_Write_String((uint8_t*)"Roll CW");
		LCD_Write_Command(0xC0); 
		LCD_Write_String((uint8_t*)"N_Roll ");
		LCD_Write_Number((int)adjustment);
		LCD_Write_String((uint8_t*)" deg");
		StepperMotor_RotateClockwise(period, (uint16_t)(adjustment * steps_for_180_degrees / 180.0));
	 } else if (adjustment < 0) 
	 {
		LCD_Write_String((uint8_t*)"Roll CCW");
		LCD_Write_Command(0xC0); 
		LCD_Write_String((uint8_t*)"N_Roll ");
		LCD_Write_Number((int)-adjustment);
		LCD_Write_String((uint8_t*)" deg");
		StepperMotor_RotateAnticlockwise(period, (uint16_t)(-adjustment * steps_for_180_degrees / 180.0));
	 }
	 
	_delay_ms(1000);
}


/**
 * ================================================================
 * @Fn             - altitudeHasIncreased
 * @brief          - Checks if the current altitude has increased from the previous reading
 * @param [in]     - currentAltitude: int value of the current altitude
 * @param [in]     - previousAltitude: pointer to int storing the previous altitude
 * @retval         - int: 1 if altitude has increased, 0 otherwise
 * Note            - Updates the previousAltitude if an increase is detected
 * ================================================================
 */

int altitudeHasIncreased(int currentAltitude, int* previousAltitude) 
{
	if (currentAltitude > *previousAltitude) {
		*previousAltitude = currentAltitude;
		return 1;
	}
	return 0;
}


/**
 * ================================================================
 * @Fn             - altitudeHasDecreased
 * @brief          - Checks if the current altitude has decreased from the previous reading
 * @param [in]     - currentAltitude: int value of the current altitude
 * @param [in]     - previousAltitude: pointer to int storing the previous altitude
 * @retval         - int: 1 if altitude has decreased, 0 otherwise
 * Note            - Updates the previousAltitude if a decrease is detected
 * ================================================================
 */

int altitudeHasDecreased(int currentAltitude, int* previousAltitude) 
{
	if (currentAltitude < *previousAltitude) {
		*previousAltitude = currentAltitude;
		return 1;
	}
	return 0;
}
//

/**
 * ================================================================
 * @Fn             - millis
 * @brief          - Returns the number of milliseconds since the program started
 * @param [in]     - None
 * @retval         - unsigned long: number of milliseconds
 * Note            - Uses volatile variable updated by timer interrupt
 * ================================================================
 */

unsigned long millis() {
	unsigned long ms;
	cli(); // Disable interrupts
	ms = milliseconds;
	sei(); // Enable interrupts
	return ms;
}


/**
 * ================================================================
 * @Fn             - timer1_init
 * @brief          - Initializes Timer1 for millisecond counting
 * @param [in]     - None
 * @retval         - None
 * Note            - Sets up Timer1 in CTC mode with 1ms interval
 * ================================================================
 */

void timer1_init() {
	// Configure Timer1 for CTC mode
	TCCR1B |= (1 << WGM12);
	// Set compare value for 1ms at 16MHz clock with prescaler 64
	OCR1A = 249;
	// Enable compare interrupt
	TIMSK |= (1 << OCIE1A);
	// Start timer with prescaler 64
	TCCR1B |= (1 << CS11) | (1 << CS10);
}


/**
 * ================================================================
 * @Fn             - TIMER1_COMPA_vect
 * @brief          - Timer1 Compare Match A Interrupt Service Routine
 * @param [in]     - None
 * @retval         - None
 * Note            - Increments milliseconds counter every 1ms
 * ================================================================
 */

ISR(TIMER1_COMPA_vect) {
	milliseconds++;
}


/**
 * ================================================================
 * @Fn             - display_error
 * @brief          - Displays an error code on the LCD
 * @param [in]     - error_code: Uint8t value of the error code
 * @retval         - None
 * Note            - Clears LCD and displays "Error: " followed by the error code
 * ================================================================
 */

void display_error(Uint8t error_code) 
{
	LCD_Clear();
	LCD_Write_String((Uint8t*)"Error: ");
	LCD_Write_Number(error_code);
	_delay_ms(2000);
}