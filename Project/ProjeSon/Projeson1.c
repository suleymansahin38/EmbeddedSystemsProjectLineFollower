#include "stm32f10x.h"    // Essential header file for STM32F10x series microcontrollers; includes register definitions and basic functions. Used for stm32f103c8t6.             
#include "Clockconfig.h"	// Header file that written for configuring the clock settings of the microcontroller with 72 MHz clock frequency.
#include "delay.h"				// Provides software-based delay functions for time-varying operations.
#include "stdio.h"				// Standard C library and header used for input/output functions.
#include "Contor.h"				// Custom header file that written and used for PID control logic, for variable definitions, and for algorithms.



void GPIO_config(void);		// Configures the GPIO pins, including the GPIO pin modes such as input/output, pull-up/pull-down settings, and other functionalities.
void Timer_config(void);	// Configures timer peripherals for specific timing operations, for PWM generation, and for input-output capture, compare mode.
void delay_miliseconds(uint32_t ms);		// Implemented a delay in milliseconds, allowing precise timing for operations.
void delay_microseconds(uint32_t us);		// Implements a delay in microseconds for more precise timing operations, used mostly for HC SR 04 ultrasonic sensor measurement.
uint32_t Read_distance(void);						// Reads the distance from HC SR 04 and returns the measured value in a 32-bit unsigned integer.

void right_pwmcontrol (void);						// Controls the PWM signal for the right motor, and allowing right motor speed adjustments, and enables independent speed control.
void left_pwmcontrol (void);						// Controls the PWM signal for the left motor, and allowing left motor speed adjustments, and enables independent speed control.
void Controller_control_logic(void);		// Implements the main control logic by including PID variables, PID algorithm fpr line following logic.
void left_sensor (int T_2_3,int T_2_4);	// The function processes data from the left sensor of the car. The parameters are used for sensor pin configurations by using Timer duty cycles, and changes ch3 and ch4 values.
void right_sensor (int T_3_3,int T_3_4); // The function processes data from the right sensor of the car. The parameters are used for sensor pin configurations by using Timer duty cycles, and changes ch3 and ch4 values.
void sharp_turn_handle(void);						// This function handles sharp turns by adjusting left motor and right motor speeds and directions to maintain line following accurately.

  static int sensor_read_value = 0x00000000;  // The integer stores the value that read from the QTR-8 sensor array. It is initialized to zero and declared as "static" to retain its value across function calls.
  int left_motor_speed;												// Variable to store the speed of the left motor, which can be dynamically updated.
  int right_motor_speed;											// Variable to store the speed of the right motor, which can be dynamically updated.
  const uint8_t right_motor_speedMax = 100;		// Constant defining the maximum speed for the right motor, represented as an 8-bit unsigned integer.
	const uint8_t left_motor_speedMax = 100;    // Constant defining the maximum speed for the left motor, represented as an 8-bit unsigned integer.

	
void General_Init(void){ //This function initializes the PID controller parameters, motor base speeds to their default values. The function also clears the error and sensor information arrays for fresh operation.


  Controller->pid_position =0;		// Initialize the PID controller's position to zero
  Controller-> Kp = 0.0151; 					// Proportional Gain, determines response strength to current error
	Controller-> Ki = 0.0031;  		  // Integral Gain, accounts for accumulated error over time
	Controller-> Kd = 10;						// Derivative Gain, predicts future error based on its rate of change

																	// This code block initializes the PID components and values to zero.
	Controller-> proportional = 0;	// Proportional component of PID. Assigned to zero at the beginning.
	Controller->integrational= 0;		// Integral component of PID. Assigned to zero at the beginning.
	Controller->Derivative= 0;			// Derivative component of PID. Assigned to zero at the beginning.
  Controller-> total_error = 0;		// Total error value for PID calculation. Assigned to zero at the beginning.
 
																					 // This code block sets the base speed of the motors.
	Controller-> right_motor_speedBase = 50; // Base speed defined for the right motor.
	Controller-> left_motor_speedBase = 50; // 	Base speed defined for the left motor.
	Controller-> Var_ARR = 10;							
  Controller-> last_error = 0;            // It stores the last calculated error. Could be changed by hardware.

  Controller-> motor_speed= 0;            // Current motor speed. It is adjustable.
	Controller-> variable = 0;							// Additional variable.
  Controller-> actv_cnt =0;								// Active count.
	Controller-> last_end=0;								// Last end position for the sensors.
  Controller->  counted_white=0;          // Counter used to determine and collect white line detection.
	
	// This for loop is used for the initialization of the error data array to use it later.
 for (int i = 0; i < 10; ++i) {
    Controller->error_data[i] = 0;  // The array is defined to store recent errors.
}
  // This for loop is used for the initialization of the infos array to use it later.
  for (int abc = 0; abc < 10; ++abc) {
    Controller->infos[abc] = 0;     // The array is defined to store sensor information and status.
}
	
}
void GPIO_configuration (void)
{
	RCC->APB2ENR |= (1<<2); // Clock activation is enabled for GPIOA
	RCC->APB2ENR |=	(1<<3); // Clock activation is enabled for GPIOA
	
	GPIOA->CRL &= ~(0xffffffff); 	// Clear all configuration bits for GPIOA pin 0,1,2,3,4,5,6, and 7.
	GPIOB->CRL &= ~(0xff); 				// Clear configuration bits for GPIOB pin 0, and 1.
	GPIOB->CRH &= ~(0xffffffff);	// Clear configuration bits for GPIOB pin 8,9,10,11,12,13,14 and 15.
	
	GPIOB->CRL |= 0x000000BB; // PB0 pin is set as alternate function push-pull with a max speed of 50 MHz.
  GPIOB->CRH |= 0x0000BB00; // PB1 pin is set as alternate function push-pull with a max speed of 50 MHz.

// PB14 pin is set as input with Pull-Down configuration
	
GPIOB->CRH &= ~(0xF << (4 * 6));   // Clear bits for PB14 (4 bits per pin, pin 14 is in CRH)
GPIOB->CRH |= (0x8 << (4 * 6));    // Set CNF[2:1] = 10 for input mode with pull-down
GPIOB->ODR &= ~(1 << 14);           // Enable pull-down by setting the corresponding ODR bit to 0.

}

void motor_config (int left_motor_speed, int right_motor_speed) // This function adjusts the motor's pwm duty cycles to adjust speed.
{
		if(left_motor_speed < 0) // The if statement checks if the left motor speed is negative.
	{
	  right_sensor ((Controller->Var_ARR)*0,-1*(Controller->Var_ARR)*left_motor_speed); // Reverse the left motor by setting the appropriate sensor signals.
	}
	else
	{
	  right_sensor ((Controller->Var_ARR)*left_motor_speed,(Controller->Var_ARR)*0);	// Move the left motor forward by setting the appropriate sensor signals
	}
	if(right_motor_speed < 0) // The if statement checks if the right motor speed is negative.
	{
		left_sensor ((Controller->Var_ARR)*0,-1*(Controller->Var_ARR)*right_motor_speed);  // The statement reverses the right motor by setting the appropriate sensor signals.
	}
	else
	{
		left_sensor ((Controller->Var_ARR)*right_motor_speed,(Controller->Var_ARR)*0); // Move the right motor forward by setting the appropriate sensor signals
	}

}

void sensor_configuration (void) // GPIO pin configurations assigned for QTR8RC sensor array.
{
	// At first, change the capacitors by writting ODR value as 1.
	GPIOA->CRL |= 0x33333333;  // Set GPIOA pins 0-7 as output with maximum speed (50 MHz), push-pull mode
	GPIOA->CRL &= ~(0xF << (0 * 4)) & ~(0xF << (1 * 4)) & ~(0xF << (2 * 4)) & ~(0xF << (3 * 4)) & ~(0xF << (4 * 4)) & ~(0xF << (5 * 4)) & ~(0xF << (6 * 4)) & ~(0xF << (7 * 4)); // Clear GPIOA pin configurations for pins 0-7
	GPIOA->ODR |= 0xFF;		 // Set GPIOA pins 0-7 to high (logical 1) to charge the capacitors
	
	delay_us(12);	// Wait for a short period (12 microseconds) to ensure capacitors are fully charged
	
	// When the delay in microseconds occured, set the sensors as input to read the data from "Controller_position".
	GPIOA->CRL &= ~(3<<28) & ~(3<<24) & ~(3<<20) & ~(3<<16) & ~(3<<12) & ~(3<<8) & ~(3<<4) & ~(3<<0); // Set GPIOA pins 0-7 as input mode by resetting configuration bits.
	GPIOA->CRL |= (2<<30) | (2<<26) | (2<<22) | (2<<18) | (2<<14) | (2<<10) | (2<<6) | (2<<2);	// Set GPIOA pins 0-7 with input mode using pull-up/pull-down resistors 
	delay_ms(6);	// Wait for a short period (6 milliseconds) to allow the sensor readings to stabilize
	
}

int Sensor_Read() // With this function, each sensor is checked one by one to determine if the sensors detects the line as logical high.
{	
	sensor_configuration(); // The function configures GPIO pins for the QTR8RC sensor array.
    int real_position = 0;	// Variable to store the weighted sum of active sensor positions
		int real_var = 0;				 // Additional variable used in Controller.h . 
    int real_actv = 0;				// Counter for the number of active sensors detecting the line
	
		// Check each sensor's digital input and update the position, active count, and sensor state for line detection logic.
		if(((GPIOA->IDR & (1<<0))==(1<<0)))  // Sensor 1
			{
				sensor_read_value = 0x00000001;	
				real_position+=1000;
				real_actv++;
				Controller-> last_end=1; // Means robot tend to right
			}
		if(((GPIOA->IDR & (1<<1))==(1<<1))) // Sensor 2
			{
				sensor_read_value = 0x00000010;	
        real_position+=2000;
				real_actv++;
			}	
		if(((GPIOA->IDR & (1<<2))==(1<<2)))  // Sensor 3
			{	
				sensor_read_value = 0x00000100;	
				real_position+=3000;
				real_actv++;
			}
		if(((GPIOA->IDR & (1<<3))==(1<<3)))  // Sensor 4
			{	
				sensor_read_value = 0x00001000;	
				real_position+=4000;
				real_actv++;
			}
		if(((GPIOA->IDR & (1<<4))==(1<<4)))  // Sensor 5
			{	
				sensor_read_value = 0x00010000;	
				real_position+=5000;
				real_actv++;
			}
		if(((GPIOA->IDR & (1<<5))==(1<<5)))  // Sensor 6
			{	
				sensor_read_value = 0x00100000;	
        real_position+=6000;
				real_actv++;
			}
		if(((GPIOA->IDR & (1<<6))==(1<<6)))  // Sensor 7
			{	
				sensor_read_value = 0x01000000;	
				real_position+=7000;
				real_actv++;
			}
		if(((GPIOA->IDR & (1<<7))==(1<<7)))  // Sensor 8
			{	
				sensor_read_value = 0x10000000;	
				real_position+=8000;
				real_actv++;
				Controller->last_end=0;		// Means robot tend to left
			}
			
			// This code block stores individual sensor states into the Controller's info array. 
			
		Controller->infos[0] = (GPIOA->IDR & (1<<0));
		Controller->infos[1] = (GPIOA->IDR & (1<<1));
		Controller->infos[2] = (GPIOA->IDR & (1<<2));
		Controller->infos[3] = (GPIOA->IDR & (1<<3));
		Controller->infos[4] = (GPIOA->IDR & (1<<4));
		Controller->infos[5] = (GPIOA->IDR & (1<<5));
		Controller->infos[6] = (GPIOA->IDR & (1<<6));
		Controller->infos[7] = (GPIOA->IDR & (1<<7));	
			
			// Calculate the average position value based on active sensors
			Controller->pid_position = real_position/real_actv;
      Controller->actv_cnt = real_actv;
			Controller->variable = real_var;			
			
			// Check if no sensors detect the black line.
		if (Controller->actv_cnt == 0) // No black line detecting 
		{
			Controller->counted_white++;
		}
		else
		{
			Controller->counted_white = 0;
		}
			return real_position/real_actv; // Return the calculated position value
}

void right_pwmcontrol (void) // PWM Configuration for TIM2
{
 RCC->APB2ENR |= (1 << 3) | (1 << 0); // Enable clocks for GPIOB and AFIO (Alternate Function I/O)
 RCC->APB1ENR |= (1 << 0);// Enable clock for TIM2.
 AFIO->MAPR |= (1 << 8) | (1 << 9); // Full remap for TIM2
 TIM2->CCER |= (1 << 12);	// Enable capture/compare for channel 4 (CC4E)
 TIM2->CCER |= (1 << 8);	// Enable capture/compare for channel 3 (CC3E)
 TIM2->CR1 |= (1 << 7); // Auto-reload preload (ARPE) enable (ARPE)
 TIM2->CCMR2 |= (1 << 13) | (1 << 14) | (1 << 11); // Configure channel 4 for PWM mode and enable preload
 TIM2->CCMR2 |= (1 << 5) | (1 << 6) | (1 << 3); // Configure channel 3 for PWM mode and enable preload

 

 TIM2->PSC = 72-1; // Prescaler: Divide 72 MHz by 72
 TIM2->ARR = 1000; // Auto-reload: Set ARR to 1000, resulting in a PWM frequency of 1 kHz

}

void left_pwmcontrol (void) // PWM Configuration for TIM3
{
 RCC->APB2ENR |= (1 << 3) | (1 << 0); // Enable GPIOB (IOPBEN) and AFIO (AFIOEN)
 RCC->APB1ENR |= (1 << 1); // Enable clock for TIM3 (TIM3EN)
 
 TIM3->CCER |= (1 << 12);	// Enable capture/compare for channel 4 (CC4E)
 TIM3->CCER |= (1 << 8);	 // Enable capture/compare for channel 3 (CC3E)
 TIM3->CR1 |= (1 << 7); //Auto-reload preload (ARPE) enable
 TIM3->CCMR2 |= (1 << 13) | (1 << 14) | (1 << 11); // Configure channel 4 for PWM mode and enable preload
 TIM3->CCMR2 |= (1 << 5) | (1 << 6) | (1 << 3); // Configure channel 3 for PWM mode and enable preload



 TIM3->PSC = 72-1; // Prescaler: Divide 72 MHz by 72
 TIM3->ARR = 1000; // Auto-reload: Set ARR to 1000, resulting in a PWM frequency of 1 kHz
 
}

void left_sensor (int T_2_3,int T_2_4)
{

TIM2->CCR3= T_2_3; // Update capture/compare register 3 (Channel 3) for TIM2 with the duty cycle value
TIM2->CCR4= T_2_4; // Update capture/compare register 4 (Channel 4) for TIM2 with the duty cycle value
TIM2->EGR |= (1 << 0); // UG: Update generation, reinitialize the timer and apply new values
TIM2->CR1 |= (1 << 0); // CEN: Counter enable, starts the timer
}

void right_sensor (int T_3_3,int T_3_4)
{

TIM3->CCR4= T_3_4; // Update capture/compare register 4 (Channel 4) for TIM3 with the duty cycle value
TIM3->CCR3= T_3_3; // Update capture/compare register 3 (Channel 3) for TIM3 with the duty cycle value
TIM3->EGR |= (1 << 0); // UG: Update generation, reinitialize the timer and apply new values
TIM3->CR1 |= (1 << 0); // CEN: Counter enable, starts the timer
}



void sharp_turn_handle() {	// The function detects and handles sharp turns.
	
	if (Controller->counted_white < 25) // Check if the number of white line counts is less than 25 to confirm no black line detection
	{
		if (Controller->last_end == 1){  // Check the last direction indicated by the sensors
			// Robot tends to the right, adjust motor speeds for a left turn
			left_motor_speed=15;
			right_motor_speed=-25;
			motor_config(left_motor_speed, right_motor_speed); 
		}
		else if(Controller->last_end==0)
		{
			 // Robot tends to the left, adjust motor speeds for a right turn
			left_motor_speed=-25;
			right_motor_speed=15;
			motor_config(left_motor_speed, right_motor_speed);
		}
		 else;
	}
	else 
	{
		if (Controller->last_end == 1){
			// Robot tends to the right, execute a sharper left turn
			left_motor_speed=70;
			right_motor_speed=-50;
		}
		else
		{
			// Robot tends to the left, execute a sharper right turn
			left_motor_speed=-70;
			right_motor_speed=50;
		}
	}
}



void pst_error (int error)  // The function stores the last error for use in Integral Gain
{
  for (int aa = 9; aa > 0; aa--) { // Shift the stored error data array to make room for the new error
      Controller->error_data[aa] = Controller->error_data[aa-1];
      Controller->error_data[0] = error;
	}
}

int err_sums (int index)  // The function sums the stored errors to calculate Integral Gain
{
  int sum = 0; // Initialize the sum variable
  for (Controller->k = 0; Controller->k < index; Controller->k ++)  // Iterate over the error data array up to the specified index
  {
      sum +=Controller->error_data[Controller->k];
  }
  return sum; // Return the calculated sum of errors
}

void Controller_control_logic()  // The function aims the controller logic for motor control based on PID.
	{
		// Read the current sensor position
	Controller->pid_position = Sensor_Read();	
		// Calculate the error (difference between reference and current position)
  int error = 4500 - (Controller->pid_position); // Reference is 4500 which can be evaluated is the average of two middle sensor
	pst_error(error); // Store the error for integral calculation

		// With this code block, it calculates the PID components
  Controller->proportional = error;
  Controller->integrational = err_sums(5);  // Integral term (sum of the last 5 errors)
  Controller->Derivative = error - Controller->last_error; // Derivative term (rate of error change)

  Controller->last_error = error; // Update the last error for future derivative calculations
	
		// This code line calculates the motor speed adjustment using the PID formula
  Controller->motor_speed = (Controller->proportional)*(Controller->Kp) + (Controller->integrational)*(Controller->Ki) + (Controller->Derivative)*(Controller->Kd);
  
  left_motor_speed = Controller->left_motor_speedBase + Controller->motor_speed; // This line Adjusts the left motor speed based on the PID controller's output
  int right_motor_speed = Controller->right_motor_speedBase - Controller->motor_speed; // Control the right motor speed by using the PID Controller values of the system.
  
  if (left_motor_speed > left_motor_speedMax) // Limit the left motor speed to its maximum allowed value
	{
    left_motor_speed = left_motor_speedMax;
	}
  if (right_motor_speed > right_motor_speedMax) // Limit the right motor speed to its maximum allowed value
	{
    right_motor_speed = right_motor_speedMax;
	}
	
	// If no active sensors are detecting the line (can be considered as the end of line condition)
	if (Controller->actv_cnt==0) // Handle the sharp turn to relocate the line
	{
		sharp_turn_handle();
	}

	motor_config(left_motor_speed, right_motor_speed); // Configure the motors with the calculated speeds
}



int main()
{	
  General_Init();
	initClockPLL();
	GPIO_configuration();
	TIM2Config();
	right_pwmcontrol ();
  left_pwmcontrol();
	    uint32_t distance; // Variable to store the distance measured by the sensor

    GPIO_config();
    Timer_config();

while(1) // Infinite loop to control the robot
{
	distance = Read_distance(); // This line measures the distance by using the hc sr 04 ultrasonic sensor.
	 if (distance <= 20) { // Check if the distance is below or equal to 20 cm.
		 // Stop the motors if an obstacle is detected within 20 units
						left_motor_speed = 0;
            right_motor_speed = 0;
            motor_config(left_motor_speed, right_motor_speed); // Apply the stop command
        } else {

            Controller_control_logic();  // Otherwise, run the main control logic by applying original line following using PID controller.
        }
	 	
}
return 0; // Program ends.
}

// GPIO Configuration for Ultrasonic Sensor (TRIG and ECHO)
void GPIO_config(void) {
        
    RCC->APB2ENR |= (1 << 3); // Enable clock for GPIOB

    // This code block configures PB13 (TRIG) pin as Output, with Push-Pull configuration with 50 MHz output.
    GPIOB->CRH &= ~(0xF << (4 * 5));   // Clear the configuration bits for PB13
    GPIOB->CRH |= (0x3 << (4 * 5));    // Mode: Output 50MHz
    GPIOB->CRH |= (0x0 << (4 * 5 + 2)); // CNF: General-purpose output push-pull

    // Configure PB14 (ECHO) as Input, Pull-Down
    GPIOB->CRH &= ~(0xF << (4 * 6));   // Clear the configuration bits for PB14
    GPIOB->CRH |= (0x8 << (4 * 6));    // CNF: Input with pull-down
    GPIOB->ODR &= ~(1 << 14);          // Pull-down etkinlestir
}

// Timer (TIM4) Configuration
void Timer_config(void) {
    // Enable clock activation for TIM4.
    RCC->APB1ENR |= (1 << 2); // This code line enables TIM4 clock activation.

    // Prescaler and period configurations.
    TIM4->PSC = 72 - 1; // 72 MHz / 72 = 1 MHz (1 µs per tick)
    TIM4->ARR = 0xFFFF; // Max timer value that will counted by the timer.
    TIM4->CR1 |= (1 << 0); // Enable the timer.
}

// Microsecond delay function.
void delay_microseconds(uint32_t us) {
    TIM4->CNT = 0; // Sayaç sifirla
    while (TIM4->CNT < us); // Belirtilen mikro saniyeye kadar bekle
}

// Milisecond delay function.
void delay_miliseconds(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000); // 1 ms = 1000 µs
    }
}

// Distance measuring function.
	uint32_t Read_distance(void) {
    uint32_t start, end; // Variables to store the timer counts at the start and end of the echo pulse


    // With this code block, set the TRIG pin HIGH for 10µs to send a trigger pulse
    GPIOB->BSRR = (1 << 13); // set PB13 HIGH for TRIG
    delay_us(10); // Wait for 10 us.
    GPIOB->BRR = (1 << 13); // set PB13 LOW for TRIG

    // Wait for the ECHO pin to go HIGH (start of echo pulse)
    while (!(GPIOB->IDR & (1 << 14))); // Loop until PB14 (ECHO) reads HIGH
    start = TIM4->CNT;  // Record the timer count at the start of the echo pulse

    // Wait for the ECHO pin to go LOW (end of echo pulse)
    while (GPIOB->IDR & (1 << 14)); // Loop until PB14 (ECHO) reads LOW
    end = TIM4->CNT;                // Record the timer count at the end of the echo pulse

    // Calculate the duration of the echo pulse and convert it to distance in cm
    uint32_t time = end - start; // Calculate the time duration in timer counts
    return (time * 0.0343) / 2; // Convert time to distance (cm), using speed of sound (343 m/s)
}
