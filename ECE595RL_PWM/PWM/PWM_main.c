/**
 * @file PWM_main.c
 * @brief Main source code for the PWM program.
 *
 * This file contains the main entry point for the PWM program.
 * It uses Timer A0 to generate PWM signals that will be used to drive the DC motors.
 * Then, it uses the edge-triggered interrupts from the bump sensors to detect a collision,
 * which should immediately stop the motors from running.
 *
 * Timer A1 is used to generate periodic interrupts at a rate of 10 Hz, while Timer A2
 * is used to generate PWM signals to drive two servos.
 *
 * @author Michael Granberry, Abdullah Hendy, Aaron Nanas
 *
 */

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/GPIO.h"
#include "../inc/Bumper_Sensors.h"
#include "../inc/EUSCI_A0_UART.h"
#include "../inc/Timer_A0_PWM.h"
#include "../inc/Timer_A1_Interrupt.h"
#include "../inc/Timer_A2_PWM.h"
#include "../inc/Motor.h"

// Global variable used to store the current state of the bumper sensors when an interrupt
// occurs (Bumper_Sensors_Handler). It will get updated on each interrupt event.
uint8_t bumper_sensor_value;

// Global variable that gets set in Bumper_Sensors_Handler.
// This is used to detect if any collisions occurred
uint8_t collision_detected = 0;


/**
 * @brief Bumper sensor interrupt handler function.
 *
 * This is the interrupt handler for the bumper sensor interrupts. It is called when a falling edge event is detected on
 * any of the bumper sensor pins. The function checks if a collision has already been detected; if not, it prints a collision
 * detection message along with the bumper sensor state and sets a collision flag to prevent further detections.
 *
 * @param bumper_sensor_state An 8-bit unsigned integer representing the bump sensor states at the time of the interrupt.
 *
 * @return None
 */
void Bumper_Sensors_Handler(uint8_t bumper_sensor_state)
{
    if (collision_detected == 0)
    {
        printf("Collision Detected! Bumper Sensor State: 0x%02X\n", bumper_sensor_state);
        collision_detected = 1;
    }
}

/**
 * @brief User-defined function executed by Timer A1 using a periodic interrupt at a rate of 10 Hz.
 *
 * This task is executed by Timer A1 every time a periodic interrupt occurs at a rate of 10 Hz.
 * When an interrupt occurs and a collision has not been detected, it turns off the
 * back red LEDs and toggles the front yellow LEDs. But if a collision has been detected,
 * it turns off the front yellow LEDs and toggles the back red LEDs.
 *
 * @return None
 */
void Timer_A1_10_Hz_Task(void)
{
    if (collision_detected == 0)
    {
        P8->OUT ^= 0x21;
        P8->OUT &= ~0xC0;
    }
    else
    {
        P8->OUT ^= 0xC0;
        P8->OUT &= ~0x21;
    }
}

/**
 * @brief Execute a predefined drive pattern using the motors.
 *
 * This function executes a predefined drive pattern using the motors. It involves a sequence of motor commands
 * to create specific movements. The sequence consists of:
 *
 * 1. Setting both motors to move forward with a 50% duty cycle for a duration of 2 seconds.
 * 2. Stopping the motors for 2 seconds.
 * 3. Setting both motors to move left with a 30% duty cycle for 2 seconds.
 * 4. Stopping the motors for 2 seconds.
 * 5. Setting both motors to move right with a 30% duty cycle for 2 seconds.
 * 6. Stopping the motors for 2 seconds.
 * 7. Setting both motors to move backward with a 30% duty cycle for 2 seconds.
 * 8. Stopping the motors for 2 seconds.
 *
 * @note This function assumes that the Motor_Forward, Motor_Left, Motor_Right, Motor_Backward,
 * and Motor_Stop functions are properly implemented and configured.
 *
 * @note The Clock_Delay1ms function is used to introduce delays between motor actions.
 *
 * @return None
 */
void Drive_Pattern_1()
{
    // Set PWM to 50% Duty Cycle
    Motor_Forward(7500, 7500);
    Clock_Delay1ms(2000);

    // Stop the motors
    Motor_Stop();
    Clock_Delay1ms(2000);

    // Set PWM to 30% Duty Cycle
    Motor_Left(4500, 4500);
    Clock_Delay1ms(2000);

    // Stop the motors
    Motor_Stop();
    Clock_Delay1ms(2000);

    // Set PWM to 30% Duty Cycle
    Motor_Right(4500, 4500);
    Clock_Delay1ms(2000);

    // Stop the motors
    Motor_Stop();
    Clock_Delay1ms(2000);

    // Set PWM to 30% Duty Cycle
    Motor_Backward(4500, 4500);
    Clock_Delay1ms(2000);

    // Stop the motors
    Motor_Stop();
    Clock_Delay1ms(2000);
}

void Handle_Collision()
{
    // Stop the motors
    Motor_Stop();

    // Make a function call to Clock_Delay1ms(2000)
    Clock_Delay1ms(2000);

    // Move the motors backward with 30% duty cycle
    Motor_Backward(4500, 4500);

    // Make a function call to Clock_Delay1ms(3000)
    Clock_Delay1ms(3000);

    // Stop the motors
    Motor_Stop();

    // Make a function call to Clock_Delay1ms(1000)
    Clock_Delay1ms(1000);

    // Make the robot turn to the right with 10% duty cycle
    Motor_Right(1500, 1500);

    // Make a function call to Clock_Delay1ms(5000)
    Clock_Delay1ms(5000);

    // Stop the motors
    Motor_Stop();

    // Make a function call to Clock_Delay1ms(2000)
    Clock_Delay1ms(2000);

    // Set the collision_detected flag to 0
    collision_detected = 0;
}

int main(void)
{
    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED
    LED1_Init();
    LED2_Init();

    // Initialize the front and back LEDs
    P8_Init();

    // Initialize the buttons
    Buttons_Init();

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();

    // Initialize the bumper sensors which will be used to generate external I/O-triggered interrupts
    Bumper_Sensors_Init(&Bumper_Sensors_Handler);

    // Initialize Timer A1 with interrupts enabled
    // Default frequency is set to 10 Hz
    Timer_A1_Interrupt_Init(&Timer_A1_10_Hz_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initialize Timer A2 with a period of 50 Hz
    // Timer A2 is used to drive two servos
    Timer_A2_PWM_Init(60000, 0, 0);

    // Initialize the motors
    Motor_Init();

    // Initialize collision_detected flag
    collision_detected = 0;

    // Enable the interrupts used by the bumper sensors and Timer A1
    EnableInterrupts();

    while(1)
    {
//        Drive_Pattern_1();

        // Rotate to 0
        Timer_A2_Update_Duty_Cycle_1(1700);
        Timer_A2_Update_Duty_Cycle_2(1700);
        LED2_Output(RGB_LED_RED);
        Clock_Delay1ms(5000);

        // Rotate to 180
        Timer_A2_Update_Duty_Cycle_1(7000);
        Timer_A2_Update_Duty_Cycle_2(7000);
        LED2_Output(RGB_LED_BLUE);
        Clock_Delay1ms(5000);

//        if (collision_detected == 0)
//        {
//            // Move forward for an indefinite amount of time with 50% duty cycle
//            Motor_Forward(7500, 7500);
//        }
//        else
//        {
//            Handle_Collision();
//        }
    }
}
