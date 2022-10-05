/**
  ******************************************************************************
  * @file    hardware_stm_gpio.c 
  * @author  jdiyora@andrew.cmu.edu
  * @version 1.0
  * @date    Oct-2022
  * @brief   Controls the LED's on the nucleo board
  ******************************************************************************
  */

#include "hardware_stm_gpio.h"
#include "nucleo_led.h"

/************************************
* Initializes LED1 on the nucleo Board which is connected to Port B Pin 0
*************************************/
void init_LED1(void )
{
    // Call something from hardware_stm_gpio   
    initGpioB0AsOutput();
}

void toggle_LED1( void )
{
    // Call something else from hardware_stm_gpio
    toggleGPIOB0();
}

void high_LED1( void )
{
    // LED on
    setGPIOB0();
}

void low_LED1( void )
{
    // LED off
    clearGPIOB0();
}

void init_InputPC6( void )
{
    //input PORTC6
    initGpioB0AsOutput();
}

void checkstatusPC6( void )
{
    //polling
    checkGPIOC6();
}
