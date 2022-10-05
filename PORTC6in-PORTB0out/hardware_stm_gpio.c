/**
  ******************************************************************************
  * @file    hardware_stm_gpio.c 
  * @author  mortamar@andrew.cmu.edu
  * @version 1.0
  * @date    Septembr-2021
  * @brief   Controls STM32F446ze GPIO
  ******************************************************************************
  */

#include "hardware_stm_gpio.h"
#include "stm32f4xx_rcc_mort.h"

//led 1 is connected to PB0. 
// GPIO B addresses: 0x4002 0400 - 0x4002 07FF
// GPIO C addresses: 0x4002 0800 - 0x4002 0BFF


/* MACRO definitions----------------------------------------------------------*/
//Port B addresses:
#define PORTB_BASE_ADDRESS ((uint32_t)0x40020400)        //The first address in memory corresponding to Port B (this is in the user manual!)
// I gave you the first one, now you fill in the rest, check in the user manual what is the offset from the base address for each register!
#define PORTB_MODER_REGISTER (PORTB_BASE_ADDRESS + 0x00) //replace the question mark with the correct offset!
#define PORTB_OTYPER_REGISTER (PORTB_BASE_ADDRESS + 0x04)
#define PORTB_OSPEEDR_REGISTER (PORTB_BASE_ADDRESS + 0x08)
#define PORTB_PUPDR_REGISTER (PORTB_BASE_ADDRESS + 0x0C)
#define PORTB_IDR_REGISTER (PORTB_BASE_ADDRESS + 0x10)
#define PORTB_ODR_REGISTER (PORTB_BASE_ADDRESS + 0x14)
#define PORTB_BSRRL_REGISTER (PORTB_BASE_ADDRESS + 0x18)
#define PORTB_BSRR_REGISTER (PORTB_BASE_ADDRESS + 0x18)
#define PORTB_BSRRH_REGISTER (PORTB_BASE_ADDRESS + 0x1A)
#define PORTB_LCKR_REGISTER (PORTB_BASE_ADDRESS + 0x1C)
#define PORTB_AFR1_REGISTER (PORTB_BASE_ADDRESS + 0x20)
#define PORTB_AFR2_REGISTER (PORTB_BASE_ADDRESS + 0x24)
#define PORTB_OSPEEDR_REGISTER (PORTB_BASE_ADDRESS + 0x08)
//Port C addresses:

//flags MODER Register:

//flags OTYPER Register:

//flags OSPEEDR Register:

//flags PUPDR Register:

//input data register:

//flags AFR1 Register:

//flags ODR Register:


/* function definitions----------------------------------------------------------*/

void initGpioC6AsInput( void )
{
    /* GPIOC Peripheral clock enable */
    RCC -> AHB1ENR |= (1<<2);
    /* GPIOC Pin 6 as input*/
    GPIOC -> MODER |= (0<<12) | (0<<13);
    /*PUSH-PULL Pin*/
    GPIOC -> OTYPER |= (0<<6);
    /*GPIOC pin 6 high speed */
    GPIOC -> OSPEEDR |= (1<<12) | (1<<13);
    /*Configure pulled-down*/
    GPIOC -> PUPDR |= (0<<12) | (1<<13);

}


void initGpioB0AsOutput( void )
{
    /* GPIOB Peripheral clock enable */
    RCC -> AHB1ENR |= (1<<1);
    /* GPIOB0 configured as output */
    GPIOB -> MODER |= 0x00000001;
    /*GPIOB0 configured as push-pull */
    GPIOB -> OTYPER |= 0x00000000;
    /*GPIOB0 configured floating */
    GPIOB -> OSPEEDR |= 0x00000003;
    /* No pull up pull down */
    //GPIOB -> PUPDR |= 0x00000000; //Optional for Output
    /* GPIOB0 driven high to start out with */
    GPIOB -> ODR |= 0x00000001;

}



void toggleGPIOB0( void )
{
    uint32_t value;
    uint32_t  *reg_pointer;
    //get the current value of the pin 
    reg_pointer = (uint32_t *)PORTB_ODR_REGISTER; 
    value = *reg_pointer & 0b01;

    if (value > 0)
    {
        //if high, clear the bit
        clearGPIOB0();
        
    }
    else
    {
        //if low, set the bit
        setGPIOB0();
    } 
}



void setGPIOB0( void )
{
    GPIOB -> ODR |= 0x00000001;
}
void clearGPIOB0( void )
{
    GPIOB -> ODR &= ~0x00000001;
}

void checkGPIOC6(void)
{

    if(GPIOC -> IDR & (1<<6))
    {
        setGPIOB0();
    }

    else 
    {
        clearGPIOB0();
    }  
}