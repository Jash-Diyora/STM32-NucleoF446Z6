/**
  ******************************************************************************
  * @file    hardware_stm_gpio.c 
  * @author  jdiyora@andrew.cmu.edu
  * @version 1.0
  * @date    Oct-2022
  * @brief   Controls STM32F446ze GPIO
  ******************************************************************************
  */

#include "hardware_stm_gpio.h"
//#include "mbed2/299/TARGET_NUCLEO_F446ZE/TARGET_STM/TARGET_STM32F4/TARGET_NUCLEO_F446ZE/device/stm32f446xx.h"
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
      uint32_t  * reg_pointer; 
      /* GPIOC Peripheral clock enable */
   
    /* GPIOC Pin 6 as input*/
    
    /*PUSH-PULL Pin*/
    
    /*GPIOC pin 6 high speed */
    
    /*Configure pulled-down*/
    
}


void initGpioB0AsOutput( void )
{
    
    //   uint32_t  * reg_pointer;
    //   /* GPIOB Peripheral clock enable */
    //   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    // /* GPIOB0 configured as output */
    //   reg_pointer = (uint32_t *)PORTB_MODER_REGISTER;
    //   *reg_pointer = *reg_pointer & (~((uint32_t) GPIO_0_MODER));
    //   *reg_pointer = *reg_pointer| GPIO_0_MODER_OUT;
    
    // /*GPIOB0 configured as push-pull */
    
    // /*GPIOB0 configured floating */
    
    // /* GPIOB0 driven high to start out with */
    



    /* GPIOB Peripheral clock enable */
    RCC -> AHB1ENR |= (1<<1);
    /* GPIOB0 configured as output */
    GPIOB -> MODER |= 0x00000001;
    /*GPIOB0 configured as push-pull */
    GPIOB -> OTYPER |= 0x00000000;
    /*GPIOB0 configured floating */
    GPIOB -> OSPEEDR |= 0x00000003;
    /* No pull up pull down */
    //GPIOB -> PUPDR = 0x00000000;
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

uint32_t checkGPIOC6(void)
{
 uint32_t valueC6;

 return valueC6;   
}
