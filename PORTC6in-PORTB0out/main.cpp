#include "main.h"
#include "debug_mort.h"
#include "nucleo_led.h"

// int main (void) //LED Blink PORTB0
// {
//     /* Initializations */
//     init_LED1(); //uncomment once you have filled in the function
//     while(1)
//     {
      
//       toggle_LED1(); //uncomment once you have filled in the function
//       debugprintHelloWorld();
    
//     }
// }

int main (void) //LED Indicator PORTC6 -> PORTB0
{
    /* Initializations */
    init_LED1(); //uncomment once you have filled in the function
    init_InputPC6();
    while(1)
    {
        checkstatusPC6();
    }
}