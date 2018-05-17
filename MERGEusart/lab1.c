/*
 * lab1.c
 */

#include "usart.h"

#include <avr/io.h>
#include <util/delay.h>





void exemplu_usart()
{
    for(;;)
    {
        USART0_print("Salut!\n\r");
        _delay_ms(10);
    }
}




int main()
{
    USART0_init();

    exemplu_usart();
    
    return 0;
}

