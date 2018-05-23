#include "usart.h"
#include <stdlib.h>
/*
 * Functie de initializare a controllerului USART
 */
void USART0_init()
{
    /* seteaza baud rate la 9600 */
    UBRR0H = 0;
    UBRR0L = 103;

    /* porneste transmitatorul */
    UCSR0B = (1<<TXEN0) | (1<<RXEN0);

    /* seteaza formatul frame-ului: 8 biti de date, 1 biti de stop, paritate fara */
    UCSR0C &= ~(1<<USBS0);
    UCSR0C |= (2<<UCSZ00);
    UCSR0C &= ~(3<<UPM00);
}

/*
 * Functie ce transmite un caracter prin USART
 *
 * @param data - caracterul de transmis
 */
void USART0_transmit(char data)
{
    /* asteapta pana bufferul e gol */
    while(!(UCSR0A & (1<<UDRE0)));

    /* pune datele in buffer; transmisia va porni automat in urma scrierii */
    UDR0 = data;
}

/*
 * Functie ce primeste un caracter prin USART
 *
 * @return - caracterul primit
 */
char USART0_receive()
{
    /* asteapta cat timp bufferul e gol */
    while(!(UCSR0A & (1<<RXC0)));

    /* returneaza datele din buffer */
    return UDR0;
}


double USART0_receive_own()
{
    char x[10];
    int i;
    char temp;
    for (i=0;i<9;i++){
        temp = USART0_receive();
        if (temp == '\r'){
            temp = USART0_receive();
            if (temp == '\n'){
                break;
            }
        }
        x[i] = temp;
    }
    x[i] = 0;
    double fin = atof(x);
    return fin;
}

/*
 * Functie ce transmite un sir de caractere prin USART
 *
 * @param data - sirul (terminat cu '\0') de transmis
 */
void USART0_print(const char *data)
{
    while(*data != '\0')
        USART0_transmit(*data++);
}

/*
 * Functie ce transmite un caracter prin USART
 *
 * @param data - caracterul de transmis
 * @param stream - nefolosit
 *
 * @return - intotdeauna returneaza 0
 */
int USART0_printf(char data, FILE *stream)
{
    /* asteapta pana bufferul e gol */
    while(!(UCSR0A & (1<<UDRE0)));

    /* pune datele in buffer; transmisia va porni automat in urma scrierii */
    UDR0 = data;
    return 0;
}
