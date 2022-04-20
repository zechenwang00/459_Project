#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include "lib/ssd1306.h"

/*
serial_init - Initialize the USART port
*/
void serial_init ( unsigned short ubrr ) {
	UBRR0 = ubrr ; // Set baud rate
	UCSR0B |= (1 << TXEN0 ); // Turn on transmitter
	UCSR0B |= (1 << RXEN0 ); // Turn on receiver
	UCSR0C = (3 << UCSZ00 ); // Set for async . operation , no parity ,
	// one stop bit , 8 data bits
}

/*
serial_out - Output a byte to the USART0 port
*/
void serial_out ( char ch )
{
	while (( UCSR0A & (1 << UDRE0 )) == 0);
	UDR0 = ch ;
}

/*
serial_in - Read a byte from the USART0 and return it
*/
char serial_in ()
{
	while ( !( UCSR0A & (1 << RXC0 )) );
	return UDR0 ;
}


int main(void)
{
	// initialize
	serial_init(47);
	char message;

    uint8_t addr = SSD1306_ADDRESS;

    // init ssd1306
    SSD1306_Init (addr);
    SSD1306_ClearScreen ();

	while(1)
	{
		message = serial_in();

        SSD1306_SetPosition(0,1);
        SSD1306_DrawString(message);
        SSD1306_UpdateScreen(addr);
        _delay_ms(50);
	}
}


// ref: https://ece-classes.usc.edu/ee459/library/documents/Serial_Port.pdf