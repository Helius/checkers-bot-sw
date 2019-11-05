#include <stdio.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>
#include <stdlib.h>

#include <uart.h>
#include <misc.h>

void togglePin(void)
{
	// toggle pin
	PORTB ^= (1 << PIN5);
	// wait for 250ms
	_delay_ms(250);
}


void uart_rx_handler(unsigned char ch)
{
	togglePin();
}

int main(void)
{
	
	uart_init();
	set_receive_interrupt_handler(&uart_rx_handler);
	msg("hello world, how are you?\n\r");
	
	DDRB |= (1 << PIN5);

	togglePin();

	// delay for enabling bluetooth board power
	// allow to program board via serial port
	_delay_ms(3000);

	sei();

	while(1)
	{
		togglePin();
	}
}

