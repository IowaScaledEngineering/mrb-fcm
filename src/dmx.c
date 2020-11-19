#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "dmx.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#undef BAUD
#define BAUD 250000
#include <util/setbaud.h>

uint8_t dmx_channels[MAX_DMX_CHANNELS];
uint16_t dmx_TxIndex = 0;

void dmx_initialize()
{
	PORTD |= _BV(PD3);
	DDRD &= ~(_BV(PD3) | _BV(PD2));
	PRR0 &= ~_BV(PRUSART1);
	UBRR1 = UBRR_VALUE;
	UCSR1A = (USE_2X)?_BV(U2X1):0;
	UCSR1B = 0;
	UCSR1C = _BV(USBS1) | _BV(UCSZ11) | _BV(UCSZ10); // 8 data bits, 2 stop bit

	memset(&dmx_channels, 0, MAX_DMX_CHANNELS);
	dmx_TxIndex = 0;
}

#undef BAUD

bool dmx_isSending()
{
	if (UCSR1B & _BV(UDRIE1))
		return true; // Still running the last one
	return false;
}

ISR(USART1_UDRE_vect)
{
	UDR1 = dmx_channels[dmx_TxIndex++];  //  Get next byte and write to UART

	if (dmx_TxIndex >= MAX_DMX_CHANNELS)
	{
		//  Done sending data to UART, disable UART interrupt
		UCSR1B &= ~_BV(UDRIE1);
		dmx_TxIndex = 0;
	}
}

void dmx_setChannel(uint16_t chanNum, uint8_t level)
{
	if (chanNum >= MAX_DMX_CHANNELS || chanNum == 0)
		return;
	dmx_channels[chanNum-1] = level;
}

void dmx_all_off(void)
{
	memset(&dmx_channels, 0, MAX_DMX_CHANNELS);
}

void dmx_startXmit()
{
	if(dmx_isSending())
		return;

	PORTD |= _BV(PD3);
	DDRD |= _BV(PD3);
	UCSR1B &= ~_BV(TXEN1);
	_delay_us(8);
	PORTD &= ~_BV(PD3);
	_delay_us(92);
	PORTD |= _BV(PD3);
	_delay_us(12);

	dmx_TxIndex = 0;
	UCSR1B |= _BV(TXEN1);
	UDR1 = 0;
	UCSR1B |= _BV(UDRIE1);

}


