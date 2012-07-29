/*************************************************************************
Title:    MRBus Fast Clock Master
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan D. Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "ds1302.h"
#include "lcd.h"

#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbus_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define MRBUS_TX_PKT_READY MRBEE_TX_PKT_READY
#define MRBUS_RX_PKT_READY MRBEE_RX_PKT_READY
#define mrbux_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbusInit mrbeeInit
#define mrbusPacketTransmit mrbeePacketTransmit
#endif

#include "mrbus.h"

extern uint8_t mrbus_activity;
extern uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_state;

uint8_t mrbus_dev_addr = 0;

// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint8_t decisecs;
volatile uint8_t status=0;

#define STATUS_READ_INPUTS 0x01
#define STATUS_FAST_ACTIVE 0x02
#define STATUS_FAST_AMPM   0x04
#define STATUS_REAL_AMPM   0x08

#define FAST_MODE (status & STATUS_FAST_ACTIVE)

/* Scale factor defines the fast time ratio, or the number of minutes that pass to each real minute */
/* Options:

1 = 1:1, 2 = 2:1, 3 = 3:1, 4 = 4:1
6 = 6:1, 8 = 8:1, 10 = 10:1, 12 = 12:1
16 = 16:1, 20 = 20:1, 24 = 24:1, 30 = 30:1
32 = 32:1, 36 = 36:1, 40 = 40:1, 60 = 60:1

*/

uint8_t scaleFactor = 1;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t month;
	uint16_t year;
} TimeData;

TimeData realTime;
TimeData fastTime;

void initTimeData(TimeData* t)
{
	t->seconds = t->minutes = t->hours = 0;
	t->year = 2012;
	t->month = t->day = 1;
}

volatile uint8_t clock_A=0, clock_B=0, debounced_state=0;

uint8_t debounce(uint8_t raw_inputs)
{
  uint8_t delta = raw_inputs ^ debounced_state;   //Find all of the changes
  uint8_t changes;

  clock_A ^= clock_B;                     //Increment the counters
  clock_B  = ~clock_B;

  clock_A &= delta;                       //Reset the counters if no changes
  clock_B &= delta;                       //were detected.

  changes = ~((~delta) | clock_A | clock_B);
  debounced_state ^= changes;
  debounced_state &= 0xFC;
  return(changes & ~(debounced_state));
}


void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR1C = 0;
	TIMSK1 = _BV(TOIE1);
	ticks = 0;
	decisecs = 0;
}

ISR(TIMER1_OVF_vect)
{
	TCNT1 += 0xF3CB;

	if (ticks & 0x01)
		status |= STATUS_READ_INPUTS;

	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
	}
}


void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (mrbus_rx_buffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != mrbus_rx_buffer[MRBUS_PKT_DEST] && mrbus_dev_addr != mrbus_rx_buffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, mrbus_rx_buffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 6;
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'a';
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	} 
	else if ('W' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = mrbus_rx_buffer[7];
		if (MRBUS_EE_DEVICE_ADDR == mrbus_rx_buffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	
	}
	else if ('R' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'r';
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6]);			
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	}

	// FIXME:  Insert code here to handle incoming packets specific
	// to the device.

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	mrbus_state &= (~MRBUS_RX_PKT_READY);
	return;	
}

void init(void)
{
	// FIXME:  Do any initialization you need to do here.
	// Kill watchdog
    MCUSR = 0;
	wdt_reset();
	wdt_disable();

	DDRD &= ~(0xF2);
	DDRC &= ~(0x10);
		
	ds1302_init();
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	
	initTimeData(&realTime);
	initTimeData(&fastTime);
	
	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
}


void drawBigTime(TimeData* t, uint8_t useAMPM)
{
	uint8_t hours = t->hours;
	if (useAMPM)
	{
		if (0 == hours)
			hours = 12;
		else if (hours > 12)
			hours -= 12;
	}

	if (!useAMPM || hours >= 10)
		lcd_putc_big(0, (hours / 10) % 10);
	lcd_putc_big(1, hours % 10);

	lcd_putc_big(2, (t->minutes / 10) % 10);
	lcd_putc_big(3, t->minutes % 10);
}

void drawSplashScreen()
{
	lcd_setup_bigdigits();
	lcd_gotoxy(0,0);
//            00000000001111111111
//            01234567890123456789	
	lcd_gotoxy(0,0);
	lcd_puts("  Fast Clock Master ");
	lcd_gotoxy(0,1);
	lcd_puts("MRBus Enabled   v1.0");
	lcd_gotoxy(0,2);
	lcd_puts("2012 Iowa Scaled Eng");
	lcd_gotoxy(0,3);
	lcd_puts("  www.iascaled.com  ");
	_delay_ms(1000);
	lcd_clrscr();	
}

void drawSoftKeys(char* key1Text, char* key2Text, char* key3Text, char* key4Text)
{
	uint8_t i;

	lcd_gotoxy(0,4);
	for(i=0; i<20; i++)
		lcd_putc(' ');

	lcd_gotoxy(0,3);
	lcd_puts(key1Text);
	lcd_gotoxy(5,3);
	lcd_puts(key2Text);
	lcd_gotoxy(10,3);
	lcd_puts(key3Text);
	lcd_gotoxy(15,3);
	lcd_puts(key4Text);
}

void drawLittleTime(TimeData* t, uint8_t useAMPM)
{
	lcd_gotoxy(10,2);
	
	if (status & STATUS_FAST_AMPM)
		printDec2Dig(t->hours);
	else
		printDec2DigWZero(t->hours);
	lcd_putc(':');
	printDec2DigWZero(t->minutes);
	lcd_putc(':');					
	printDec2DigWZero(t->seconds);
	if (useAMPM)
	{
		if (t->hours < 12)
			lcd_putc('A');
		else
			lcd_putc('P');
		lcd_putc('M');
	}
	else
	{
		lcd_putc(' ');
		lcd_putc(' ');
	}

}



typedef enum
{
	SCREEN_MAIN_DRAW = 0,
	SCREEN_MAIN_IDLE = 1,
	SCREEN_MAIN_UPDATE_TIME = 2,

	SCREEN_CONF_MENU_DRAW = 10,
	SCREEN_CONF_MENU_IDLE = 11,

	SCREEN_CONF_R1224_SETUP = 100,
	SCREEN_CONF_R1224_DRAW = 101,
	SCREEN_CONF_R1224_IDLE = 102,

	SCREEN_CONF_F1224_SETUP = 110,
	SCREEN_CONF_F1224_DRAW = 111,
	SCREEN_CONF_F1224_IDLE = 112,

	
	SCREEN_DONT_KNOW = 255

} ScreenState;

#define SOFTKEY_1 0x10
#define SOFTKEY_2 0x20
#define SOFTKEY_3 0x40
#define SOFTKEY_4 0x80

typedef struct
{
	const char* configName;
	ScreenState configScreen;
} ConfigurationOption;

ConfigurationOption configurationOptions[] = 
{
//   012345678901234
  { "Real 12/24 Ind", SCREEN_CONF_R1224_SETUP },
  { "Real Time     ", 0 },
  { "Real Date     ", 0 },  
  { "Fast 12/24 Ind", SCREEN_CONF_F1224_SETUP },
  { "Fast Ratio     ", 0 },  
  { "Fast Start Time", 0 },
  
};

#define NUM_CONF_OPTIONS  (sizeof(configurationOptions)/sizeof(ConfigurationOption))


int main(void)
{
	uint8_t ds1302Buffer[10];
	uint8_t buttonsPressed=0, colon=0;
	uint8_t idler=0;
	uint8_t configMenuOption = 0, confSaveVar;
	uint8_t idlerChars[4] = {'/', '-', '\\', '|'};
	ScreenState screenState = SCREEN_MAIN_DRAW;
	// Application initialization
	init();

	DDRD &= ~(0xF8);

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusInit();

	ds1302_transact(0x81, 1, ds1302Buffer);
	if (ds1302Buffer[0] & 0x80)
	{
		ds1302Buffer[0] &= 0x7F;
		// Crap, the clock's been halted, get it started again
		ds1302_transact(0x80, 1, ds1302Buffer);
	}

	drawSplashScreen();

	sei();	

	while (1)
	{
#ifdef MRBEE
		mrbeePoll();
#endif
		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();

		if (status & STATUS_READ_INPUTS)
		{
			status &= ~(STATUS_READ_INPUTS);
			buttonsPressed = debounce((PIND & 0xF8) | (PINC & _BV(PC4)>>2));
		}

/*

Possible states:

MAIN:
 - Displays:
   - Big time
   - AM/PM if in A/P mode or 24 if in 24h mode
   - REAL if in real mode or F1:XX if in FAST
   - 
   
 - Soft Keys:
   - CONF
   - FAST (if real) or REAL (if fast)
   - HOLD (if fast)
   - RST  (if Fast)

CONF:
 - Displays: Scrolly list

 - Soft Keys:
   - UP
   - DOWN
   - SET
   - BACK


*/

		switch(screenState)
		{
		
			case SCREEN_MAIN_DRAW:
				if (FAST_MODE)
				{
					lcd_gotoxy(16,1);
					lcd_puts("    ");
					lcd_gotoxy(16,1);				
					printDec2Dig(scaleFactor);
					lcd_puts(":1");
				}
				else
				{
					lcd_gotoxy(16,1);
					lcd_puts("REAL");				
				}

				drawSoftKeys(FAST_MODE?"REAL":"FAST",  FAST_MODE?"HOLD":"", FAST_MODE?"RST":"", "CONF");
				// Intentional fall-through

			case SCREEN_MAIN_UPDATE_TIME:
				lcd_gotoxy(16,0);
				if (FAST_MODE)
				{
					if (status & STATUS_FAST_AMPM)
						lcd_puts((fastTime.hours<12)?"AM":"PM");
					else
						lcd_puts("24");
					drawBigTime(&fastTime, status & STATUS_FAST_AMPM);
					drawLittleTime(&fastTime, status & STATUS_REAL_AMPM);
				} else {
				
					if (status & STATUS_REAL_AMPM)
						lcd_puts((realTime.hours<12)?"AM":"PM");
					else
						lcd_puts("24");
					drawBigTime(&realTime, status & STATUS_REAL_AMPM);
					drawLittleTime(&realTime, status & STATUS_REAL_AMPM);
				}
				screenState = SCREEN_MAIN_IDLE;
				break;

			case SCREEN_MAIN_IDLE:
				lcd_gotoxy(0,2);
				printHex(debounced_state);
				lcd_putc(' ');
				printHex(buttonsPressed);
				lcd_putc(' ');
				printHex((PIND & 0xF8) | ((PINC & _BV(PC4))>>2));
			
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					status ^= STATUS_FAST_ACTIVE;
					screenState = SCREEN_MAIN_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (FAST_MODE && (SOFTKEY_2 & buttonsPressed))
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (FAST_MODE && (SOFTKEY_3 & buttonsPressed))
				{
					screenState = SCREEN_CONF_MENU_DRAW;
					configMenuOption = 0;
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case SCREEN_CONF_MENU_DRAW:
				lcd_clrscr();
				drawSoftKeys((configMenuOption>0)?" UP ":"",  (configMenuOption < NUM_CONF_OPTIONS-1)?"DOWN":"", "SLCT", "BACK");

				{
					uint8_t i, baseOptionCount = (configMenuOption / 3) * 3;
					for(i=0; i<3; i++)
					{
						if (i+baseOptionCount >= NUM_CONF_OPTIONS)
							continue;
						if (i+baseOptionCount == configMenuOption)
						{
							lcd_gotoxy(0, i);
							lcd_putc('>');
						}

						lcd_gotoxy(2, i);
						lcd_puts(configurationOptions[i+baseOptionCount].configName);
					}
				}

				screenState = SCREEN_CONF_MENU_IDLE;
				break;
				
			case SCREEN_CONF_MENU_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					if (configMenuOption > 0)
						configMenuOption--;
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					if (configMenuOption < NUM_CONF_OPTIONS-1)
						configMenuOption++;
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					lcd_clrscr();
					screenState = SCREEN_MAIN_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed && (configurationOptions[configMenuOption].configScreen))
				{
					screenState = configurationOptions[configMenuOption].configScreen;
					lcd_clrscr();
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;			
				break;

			case SCREEN_CONF_R1224_SETUP:
				confSaveVar = (status & STATUS_REAL_AMPM);
				lcd_gotoxy(0,0);
				lcd_puts("Real Time 12/24H:");
				drawSoftKeys("12hr",  "24hr", "SAVE", "CNCL");
				// Intentional fall-through

			case SCREEN_CONF_R1224_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts("[ ] 12H (AM/PM)");
				lcd_gotoxy(0,2);
				lcd_puts("[ ] 24H (0000h)");
				lcd_gotoxy(1, (status & STATUS_REAL_AMPM)?1:2);
				lcd_putc('*');
				screenState = SCREEN_CONF_R1224_IDLE;
				break;

			case SCREEN_CONF_R1224_IDLE:

				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					status |= STATUS_REAL_AMPM;
					screenState = SCREEN_CONF_R1224_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					status &= ~STATUS_REAL_AMPM;
					screenState = SCREEN_CONF_R1224_DRAW;
				}
				else if ((SOFTKEY_3 | SOFTKEY_4) & buttonsPressed)
				{
					if (!(SOFTKEY_3 & buttonsPressed))
					{
						status &= ~STATUS_REAL_AMPM;
						status |= confSaveVar;
					} else {
						// FIXME:  Save value
						
					}
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
				}				
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


			case SCREEN_CONF_F1224_SETUP:
				confSaveVar = (status & STATUS_REAL_AMPM);
				lcd_gotoxy(0,0);
				lcd_puts("Fast Time 12/24H:");
				drawSoftKeys("12hr",  "24hr", "SAVE", "CNCL");
				// Intentional fall-through

			case SCREEN_CONF_F1224_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts("[ ] 12H (AM/PM)");
				lcd_gotoxy(0,2);
				lcd_puts("[ ] 24H (0000h)");
				lcd_gotoxy(1, (status & STATUS_FAST_AMPM)?1:2);
				lcd_putc('*');				
				screenState = SCREEN_CONF_F1224_IDLE;
				break;

			case SCREEN_CONF_F1224_IDLE:

				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					status |= STATUS_FAST_AMPM;
					screenState = SCREEN_CONF_F1224_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					status &= ~STATUS_FAST_AMPM;
					screenState = SCREEN_CONF_F1224_DRAW;
				}
				else if ((SOFTKEY_3 | SOFTKEY_4) & buttonsPressed)
				{
					if (!(SOFTKEY_3 & buttonsPressed))
					{
						status &= ~STATUS_FAST_AMPM;
						status |= confSaveVar;
					} else {
						// FIXME:  Save value
						
					}
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
				}				
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;
			
			default:
				lcd_gotoxy(0,1);
				lcd_puts("Code off in lalaland");
				break;		
		
		}


			
		// FIXME: Do any module-specific behaviours here in the loop.
		if (decisecs >= 10)
		{

			ds1302_transact(0xBF, 4, ds1302Buffer);
			realTime.hours = (ds1302Buffer[2] & 0x0F) + 10 * (((ds1302Buffer[2] & 0x30)>>4));
			realTime.minutes = (ds1302Buffer[1] & 0x0F) + 10 * (((ds1302Buffer[1] & 0x30)>>4));
			realTime.seconds = (ds1302Buffer[0] & 0x0F) + 10 * (((ds1302Buffer[0] & 0x30)>>4));

			switch(screenState)
			{
				case SCREEN_MAIN_IDLE:
				case SCREEN_MAIN_UPDATE_TIME:
					screenState = SCREEN_MAIN_UPDATE_TIME;
					if (0xA5 == colon)
						colon = ' ';
					else
						colon = 0xA5;
					
					lcd_gotoxy(7,0);
					lcd_putc(colon);
					lcd_gotoxy(7,1);
					lcd_putc(colon);
					break;
				
				case SCREEN_MAIN_DRAW:
				default:
					break;
			}

			decisecs -= 10;
			lcd_gotoxy(19,0);
			lcd_putc(idlerChars[idler]);
			idler = (idler + 1) % sizeof(idlerChars);
			
		}		


		// If we have a packet to be transmitted, try to send it here
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
			uint8_t bus_countdown;

			// Even while we're sitting here trying to transmit, keep handling
			// any packets we're receiving so that we keep up with the current state of the
			// bus.  Obviously things that request a response cannot go, since the transmit
			// buffer is full.
			if (mrbus_state & MRBUS_RX_PKT_READY)
				PktHandler();


			if (0 == mrbusPacketTransmit())
			{
				mrbus_state &= ~(MRBUS_TX_PKT_READY);
				break;
			}

#ifndef MRBEE
			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			bus_countdown = 20;
			while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
			{
				//clrwdt();
				_delay_ms(1);
				if (mrbus_state & MRBUS_RX_PKT_READY) 
					PktHandler();
			}
#endif
		}
	}
}



