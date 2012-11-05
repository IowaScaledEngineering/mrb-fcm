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
#include "string.h"

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
extern uint8_t mrbus_priority;

uint8_t mrbus_dev_addr = 0;

// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs=0;
volatile uint16_t updateXmitInterval=20;
volatile uint16_t screenUpdateDecisecs=0;
volatile uint16_t fastDecisecs=0;
volatile uint8_t status=0;

#define STATUS_READ_INPUTS 0x01
#define STATUS_FAST_ACTIVE 0x02
#define STATUS_FAST_AMPM   0x04
#define STATUS_REAL_AMPM   0x08
#define STATUS_FAST_HOLDING 0x10

#define FAST_MODE (status & STATUS_FAST_ACTIVE)
#define FASTHOLD_MODE (status & STATUS_FAST_HOLDING)

#define EE_ADDR_FAST_START_H   0x30
#define EE_ADDR_FAST_START_M   0x31
#define EE_ADDR_FAST_START_S   0x32

#define EE_ADDR_FAST_RATIO     0x38

uint8_t scaleFactor = 1;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t dayOfWeek;
	uint8_t day;
	uint8_t month;
	uint16_t year;
} TimeData;

TimeData realTime;
TimeData fastTime;

void initTimeData(TimeData* t)
{
	t->seconds = t->minutes = t->hours = 0;
	t->dayOfWeek = 0;
	t->year = 2012;
	t->month = t->day = 1;
}

void incrementTime(TimeData* t, uint8_t incSeconds)
{
	uint16_t i = t->seconds + incSeconds;

	while(i >= 60)
	{
		t->minutes++;
		i -= 60;
	}
	t->seconds = (uint8_t)i;
	
	while(t->minutes >= 60)
	{
		t->hours++;
		t->minutes -= 60;
	}
	
	if (t->hours >= 24)
		t->hours %= 24;
}


void FastTimeStartToFlash(TimeData* t)
{
	eeprom_write_byte((uint8_t*)(uint16_t)EE_ADDR_FAST_START_H, t->hours);
	eeprom_write_byte((uint8_t*)(uint16_t)EE_ADDR_FAST_START_M, t->minutes);
	eeprom_write_byte((uint8_t*)(uint16_t)EE_ADDR_FAST_START_S, t->seconds);			
}
void FlashToFastTimeStart(TimeData* t)
{
	initTimeData(t);
	t->hours = eeprom_read_byte((uint8_t*)(uint16_t)EE_ADDR_FAST_START_H);
	t->minutes = eeprom_read_byte((uint8_t*)(uint16_t)EE_ADDR_FAST_START_M);
	t->seconds = eeprom_read_byte((uint8_t*)(uint16_t)EE_ADDR_FAST_START_S);			

	if (t->hours > 23 || t->minutes > 59 || t->seconds > 59)
	{
		initTimeData(t);
		FastTimeStartToFlash(t);
	}
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

const char* monthNames[13] = { "Unk", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
const uint8_t monthDays[13] = { 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }; // Days in each month

typedef enum
{
	SCREEN_MAIN_DRAW = 0,
	SCREEN_MAIN_IDLE = 1,
	SCREEN_MAIN_UPDATE_TIME = 2,

	SCREEN_CONF_MENU_DRAW = 10,
	SCREEN_CONF_MENU_IDLE = 11,

	SCREEN_FAST_RESET_DRAW = 14,
	SCREEN_FAST_RESET_IDLE = 15,

	SCREEN_CONF_R1224_SETUP = 100,
	SCREEN_CONF_R1224_DRAW = 101,
	SCREEN_CONF_R1224_IDLE = 102,

	SCREEN_CONF_F1224_SETUP = 110,
	SCREEN_CONF_F1224_DRAW = 111,
	SCREEN_CONF_F1224_IDLE = 112,

	SCREEN_CONF_FRATIO_SETUP = 115,
	SCREEN_CONF_FRATIO_DRAW = 116,
	SCREEN_CONF_FRATIO_IDLE = 117,
	
	SCREEN_CONF_RTIME_SETUP = 120,
	SCREEN_CONF_RTIME_DRAW  = 121,
	SCREEN_CONF_RTIME_IDLE  = 122,
	SCREEN_CONF_RTIME_CONFIRM = 123,

	SCREEN_CONF_FSTART_SETUP = 130,
	SCREEN_CONF_FSTART_DRAW  = 131,
	SCREEN_CONF_FSTART_IDLE  = 132,
	SCREEN_CONF_FSTART_CONFIRM = 133,

	SCREEN_CONF_PKTINT_SETUP = 140,
	SCREEN_CONF_PKTINT_DRAW = 141,
	SCREEN_CONF_PKTINT_IDLE = 142,

	SCREEN_CONF_RDATE_SETUP = 150,
	SCREEN_CONF_RDATE_DRAW  = 151,
	SCREEN_CONF_RDATE_IDLE  = 152,
	
	
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

const ConfigurationOption configurationOptions[] = 
{
  { "Real 12/24 Ind", SCREEN_CONF_R1224_SETUP },
  { "Real Time     ", SCREEN_CONF_RTIME_SETUP },
  { "Real Date     ", SCREEN_CONF_RDATE_SETUP },  
  { "Fast 12/24 Ind", SCREEN_CONF_F1224_SETUP },
  { "Fast Ratio     ", SCREEN_CONF_FRATIO_SETUP },  
  { "Fast Start Time", SCREEN_CONF_FSTART_SETUP },
  { "Time Pkt Interval", SCREEN_CONF_PKTINT_SETUP },
  
};

const ConfigurationOption ratioOptions[] = 
{
  { "1:1", 1 },
  { "2:1", 2 },
  { "3:1", 3 },
  { "4:1", 4 },
  { "6:1", 6 },
  { "8:1", 8 },
  { "10:1", 10 },
  { "12:1", 12 },
  { "16:1", 16 },
  { "20:1", 20 },
  { "24:1", 24 },
  { "30:1", 30 },
  { "32:1", 32 },
  { "36:1", 36 },
  { "40:1", 40 },
  { "60:1", 60 }
};

#define NUM_RATIO_OPTIONS  (sizeof(ratioOptions)/sizeof(ConfigurationOption))

#define NUM_CONF_OPTIONS  (sizeof(configurationOptions)/sizeof(ConfigurationOption))


void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR1C = 0;
	TIMSK1 = _BV(TOIE1);
	ticks = 0;
	decisecs = 0;
	fastDecisecs = 0;
	screenUpdateDecisecs = 0;
}

ISR(TIMER1_OVF_vect)
{
	TCNT1 += 0xF3CB;

	if (ticks & 0x01)
		status |= STATUS_READ_INPUTS;

	if (++ticks >= 10)
	{
		ticks = 0;
		if (STATUS_FAST_ACTIVE == (status & (STATUS_FAST_ACTIVE | STATUS_FAST_HOLDING)))
			fastDecisecs += scaleFactor;
		decisecs++;
		screenUpdateDecisecs++;
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
	uint8_t i, j;
	uint8_t ds1302Buffer[10];	
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
	
	ds1302_transact(0xBF, 7, ds1302Buffer);
	realTime.seconds = (ds1302Buffer[0] & 0x0F) + 10 * (((ds1302Buffer[0] & 0x70)>>4));
	realTime.minutes = (ds1302Buffer[1] & 0x0F) + 10 * (((ds1302Buffer[1] & 0x70)>>4));
	realTime.hours = (ds1302Buffer[2] & 0x0F) + 10 * (((ds1302Buffer[2] & 0x30)>>4));			
	realTime.day = (ds1302Buffer[3] & 0x0F) + 10 * (((ds1302Buffer[3] & 0x30)>>4));
	realTime.month = (ds1302Buffer[4] & 0x0F) + (ds1302Buffer[4] & 0x10)?10:0;
	realTime.year = 2000 + (ds1302Buffer[3] & 0x0F) + 10 * (ds1302Buffer[3]>>4);

	if (ds1302Buffer[0] & 0x80)
	{
		// Crap, the clock's been halted, get it started again
		// Set write enable
		ds1302Buffer[0] = 0x00;
		ds1302_transact(0x8E, 1, ds1302Buffer);
		// Clear the clock halting bit
		ds1302Buffer[0] &= 0x7F;
		ds1302_transact(0x80, 1, ds1302Buffer);
	}

	// Clear DS1302 WR enable
	ds1302Buffer[0] = 0x80;
	ds1302_transact(0x8E, 1, ds1302Buffer);	

	initTimeData(&fastTime);
	FlashToFastTimeStart(&fastTime);
	
	scaleFactor = 0;	
	j = eeprom_read_byte((uint8_t*)EE_ADDR_FAST_RATIO);
	for(i=0; i<NUM_RATIO_OPTIONS; i++)
	{
		if (ratioOptions[i].configScreen == j)
		{
			scaleFactor = j;
			break;
		}	
	
	}
	if (0 == scaleFactor)
	{
		scaleFactor = ratioOptions[0].configScreen;
		eeprom_write_byte((uint8_t*)EE_ADDR_FAST_RATIO, scaleFactor);
	}	
	
	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	
	updateXmitInterval = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
			| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	updateXmitInterval = max(1, updateXmitInterval);
			
	// If the update interval is garbage, set it to 2 seconds
	if (0xFFFF == updateXmitInterval)
	{
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, 20);
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, 0);
		updateXmitInterval = 20;
	}
}

void drawBigHold()
{
	lcd_putc_big(0, 10);
	lcd_putc_big(1, 0);
	lcd_putc_big(2, 11);
	lcd_putc_big(3, 12);
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

int main(void)
{
	uint8_t ds1302Buffer[10];
	uint8_t buttonsPressed=0, colon=0;
	uint8_t configMenuOption = 0, confSaveVar=0;
	uint8_t vitalChange=0;
	TimeData tempTime;
	ScreenState screenState = SCREEN_MAIN_DRAW;
	// Application initialization
	init();

	DDRD &= ~(0xF8);

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusInit();
	mrbus_priority = 1;  // We're a clock, highest priority

	drawSplashScreen();

	sei();	

	while (1)
	{
//		loopCount++;
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
					if (FASTHOLD_MODE)
						drawBigHold();
					else
						drawBigTime(&fastTime, status & STATUS_FAST_AMPM);
					drawLittleTime(&realTime, status & STATUS_REAL_AMPM);
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

			case SCREEN_FAST_RESET_DRAW:
				lcd_clrscr();
				lcd_gotoxy(0,3);
				lcd_puts("!! CONFIRM !!");
				lcd_gotoxy(1,2);
				lcd_puts("Reset Fast Clock");
				lcd_gotoxy(1,3);
				lcd_puts("to Start Time?");
				drawSoftKeys("YES",  "", "", "NO");
				screenState = SCREEN_FAST_RESET_IDLE;
				break;
			
			case SCREEN_FAST_RESET_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_FAST_RESET_DRAW;
					FlashToFastTimeStart(&fastTime);
					vitalChange = 1;
					screenState = SCREEN_MAIN_UPDATE_TIME;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case SCREEN_MAIN_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					status ^= STATUS_FAST_ACTIVE;
					vitalChange = 1;
					if (FAST_MODE)
						status |= FASTHOLD_MODE;
					screenState = SCREEN_MAIN_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
					configMenuOption = 0;

				}
				else if (FAST_MODE && (SOFTKEY_2 & buttonsPressed))
				{
					status ^= STATUS_FAST_HOLDING;
					vitalChange = 1;
					screenState = SCREEN_MAIN_UPDATE_TIME;
				}
				else if (FAST_MODE && (SOFTKEY_3 & buttonsPressed))
				{
					screenState = SCREEN_FAST_RESET_DRAW;
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

			case SCREEN_CONF_FRATIO_SETUP:
				{
					uint8_t i;
					confSaveVar = 0;
					for(i=0; i<NUM_RATIO_OPTIONS; i++)
					{
						if (scaleFactor == ratioOptions[i].configScreen)
						{
							confSaveVar = i;
							break;
						}
					}
				}
				screenState = SCREEN_CONF_FRATIO_DRAW;
				break;

			case SCREEN_CONF_FRATIO_DRAW:
				lcd_clrscr();
				drawSoftKeys((confSaveVar>0)?" UP ":"",  (confSaveVar < NUM_RATIO_OPTIONS-1)?"DOWN":"", "SLCT", "CNCL");
				{
					uint8_t i, baseOptionCount = (confSaveVar / 3) * 3;
					for(i=0; i<3; i++)
					{
						if (i+baseOptionCount >= NUM_RATIO_OPTIONS)
							continue;
						if (i+baseOptionCount == confSaveVar)
						{
							lcd_gotoxy(0, i);
							lcd_putc('>');
						}

						lcd_gotoxy(2, i);
						lcd_puts(ratioOptions[i+baseOptionCount].configName);
					}
				}
				screenState = SCREEN_CONF_FRATIO_IDLE;
				break;
				
			case SCREEN_CONF_FRATIO_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					if (confSaveVar > 0)
						confSaveVar--;
					screenState = SCREEN_CONF_FRATIO_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					if (confSaveVar < NUM_RATIO_OPTIONS-1)
						confSaveVar++;
					screenState = SCREEN_CONF_FRATIO_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					scaleFactor = ratioOptions[confSaveVar].configScreen;
					eeprom_write_byte((uint8_t*)EE_ADDR_FAST_RATIO, scaleFactor);
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;			
				break;

			case SCREEN_CONF_RDATE_SETUP:
				memcpy(&tempTime, &realTime, sizeof(TimeData));
				// Sanity test
				if (tempTime.month > 12 || tempTime.month < 1)
					tempTime.month = 1;
				if (tempTime.day < 1)
					tempTime.day = 1;
				if (tempTime.day > monthDays[tempTime.month])
					tempTime.day = monthDays[tempTime.month];
				if (tempTime.year < 2000 || tempTime.year > 2099)
					tempTime.year = 2012;
				
				confSaveVar = 0;
				lcd_gotoxy(0,0);
				lcd_puts("Current Real Date:");

				// Intentional fall-through

			case SCREEN_CONF_RDATE_DRAW:
				lcd_gotoxy(0, 1);
				printDec2Dig(tempTime.day);
				lcd_putc(' ');
				lcd_puts(monthName[tempTime.month]);
				lcd_puts(" 20");
				printDec2DigWZero(tempTime.year % 100);
				lcd_gotoxy(0,2);
				lcd_puts("            ");
				switch(confSaveVar)
				{
					case 0: // Day
						lcd_gotoxy(0, 2);
						lcd_puts("^^");
					
					case 1: // Month
						lcd_gotoxy(3, 2);
						lcd_puts("^^^");						
						break;
						
					case 2: // Year
						lcd_gotoxy(7, 2);
						lcd_puts("  ^^");
						break;
				}


				{
					// Check that our day is actually in range  Otherwise, throw up "DATE ERR"
					uint8_t dateError = 0;
					uint8_t monthDaysWithLeap = monthDays[tempTime.month];
					if (tempTime.month == 2 && isLeapYear(tempTime.year))
						monthDaysWithLeap++;

					if (tempTime.day > monthDaysWithLeap)
						dateError = 1;

					lcd_gotoxy(15,1);
					lcd_puts(dateError?"DATE":"    ");
					lcd_gotoxy(15,2);
					lcd_puts(dateError?"ERR ":"    ");

					drawSoftKeys(" ++ ",  " -- ", " >> ", dateError?"    ":" GO ");
				}
				screenState = SCREEN_CONF_RTIME_IDLE;
				break;


			case SCREEN_CONF_RDATE_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							// Day of Month
							if (tempTime.day < 31)
								tempTime.day++;
							else
								tempTime.day = 1;
							break;

						case 1:
							// Month
							if (tempTime.month < 12)
								tempTime.month++;
							else
								tempTime.month = 1;
							break;

						case 2:
							// Year
							tempTime.year++;
							if(tempTime.year > 2099 || tempTime.year < 2012)
								tempTime.year = 2012;
							break;
					}
					screenState = SCREEN_CONF_RTIME_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							// Day of Month
							if (tempTime.day > 1)
								tempTime.day--;
							else
								tempTime.day = 31;
							break;

						case 1:
							// Month
							if (tempTime.month > 1)
								tempTime.month--;
							else
								tempTime.month = 12;
							break;

						case 2:
							// Seconds
							if (tempTime.year > 2012)
								tempTime.year--;
							else
								tempTime.year = 2099;
							break;
					}
					screenState = SCREEN_CONF_RDATE_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					confSaveVar = (confSaveVar+1)%3;
					screenState = SCREEN_CONF_RDATE_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					uint8_t monthDaysWithLeap = monthDays[tempTime.month];

					screenState = SCREEN_CONF_RDATE_DRAW;

					if (tempTime.month == 2 && isLeapYear(tempTime.year))
						monthDaysWithLeap++;

					if (tempTime.day <= monthDaysWithLeap)
					{
						drawSoftKeys("BACK",  "", "SAVE", "CNCL");
						screenState = SCREEN_CONF_RDATE_CONFIRM;
					}
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


			case SCREEN_CONF_RDATE_CONFIRM:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_CONF_RDATE_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					memcpy(&realTime, &tempTime, sizeof(TimeData));

					// Save crap back to RTC
					// Set write enable
					ds1302Buffer[0] = 0x00;
					ds1302_transact(0x8E, 1, ds1302Buffer);

					// Write time registers
					ds1302Buffer[0] = (tempTime.seconds % 10) | (0x70 & (((tempTime.seconds/10)%10)<<4));
					ds1302_transact(0x80, 1, ds1302Buffer);
					ds1302Buffer[0] = (tempTime.minutes % 10) | (0x70 & (((tempTime.minutes/10)%10)<<4));
					ds1302_transact(0x82, 1, ds1302Buffer);
					ds1302Buffer[0] = (tempTime.hours % 10) | (0x30 & (((tempTime.hours/10)%10)<<4));
					ds1302_transact(0x84, 1, ds1302Buffer);

					// Clear WR enable
					ds1302Buffer[0] = 0x80;
					ds1302_transact(0x8E, 1, ds1302Buffer);

					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				buttonsPressed = 0;	
				break;
							
			// END OF DATE CONFIG SCREENS
			
			// START OF TIME CONFIG SCREENS

			case SCREEN_CONF_RTIME_SETUP:
				memcpy(&tempTime, &realTime, sizeof(TimeData));
				confSaveVar = 0;
				lcd_gotoxy(0,0);
				lcd_puts("Current Real Time:");
				// Intentional fall-through

			case SCREEN_CONF_RTIME_DRAW:
				drawSoftKeys(" ++ ",  " -- ", " >> ", " GO ");
				lcd_gotoxy(0, 1);
				printDec2Dig(tempTime.);
				else
					printDec2DigWZero(tempTime.hours);
				lcd_putc(':');
				printDec2DigWZero(tempTime.minutes);
				lcd_putc(':');
				printDec2DigWZero(tempTime.seconds);
				lcd_putc(' ');

				if (status & STATUS_REAL_AMPM)
				{
					if (tempTime.hours < 12)
						lcd_putc('A');
					else
						lcd_putc('P');
					lcd_putc('M');
				}
				lcd_gotoxy(0,2);
				lcd_puts("         ");
				lcd_gotoxy(0 + 3*confSaveVar,2);
				lcd_puts("^^");
				screenState = SCREEN_CONF_RTIME_IDLE;
				break;

			case SCREEN_CONF_RTIME_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							// Hours
							tempTime.hours = (tempTime.hours+1) % 24;
							break;
						case 1:
							// Minutes
							tempTime.minutes = (tempTime.minutes+1) % 60;
							break;
						case 2:
							// Seconds
							tempTime.seconds = (tempTime.seconds+1) % 60;
							break;
					}
					screenState = SCREEN_CONF_RTIME_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							// Hours
							if (tempTime.hours)
								tempTime.hours--;
							else
								tempTime.hours = 23;
							break;
						case 1:
							// Minutes
							if (tempTime.minutes)
								tempTime.minutes--;
							else
								tempTime.minutes = 59;
							break;
						case 2:
							// Seconds
							if (tempTime.seconds)
								tempTime.seconds--;
							else
								tempTime.seconds = 59;
							break;
					}
					screenState = SCREEN_CONF_RTIME_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					confSaveVar = (confSaveVar+1)%3;
					screenState = SCREEN_CONF_RTIME_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					drawSoftKeys("BACK",  "", "SAVE", "CNCL");
					screenState = SCREEN_CONF_RTIME_CONFIRM;
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;

			case SCREEN_CONF_RTIME_CONFIRM:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_CONF_RTIME_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					memcpy(&realTime, &tempTime, sizeof(TimeData));

					// Save crap back to RTC
					// Set write enable
					ds1302Buffer[0] = 0x00;
					ds1302_transact(0x8E, 1, ds1302Buffer);

					// Write time registers
					ds1302Buffer[0] = (tempTime.seconds % 10) | (0x70 & (((tempTime.seconds/10)%10)<<4));
					ds1302_transact(0x80, 1, ds1302Buffer);
					ds1302Buffer[0] = (tempTime.minutes % 10) | (0x70 & (((tempTime.minutes/10)%10)<<4));
					ds1302_transact(0x82, 1, ds1302Buffer);
					ds1302Buffer[0] = (tempTime.hours % 10) | (0x30 & (((tempTime.hours/10)%10)<<4));
					ds1302_transact(0x84, 1, ds1302Buffer);

					// Clear WR enable
					ds1302Buffer[0] = 0x80;
					ds1302_transact(0x8E, 1, ds1302Buffer);

					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;

			case SCREEN_CONF_FSTART_SETUP:
				FlashToFastTimeStart(&tempTime);
				confSaveVar = 0;
				lcd_gotoxy(0,0);
				lcd_puts("Fast Clk Start Time:");
				// Intentional fall-through

			case SCREEN_CONF_FSTART_DRAW:
				drawSoftKeys(" ++ ",  " -- ", " >> ", " GO ");
				lcd_gotoxy(0, 1);
				if (status & STATUS_FAST_AMPM)
					printDec2Dig(tempTime.hours);
				else
					printDec2DigWZero(tempTime.hours);
				lcd_putc(':');
				printDec2DigWZero(tempTime.minutes);
				lcd_putc(':');
				printDec2DigWZero(tempTime.seconds);
				lcd_putc(' ');

				if (status & STATUS_FAST_AMPM)
				{
					if (tempTime.hours < 12)
						lcd_putc('A');
					else
						lcd_putc('P');
					lcd_putc('M');
				}

				confSaveVar %= 3;
				lcd_gotoxy(0,2);
				lcd_puts("         ");
				lcd_gotoxy(0 + 3*confSaveVar,2);
				lcd_puts("^^");
				screenState = SCREEN_CONF_FSTART_IDLE;
				break;

			case SCREEN_CONF_FSTART_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							// Hours
							tempTime.hours = (tempTime.hours+1) % 24;
							break;
						case 1:
							// Minutes
							tempTime.minutes = (tempTime.minutes+1) % 60;
							break;
						case 2:
							// Seconds
							tempTime.seconds = (tempTime.seconds+1) % 60;
							break;
					}
					screenState = SCREEN_CONF_FSTART_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							// Hours
							if (tempTime.hours)
								tempTime.hours--;
							else
								tempTime.hours = 23;
							break;
						case 1:
							// Minutes
							if (tempTime.minutes)
								tempTime.minutes--;
							else
								tempTime.minutes = 59;
							break;
						case 2:
							// Seconds
							if (tempTime.seconds)
								tempTime.seconds--;
							else
								tempTime.seconds = 59;
							break;
					}
					screenState = SCREEN_CONF_FSTART_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					confSaveVar = (confSaveVar+1)%3;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					drawSoftKeys("BACK",  "", "SAVE", "CNCL");
					screenState = SCREEN_CONF_FSTART_CONFIRM;
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;

			case SCREEN_CONF_FSTART_CONFIRM:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_CONF_FSTART_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					memcpy(&fastTime, &tempTime, sizeof(TimeData));
					FastTimeStartToFlash(&fastTime);
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
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

		if(FAST_MODE && !FASTHOLD_MODE && fastDecisecs >= 10)
		{
			uint8_t fastTimeSecs = fastDecisecs / 10;
			incrementTime(&fastTime, fastTimeSecs);
			fastDecisecs -= fastTimeSecs * 10;
		}
		

		// FIXME: Do any module-specific behaviours here in the loop.
		if (screenUpdateDecisecs >= 10)
		{
			// Reading optimizer
			// If we don't know the date or if we're in the range where we're at risk of 
			// changing dates

			ds1302_transact(0xBF, 7, ds1302Buffer);
			realTime.seconds = (ds1302Buffer[0] & 0x0F) + 10 * (((ds1302Buffer[0] & 0x70)>>4));
			realTime.minutes = (ds1302Buffer[1] & 0x0F) + 10 * (((ds1302Buffer[1] & 0x70)>>4));
			realTime.hours = (ds1302Buffer[2] & 0x0F) + 10 * (((ds1302Buffer[2] & 0x30)>>4));			
			realTime.day = (ds1302Buffer[3] & 0x0F) + 10 * (((ds1302Buffer[3] & 0x30)>>4));
			realTime.month = (ds1302Buffer[4] & 0x0F) + (ds1302Buffer[4] & 0x10)?10:0;
			realTime.year = 2000 + (ds1302Buffer[3] & 0x0F) + 10 * (ds1302Buffer[3]>>4);
			
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
			
			screenUpdateDecisecs -= 10;
			lcd_gotoxy(19,0);
			
		}		

		/* If we need to send a packet and we're not already busy... */
		if (decisecs >= updateXmitInterval)
		{
			vitalChange = 1;
			decisecs -= updateXmitInterval;
		}			
		
		
		if (vitalChange && !(mrbus_state & (MRBUS_TX_BUF_ACTIVE | MRBUS_TX_PKT_READY)))
		{
			uint8_t flags = 0;
			
			if (FAST_MODE)
				flags |= 0x01;
			if (FASTHOLD_MODE)
				flags |= 0x02;

			if (status & STATUS_REAL_AMPM)
				flags |= 0x04;

			if (status & STATUS_FAST_AMPM)
				flags |= 0x08;


#define STATUS_FAST_AMPM   0x04
#define STATUS_REAL_AMPM   0x08
			
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 17;			
			mrbus_tx_buffer[5] = 'T';
			mrbus_tx_buffer[6] = realTime.hours;
			mrbus_tx_buffer[7] = realTime.minutes;
			mrbus_tx_buffer[8] = realTime.seconds;
			mrbus_tx_buffer[9] = flags;
			mrbus_tx_buffer[10] = fastTime.hours;
			mrbus_tx_buffer[11] = fastTime.minutes;			
			mrbus_tx_buffer[12] = fastTime.seconds;
			mrbus_tx_buffer[13] = scaleFactor;
			mrbus_tx_buffer[14] = 0xFF & (realTime.year>>4);
			mrbus_tx_buffer[15] = ((realTime.year<<4) & 0xF0) | (0x0F & realTime.month);
			mrbus_tx_buffer[16] = realTime.day;
			mrbus_state |= MRBUS_TX_PKT_READY;
			vitalChange = 0;
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



