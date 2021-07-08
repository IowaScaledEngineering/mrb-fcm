/*************************************************************************
Title:    MRBus Fast Clock Master v2.1
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2020 Nathan D. Holmes (maverick@drgw.net)
     & Michael Petersen (railfan@drgw.net)

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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>

#include "float16.h"
#include "lcd.h"
#include "avr-i2c-master.h"
#include "rv3129.h"
#include "dmx.h"

#include "ff.h"
#include "diskio.h"
#include "mmc-avr.h"


#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbusTxQueue mrbeeTxQueue
#define mrbusRxQueue mrbeeRxQueue
#else
#include "mrbus.h"
#endif

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t mrbus_dev_addr = 0;
uint8_t thSourceAddr = 0;
uint16_t thTimeoutReset = 600;
uint16_t thTimeout = 0;

#define BACKLIGHT_ALWAYS_ON 0xFF
uint8_t blTimeout = BACKLIGHT_ALWAYS_ON;
uint8_t blTimeoutReset = BACKLIGHT_ALWAYS_ON;

volatile uint16_t decisecs=0;
volatile uint16_t fastDecisecs=0;

uint16_t updateXmitInterval=20;
uint16_t scaleFactor = 10;

volatile uint8_t status=0;
volatile uint8_t events=0;


#define EVENT_READ_INPUTS   0x01
#define EVENT_UPDATE_SCREEN 0x02
#define EVENT_SECOND_TICK   0x04
#define EVENT_SEND_DMX      0x80

float celsiusTemp = 0;
uint8_t relHumidity = 0;
uint8_t thVoltage = 0;
uint8_t thAlternator = 0;

#define TH_ALTERNATOR_MAX 8

#define STATUS_FAST_ACTIVE  0x02
#define STATUS_FAST_AMPM    0x04
#define STATUS_REAL_AMPM    0x08
#define STATUS_FAST_HOLDING 0x10 // This hold flag indicates we're actually in hold
#define STATUS_FAST_HOLD    0x20  // This flag indicates that we start going into fast in hold
#define STATUS_TEMP_DEG_F   0x40


#define TIME_FLAGS_DISP_FAST       0x01
#define TIME_FLAGS_DISP_FAST_HOLD  0x02
#define TIME_FLAGS_DISP_REAL_AMPM  0x04
#define TIME_FLAGS_DISP_FAST_AMPM  0x08

#define FAST_MODE (status & STATUS_FAST_ACTIVE)
#define FASTHOLD_MODE (status & STATUS_FAST_HOLDING)


#define EE_ADDR_CONF_FLAGS     0x20

#define CONF_FLAG_FAST_AMPM          0x04
#define CONF_FLAG_REAL_AMPM          0x08
#define CONF_FLAG_FAST_HOLD_START    0x20
#define CONF_FLAG_TEMP_DEG_F         0x40

#define EE_ADDR_FAST_START1_H   0x30
#define EE_ADDR_FAST_START1_M   0x31
#define EE_ADDR_FAST_START1_S   0x32
#define EE_ADDR_FAST_START2_H   0x33
#define EE_ADDR_FAST_START2_M   0x34
#define EE_ADDR_FAST_START2_S   0x35
#define EE_ADDR_FAST_START3_H   0x36
#define EE_ADDR_FAST_START3_M   0x37
#define EE_ADDR_FAST_START3_S   0x38

#define EE_ADDR_FAST_RATIO_H    0x3A
#define EE_ADDR_FAST_RATIO_L    0x3B
#define EE_ADDR_TH_SRC_ADDR     0x3C
#define EE_ADDR_TH_TIMEOUT_H    0x3D
#define EE_ADDR_TH_TIMEOUT_L    0x3E
#define EE_ADDR_BL_TIMEOUT      0x3F

void setBacklightTimeout(uint8_t blTimeoutSeconds)
{
	eeprom_write_byte((uint8_t*)EE_ADDR_BL_TIMEOUT, blTimeoutSeconds);
}

uint8_t getBacklightTimeout()
{
	uint8_t tmpVal = 	(uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_BL_TIMEOUT);

	if (tmpVal < 10)
	{
		tmpVal = 0xFF;
		setBacklightTimeout(tmpVal);
	}
	return tmpVal;
}

uint32_t loopCount = 0;

void initTimeData(TimeData* t)
{
	t->seconds = t->minutes = t->hours = 0;
	t->dayOfWeek = 0;
	t->year = 2020;
	t->month = t->day = 1;
}

void writeDefaultEEPROMConfig()
{
	uint16_t u16 = 40;

	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, 0x10);

	u16 = 20;
	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, u16 & 0xFF);
	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, 0xFF & (u16>>8));

	eeprom_write_byte((uint8_t*)(EE_ADDR_CONF_FLAGS), CONF_FLAG_FAST_AMPM | CONF_FLAG_REAL_AMPM | CONF_FLAG_FAST_HOLD_START | CONF_FLAG_TEMP_DEG_F);
	
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START1_H), 6);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START1_M), 0);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START1_S), 0);

	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START2_H), 12);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START2_M), 0);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START2_S), 0);

	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START3_H), 18);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START3_M), 0);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START3_S), 0);

	u16 = 40;
	eeprom_write_byte((uint8_t*)EE_ADDR_FAST_RATIO_L, u16 & 0xFF);
	eeprom_write_byte((uint8_t*)EE_ADDR_FAST_RATIO_H, 0xFF & (u16>>8));

	eeprom_write_byte((uint8_t*)(EE_ADDR_TH_SRC_ADDR), 0); // No TH by default
	
	u16 = 600; // 60 second timeout on the TH
	eeprom_write_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_L, u16 & 0xFF);
	eeprom_write_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_H, 0xFF & (u16>>8));

	setBacklightTimeout(0xFF);
}

void firstTimeInitConfig()
{
	TimeData initFastTime;
	initTimeData(&initFastTime);
	rv3129_systemReset();
	rv3129_writeFastTime(&initFastTime);
	rv3129_writeTime(&initFastTime);
	rv3129_writeDate(&initFastTime);
	writeDefaultEEPROMConfig();
}


void blankCursorLine()
{
	lcd_gotoxy(0,2);
	lcd_puts_p(PSTR("                    "));
}

void storeConfiguration(uint8_t confStatus)
{
	eeprom_write_byte((uint8_t*)(uint16_t)EE_ADDR_CONF_FLAGS, (confStatus & (STATUS_FAST_AMPM | STATUS_REAL_AMPM | STATUS_FAST_HOLD | STATUS_TEMP_DEG_F)));
}

TimeData realTime;
TimeData fastTime;

DWORD get_fattime (void)
{
	/* Get local time */
	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)(realTime.year - 1980) << 25)
			| ((DWORD)realTime.month << 21)
			| ((DWORD)realTime.day << 16)
			| ((DWORD)realTime.hours << 11)
			| ((DWORD)realTime.minutes<< 5)
			| ((DWORD)realTime.seconds >> 1);
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


void FastTimeStartToFlash(TimeData* t, uint8_t whichStart)
{
	whichStart *= 3;
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START1_H + whichStart), t->hours);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START1_M + whichStart), t->minutes);
	eeprom_write_byte((uint8_t*)(EE_ADDR_FAST_START1_S + whichStart), t->seconds);
}

void FlashToFastTimeStart(TimeData* t, uint8_t whichStart)
{
	initTimeData(t);
	whichStart *= 3;
	t->hours = eeprom_read_byte((uint8_t*)EE_ADDR_FAST_START1_H + whichStart);
	t->minutes = eeprom_read_byte((uint8_t*)EE_ADDR_FAST_START1_M + whichStart);
	t->seconds = eeprom_read_byte((uint8_t*)EE_ADDR_FAST_START1_S + whichStart);

	if (t->hours > 23 || t->minutes > 59 || t->seconds > 59)
	{
		initTimeData(t);
		FastTimeStartToFlash(t, whichStart);
	}
}

typedef struct
{
	uint16_t clock_A;
	uint16_t clock_B;
	uint16_t debounced_state;
} DebounceState;

void initDebounceState(DebounceState* d, uint16_t initialState)
{
	d->clock_A = d->clock_B = 0;
	d->debounced_state = initialState;
}

uint16_t debounce(uint16_t raw_inputs, DebounceState* d)
{
	uint16_t delta = raw_inputs ^ d->debounced_state;   //Find all of the changes
	uint16_t changes;

	d->clock_A ^= d->clock_B;                     //Increment the counters
	d->clock_B  = ~d->clock_B;

	d->clock_A &= delta;                       //Reset the counters if no changes
	d->clock_B &= delta;                       //were detected.

	changes = ~((~delta) | d->clock_A | d->clock_B);
	d->debounced_state ^= changes;
	return(changes & ~(d->debounced_state));
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
	SCREEN_CONF_FRATIO_CONFIRM = 118,
	
	SCREEN_CONF_RTIME_SETUP = 120,
	SCREEN_CONF_RTIME_DRAW  = 121,
	SCREEN_CONF_RTIME_IDLE  = 122,
	SCREEN_CONF_RTIME_CONFIRM = 123,

	SCREEN_CONF_FSTART1_SETUP = 127,
	SCREEN_CONF_FSTART2_SETUP = 128,
	SCREEN_CONF_FSTART3_SETUP = 129,	
	SCREEN_CONF_FSTART_COMMON_SETUP = 130,
	SCREEN_CONF_FSTART_DRAW  = 131,
	SCREEN_CONF_FSTART_IDLE  = 132,
	SCREEN_CONF_FSTART_CONFIRM = 133,

	SCREEN_CONF_FSHOLD_SETUP = 135,
	SCREEN_CONF_FSHOLD_DRAW  = 136,
	SCREEN_CONF_FSHOLD_IDLE  = 137,
	SCREEN_CONF_FSHOLD_CONFIRM = 138,

	SCREEN_CONF_PKTINT_SETUP = 140,
	SCREEN_CONF_PKTINT_DRAW = 141,
	SCREEN_CONF_PKTINT_IDLE = 142,
	SCREEN_CONF_PKTINT_CONFIRM = 143,
	
	SCREEN_CONF_RDATE_SETUP = 150,
	SCREEN_CONF_RDATE_DRAW  = 151,
	SCREEN_CONF_RDATE_IDLE  = 152,
	SCREEN_CONF_RDATE_CONFIRM = 153,

	SCREEN_CONF_ADDR_SETUP = 160,
	SCREEN_CONF_ADDR_DRAW  = 161,
	SCREEN_CONF_ADDR_IDLE  = 162,
	SCREEN_CONF_ADDR_CONFIRM = 163,	
	
	SCREEN_CONF_THADDR_SETUP = 170,
	SCREEN_CONF_THADDR_DRAW  = 171,
	SCREEN_CONF_THADDR_IDLE  = 172,
	SCREEN_CONF_THADDR_CONFIRM = 173,
	
	SCREEN_CONF_TEMPU_SETUP = 176,
	SCREEN_CONF_TEMPU_DRAW = 177,
	SCREEN_CONF_TEMPU_IDLE = 178,	

	SCREEN_CONF_THTIMEOUT_SETUP = 180,
	SCREEN_CONF_THTIMEOUT_DRAW  = 181,
	SCREEN_CONF_THTIMEOUT_IDLE  = 182,
	SCREEN_CONF_THTIMEOUT_CONFIRM = 183,

	SCREEN_CONF_BACKLITE_SETUP = 235,
	SCREEN_CONF_BACKLITE_DRAW  = 236,
	SCREEN_CONF_BACKLITE_IDLE  = 237,
	SCREEN_CONF_BACKLITE_OFF   = 238,

	SCREEN_CONF_RESET_SETUP = 245,
	SCREEN_CONF_RESET_DRAW  = 246,
	SCREEN_CONF_RESET_IDLE  = 247,

	SCREEN_CONF_DIAG_SETUP = 250,
	SCREEN_CONF_DIAG_DRAW  = 251,
	SCREEN_CONF_DIAG_IDLE  = 252,
	
	SCREEN_DONT_KNOW = 255

} ScreenState;

#define SOFTKEY_1      0x0001
#define SOFTKEY_2      0x0002
#define SOFTKEY_3      0x0004
#define SOFTKEY_4      0x0008
#define SOFTKEY_1_LONG 0x0010
#define SOFTKEY_2_LONG 0x0020
#define SOFTKEY_3_LONG 0x0040
#define SOFTKEY_4_LONG 0x0080


#define EXT_KEY_1     0x0100
#define EXT_KEY_2     0x0200
#define EXT_AUX_IO1   0x0400
#define EXT_AUX_IO2   0x0800
#define EXT_SDDET     0x1000



typedef struct
{
	const char* configName;
	ScreenState configScreen;
} ConfigurationOption;

const ConfigurationOption configurationOptions[] = 
{
  { "Real 12/24",     SCREEN_CONF_R1224_SETUP },
  { "Real Time     ", SCREEN_CONF_RTIME_SETUP },
  { "Real Date     ", SCREEN_CONF_RDATE_SETUP },  
  { "Fast 12/24",      SCREEN_CONF_F1224_SETUP },
  { "Fast Ratio     ", SCREEN_CONF_FRATIO_SETUP },  
  { "Fast Start Hold", SCREEN_CONF_FSHOLD_SETUP },  
  { "Fast Start Time 1", SCREEN_CONF_FSTART1_SETUP },
  { "Fast Start Time 2", SCREEN_CONF_FSTART2_SETUP },
  { "Fast Start Time 3", SCREEN_CONF_FSTART3_SETUP },    
  { "Time Pkt Interval", SCREEN_CONF_PKTINT_SETUP },
  { "Clock Address",   SCREEN_CONF_ADDR_SETUP },
  { "TH Address",     SCREEN_CONF_THADDR_SETUP },
  { "TH Timeout", SCREEN_CONF_THTIMEOUT_SETUP },
  { "Temperature Units", SCREEN_CONF_TEMPU_SETUP },
  { "Backlight Timeout",  SCREEN_CONF_BACKLITE_SETUP },
  { "Diagnostics",    SCREEN_CONF_DIAG_SETUP },
  { "Factory Reset",      SCREEN_CONF_RESET_SETUP },  
};

#define NUM_RATIO_OPTIONS  (sizeof(ratioOptions)/sizeof(ConfigurationOption))
#define NUM_CONF_OPTIONS  (sizeof(configurationOptions)/sizeof(ConfigurationOption))

#define isLeapYear(y)  (0 == ((y) % 4))

// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

void initialize100HzTimer(void)
{
	// Set up timer 3 for 100Hz interrupts
	TCNT3 = 0;
	OCR3A = 0x0752;
	TCCR3A = 0;
	TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);
	TCCR3C = 0;
	TIFR3 |= _BV(OCF3A);
	TIMSK3 |= _BV(OCIE3A);
	
	decisecs = 0;
	fastDecisecs = 0;
}

ISR(TIMER3_COMPA_vect)
{
	static uint8_t ticks = 0;
	static uint8_t screenUpdateTicks = 0;
	static uint8_t scaleTenthsAccum = 0;
	
	if (ticks & 0x01)
		events |= EVENT_READ_INPUTS;

	if (++screenUpdateTicks >= 100)
	{
		events |= EVENT_UPDATE_SCREEN  | EVENT_SECOND_TICK;
		screenUpdateTicks = 0;
	}

	if (++ticks >= 10)
	{
		events |= EVENT_SEND_DMX;

		ticks = 0;
		if (STATUS_FAST_ACTIVE == (status & (STATUS_FAST_ACTIVE | STATUS_FAST_HOLDING)))
		{
			fastDecisecs += scaleFactor / 10;
			scaleTenthsAccum += scaleFactor % 10;
			if (scaleTenthsAccum > 10)
			{
				fastDecisecs++;
				scaleTenthsAccum -= 10;
			}
			
		}
		decisecs++;
	}
	mmc_disk_timerproc();
}

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
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
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		
		switch(rxBuffer[6])
		{
			case MRBUS_EE_DEVICE_ADDR:
				mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
				break;
		}
				txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('S' == rxBuffer[MRBUS_PKT_TYPE] && thSourceAddr == rxBuffer[MRBUS_PKT_SRC] && rxBuffer[MRBUS_PKT_LEN] >= 10)
	{
		// Format of v3 TH 
		//  data[6-7] - Float16 format temperature
		//  data[8] - Humidity (0-100)
		//  data[9] - Battery voltage
		
		uint16_t c16Temp = (((uint16_t)rxBuffer[6])<<8) + (uint16_t)rxBuffer[7];
		celsiusTemp = F16toF32((float16_t)c16Temp);
		relHumidity = rxBuffer[8];
		thVoltage = rxBuffer[9];
		thTimeout = thTimeoutReset;
	}
	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 
	return;	
}

#define SDCARD_INSERTED_PIN  PD5
#define SDCARD_CS_PIN        PD7

void initializeSD()
{
	DDRD &= ~(_BV(SDCARD_INSERTED_PIN));
	PORTD |= _BV(SDCARD_CS_PIN) | _BV(SDCARD_INSERTED_PIN);
	DDRD  |= _BV(SDCARD_CS_PIN);


	PORTB |= _BV(SDCARD_MOSI_PIN) | _BV(SDCARD_SCLK_PIN) | _BV(SDCARD_MISO_PIN);
	DDRB  |= _BV(SDCARD_MOSI_PIN) | _BV(SDCARD_SCLK_PIN);
	DDRB  &= ~(_BV(SDCARD_MISO_PIN));

	SPCR = 0x52;			/* Enable SPI function in mode 0 */
	SPSR = 0x01;			/* SPI 2x mode */
	
}

bool sd_isInserted(uint16_t switches)
{
	return (switches & EXT_SDDET)?false:true;
}


#define LCD_BACKLIGHT_PIN  PB4

void lcd_backlightOn()
{
	DDRB |= _BV(LCD_BACKLIGHT_PIN);
	PORTB |= _BV(LCD_BACKLIGHT_PIN);
}

void lcd_backlightOff()
{
	DDRB |= _BV(LCD_BACKLIGHT_PIN);
	PORTB &= ~(_BV(LCD_BACKLIGHT_PIN));
}

#define PANEL_SWITCH_MASK (_BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5))

#define EXT_SWITCH_MASK (_BV(PA3) | _BV(PA4))

void initializeSwitches(void)
{
	DDRC &= ~(PANEL_SWITCH_MASK);  // Make inputs
	PORTC |= (PANEL_SWITCH_MASK);  // Turn on pull-ups
	
	DDRA &= ~(EXT_SWITCH_MASK);
	PORTA |= EXT_SWITCH_MASK;

	DDRA &= ~(_BV(PA7)); // Make AUX_IO2 an input
	PORTA |= _BV(PA7); // turn on pullup for AUX_IO2

	DDRD &= ~(_BV(PD4)); // Make AUX_IO1 an input
	PORTD |= _BV(PD4); // turn on pull-up

}

uint16_t readSwitches()
{
	uint16_t switchStates = (PINC & PANEL_SWITCH_MASK)>>2;
	switchStates |= (PINA & _BV(PA3))?EXT_KEY_1:0;
	switchStates |= (PINA & _BV(PA4))?EXT_KEY_2:0;
	switchStates |= (PIND & _BV(PD4))?EXT_AUX_IO1:0;
	switchStates |= (PINA & _BV(PA7))?EXT_AUX_IO2:0;
	switchStates |= (PIND & _BV(SDCARD_INSERTED_PIN))?EXT_SDDET:0;
	return switchStates;
	
}

void init(void)
{
	// Kill watchdog
	MCUSR = 0;
	wdt_reset();
	wdt_enable(WDTO_1S);

	// This must be set before the SD card stuff comes on, otherwise
	// if PB4 is seen as an input it can mess with master mode
	DDRB |= _BV(LCD_BACKLIGHT_PIN);

	initializeSwitches();
	initializeSD();
	i2c_master_init();
	dmx_initialize();
	
	// Set Up LCD Panel
	wdt_reset();
	lcd_backlightOn();
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_gotoxy(0,0);

	// Set up tick timer
	initialize100HzTimer();

	// Because scale factor is used in the interrupt, read it up front
	scaleFactor = (uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_FAST_RATIO_L) + (((uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_FAST_RATIO_H))<<8);
	if (scaleFactor < 1 || scaleFactor > 999)
		scaleFactor = 40;

	// Enable interrupts
	sei();

	// Setup real time data
	initTimeData(&realTime);
	// Get current real time from RTC
	rv3129_readTime(&realTime);

	initTimeData(&fastTime);
	if (!rv3129_readFastTime(&fastTime))
		FlashToFastTimeStart(&fastTime, 0); // If the read failed, go get the last one
	
	status = eeprom_read_byte((uint8_t*)EE_ADDR_CONF_FLAGS);
	
	thSourceAddr = eeprom_read_byte((uint8_t*)EE_ADDR_TH_SRC_ADDR);
	if(0xFF == thSourceAddr)
		thSourceAddr = 0;
	eeprom_write_byte((uint8_t*)EE_ADDR_TH_SRC_ADDR, thSourceAddr);

	thTimeoutReset = (uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_L) 
			| (((uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_H)) << 8);

	if (thTimeoutReset < 10 || thTimeoutReset > 9999)
	{
		thTimeoutReset = 600;
	}
			
	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	
	updateXmitInterval = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
			| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	updateXmitInterval = max(1, updateXmitInterval);
			
	// If the update interval is garbage, set it to 2 seconds
	if (0xFFFF == updateXmitInterval)
		updateXmitInterval = 20;
		
	blTimeout = blTimeoutReset = getBacklightTimeout();

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
	else
		lcd_putc_big(0, 13);

	lcd_putc_big(1, hours % 10);
	lcd_putc_big(2, (t->minutes / 10) % 10);
	lcd_putc_big(3, t->minutes % 10);
}

void drawSplashScreen()
{
	lcd_setup_bigdigits();
	lcd_gotoxy(0,0);
	//               00000000001111111111
	//               01234567890123456789
	//                        ||
	lcd_gotoxy(0,0);
	lcd_puts_p(PSTR("     PaceSetter"));
	lcd_gotoxy(0,1);
	lcd_puts_p(PSTR(" Scale Clock System"));
	lcd_gotoxy(0,2);
	lcd_puts_p(PSTR("Iowa Scaled Eng 2021"));
	lcd_gotoxy(0,3);
	lcd_puts_p(PSTR("  www.iascaled.com  "));
	
	for(uint8_t i=0; i<20; i++)
	{
		wdt_reset();
		_delay_ms(100);
	}
	lcd_clrscr();	
}
/*
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
}*/

void drawSoftKeys_p(const char* key1Text, const char* key2Text, const char* key3Text, const char* key4Text)
{
	uint8_t i;

	lcd_gotoxy(0,3);
	for(i=0; i<20; i++)
		lcd_putc(' ');

	lcd_gotoxy(0,3);
	lcd_puts_p(key1Text);
	lcd_gotoxy(5,3);
	lcd_puts_p(key2Text);
	lcd_gotoxy(10,3);
	lcd_puts_p(key3Text);
	lcd_gotoxy(15,3);
	lcd_puts_p(key4Text);
}


void drawLittleDate(TimeData* t)
{
	lcd_gotoxy(9,2);
	printDec2Dig(t->day);
	lcd_putc('-');
	lcd_puts(monthNames[t->month]);
	lcd_puts("-20");
	printDec2DigWZero(t->year % 100);
}

void drawLittleTempHum()
{
	uint8_t tempInUnits = 0;
	
	if (status & STATUS_TEMP_DEG_F)
		tempInUnits = (uint8_t)(celsiusTemp * 9.0 / 5.0) + 32;
	else
		tempInUnits = (uint8_t)(celsiusTemp);
	lcd_gotoxy(9,2);
	//        01111111111
	//        90123456789
//	lcd_puts("           ");
	//           90123456789
	// Format is DDD*F XXX%H
	
	printDec3Dig(tempInUnits);
	lcd_putc(0xDF); // Degree symbol
	lcd_putc((status & STATUS_TEMP_DEG_F)?'F':'C');
	lcd_putc(' ');
	printDec3Dig(relHumidity);
	lcd_puts("%H");
}

void drawLittleFast(TimeData* t)
{
	lcd_gotoxy(0,2);

	if (status & (STATUS_FAST_HOLDING | STATUS_FAST_HOLD))
		lcd_puts("FH:");
	else
		lcd_puts("FC:");
	
	if (status & STATUS_FAST_AMPM)
	{
		uint8_t hours = t->hours % 12;
		if (0 == hours)
			hours = 12;
		printDec2Dig(hours);
	}
	else
		printDec2DigWZero(t->hours);
	
	printDec2DigWZero(t->minutes);
	
	if (status & STATUS_FAST_AMPM)
	{
		if (t->hours < 12)
			lcd_putc('A');
		else
			lcd_putc('P');
	}
	else
	{
		lcd_putc('h');
	}	
}


void drawLittleTime(TimeData* t, uint8_t useAMPM)
{
	lcd_gotoxy(9,2);
	lcd_putc(' ');
	
	if (useAMPM)
	{
		uint8_t hours = t->hours % 12;
		if (0 == hours)
			hours = 12;

		printDec2Dig(hours);
	}
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
	uint8_t buttonsPressed=0, colon=0;
	uint8_t configMenuOption = 0, confSaveVar=0;
	uint8_t vitalChange=0;
	TimeData tempTime;
	uint8_t tempVar = 0;
	uint16_t tempVar16 = 0;
	uint16_t kloopsPerSec=0;
	char screenLineBuffer[21];
	uint8_t buttonLongPressCounters[4] = {3,3,3,3};
	DebounceState d;
	ScreenState screenState = SCREEN_MAIN_DRAW;
	bool mounted = false;
	uint8_t sdMountCode = 42;
	uint8_t sdMountRetries = 0;
	FATFS FatFs;
	
	// Application initialization
	init();
	initDebounceState(&d, 0xFFFF);

	// Initialize MRBus core
#ifdef MRBEE
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();
#else
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();
	// MRBee doesn't have a concept of arbitration, so there's no priority level
	// We're a clock, highest priority
	mrbusSetPriority(1);
#endif

	drawSplashScreen();
	wdt_reset();
	loopCount = 0;
	kloopsPerSec = 0;

	mounted = false;

	while (1)
	{
		loopCount++;
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
		{
			PktHandler();
		}

		if (events & EVENT_SECOND_TICK)
		{
			events &= ~(EVENT_SECOND_TICK);
			if (0 == blTimeout)
			{
				lcd_backlightOff();
			}
			else if (blTimeout != BACKLIGHT_ALWAYS_ON && blTimeout > 0)
			{
				blTimeout -= 1;
			}
		}

		if (events & EVENT_READ_INPUTS)
		{
			events &= ~(EVENT_READ_INPUTS);
			buttonsPressed = debounce(readSwitches(), &d);

			// If any button is pressed, reset backlight delay
			if (buttonsPressed)
			{
				if (0 == blTimeout)
				{
					// If we're timed out and the backlight is off, eat the first keystroke
					buttonsPressed &= 0xF0;
					lcd_backlightOn();
				}
				blTimeout = blTimeoutReset;
			}


			for(uint8_t btn=0; btn<4; btn++)
			{
				if (buttonsPressed & (1<<btn))
					buttonLongPressCounters[btn] = 25; // On initial press, we set a 0.5s delay before rapid

				if (d.debounced_state & (1<<btn))
					buttonLongPressCounters[btn] = 25; // Long delay if the button is up, too
				else
				{
					// Button is down
					if (buttonLongPressCounters[btn])
						buttonLongPressCounters[btn]--;
					else
					{
						buttonsPressed |= (1<<(btn+4));
						buttonLongPressCounters[btn] = 5; // Repeat time
					}
				}
			}
		}

		if (events & EVENT_SEND_DMX)
		{
			events &= ~(EVENT_SEND_DMX);
			// Calculate DMX conditions here
			if (!(d.debounced_state & EXT_KEY_1))
			{
				dmx_setChannel(1, 255);
				dmx_setChannel(2, 255);
			} else {
				dmx_setChannel(1, 0);
				dmx_setChannel(2, 0);
			}
			
			if (!(d.debounced_state & EXT_KEY_2))
			{
				dmx_setChannel(3, 255);
			} else {
				dmx_setChannel(3, 0);
			}
			
			dmx_startXmit();
		}

		// Check SD card - try to mount if we're not mounted already
		// If the card is ejected, reset mounted status
		if (!sd_isInserted(d.debounced_state))
		{
			mounted = false;
			sdMountCode = 42;
			sdMountRetries = 0;
		} else if (!mounted) {
			// Try mounting
			if (sdMountRetries < 3)
			{
				sdMountCode = f_mount(&FatFs, "", 1);
				if (FR_OK == sdMountCode)
				{
					mounted = true;
					sdMountRetries = 0;
				} else {
					sdMountRetries++;
				}
			} // Otherwise we've exceeded our retries and we're done
			
		} else {
			// Card in and mounted
			sdMountCode = 77;
			sdMountRetries = 0;
		}

/* Example file write
		if (mounted)
		{
			FIL fsrc;
			sdFileOpenCode = f_open(&fsrc, "test2.txt", FA_READ | FA_WRITE | FA_OPEN_APPEND);
			f_printf(&fsrc, "Loop execution %d\n", loop++);
			f_close(&fsrc);
		} else {
			sdFileOpenCode = 42;
		}*/


		switch(screenState)
		{
		
			case SCREEN_MAIN_DRAW:
				lcd_gotoxy(16,1);
				lcd_puts((FAST_MODE)?"FAST":"REAL");
				drawSoftKeys_p(FAST_MODE?PSTR("REAL"):PSTR("FAST"),  FAST_MODE?(FASTHOLD_MODE?PSTR("RUN"):PSTR("HOLD")):PSTR(""), FAST_MODE?PSTR("RST"):PSTR(""), PSTR("CONF"));
				// Intentional fall-through

			case SCREEN_MAIN_UPDATE_TIME:
				lcd_gotoxy(16,0);
				if (FAST_MODE)
				{
					if (status & STATUS_FAST_AMPM)
						lcd_puts_p((fastTime.hours<12)?PSTR("AM"):PSTR("PM"));
					else
						lcd_puts_p(PSTR("24"));

					if (FASTHOLD_MODE)
						drawBigHold();
					else
						drawBigTime(&fastTime, status & STATUS_FAST_AMPM);
						
					rv3129_writeFastTime(&fastTime);
					// If we have a TH node and packet within timeout, alternate between real and TH
					if (0 != thSourceAddr && 0 != thTimeout && thAlternator >= (TH_ALTERNATOR_MAX/2))
						drawLittleTempHum();
					else
						drawLittleTime(&realTime, status & STATUS_REAL_AMPM);
				
					
					// blank out the the small fast time
					lcd_gotoxy(0,2);
					lcd_puts("        ");
					lcd_gotoxy(1,2);
					printDec2Dig(scaleFactor / 10);
					lcd_putc('.');
					lcd_putc('0' + scaleFactor % 10);
					lcd_puts(":1");

				} else {
				
					if (status & STATUS_REAL_AMPM)
						lcd_puts_p((realTime.hours<12)?PSTR("AM"):PSTR("PM"));
					else
						lcd_puts_p(PSTR("24"));
					drawBigTime(&realTime, status & STATUS_REAL_AMPM);

					// If we have a TH node and packet within timeout, alternate between real and TH
					if (0 != thSourceAddr && 0 != thTimeout && thAlternator >= (TH_ALTERNATOR_MAX/2))
						drawLittleTempHum();
					else
						drawLittleDate(&realTime);

					drawLittleFast(&fastTime);
//					drawLittleTime(&realTime, status & STATUS_REAL_AMPM);
				}

				lcd_gotoxy(19,0);
				if (mounted)
				{
					lcd_putc('M');
					//printDec2Dig(sdFileOpenCode);
				}
				else
					lcd_putc(sd_isInserted(d.debounced_state)?'S':' ');
				screenState = SCREEN_MAIN_IDLE;
				break;

			case SCREEN_FAST_RESET_DRAW:
				lcd_clrscr();
				lcd_gotoxy(3,0);
				lcd_puts_p(PSTR("!! CONFIRM !!"));
				lcd_gotoxy(2,1);
				lcd_puts_p(PSTR("Reset Fast Clock"));
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("to Which Start Time?"));
				{
					uint8_t i;
					TimeData fs;
					// This is essentially the guts of drawSoftKeys(), but done here to
					// avoid building up a bunch of buffers that I don't need or care about
					lcd_gotoxy(0,3);
					for(i=0; i<20; i++)
						lcd_putc(' ');

					for(i=0; i<3; i++)
					{
						FlashToFastTimeStart(&fs, i);
						lcd_gotoxy(0 + 5*i,3);
						printDec2DigWZero(fs.hours);
						printDec2DigWZero(fs.minutes);
					}

					lcd_gotoxy(15,3);
					lcd_puts("CNCL");
				}


				screenState = SCREEN_FAST_RESET_IDLE;
				break;
			
			case SCREEN_FAST_RESET_IDLE:
				// Switchy goodness
				if ((SOFTKEY_1 | SOFTKEY_2 | SOFTKEY_3) & buttonsPressed)
				{
					if (SOFTKEY_1 & buttonsPressed)
						FlashToFastTimeStart(&fastTime, 0);
					else if (SOFTKEY_2 & buttonsPressed)
						FlashToFastTimeStart(&fastTime, 1);
					else if (SOFTKEY_3 & buttonsPressed)
						FlashToFastTimeStart(&fastTime, 2);
					vitalChange = 1;
					lcd_clrscr();
					screenState = SCREEN_MAIN_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					lcd_clrscr();
					screenState = SCREEN_MAIN_DRAW;
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
					if (status & STATUS_FAST_HOLD)
						status |= STATUS_FAST_HOLDING;
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
					screenState = SCREEN_MAIN_DRAW;
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
				drawSoftKeys_p((configMenuOption>0)?PSTR(" UP "):PSTR(""),  (configMenuOption < NUM_CONF_OPTIONS-1)?PSTR("DOWN"):PSTR(""), PSTR("SLCT"), PSTR("BACK"));

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
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
				{
					if (configMenuOption > 0)
						configMenuOption--;
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
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
				drawSoftKeys_p(PSTR("12hr"),  PSTR("24hr"), PSTR("SAVE"), PSTR("CNCL"));
				// Intentional fall-through

			case SCREEN_CONF_R1224_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts_p(PSTR("[ ] 12H (AM/PM)"));
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("[ ] 24H (0000h)"));
				lcd_gotoxy(1, (confSaveVar)?1:2);
				lcd_putc('*');
				screenState = SCREEN_CONF_R1224_IDLE;
				break;

			case SCREEN_CONF_R1224_IDLE:

				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					confSaveVar = STATUS_REAL_AMPM;
					screenState = SCREEN_CONF_R1224_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					confSaveVar = 0;
					screenState = SCREEN_CONF_R1224_DRAW;
				}
				else if ((SOFTKEY_3 | SOFTKEY_4) & buttonsPressed)
				{
					if (SOFTKEY_3 & buttonsPressed)
					{
						status &= ~STATUS_REAL_AMPM;
						status |= ((confSaveVar)?STATUS_REAL_AMPM:0);
						storeConfiguration(status);
					}
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
				}				
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


			case SCREEN_CONF_F1224_SETUP:
				confSaveVar = (status & STATUS_FAST_AMPM);
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("Fast Time 12/24H:"));
				drawSoftKeys_p(PSTR("12hr"),  PSTR("24hr"), PSTR("SAVE"), PSTR("CNCL"));
				// Intentional fall-through

			case SCREEN_CONF_F1224_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts_p(PSTR("[ ] 12H (AM/PM)"));
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("[ ] 24H (0000h)"));
				lcd_gotoxy(1, (confSaveVar)?1:2);
				lcd_putc('*');				
				screenState = SCREEN_CONF_F1224_IDLE;
				break;

			case SCREEN_CONF_F1224_IDLE:

				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					confSaveVar = STATUS_FAST_AMPM;
					screenState = SCREEN_CONF_F1224_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					confSaveVar = 0;
					screenState = SCREEN_CONF_F1224_DRAW;
				}
				else if ((SOFTKEY_3 | SOFTKEY_4) & buttonsPressed)
				{
					if (SOFTKEY_3 & buttonsPressed)
					{
						status &= ~STATUS_FAST_AMPM;
						status |= ((confSaveVar)?STATUS_FAST_AMPM:0);
						storeConfiguration(status);
					}
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
				}				
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


			case SCREEN_CONF_FSHOLD_SETUP:
				confSaveVar = status & STATUS_FAST_HOLD;
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("Fast Start Mode"));
				drawSoftKeys_p(PSTR("RUN"),  PSTR("HOLD"), PSTR("SAVE"), PSTR("CNCL"));
				// Intentional fall-through

			case SCREEN_CONF_FSHOLD_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts_p(PSTR("[ ] RUN"));
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("[ ] HOLD"));

				lcd_gotoxy(1, (confSaveVar)?2:1);
				lcd_putc('*');				
				screenState = SCREEN_CONF_FSHOLD_IDLE;
				break;

			case SCREEN_CONF_FSHOLD_IDLE:

				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					confSaveVar = 0;
					screenState = SCREEN_CONF_FSHOLD_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					confSaveVar = STATUS_FAST_HOLD;
					screenState = SCREEN_CONF_FSHOLD_DRAW;
				}
				else if ((SOFTKEY_3 | SOFTKEY_4) & buttonsPressed)
				{
					if (SOFTKEY_3 & buttonsPressed)
					{
						status &= ~STATUS_FAST_HOLD;
						status |= ((confSaveVar)?STATUS_FAST_HOLD:0);
						storeConfiguration(status);
					}
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
				}				
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


			case SCREEN_CONF_FRATIO_SETUP:
				confSaveVar = 0;
				tempVar16 = scaleFactor;
				lcd_clrscr();
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("Fast Clock Ratio:"));
				screenState = SCREEN_CONF_FRATIO_DRAW;
				break;

			case SCREEN_CONF_FRATIO_DRAW:
				lcd_gotoxy(0,1);
				printDec2DigWZero(tempVar16/10);
				lcd_putc('.');
				lcd_putc('0' + tempVar16 % 10);
				lcd_puts(":1");

				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("            "));
				switch(confSaveVar)
				{
					case 0: // Integer
						lcd_gotoxy(0, 2);
						lcd_puts("^^");
						break;
					
					case 1: // Decimal
						lcd_gotoxy(3, 2);
						lcd_putc('^');
						break;
				}				
				drawSoftKeys_p(PSTR(" ++ "), PSTR(" -- "), PSTR(" >> "), PSTR(" GO "));

				screenState = SCREEN_CONF_FRATIO_IDLE;
				break;
				
			case SCREEN_CONF_FRATIO_IDLE:
				// Switchy goodness
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							if ((tempVar16 / 10) < 99)
								tempVar16 += 10;
							break;
						
						case 1:
							if ((tempVar16 % 10) < 9)
								tempVar16++;
							break;
					
					}
					screenState = SCREEN_CONF_FRATIO_DRAW;
				}
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							if ((tempVar16 / 10) >= 1)
								tempVar16 = max(tempVar16 - min(tempVar16, 10), 1);

							break;
						
						case 1:
							if ((tempVar16 % 10) > 0)
								tempVar16 = max(tempVar16-1, 1);
							break;
					
					}
					screenState = SCREEN_CONF_FRATIO_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					confSaveVar = (confSaveVar+1)%2;
					screenState = SCREEN_CONF_FRATIO_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					blankCursorLine();
					drawSoftKeys_p(PSTR("BACK"),  PSTR(""), PSTR("SAVE"), PSTR("CNCL"));
					screenState = SCREEN_CONF_FRATIO_CONFIRM;
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;			
				break;

			case SCREEN_CONF_FRATIO_CONFIRM:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_CONF_FRATIO_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					if (tempVar16 < 1 || tempVar16 > 999)
						tempVar16 = 40;

					eeprom_write_byte((uint8_t*)EE_ADDR_FAST_RATIO_L, tempVar16 & 0xFF);
					eeprom_write_byte((uint8_t*)EE_ADDR_FAST_RATIO_H, 0xFF & (tempVar16>>8));
					scaleFactor = (uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_FAST_RATIO_L) + (((uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_FAST_RATIO_H))<<8);
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
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
				lcd_puts_p(PSTR("Current Real Date:"));

				// Intentional fall-through

			case SCREEN_CONF_RDATE_DRAW:
				lcd_gotoxy(0, 1);
				printDec2Dig(tempTime.day);
				lcd_putc(' ');
				lcd_puts(monthNames[tempTime.month]);
				lcd_puts_p(PSTR(" 20"));
				printDec2DigWZero(tempTime.year % 100);
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("            "));
				switch(confSaveVar)
				{
					case 0: // Day
						lcd_gotoxy(0, 2);
						lcd_puts_p(PSTR("^^"));
						break;
					
					case 1: // Month
						lcd_gotoxy(3, 2);
						lcd_puts_p(PSTR("^^^"));
						break;
						
					case 2: // Year
						lcd_gotoxy(7, 2);
						lcd_puts_p(PSTR("  ^^"));
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
					lcd_puts_p(dateError?PSTR("DATE"):PSTR("    "));
					lcd_gotoxy(15,2);
					lcd_puts(dateError?PSTR("ERR "):PSTR("    "));

					drawSoftKeys_p(PSTR(" ++ "),  PSTR(" -- "), PSTR(" >> "), dateError?PSTR("    "):PSTR(" GO "));
				}
				screenState = SCREEN_CONF_RDATE_IDLE;
				break;


			case SCREEN_CONF_RDATE_IDLE:
				// Switchy goodness
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
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
					screenState = SCREEN_CONF_RDATE_DRAW;
				}
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
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
							// Year
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
						blankCursorLine();
						drawSoftKeys_p(PSTR("BACK"),  PSTR(""), PSTR("SAVE"), PSTR("CNCL"));
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
					rv3129_writeDate(&tempTime);
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
				lcd_puts_p(PSTR("Current Real Time:"));
				// Intentional fall-through

			case SCREEN_CONF_RTIME_DRAW:
				drawSoftKeys_p(PSTR(" ++ "),  PSTR(" -- "), PSTR(" >> "), PSTR(" GO "));
				lcd_gotoxy(0, 1);
				if (status & STATUS_REAL_AMPM)
				{
					uint8_t hours = tempTime.hours % 12;
					if (0 == hours)
						hours = 12;
					printDec2Dig(hours);
				}
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
				lcd_puts_p(PSTR("         "));
				lcd_gotoxy(0 + 3*confSaveVar,2);
				lcd_puts("^^");
				screenState = SCREEN_CONF_RTIME_IDLE;
				break;

			case SCREEN_CONF_RTIME_IDLE:
				// Switchy goodness
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
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
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
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
					blankCursorLine();
					drawSoftKeys_p(PSTR("BACK"),  PSTR(""), PSTR("SAVE"), PSTR("CNCL"));
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
					rv3129_writeTime(&tempTime);
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;

			case SCREEN_CONF_FSTART1_SETUP:
				tempVar = 0;
				screenState = SCREEN_CONF_FSTART_COMMON_SETUP;
				break;

			case SCREEN_CONF_FSTART2_SETUP:
				tempVar = 1;
				screenState = SCREEN_CONF_FSTART_COMMON_SETUP;
				break;

			case SCREEN_CONF_FSTART3_SETUP:
				tempVar = 2;
				screenState = SCREEN_CONF_FSTART_COMMON_SETUP;
				break;
				

			case SCREEN_CONF_FSTART_COMMON_SETUP:
				FlashToFastTimeStart(&tempTime, tempVar);
				confSaveVar = 0;
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("Fast Start Time"));
				lcd_putc('1' + tempVar);
				lcd_putc(':');
				
				// Intentional fall-through

			case SCREEN_CONF_FSTART_DRAW:
				drawSoftKeys_p(PSTR(" ++ "),  PSTR(" -- "), PSTR(" >> "), PSTR(" GO "));
				lcd_gotoxy(0, 1);
				
				if (status & STATUS_FAST_AMPM)
				{
					uint8_t hours = tempTime.hours % 12;
					if (0 == hours)
						hours = 12;

					printDec2Dig(hours);
				}
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
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
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
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
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
					screenState = SCREEN_CONF_FSTART_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					blankCursorLine();
					drawSoftKeys_p(PSTR("BACK"),  PSTR(""), PSTR("SAVE"), PSTR("CNCL"));
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
					FastTimeStartToFlash(&fastTime, tempVar);
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


//  Backlight Timeout Configuration
//  00000000001111111111
//  01234567890123456789
// [Backlight Timeout   ]
// [ Seconds: yyys      ]
// [          ^         ]
// [ +++  >>> SAVE CNCL ]
// nnn = max speed %
// yy.y = ramp time

			case SCREEN_CONF_BACKLITE_SETUP:
				lcd_clrscr();
				lcd_puts_p(PSTR("Backlight Timeout"));
				lcd_gotoxy(1, 1);
				lcd_puts_p(PSTR("Seconds: "));
				lcd_gotoxy(10, 2);
				lcd_puts("^^^^");
				tempVar = blTimeoutReset;
				if (0xFF == tempVar)
					tempVar = 0;  // For the purposes of the menu, 0 is don't turn the backlight off, not 0xFF

				drawSoftKeys_p(PSTR(" ++ "), PSTR(" -- "), PSTR("SAVE"), PSTR("CNCL"));
				screenState = SCREEN_CONF_BACKLITE_DRAW;
				break;

			case SCREEN_CONF_BACKLITE_DRAW:
				lcd_gotoxy(10,1);
				if (0 == tempVar)
					snprintf(screenLineBuffer, sizeof(screenLineBuffer), "None");
				else
					snprintf(screenLineBuffer, sizeof(screenLineBuffer), "%03ds", tempVar);
				lcd_puts(screenLineBuffer);
				screenState = SCREEN_CONF_BACKLITE_IDLE;
				break;

			case SCREEN_CONF_BACKLITE_IDLE:
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
				{
					if (tempVar < 250)
						tempVar += 10;
					screenState = SCREEN_CONF_BACKLITE_DRAW;
				}
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
				{
					if (tempVar > 0)
						tempVar -= 10;
					screenState = SCREEN_CONF_BACKLITE_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					// Put the functions back in their bitmasks
					if (0 == tempVar)
						tempVar = 0xFF;
					setBacklightTimeout(tempVar);
					blTimeout = blTimeoutReset = getBacklightTimeout();
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					// Cancel
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
				}

				// Buttons handled, clear
				buttonsPressed = 0;
				break;


//  Diagnostic Screen
//  00000000001111111111
//  01234567890123456789
// [DIAGS 0xAA TTTC xxkl]
// [TH:0xAA 0.0V [-----]]
// [TH:None      [-----]]
// [XXXXXX vx.y LR IR IL]
// [PHSA PHSB      BACK ]
// F:n - n=Y/N for faulted

			case SCREEN_CONF_DIAG_SETUP:
				{
					int8_t tempC = rv3129_readTemperature();
					
					if (status & STATUS_TEMP_DEG_F)
						tempC = (int8_t)(((int16_t)tempC) * 9 / 5 + 32);
					
					snprintf(screenLineBuffer, sizeof(screenLineBuffer), "DIAG 0x%02X %3d%c %3ukl", mrbus_dev_addr, tempC, (status & STATUS_TEMP_DEG_F)?'F':'C', min(999,kloopsPerSec));
					lcd_gotoxy(0,0);
					lcd_puts(screenLineBuffer);

					if (0 != thTimeout && 0 != thSourceAddr)
						snprintf(screenLineBuffer, sizeof(screenLineBuffer), "TH:0x%02X %1d.%1dV [%c%c%c%c%c]", thSourceAddr, thVoltage/10, thVoltage % 10,
							(d.debounced_state & EXT_KEY_1)?'-':'1',
							(d.debounced_state & EXT_KEY_2)?'-':'2', 
							(d.debounced_state & EXT_AUX_IO1)?'-':'A', 
							(d.debounced_state & EXT_AUX_IO2)?'-':'B',
							(d.debounced_state & EXT_SDDET)?'-':'C');
					else
						snprintf(screenLineBuffer, sizeof(screenLineBuffer), "TH:None      [%c%c%c%c%c]",
							(d.debounced_state & EXT_KEY_1)?'-':'1',
							(d.debounced_state & EXT_KEY_2)?'-':'2', 
							(d.debounced_state & EXT_AUX_IO1)?'-':'A', 
							(d.debounced_state & EXT_AUX_IO2)?'-':'B',
							(d.debounced_state & EXT_SDDET)?'-':'C');
					lcd_gotoxy(0,1);
					lcd_puts(screenLineBuffer);
					
					
					snprintf(screenLineBuffer, sizeof(screenLineBuffer), "%06lX v%d.%d", GIT_REV, SWREV_MAJOR, SWREV_MINOR);
					lcd_gotoxy(0,2);
					lcd_puts(screenLineBuffer);
				}
				
				
/*				if (0 != thTimeout && 0 != thSourceAddr)
				{

					lcd_puts_p(PSTR("TH :"));
					printDec2Dig(thVoltage / 10);
					lcd_putc('.');
					lcd_putc((thVoltage % 10) + '0');
					lcd_putc('V');
				}
				else
					lcd_puts("         ");
			
				
				{
					int8_t tempC = rv3129_readTemperature();
					snprintf(screenLineBuffer, sizeof(screenLineBuffer), "%3.3dC", tempC);
					lcd_gotoxy(0, 2);
					lcd_puts(screenLineBuffer);
				}*/
				
				drawSoftKeys_p(PSTR(""),  PSTR(""), PSTR(""), PSTR("BACK"));
				screenState = SCREEN_CONF_DIAG_IDLE;
				break;

			case SCREEN_CONF_DIAG_IDLE:
				if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				buttonsPressed = 0;

				// 1-sec refreshes are handled in the main loop as a special case

				break;
				
//  Factory Reset Screen
//  00000000001111111111
//  01234567890123456789
// [CLEAR ALL SETTINGS? ]
// [ Press YES n more   ]
// [ times to confirm   ]
// [ YES! YES!     CNCL ]

			case SCREEN_CONF_RESET_SETUP:
				lcd_clrscr();
				confSaveVar = 5;
				screenState = SCREEN_CONF_RESET_DRAW;
				break;

			case SCREEN_CONF_RESET_DRAW:
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("CLEAR ALL SETTINGS?"));

				if (confSaveVar > 0)
				{
					lcd_gotoxy(1,1);
					lcd_puts_p(PSTR("Press YES n more"));
					lcd_gotoxy(1,2);
					lcd_puts_p(PSTR("times to confirm"));
					lcd_gotoxy(11,1);
					lcd_putc(confSaveVar + '0');
					drawSoftKeys_p(PSTR("YES!"), PSTR(""), PSTR(""), PSTR("CNCL"));
				} else { 
					lcd_gotoxy(0,1);
					lcd_puts_p(PSTR("    REALLY ERASE    "));
					lcd_gotoxy(0,2);
					lcd_puts_p(PSTR("    EVERYTHING?     "));
					drawSoftKeys_p(PSTR(""), PSTR("YES!"), PSTR(""), PSTR("CNCL"));
				}
				screenState = SCREEN_CONF_RESET_IDLE;
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;

			case SCREEN_CONF_RESET_IDLE:
				if (SOFTKEY_1 & buttonsPressed)
				{
					if (confSaveVar > 0)
						confSaveVar--;
					screenState = SCREEN_CONF_RESET_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					if (confSaveVar == 0)
					{
						lcd_clrscr();
						lcd_gotoxy(0,0);
						lcd_puts_p(PSTR("RESETTING..."));
						lcd_gotoxy(0,1);
						lcd_puts_p(PSTR("EEPROM... "));
						firstTimeInitConfig();
						lcd_puts_p(PSTR("done"));
						lcd_gotoxy(0,2);
						lcd_puts_p(PSTR("Reboot"));
						while(1); // Just stall and wait for a WDT reset
					}
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;
				
				

			case SCREEN_CONF_PKTINT_SETUP:
				tempVar16 = min(9999, max(updateXmitInterval, 1));
				confSaveVar = 0;
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("Time Pkt Interval:"));
				// Intentional fall-through

			case SCREEN_CONF_PKTINT_DRAW:
				lcd_gotoxy(0, 1);
				printDec3DigWZero(tempVar16/10);
				lcd_putc('.');
				lcd_putc('0' + tempVar16 % 10);
				lcd_puts(" sec");
				lcd_gotoxy(0,2);
				lcd_puts("            ");
				if (3 == confSaveVar)
					lcd_gotoxy(confSaveVar+1, 2);
				else
					lcd_gotoxy(confSaveVar, 2);
				
				lcd_putc('^');
				drawSoftKeys_p(PSTR(" ++ "),  PSTR(" -- "), PSTR(" >> "), PSTR(" GO "));
				screenState = SCREEN_CONF_PKTINT_IDLE;
				break;

			case SCREEN_CONF_PKTINT_IDLE:
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							if ((tempVar16 / 1000) % 10 != 9)
								tempVar16 += 1000;
							else
								tempVar16 -= 9000;
							break;
						case 1:
							if ((tempVar16 / 100) % 10 != 9)
								tempVar16 += 100;
							else
								tempVar16 -= 900;
							break;
						case 2:
							if ((tempVar16 / 10) % 10 != 9)
								tempVar16 += 10;
							else
								tempVar16 -= 90;
							break;
						case 3:
							if (tempVar16 % 10 != 9)
								tempVar16 += 1;
							else
								tempVar16 -= 9;
							break;
					}
					screenState = SCREEN_CONF_PKTINT_DRAW;
				}
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							if ((tempVar16 / 1000) % 10 != 0)
								tempVar16 -= 1000;
							else
								tempVar16 += 9000;
							break;
						case 1:
							if ((tempVar16 / 100) % 10 != 0)
								tempVar16 -= 100;
							else
								tempVar16 += 900;
							break;
						case 2:
							if ((tempVar16 / 10) % 10 != 0)
								tempVar16 -= 10;
							else
								tempVar16 += 90;
							break;
						case 3:
							if (tempVar16 % 10 != 0)
								tempVar16 -= 1;
							else
								tempVar16 += 9;
							break;
					}
					screenState = SCREEN_CONF_PKTINT_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					confSaveVar = (confSaveVar+1)%4;
					screenState = SCREEN_CONF_PKTINT_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					blankCursorLine();
					drawSoftKeys_p(PSTR("BACK"),  PSTR(""), PSTR("SAVE"), PSTR("CNCL"));
					screenState = SCREEN_CONF_PKTINT_CONFIRM;
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


			case SCREEN_CONF_PKTINT_CONFIRM:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_CONF_PKTINT_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{

					eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, 0xFF & tempVar16);
					eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, 0xFF & (tempVar16>>8));

					updateXmitInterval = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
						| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				buttonsPressed = 0;	
				break;

			// END PACKET INTERVAL


				
			case SCREEN_CONF_ADDR_SETUP:
				tempVar = mrbus_dev_addr;
				if (0x00 == tempVar || 0xFF == tempVar)
					tempVar = 0x01;

				confSaveVar = 0;
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("Master Clock Addr:"));

				// Intentional fall-through

			case SCREEN_CONF_ADDR_DRAW:
				lcd_gotoxy(0, 1);
				lcd_puts("0x");
				printHex(tempVar);
				lcd_gotoxy(0,2);
				lcd_puts("            ");

				lcd_gotoxy(2 + confSaveVar, 2);
				lcd_putc('^');
				drawSoftKeys_p(PSTR(" ++ "),  PSTR(" -- "), PSTR(" >> "), PSTR(" GO "));
				screenState = SCREEN_CONF_ADDR_IDLE;
				break;


			case SCREEN_CONF_ADDR_IDLE:
				// Switchy goodness
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
				{
					if (0 == confSaveVar)
						tempVar += 0x10;
					else
						tempVar += 0x01;

					if (tempVar == 0xFF)
						tempVar = 0xFE;
					else if (tempVar == 0x00)
						tempVar = 0x01;

					screenState = SCREEN_CONF_ADDR_DRAW;
				}
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
				{
					if (0 == confSaveVar)
						tempVar -= 0x10;
					else
						tempVar -= 0x01;

					if (tempVar == 0xFF)
						tempVar = 0xFE;
					else if (tempVar == 0x00)
						tempVar = 0x01;

					screenState = SCREEN_CONF_ADDR_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					confSaveVar = (confSaveVar+1)%2;
					screenState = SCREEN_CONF_ADDR_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					blankCursorLine();
					drawSoftKeys_p(PSTR("BACK"),  PSTR(""), PSTR("SAVE"), PSTR("CNCL"));
					screenState = SCREEN_CONF_ADDR_CONFIRM;
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


			case SCREEN_CONF_ADDR_CONFIRM:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_CONF_ADDR_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, tempVar);
					mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				buttonsPressed = 0;	
				break;

			case SCREEN_CONF_TEMPU_SETUP:
				confSaveVar = (status & STATUS_REAL_AMPM);
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("Temperature Units"));
				drawSoftKeys_p(PSTR(" F  "),  PSTR(" C  "), PSTR("SAVE"), PSTR("CNCL"));
				// Intentional fall-through

			case SCREEN_CONF_TEMPU_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts_p(PSTR("[ ] Degrees F"));
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("[ ] Degrees C"));
				lcd_gotoxy(1, (confSaveVar)?1:2);
				lcd_putc('*');
				screenState = SCREEN_CONF_TEMPU_IDLE;
				break;

			case SCREEN_CONF_TEMPU_IDLE:

				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					confSaveVar = STATUS_TEMP_DEG_F;
					screenState = SCREEN_CONF_TEMPU_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
				{
					confSaveVar = 0;
					screenState = SCREEN_CONF_TEMPU_DRAW;
				}
				else if ((SOFTKEY_3 | SOFTKEY_4) & buttonsPressed)
				{
					if (SOFTKEY_3 & buttonsPressed)
					{
						status &= ~STATUS_TEMP_DEG_F;
						status |= ((confSaveVar)?STATUS_TEMP_DEG_F:0);
						storeConfiguration(status);
					}
					lcd_clrscr();
					screenState = SCREEN_CONF_MENU_DRAW;
				}				
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


			case SCREEN_CONF_THADDR_SETUP:
				tempVar = thSourceAddr;
				if (0xFF == tempVar)
					tempVar = 0x01;

				confSaveVar = 0;
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("Temp/Humidity Addr:"));
				// Intentional fall-through

			case SCREEN_CONF_THADDR_DRAW:

				if (0x00 == tempVar)
				{
					lcd_gotoxy(0, 1);
					lcd_puts_p(PSTR("T/H Off"));
				}
				else
				{
					lcd_gotoxy(0, 1);
					lcd_puts("0x");
					printHex(tempVar);
					lcd_puts_p(PSTR("   "));
				}
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("            "));

				lcd_gotoxy(2 + confSaveVar, 2);
				lcd_putc('^');
				drawSoftKeys_p(PSTR(" ++ "),  PSTR(" -- "), PSTR(" >> "), PSTR(" GO "));
				screenState = SCREEN_CONF_THADDR_IDLE;
				break;


			case SCREEN_CONF_THADDR_IDLE:
				// Switchy goodness
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
				{
					if (0 == confSaveVar)
						tempVar += 0x10;
					else
						tempVar += 0x01;

					if (tempVar == 0xFF)
						tempVar = 0xFE;

					screenState = SCREEN_CONF_THADDR_DRAW;
				}
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
				{
					if (0 == confSaveVar)
						tempVar -= 0x10;
					else
						tempVar -= 0x01;

					if (tempVar == 0xFF)
						tempVar = 0xFE;

					screenState = SCREEN_CONF_THADDR_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					confSaveVar = (confSaveVar+1)%2;
					screenState = SCREEN_CONF_THADDR_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					blankCursorLine();
					drawSoftKeys_p(PSTR("BACK"),  PSTR(""), PSTR("SAVE"), PSTR("CNCL"));
					screenState = SCREEN_CONF_THADDR_CONFIRM;
				}
				// Buttons handled, clear
				buttonsPressed = 0;	
				break;


			case SCREEN_CONF_THADDR_CONFIRM:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_CONF_THADDR_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					eeprom_write_byte((uint8_t*)EE_ADDR_TH_SRC_ADDR, tempVar);
					thSourceAddr = eeprom_read_byte((uint8_t*)EE_ADDR_TH_SRC_ADDR);
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				buttonsPressed = 0;	
				break;

			case SCREEN_CONF_THTIMEOUT_SETUP:
				confSaveVar = 0;
				tempVar16 = thTimeoutReset;
				lcd_clrscr();
				lcd_gotoxy(0,0);
				lcd_puts_p(PSTR("Temp/Hum Timeout:"));
				screenState = SCREEN_CONF_THTIMEOUT_DRAW;
				break;

			case SCREEN_CONF_THTIMEOUT_DRAW:
				lcd_gotoxy(0,1);
				printDec4DigWZero(tempVar16);
				lcd_puts_p(PSTR(" sec"));

				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("            "));
				lcd_gotoxy(confSaveVar, 2);
				lcd_putc('^');
				drawSoftKeys_p(PSTR(" ++ "), PSTR(" -- "), PSTR(" >> "), PSTR(" GO "));

				screenState = SCREEN_CONF_THTIMEOUT_IDLE;
				break;
				
			case SCREEN_CONF_THTIMEOUT_IDLE:
				// Switchy goodness
				if ((SOFTKEY_1 | SOFTKEY_1_LONG) & buttonsPressed)
				{
					switch(confSaveVar)
					{

						case 0:
							if ((tempVar16 / 1000) % 10 != 9)
								tempVar16 += 1000;
							else
								tempVar16 -= 9000;
							break;
						case 1:
							if ((tempVar16 / 100) % 10 != 9)
								tempVar16 += 100;
							else
								tempVar16 -= 900;
							break;
						case 2:
							if ((tempVar16 / 10) % 10 != 9)
								tempVar16 += 10;
							else
								tempVar16 -= 90;
							break;
						case 3:
							if (tempVar16 % 10 != 9)
								tempVar16 += 1;
							else
								tempVar16 -= 9;
							break;
					}
					screenState = SCREEN_CONF_THTIMEOUT_DRAW;
				}
				else if ((SOFTKEY_2 | SOFTKEY_2_LONG) & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							if ((tempVar16 / 1000) % 10 != 0)
								tempVar16 -= 1000;
							else
								tempVar16 += 9000;
							break;
						case 1:
							if ((tempVar16 / 100) % 10 != 0)
								tempVar16 -= 100;
							else
								tempVar16 += 900;
							break;
						case 2:
							if ((tempVar16 / 10) % 10 != 0)
								tempVar16 -= 10;
							else
								tempVar16 += 90;
							break;
						case 3:
							if (tempVar16 % 10 != 0)
								tempVar16 -= 1;
							else
								tempVar16 += 9;
							break;
					}
					screenState = SCREEN_CONF_THTIMEOUT_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					confSaveVar = (confSaveVar+1)%4;
					screenState = SCREEN_CONF_THTIMEOUT_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					blankCursorLine();
					drawSoftKeys_p(PSTR("BACK"),  PSTR(""), PSTR("SAVE"), PSTR("CNCL"));
					screenState = SCREEN_CONF_THTIMEOUT_CONFIRM;
				}
				
				// Buttons handled, clear
				buttonsPressed = 0;
				break;

			case SCREEN_CONF_THTIMEOUT_CONFIRM:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_CONF_THTIMEOUT_DRAW;
				}
				else if (SOFTKEY_3 & buttonsPressed)
				{
					if (tempVar16 < 10 || tempVar16 > 9999)
						tempVar16 = 10;

					eeprom_write_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_L, tempVar16 & 0xFF);
					eeprom_write_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_H, 0xFF & (tempVar16>>8));

					thTimeoutReset = (uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_L) 
						| (((uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_H)) << 8);

					screenState = SCREEN_CONF_MENU_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				buttonsPressed = 0;	
				break;
				
			
			default:
				lcd_gotoxy(0,1);
				lcd_puts_p(PSTR("Code off in lalaland"));
				lcd_gotoxy(0,2);
				lcd_puts_p(PSTR("Call Nathan"));

				break;		
		
		}
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			if(FAST_MODE && !FASTHOLD_MODE && fastDecisecs >= 10)
			{
				uint8_t fastTimeSecs = fastDecisecs / 10;
				incrementTime(&fastTime, fastTimeSecs);
				fastDecisecs -= fastTimeSecs * 10;
				if (screenState == SCREEN_MAIN_IDLE || screenState == SCREEN_MAIN_UPDATE_TIME)
					screenState = SCREEN_MAIN_UPDATE_TIME;
			}
		}

		if (events & EVENT_UPDATE_SCREEN)
		{
			events &= ~(EVENT_UPDATE_SCREEN);
			// Reading optimizer
			// If we don't know the date or if we're in the range where we're at risk of 
			// changing dates
			rv3129_readTime(&realTime);
			
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
				
				case SCREEN_CONF_DIAG_IDLE:
					// Get loop iterations here
					screenState = SCREEN_CONF_DIAG_SETUP;
					break;
				
				case SCREEN_MAIN_DRAW:
				default:
					break;
			}

			kloopsPerSec = loopCount / 1000;
			if(0 != thTimeout)
				thTimeout--;
				
			if (++thAlternator >= TH_ALTERNATOR_MAX)
				thAlternator = 0;
				
			loopCount = 0;
		}		

		/* If we need to send a packet and we're not already busy... */
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			if (decisecs >= updateXmitInterval)
			{
				vitalChange = 1;
				decisecs -= updateXmitInterval;
			}
		}
		
		if (vitalChange && !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t flags = 0;
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

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
			
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 18;
			txBuffer[5] = 'T';
			txBuffer[6] = realTime.hours;
			txBuffer[7] = realTime.minutes;
			txBuffer[8] = realTime.seconds;
			txBuffer[9] = flags;
			txBuffer[10] = fastTime.hours;
			txBuffer[11] = fastTime.minutes;
			txBuffer[12] = fastTime.seconds;
			txBuffer[13] = 0xFF & (scaleFactor>>8);
			txBuffer[14] = 0xFF & scaleFactor;
			txBuffer[15] = 0xFF & (realTime.year>>4);
			txBuffer[16] = ((realTime.year<<4) & 0xF0) | (0x0F & realTime.month);
			txBuffer[17] = realTime.day;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			vitalChange = 0;
		}



		// If we have a packet to be transmitted, try to send it here
		while(mrbusPktQueueDepth(&mrbusTxQueue))
		{
			wdt_reset();

#ifdef MRBUS
			if (0 != mrbusTransmit()) // 0 is success, all others are failure
			{
				// If we're here, we failed to start transmission due to somebody else transmitting
				// Given that our transmit buffer is full, priority one should be getting that data onto
				// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
				// to get bus time.

				// We want to wait 20ms before we try a retransmit to avoid hammering the bus
				// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
				// need to check roughly every millisecond to see if we have a new packet
				// so that we don't miss things we're receiving while waiting to transmit
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();
				}
			}
#else
			// MRBee is simple - because the XBee has a buffer, we never fail
			mrbeeTransmit();
#endif
		}
	}
}



