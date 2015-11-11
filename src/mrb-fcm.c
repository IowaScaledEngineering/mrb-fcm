/*************************************************************************
Title:    MRBus Fast Clock Master
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012-2015 Nathan D. Holmes

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
volatile uint8_t scaleTenthsAccum = 0;
uint16_t scaleFactor = 10;
volatile uint8_t status=0;

uint16_t kelvinTemp = 0;
uint8_t relHumidity = 0;
uint8_t thVoltage = 0;
uint8_t thAlternator = 0;

#define TH_ALTERNATOR_MAX 8

#define STATUS_READ_INPUTS 0x01
#define STATUS_FAST_ACTIVE 0x02
#define STATUS_FAST_AMPM   0x04
#define STATUS_REAL_AMPM   0x08
#define STATUS_FAST_HOLDING 0x10 // This hold flag indicates we're actually in hold
#define STATUS_FAST_HOLD   0x20  // This flag indicates that we start going into fast in hold
#define STATUS_TEMP_DEG_F  0x40

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

#define EE_ADDR_FAST_RATIO_H   0x3A
#define EE_ADDR_FAST_RATIO_L   0x3B
#define EE_ADDR_TH_SRC_ADDR    0x3C
#define EE_ADDR_TH_TIMEOUT_L   0x3D
#define EE_ADDR_TH_TIMEOUT_H   0x3E

uint32_t loopCount = 0;

void blankCursorLine()
{
	lcd_gotoxy(0,2);
	lcd_puts("                    ");
}

void storeConfiguration(uint8_t confStatus)
{
	eeprom_write_byte((uint8_t*)(uint16_t)EE_ADDR_CONF_FLAGS, (confStatus & (STATUS_FAST_AMPM | STATUS_REAL_AMPM | STATUS_FAST_HOLD | STATUS_TEMP_DEG_F)));
}

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
		
	SCREEN_CONF_DIAG_SETUP = 250,
	SCREEN_CONF_DIAG_DRAW  = 251,
	SCREEN_CONF_DIAG_IDLE  = 252,
	
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
  { "Node Address",   SCREEN_CONF_ADDR_SETUP },
  { "TH Address",     SCREEN_CONF_THADDR_SETUP },
  { "TH Timeout", SCREEN_CONF_THTIMEOUT_SETUP },
  { "Temperature Units", SCREEN_CONF_TEMPU_SETUP },
  { "Diagnostics",    SCREEN_CONF_DIAG_SETUP },  
};

#define NUM_RATIO_OPTIONS  (sizeof(ratioOptions)/sizeof(ConfigurationOption))
#define NUM_CONF_OPTIONS  (sizeof(configurationOptions)/sizeof(ConfigurationOption))

#define isLeapYear(y)  (0 == ((y) % 4))

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
	scaleTenthsAccum = 0;
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
		screenUpdateDecisecs++;
	}
}


volatile uint16_t busVoltageAccum=0;
volatile uint16_t busVoltage=0;
volatile uint8_t busVoltageCount=0;

ISR(ADC_vect)
{
	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/3) / 5 * 1024
        //So multiply by 150, divide by 1024, or multiply by 75 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 75) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
	}
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
	else if ('S' == rxBuffer[MRBUS_PKT_TYPE] && thSourceAddr == rxBuffer[MRBUS_PKT_SRC] && rxBuffer[MRBUS_PKT_LEN] >= 11)
	{
		// This might be a TH packet coming in
		// P:FF 20 0B 60 7B 53 00 12 5B 3C 20
		kelvinTemp = (((uint16_t)rxBuffer[7])<<8) + (uint16_t)rxBuffer[8];
		relHumidity = rxBuffer[9];
		thTimeout = thTimeoutReset;
		thVoltage = rxBuffer[10];
	}
	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 
	return;	
}

#define LCD_BACKLIGHT_PIN  PD3
#define BUZZER_PIN PC4

void readDS1302(uint8_t* ds1302Buffer)
{
	ds1302_transact(0xBF, 7, ds1302Buffer);
	realTime.seconds = (ds1302Buffer[0] & 0x0F) + 10 * (((ds1302Buffer[0] & 0x70)>>4));
	realTime.minutes = (ds1302Buffer[1] & 0x0F) + 10 * (((ds1302Buffer[1] & 0x70)>>4));
	realTime.hours = (ds1302Buffer[2] & 0x0F) + 10 * (((ds1302Buffer[2] & 0x30)>>4));			
	realTime.day = (ds1302Buffer[3] & 0x0F) + 10 * (((ds1302Buffer[3] & 0x30)>>4));
	realTime.month = (ds1302Buffer[4] & 0x0F) + ((ds1302Buffer[4] & 0x10)?10:0);
	realTime.year = 2000 + (ds1302Buffer[6] & 0x0F) + 10 * (ds1302Buffer[6]>>4);
}


void init(void)
{
	uint8_t ds1302Buffer[10];	
	// Kill watchdog
    MCUSR = 0;
	wdt_reset();
	wdt_disable();

	DDRD &= ~(0xF2);
	DDRC &= ~(0x10);
	DDRD |= _BV(LCD_BACKLIGHT_PIN);
	DDRC |= _BV(BUZZER_PIN);
	
	PORTD |= _BV(LCD_BACKLIGHT_PIN);
	PORTC &= ~_BV(BUZZER_PIN);
		
	ds1302_init();
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();


	// Setup ADC
	ADMUX  = 0x46;  // AVCC reference; ADC6 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	busVoltageAccum = 0;
	busVoltageCount = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);

	// Setup real time data
	initTimeData(&realTime);
	readDS1302(ds1302Buffer);

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
	FlashToFastTimeStart(&fastTime, 0);
	
	status = eeprom_read_byte((uint8_t*)EE_ADDR_CONF_FLAGS);
	
	scaleFactor = (uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_FAST_RATIO_L) + (((uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_FAST_RATIO_H))<<8);
	if (scaleFactor < 10 || scaleFactor > 999)
	{
		scaleFactor = 10;
		eeprom_write_byte((uint8_t*)EE_ADDR_FAST_RATIO_L, scaleFactor & 0xFF);
		eeprom_write_byte((uint8_t*)EE_ADDR_FAST_RATIO_H, 0xFF & (scaleFactor>>8));
	}	

	thSourceAddr = eeprom_read_byte((uint8_t*)EE_ADDR_TH_SRC_ADDR);
	if(0xFF == thSourceAddr)
		thSourceAddr = 0;
	eeprom_write_byte((uint8_t*)EE_ADDR_TH_SRC_ADDR, thSourceAddr);

	thTimeoutReset = (uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_L) 
			| (((uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_H)) << 8);


	if (thTimeoutReset < 10 || thTimeoutReset > 9999)
	{
		thTimeoutReset = 10;
		eeprom_write_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_L, thTimeoutReset & 0xFF);
		eeprom_write_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_H, 0xFF & (thTimeoutReset>>8));
		thTimeoutReset = (uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_L) 
			| (((uint16_t)eeprom_read_byte((uint8_t*)EE_ADDR_TH_TIMEOUT_H)) << 8);			
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
	_delay_ms(2000);
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
		tempInUnits = (uint8_t)( 32 + (((kelvinTemp - 4370) * 9) / (5 * 16)) );
	else
		tempInUnits = 0xFF & ((kelvinTemp - 4370)>>4);
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
	printDec3Dig(relHumidity / 2);
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
	uint8_t ds1302Buffer[10];
	uint8_t buttonsPressed=0, colon=0;
	uint8_t configMenuOption = 0, confSaveVar=0;
	uint8_t vitalChange=0;
	TimeData tempTime;
	uint8_t tempVar = 0;
	uint16_t tempVar16 = 0;
	uint16_t kloopsPerSec=0;
	ScreenState screenState = SCREEN_MAIN_DRAW;
	// Application initialization
	init();

	DDRD &= ~(0xF8);

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

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

	sei();	

	loopCount = 0;
	kloopsPerSec = 0;

	while (1)
	{
		loopCount++;

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
		{
			PktHandler();
		}

		if (status & STATUS_READ_INPUTS)
		{
			status &= ~(STATUS_READ_INPUTS);
			buttonsPressed = debounce((PIND & 0xF8) | (PINC & _BV(PC4)>>2));
		}

		switch(screenState)
		{
		
			case SCREEN_MAIN_DRAW:
				lcd_gotoxy(16,1);
				lcd_puts((FAST_MODE)?"FAST":"REAL");
				drawSoftKeys(FAST_MODE?"REAL":"FAST",  FAST_MODE?(FASTHOLD_MODE?"RUN":"HOLD"):"", FAST_MODE?"RST":"", "CONF");
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
						lcd_puts((realTime.hours<12)?"AM":"PM");
					else
						lcd_puts("24");
					drawBigTime(&realTime, status & STATUS_REAL_AMPM);

					// If we have a TH node and packet within timeout, alternate between real and TH
					if (0 != thSourceAddr && 0 != thTimeout && thAlternator >= (TH_ALTERNATOR_MAX/2))
						drawLittleTempHum();
					else
						drawLittleDate(&realTime);

					drawLittleFast(&fastTime);
//					drawLittleTime(&realTime, status & STATUS_REAL_AMPM);
				}
			
				screenState = SCREEN_MAIN_IDLE;
				break;

			case SCREEN_FAST_RESET_DRAW:
				lcd_clrscr();
				lcd_gotoxy(3,0);
				lcd_puts("!! CONFIRM !!");
				lcd_gotoxy(2,1);
				lcd_puts("Reset Fast Clock");
				lcd_gotoxy(0,2);
				lcd_puts("to Which Start Time?");
				{
					uint8_t i;
					TimeData fs;
					// This is essentially the guts of drawSoftKeys(), but done here to
					// avoid building up a bunch of buffers that I don't need or care about
					lcd_gotoxy(0,4);
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
				lcd_puts("Fast Time 12/24H:");
				drawSoftKeys("12hr",  "24hr", "SAVE", "CNCL");
				// Intentional fall-through

			case SCREEN_CONF_F1224_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts("[ ] 12H (AM/PM)");
				lcd_gotoxy(0,2);
				lcd_puts("[ ] 24H (0000h)");
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
				lcd_puts("Fast Start Mode");
				drawSoftKeys("RUN",  "HOLD", "SAVE", "CNCL");
				// Intentional fall-through

			case SCREEN_CONF_FSHOLD_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts("[ ] RUN");
				lcd_gotoxy(0,2);
				lcd_puts("[ ] HOLD");

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
				lcd_puts("Fast Clock Ratio:");
				screenState = SCREEN_CONF_FRATIO_DRAW;
				break;

			case SCREEN_CONF_FRATIO_DRAW:
				lcd_gotoxy(0,1);
				printDec2DigWZero(tempVar16/10);
				lcd_putc('.');
				lcd_putc('0' + tempVar16 % 10);
				lcd_puts(":1");

				lcd_gotoxy(0,2);
				lcd_puts("            ");
				switch(confSaveVar)
				{
					case 0: // Integer
						lcd_gotoxy(0, 2);
						lcd_puts("^^");
						break;
					
					case 1: // Decimal
						lcd_gotoxy(3, 2);
						lcd_puts("^");						
						break;
				}				
				drawSoftKeys(" ++ ", " -- ", " >> ", " GO ");

				screenState = SCREEN_CONF_FRATIO_IDLE;
				break;
				
			case SCREEN_CONF_FRATIO_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
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
				else if (SOFTKEY_2 & buttonsPressed)
				{
					switch(confSaveVar)
					{
						case 0:
							if ((tempVar16 / 10) > 1)
								tempVar16 -= 10;
							break;
						
						case 1:
							if ((tempVar16 % 10) > 0)
								tempVar16--;
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
					drawSoftKeys("BACK",  "", "SAVE", "CNCL");
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
					if (tempVar16 < 10 || tempVar16 > 999)
						tempVar16 = 10;

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
				lcd_puts("Current Real Date:");

				// Intentional fall-through

			case SCREEN_CONF_RDATE_DRAW:
				lcd_gotoxy(0, 1);
				printDec2Dig(tempTime.day);
				lcd_putc(' ');
				lcd_puts(monthNames[tempTime.month]);
				lcd_puts(" 20");
				printDec2DigWZero(tempTime.year % 100);
				lcd_gotoxy(0,2);
				lcd_puts("            ");
				switch(confSaveVar)
				{
					case 0: // Day
						lcd_gotoxy(0, 2);
						lcd_puts("^^");
						break;
					
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
				screenState = SCREEN_CONF_RDATE_IDLE;
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
					screenState = SCREEN_CONF_RDATE_DRAW;
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
					ds1302Buffer[0] = (tempTime.day % 10) | (0x30 & (((tempTime.day/10)%10)<<4));
					ds1302_transact(0x86, 1, ds1302Buffer);
					ds1302Buffer[0] = (tempTime.month % 10) | ((tempTime.month >= 10)?0x10:0);
					ds1302_transact(0x88, 1, ds1302Buffer);
					ds1302Buffer[0] = (tempTime.year % 10) | (0xF0 & (((tempTime.year/10)%10)<<4));
					ds1302_transact(0x8C, 1, ds1302Buffer);

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
					blankCursorLine();
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
				lcd_puts("Fast Start Time");
				lcd_putc('1' + tempVar);
				lcd_putc(':');
				
				// Intentional fall-through

			case SCREEN_CONF_FSTART_DRAW:
				drawSoftKeys(" ++ ",  " -- ", " >> ", " GO ");
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
					screenState = SCREEN_CONF_FSTART_DRAW;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					blankCursorLine();
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

			case SCREEN_CONF_DIAG_SETUP:
				lcd_gotoxy(0,0);
				lcd_puts("Diagnostics");
				lcd_gotoxy(0,1);
				lcd_puts("Bus:");
				printDec2Dig(busVoltage / 10);
				lcd_putc('.');
				lcd_putc((busVoltage % 10) + '0');					
				lcd_putc('V');
				

				lcd_gotoxy(0,2);			
				if (0 != thTimeout && 0 != thSourceAddr)
				{

					lcd_puts("TH :");
					printDec2Dig(thVoltage / 10);
					lcd_putc('.');
					lcd_putc((thVoltage % 10) + '0');					
					lcd_putc('V');
				}
				else
					lcd_puts("         ");
			
				
				lcd_gotoxy(11,1);
				lcd_puts("Addr:0x");
				printHex(mrbus_dev_addr);
				
				lcd_gotoxy(12,0);
				printDec4Dig(kloopsPerSec);
				lcd_puts("kl/s");
				
				drawSoftKeys("RFSH",  "", "", "BACK");
				screenState = SCREEN_CONF_DIAG_IDLE;
				break;

			case SCREEN_CONF_DIAG_IDLE:
				if (SOFTKEY_1 & buttonsPressed)
				{
					screenState = SCREEN_CONF_DIAG_SETUP;
				}
				else if (SOFTKEY_4 & buttonsPressed)
				{
					screenState = SCREEN_CONF_MENU_DRAW;
				}
				buttonsPressed = 0;		
				break;

			case SCREEN_CONF_PKTINT_SETUP:
				tempVar16 = min(9999, max(updateXmitInterval, 1));
				confSaveVar = 0;
				lcd_gotoxy(0,0);
				lcd_puts("Time Pkt Interval:");
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
				
				lcd_puts("^");
				drawSoftKeys(" ++ ",  " -- ", " >> ", " GO ");
				screenState = SCREEN_CONF_PKTINT_IDLE;
				break;

			case SCREEN_CONF_PKTINT_IDLE:
				if (SOFTKEY_1 & buttonsPressed)
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
				else if (SOFTKEY_2 & buttonsPressed)
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
					drawSoftKeys("BACK",  "", "SAVE", "CNCL");
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
				lcd_puts("Fast Clock Addr:");

				// Intentional fall-through

			case SCREEN_CONF_ADDR_DRAW:
				lcd_gotoxy(0, 1);
				lcd_puts("0x");
				printHex(tempVar);
				lcd_gotoxy(0,2);
				lcd_puts("            ");

				lcd_gotoxy(2 + confSaveVar, 2);
				lcd_puts("^");
				drawSoftKeys(" ++ ",  " -- ", " >> ", " GO ");
				screenState = SCREEN_CONF_ADDR_IDLE;
				break;


			case SCREEN_CONF_ADDR_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
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
				else if (SOFTKEY_2 & buttonsPressed)
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
					drawSoftKeys("BACK",  "", "SAVE", "CNCL");
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
				lcd_puts("Temperature Units");
				drawSoftKeys(" F  ",  " C  ", "SAVE", "CNCL");
				// Intentional fall-through

			case SCREEN_CONF_TEMPU_DRAW:
				lcd_gotoxy(0,1);
				lcd_puts("[ ] Degrees F");
				lcd_gotoxy(0,2);
				lcd_puts("[ ] Degrees C");
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
				lcd_puts("Temp/Humidity Addr:");
				// Intentional fall-through

			case SCREEN_CONF_THADDR_DRAW:

				if (0x00 == tempVar)
				{
					lcd_gotoxy(0, 1);
					lcd_puts("T/H Off");
				}
				else
				{
					lcd_gotoxy(0, 1);
					lcd_puts("0x");
					printHex(tempVar);
					lcd_puts("   ");
				}
				lcd_gotoxy(0,2);
				lcd_puts("            ");

				lcd_gotoxy(2 + confSaveVar, 2);
				lcd_puts("^");
				drawSoftKeys(" ++ ",  " -- ", " >> ", " GO ");
				screenState = SCREEN_CONF_THADDR_IDLE;
				break;


			case SCREEN_CONF_THADDR_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
				{
					if (0 == confSaveVar)
						tempVar += 0x10;
					else
						tempVar += 0x01;

					if (tempVar == 0xFF)
						tempVar = 0xFE;

					screenState = SCREEN_CONF_THADDR_DRAW;
				}
				else if (SOFTKEY_2 & buttonsPressed)
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
					drawSoftKeys("BACK",  "", "SAVE", "CNCL");
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
				lcd_puts("Temp/Hum Timeout:");
				screenState = SCREEN_CONF_THTIMEOUT_DRAW;
				break;

			case SCREEN_CONF_THTIMEOUT_DRAW:
				lcd_gotoxy(0,1);
				printDec4DigWZero(tempVar16);
				lcd_puts(" sec");

				lcd_gotoxy(0,2);
				lcd_puts("            ");
				lcd_gotoxy(confSaveVar, 2);
				lcd_putc('^');
				drawSoftKeys(" ++ ", " -- ", " >> ", " GO ");

				screenState = SCREEN_CONF_THTIMEOUT_IDLE;
				break;
				
			case SCREEN_CONF_THTIMEOUT_IDLE:
				// Switchy goodness
				if (SOFTKEY_1 & buttonsPressed)
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
				else if (SOFTKEY_2 & buttonsPressed)
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
					drawSoftKeys("BACK",  "", "SAVE", "CNCL");
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
				lcd_puts("Code off in lalaland");
				lcd_gotoxy(0,2);
				lcd_puts("Call Nathan");

				break;		
		
		}

		if(FAST_MODE && !FASTHOLD_MODE && fastDecisecs >= 10)
		{
			uint8_t fastTimeSecs = fastDecisecs / 10;
			incrementTime(&fastTime, fastTimeSecs);
			fastDecisecs -= fastTimeSecs * 10;
			if (screenState == SCREEN_MAIN_IDLE || screenState == SCREEN_MAIN_UPDATE_TIME)
				screenState = SCREEN_MAIN_UPDATE_TIME;
		}
		

		if (screenUpdateDecisecs >= 10)
		{
			// Reading optimizer
			// If we don't know the date or if we're in the range where we're at risk of 
			// changing dates
			readDS1302(ds1302Buffer);
			
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
			
			screenUpdateDecisecs -= 10;
			kloopsPerSec = loopCount / 1000;
			if(0 != thTimeout)
				thTimeout--;
				
			if (++thAlternator >= TH_ALTERNATOR_MAX)
				thAlternator = 0;
				
			loopCount = 0;
		}		

		/* If we need to send a packet and we're not already busy... */
		if (decisecs >= updateXmitInterval)
		{
			vitalChange = 1;
			decisecs -= updateXmitInterval;
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



