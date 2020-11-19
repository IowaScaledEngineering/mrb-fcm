#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "rv3129.h"
#include "avr-i2c-master.h"

#define min(a,b)  ((a)<(b)?(a):(b))

static uint8_t toBCD(uint8_t bin)
{
	uint8_t tens = bin / 10;
	uint8_t ones = (bin % 10);
	return (tens<<4) | ones;
}

static uint8_t fromBCD(uint8_t bcd)
{
	return (bcd & 0x0F) + 10*((bcd) >> 4);
}

int8_t rv3129_readTemperature(void)
{
	uint8_t i2cBuf[10];
	uint8_t successful = 0;
	memset(i2cBuf, 0, sizeof(i2cBuf));    

	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x20; // Temperature register
	i2c_transmit(i2cBuf, 2, 0);

	memset(i2cBuf, 0, sizeof(i2cBuf));  
	i2cBuf[0] = RV3129_I2C_ADDR | 0x01;  // Read address
	i2c_transmit(i2cBuf, 2, 1);
	while(i2c_busy());
	successful = i2c_receive(i2cBuf, 2);
	if (!successful)
		return (0);
	return (int8_t)((int16_t)i2cBuf[1] - 60);
}

void rv3129_systemReset()
{
	uint8_t i2cBuf[10];
	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x04; // CONTROL_RESET register
	i2cBuf[1] = 0x10; // Initiate system reset
	i2c_transmit(i2cBuf, 2, 0);
}

uint8_t rv3129_readControl(void)
{
	uint8_t i2cBuf[10];
	uint8_t successful = 0;
	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x03; // Start of date/time page
	i2c_transmit(i2cBuf, 2, 0);

	memset(i2cBuf, 0, sizeof(i2cBuf));  
	i2cBuf[0] = RV3129_I2C_ADDR | 0x01;  // Read address
	i2c_transmit(i2cBuf, 2, 1);
	while(i2c_busy());
	successful = i2c_receive(i2cBuf, 2);
	if (!successful)
		return 0;
	return i2cBuf[1];
}

void rv3129_writeControl(void)
{
	uint8_t i2cBuf[10];
	
	memset(i2cBuf, 0, sizeof(i2cBuf));    

	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x00; // Start of control page

	i2cBuf[2] = 0x10; // Enable self-recovery, disable everything else
	i2cBuf[3] = 0x00; // Disable all interrupts
	i2cBuf[4] = 0x00; // Disable all interrupt flags
	i2cBuf[5] = 0x00; // Clear all control flags
	
	i2c_transmit(i2cBuf, 6, 1);
}

void rv3129_writeDateTime(TimeData* dt)
{
	uint8_t i2cBuf[10];
	
	memset(i2cBuf, 0, sizeof(i2cBuf));    

	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x08; // Start of date/time page

	i2cBuf[2] = 0x7F & toBCD(min(59,dt->seconds));
	i2cBuf[3] = 0x7F & toBCD(min(59,dt->minutes));
	i2cBuf[4] = 0x3F & toBCD(min(23,dt->hours));
	i2cBuf[5] = 0x3F & toBCD(min(31,dt->day));
	i2c_transmit(i2cBuf, 6, 1);
	
	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x0D; // Start of date/time page
	i2cBuf[2] = 0x3F & toBCD(min(12,dt->month));
	i2cBuf[3] = 0x7F & toBCD((uint8_t)(dt->year - 2000));
	i2c_transmit(i2cBuf, 4, 1);
}

void rv3129_writeTime(TimeData* dt)
{
	uint8_t i2cBuf[10];
	
	memset(i2cBuf, 0, sizeof(i2cBuf));    

	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x08; // Start of date/time page

	i2cBuf[2] = 0x7F & toBCD(min(59,dt->seconds));
	i2cBuf[3] = 0x7F & toBCD(min(59,dt->minutes));
	i2cBuf[4] = 0x3F & toBCD(min(23,dt->hours));
	i2c_transmit(i2cBuf, 5, 1);
}

void rv3129_writeDate(TimeData* dt)
{
	uint8_t i2cBuf[5];
	
	memset(i2cBuf, 0, sizeof(i2cBuf));    

	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x0B; // Start of date/time page
	i2cBuf[2] = toBCD(dt->day);
	i2c_transmit(i2cBuf, 3, 1);
	
	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x0D; // Start of date/time page
	i2cBuf[2] = 0x3F & toBCD(min(12,dt->month));
	i2cBuf[3] = 0x7F & toBCD((uint8_t)(dt->year - 2000));
	i2c_transmit(i2cBuf, 4, 1);
}


void rv3129_readTime(TimeData* dt)
{
	uint8_t i2cBuf[10];
	uint8_t successful = 0;
	memset(i2cBuf, 0, sizeof(i2cBuf));    

	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x08; // Start of date/time page
	i2c_transmit(i2cBuf, 2, 0);

	memset(i2cBuf, 0, sizeof(i2cBuf));  
	i2cBuf[0] = RV3129_I2C_ADDR | 0x01;  // Read address
	i2c_transmit(i2cBuf, 8, 1);
	while(i2c_busy());
	successful = i2c_receive(i2cBuf, 8);

	if (!successful)
		return;

	dt->seconds = fromBCD(i2cBuf[1] & 0x7F);
	dt->minutes = fromBCD(i2cBuf[2] & 0x7F);
	
	if (i2cBuf[3] & 0x40)
	{
		// 12-hour AM/PM mode - ick
		dt->hours = (i2cBuf[3] & 0x0F) + 10*((i2cBuf[3] & 0x10) >> 4);
		if (i2cBuf[3] & 0x20)
			dt->hours += 12;
	} else {
		// 24-hour mode
		dt->hours = (i2cBuf[3] & 0x0F) + 10*((i2cBuf[3] & 0x30) >> 4);
	}

	dt->day = fromBCD(i2cBuf[4] & 0x3F);
	dt->dayOfWeek = (i2cBuf[5] & 0x07);
	dt->month = fromBCD(i2cBuf[6] & 0x1F);
	dt->year = 2000 + fromBCD(i2cBuf[7] & 0x3F);

}

uint8_t ft_checksum(uint8_t *data)
{
	uint8_t checksum = 0xCC;
	for (uint8_t i=0; i<3; i++)
		checksum ^= data[i];
	return checksum;
}

void rv3129_writeFastTime(TimeData* dt)
{
	uint8_t i2cBuf[10];
	static uint8_t slot = 0;
	uint8_t checksum = 0;
	
	slot++;
	// Alternate between slots 
	memset(i2cBuf, 0, sizeof(i2cBuf));

	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = (slot & 0x01)?0x3A:0x3D; // Start of FT page 1 / 2
	i2cBuf[2] = dt->hours;  // 0x39
	i2cBuf[3] = dt->minutes;  //0x3A
	i2cBuf[4] = dt->seconds;  //0x3B
	checksum = ft_checksum(&i2cBuf[2]);
	i2c_transmit(i2cBuf, 5, 0);

	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x38; // Set valid pointer to #1
	i2cBuf[2] = (slot & 0x01)?1:2;
	i2cBuf[3] = checksum;
	i2c_transmit(i2cBuf, 4, 0);
	while(i2c_busy());
}

bool rv3129_readFastTime(TimeData* dt)
{
	uint8_t i2cBuf[10];
	uint8_t slot = 0;
	uint8_t storedChecksum = 0;
	uint8_t successful = 0;
	
	memset(i2cBuf, 0, sizeof(i2cBuf));    

	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = 0x38; // Fast Time Valid Slot Indicator
	i2c_transmit(i2cBuf, 2, 0);

	// Read last valid slot indicator
	memset(i2cBuf, 0, sizeof(i2cBuf));  
	i2cBuf[0] = RV3129_I2C_ADDR | 0x01;  // Read address
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy());
	successful = i2c_receive(i2cBuf, 3);
	if (!successful)
		return false;
	
	slot = i2cBuf[1];
	storedChecksum = i2cBuf[2];
	if (slot != 1 && slot != 2)
		return false;
	
	i2cBuf[0] = RV3129_I2C_ADDR;
	i2cBuf[1] = (slot & 0x01)?0x3A:0x3D; // Start of FT page 1 / 2
	i2c_transmit(i2cBuf, 2, 0);
	
	memset(i2cBuf, 0, sizeof(i2cBuf));  
	i2cBuf[0] = RV3129_I2C_ADDR | 0x01;  // Read address
	i2c_transmit(i2cBuf, 4, 1);
	while(i2c_busy());
	successful = i2c_receive(i2cBuf, 4);
	if (!successful)
		return false;

	if (storedChecksum != ft_checksum(&i2cBuf[1]))
		return false;
		

	dt->hours = i2cBuf[1];
	dt->minutes = i2cBuf[2];
	dt->seconds = i2cBuf[3];

	return true;
}
