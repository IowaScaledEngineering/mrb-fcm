#ifndef _RV3129_H_
#define _RV3129_H_

#define RV3129_I2C_ADDR 0xAC

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

int8_t rv3129_readTemperature(void);
void rv3129_systemReset();
uint8_t rv3129_readControl(void);
void rv3129_writeControl(void);
void rv3129_writeDate(TimeData* dt);
void rv3129_writeTime(TimeData* dt);
void rv3129_readTime(TimeData* dt);

#endif

