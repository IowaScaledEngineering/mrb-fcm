#ifndef _DMX_H_
#define _DMX_H_

#define MAX_DMX_CHANNELS  128

extern uint8_t dmx_channels[MAX_DMX_CHANNELS];
void dmx_initialize();
void dmx_all_off(void);
void dmx_startXmit();
void dmx_setChannel(uint16_t chanNum, uint8_t level);

#endif
