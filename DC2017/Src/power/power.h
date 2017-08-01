
#ifndef POWER_H
#define POWER_H

#include "board.h"



bool power_is_charging(void);
uint8_t power_get_battery_level(void);
void power_init(void);

#endif
