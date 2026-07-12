#ifndef MODULE_COMPASS_H
#define MODULE_COMPASS_H

#include "Global_Define.h"

typedef struct
{
	uint8_t							ID;
	float								Roll;
	float								Pitch;
	float								Heading;
} Attitude_TypeDef;

extern Attitude_TypeDef Attitude_Data;

void Compass_Init(void);

#endif
