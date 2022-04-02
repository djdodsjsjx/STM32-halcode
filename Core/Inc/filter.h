#ifndef __FILTER_H
#define __FILTER_H

#include "main.h"

extern int16_t Dog_ROL,Dog_PIT;
float Kalman_Filter(float Accel,float Gyro);	
void Filter_Get_Angle(void);

#endif
