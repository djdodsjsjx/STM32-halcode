#ifndef __SERVO_H
#define __SERVO_H
#include "stdint.h"

#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
extern void pwm_servo_init(void);
extern void position(uint8_t ID ,uint16_t degrees);

#endif
