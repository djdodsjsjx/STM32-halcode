#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"

extern void MPU6050_initialize(void); 
extern unsigned char MPU_I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);

#endif

//------------------End of File----------------------------
