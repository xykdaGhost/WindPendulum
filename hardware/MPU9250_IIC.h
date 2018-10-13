#ifndef __MPU9250_IIC_H
#define __MPU9250_IIC_H

#include "stdio.h"	
#include "sys.h" 
#include "delay.h"
#include "math.h"

#define   uchar unsigned char
#define   uint unsigned int	
#define   bool uchar
#define 	FALSE 0x00
#define 	TRUE 0x01

bool TM_I2C_ReadMulti(uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);
void I2C_GPIO_Config(void);
bool Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
void Init_MPU9250(void);
void READ_MPU9250_ACCEL(void);
void READ_MPU9250_GYRO(void);
void READ_MPU9250_MAG(void);

#endif

