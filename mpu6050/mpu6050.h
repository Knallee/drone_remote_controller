/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.

References:
  - most of the code is a port of the arduino mpu6050 library by Jeff Rowberg
    https://github.com/jrowberg/i2cdevlib
  - Mahony complementary filter for attitude estimation
    http://www.x-io.co.uk
*/


#ifndef MPU6050_H_
#define MPU6050_H_

#include <avr/io.h>
#include "mpu6050registers.h"

//definitions
#define MPU6050_ADDR (0x68 <<1) //device address - 0x68 pin low (GND), 0x69 pin high (VCC)

//enable the getattitude functions
//because we do not have a magnetometer, we have to start the chip always in the same position
//then to obtain your object attitude you have to apply the aerospace sequence
//0 disabled
//1 mahony filter
//2 dmp chip processor
#define MPU6050_GETATTITUDE 2

//definitions for raw data
//gyro and acc scale
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_2000
#define MPU6050_ACCEL_FS MPU6050_ACCEL_FS_2

#define MPU6050_GYRO_LSB_250 131.0
#define MPU6050_GYRO_LSB_500 65.5
#define MPU6050_GYRO_LSB_1000 32.8
#define MPU6050_GYRO_LSB_2000 16.4
#if MPU6050_GYRO_FS == MPU6050_GYRO_FS_250
#define MPU6050_GGAIN MPU6050_GYRO_LSB_250
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_500
#define MPU6050_GGAIN MPU6050_GYRO_LSB_500
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_1000
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_2000
#endif

#define MPU6050_ACCEL_LSB_2 16384.0
#define MPU6050_ACCEL_LSB_4 8192.0
#define MPU6050_ACCEL_LSB_8 4096.0
#define MPU6050_ACCEL_LSB_16 2048.0
#if MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_2
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_4
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_8
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_16
#endif

#define MPU6050_CALIBRATEDACCGYRO 1 //set to 1 if is calibrated
#if MPU6050_CALIBRATEDACCGYRO == 1
#define MPU6050_AXOFFSET 0
#define MPU6050_AYOFFSET 0
#define MPU6050_AZOFFSET 0
#define MPU6050_AXGAIN 16384.0
#define MPU6050_AYGAIN 16384.0
#define MPU6050_AZGAIN 16384.0
#define MPU6050_GXOFFSET -42
#define MPU6050_GYOFFSET 9
#define MPU6050_GZOFFSET -29
#define MPU6050_GXGAIN 16.4
#define MPU6050_GYGAIN 16.4
#define MPU6050_GZGAIN 16.4
#endif



#if MPU6050_GETATTITUDE == 2
//dmp definitions
//packet size
#define MPU6050_DMP_dmpPacketSize 42
//define INT0 rise edge interrupt
#define MPU6050_DMP_INT0SETUP EICRB |= (1<<ISC41) | (1<<ISC40) //<--------------------------------- change to INT6
//define enable and disable INT0 rise edge interrupt
#define MPU6050_DMP_INT0DISABLE EIMSK &= ~(1<<INT4)				//<--------------------------------- change to INT6
#define MPU6050_DMP_INT0ENABLE EIMSK |= (1<<INT4)				//<--------------------------------- change to INT6
extern volatile uint8_t mpu6050_mpuInterrupt;
extern uint8_t mpu6050_fifoBuffer[20];
#endif

//functions
extern void mpu6050_init();
extern uint8_t mpu6050_testConnection();

extern void mpu6050_setSleepDisabled();
extern void mpu6050_setSleepEnabled();

extern int8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
extern int8_t mpu6050_readByte(uint8_t regAddr, uint8_t *data);
extern void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data);
extern void mpu6050_writeByte(uint8_t regAddr, uint8_t data);
extern int8_t mpu6050_readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
extern int8_t mpu6050_readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);
extern void mpu6050_writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
extern void mpu6050_writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);


#if MPU6050_GETATTITUDE == 2
extern void mpu6050_writeWords(uint8_t regAddr, uint8_t length, uint16_t* data);
extern void mpu6050_setMemoryBank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank);
extern void mpu6050_setMemoryStartAddress(uint8_t address);
extern void mpu6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
extern uint8_t mpu6050_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify, uint8_t useProgMem);
extern uint8_t mpu6050_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, uint8_t useProgMem);
extern uint16_t mpu6050_getFIFOCount();
extern void mpu6050_getFIFOBytes(uint8_t *data, uint8_t length);
extern uint8_t mpu6050_getIntStatus();
extern void mpu6050_resetFIFO();
extern int8_t mpu6050_getXGyroOffset();
extern void mpu6050_setXGyroOffset(int8_t offset);
extern int8_t mpu6050_getYGyroOffset();
extern void mpu6050_setYGyroOffset(int8_t offset);
extern int8_t mpu6050_getZGyroOffset();
extern void mpu6050_setZGyroOffset(int8_t offset);
//base dmp
extern uint8_t mpu6050_dmpInitialize();
extern void mpu6050_dmpEnable();
extern void mpu6050_dmpDisable();
extern void mpu6050_getQuaternion(const uint8_t* packet, double *qw, double *qx, double *qy, double *qz);
extern void mpu6050_getRollPitchYaw(double qw, double qx, double qy, double qz, double *roll, double *pitch, double *yaw);
extern uint8_t mpu6050_getQuaternionWait(double *qw, double *qx, double *qy, double *qz);
#endif

#endif
