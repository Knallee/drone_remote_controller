/*
MPU6050 lib 0x02

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.
*/
#define F_CPU 16000000UL

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "mpu6050.h"
#include "..\i2chw\TWI_master.h"

#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
#include <math.h>  //include libm
#endif


volatile uint8_t buffer[14];

/*
 * read bytes from chip register
 */
int8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length, uint8_t *data) {
	if(length > 0) {
		readFlag = 1;
		TWI_Start_Transceiver_With_Data(regAddr, data , length);
		TWI_Get_Data_From_Transceiver(data, length);
	}
	return length;
}

/*
 * read 1 byte from chip register
 */
int8_t mpu6050_readByte(uint8_t regAddr, uint8_t *data) {
    return mpu6050_readBytes(regAddr, 1, data);
}

/*
 * write bytes to chip register
 */
void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data) {
	if(length > 0) {
		TWI_Start_Transceiver_With_Data(regAddr, data, length);
		while(TWI_Transceiver_Busy());
	}
}

/*
 * write 1 byte to chip register
 */
void mpu6050_writeByte(uint8_t regAddr, uint8_t data) {
    return mpu6050_writeBytes(regAddr, 1, &data);
}

/*
 * read bits from chip register
 */
int8_t mpu6050_readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    int8_t count = 0;
    if(length > 0) {
		uint8_t b;
		if ((count = mpu6050_readByte(regAddr, &b)) != 0) {
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			b &= mask;
			b >>= (bitStart - length + 1);
			*data = b;
		}
    }
    return count;
}

/*
 * read 1 bit from chip register
 */
int8_t mpu6050_readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = mpu6050_readByte(regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/*
 * write bit/bits to chip register
 */
void mpu6050_writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
	if(length > 0) {
		uint8_t b = 0;
		if (mpu6050_readByte(regAddr, &b) != 0) { //get current data <------------------------------------------------ what if byte actually is 0? Need to write byte in this case
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			mpu6050_writeByte(regAddr, b);
		}
	}
}

/*
 * write one bit to chip register
 */
void mpu6050_writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    mpu6050_readByte(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    mpu6050_writeByte(regAddr, b);
}

#if MPU6050_GETATTITUDE == 2
/*
 * write word/words to chip register
 */
void mpu6050_writeWords(uint8_t regAddr, uint8_t length, uint16_t* data) {
	if(length > 0) {
		uint8_t address[2];
		address[0] = 0;
		address[1] = 0;
		TWI_Start_Transceiver_With_Data(regAddr, address , 2);
		while(TWI_Transceiver_Busy());
	}
}

/*
 * set a chip memory bank
 */
void mpu6050_setMemoryBank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    mpu6050_writeByte(MPU6050_RA_BANK_SEL, bank);
}

/*
 * set memory start address
 */
void mpu6050_setMemoryStartAddress(uint8_t address) {
	mpu6050_writeByte(MPU6050_RA_MEM_START_ADDR, address);
}

/*
 * read a memory block
 */
void mpu6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
	mpu6050_setMemoryBank(bank, 0, 0);
	mpu6050_setMemoryStartAddress(address);
    uint8_t chunkSize;
    for (uint16_t i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        mpu6050_readBytes(MPU6050_RA_MEM_R_W, chunkSize, data + i);

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu6050_setMemoryBank(bank, 0, 0);
            mpu6050_setMemoryStartAddress(address);
        }
    }
}

/*
 * write a memory block
 */
uint8_t mpu6050_writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify, uint8_t useProgMem) {
	mpu6050_setMemoryBank(bank, 0, 0);
	mpu6050_setMemoryStartAddress(address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer = 0;
    uint8_t *progBuffer = 0;
    uint16_t i;
    uint8_t j;
    if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        if (useProgMem) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        mpu6050_writeBytes(MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

        //verify data if needed
        if (verify && verifyBuffer) {
        	mpu6050_setMemoryBank(bank, 0, 0);
            mpu6050_setMemoryStartAddress(address);
            mpu6050_readBytes(MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return 0; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu6050_setMemoryBank(bank, 0, 0);
            mpu6050_setMemoryStartAddress(address);
        }
    }
    if (verify) free(verifyBuffer);
    if (useProgMem) free(progBuffer);
    return 1;
}

/*
 * write a dmp configuration set
 */
uint8_t mpu6050_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, uint8_t useProgMem) {
    uint8_t *progBuffer = 0;
    uint8_t success, special;
    uint16_t i, j;
    if (useProgMem) {
        progBuffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    }

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) {
        if (useProgMem) {
            bank = pgm_read_byte(data + i++);
            offset = pgm_read_byte(data + i++);
            length = pgm_read_byte(data + i++);
        } else {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write
            if (useProgMem) {
                if (sizeof(progBuffer) < length) progBuffer = (uint8_t *)realloc(progBuffer, length);
                for (j = 0; j < length; j++) progBuffer[j] = pgm_read_byte(data + i + j);
            } else {
                progBuffer = (uint8_t *)data + i;
            }
            success = mpu6050_writeMemoryBlock(progBuffer, length, bank, offset, 1, 0);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (useProgMem) {
                special = pgm_read_byte(data + i++);
            } else {
                special = data[i++];
            }
            if (special == 0x01) {
                // enable DMP-related interrupts

            	//mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, 1); //setIntZeroMotionEnabled
            	//mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, 1); //setIntFIFOBufferOverflowEnabled
            	//mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, 1); //setIntDMPEnabled
            	mpu6050_writeByte(MPU6050_RA_INT_ENABLE, 0x32);  // single operation

                success = 1;
            } else {
                // unknown special command
                success = 0;
            }
        }

        if (!success) {
            if (useProgMem) free(progBuffer);
            return 0; // uh oh
        }
    }
    if (useProgMem) free(progBuffer);
    return 1;
}

/*
 * get the fifo count
 */
uint16_t mpu6050_getFIFOCount() {
	mpu6050_readBytes(MPU6050_RA_FIFO_COUNTH, 2, (uint8_t *)buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

/*
 * read fifo bytes
 */
void mpu6050_getFIFOBytes(uint8_t *data, uint8_t length) {
	mpu6050_readBytes(MPU6050_RA_FIFO_R_W, length, data);
}

/*
 * get the interrupt status
 */
uint8_t mpu6050_getIntStatus() {
	mpu6050_readByte(MPU6050_RA_INT_STATUS, (uint8_t *)buffer);
    return buffer[0];
}

/*
 * reset fifo
 */
void mpu6050_resetFIFO() {
	mpu6050_writeBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

/*
 * get gyro offset X
 */
int8_t mpu6050_getXGyroOffset() {
	mpu6050_readBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (uint8_t *)buffer);
    return buffer[0];
}

/*
 * set gyro offset X
 */
void mpu6050_setXGyroOffset(int8_t offset) {
	mpu6050_writeBits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/*
 * get gyro offset Y
 */
int8_t mpu6050_getYGyroOffset() {
	mpu6050_readBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (uint8_t *)buffer);
    return buffer[0];
}

/*
 * set gyro offset Y
 */
void mpu6050_setYGyroOffset(int8_t offset) {
	mpu6050_writeBits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

/*
 * get gyro offset Z
 */
int8_t mpu6050_getZGyroOffset() {
	mpu6050_readBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, (uint8_t *)buffer);
    return buffer[0];
}

/*
 * set gyro offset Z
 */
void mpu6050_setZGyroOffset(int8_t offset) {
	mpu6050_writeBits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}
#endif

/*
 * set sleep disabled
 */
void mpu6050_setSleepDisabled() {
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
}

/*
 * set sleep enabled
 */
void mpu6050_setSleepEnabled() {
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1);
}


/*
 * test connectino to chip
 */
uint8_t mpu6050_testConnection() {
	mpu6050_readBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, (uint8_t *)buffer);
	if(buffer[0] == 0x34)
		return 1;
	else
		return 0;
}

/*
 * initialize the accel and gyro
 */
void mpu6050_init() {
	
	//TWI_init
	TWI_Master_Initialise();
	_delay_us(10);
	
	//allow mpu6050 chip clocks to start up
	_delay_ms(100);

	//set sleep disabled
	mpu6050_setSleepDisabled();
	//wake up delay needed sleep disabled
	_delay_ms(10);

	//set clock source
	//  it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source)
	//  as the clock reference for improved stability
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	//set DLPF bandwidth to 42Hz
	mpu6050_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
    //set sample rate
	mpu6050_writeByte(MPU6050_RA_SMPLRT_DIV, 4); //1khz / (1 + 4) = 200Hz
	//set gyro range
	mpu6050_writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS);
	//set accel range
	mpu6050_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS);

}

//can not accept many request if we alreay have getattitude requests
/*
 * get raw data
 */
void mpu6050_getRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
	mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_H, 14, (uint8_t *)buffer);

    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

/*
 * get raw data converted to g and deg/sec values
 */
void mpu6050_getConvData(double* axg, double* ayg, double* azg, double* gxds, double* gyds, double* gzds) {
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	mpu6050_getRawData(&ax, &ay, &az, &gx, &gy, &gz);

	#if MPU6050_CALIBRATEDACCGYRO == 1
    *axg = (double)(ax-MPU6050_AXOFFSET)/MPU6050_AXGAIN;
    *ayg = (double)(ay-MPU6050_AYOFFSET)/MPU6050_AYGAIN;
    *azg = (double)(az-MPU6050_AZOFFSET)/MPU6050_AZGAIN;
    *gxds = (double)(gx-MPU6050_GXOFFSET)/MPU6050_GXGAIN;
	*gyds = (double)(gy-MPU6050_GYOFFSET)/MPU6050_GYGAIN;
	*gzds = (double)(gz-MPU6050_GZOFFSET)/MPU6050_GZGAIN;
	#else
    *axg = (double)(ax)/MPU6050_AGAIN;
    *ayg = (double)(ay)/MPU6050_AGAIN;
    *azg = (double)(az)/MPU6050_AGAIN;
    *gxds = (double)(gx)/MPU6050_GGAIN;
	*gyds = (double)(gy)/MPU6050_GGAIN;
	*gzds = (double)(gz)/MPU6050_GGAIN;
	#endif
}


