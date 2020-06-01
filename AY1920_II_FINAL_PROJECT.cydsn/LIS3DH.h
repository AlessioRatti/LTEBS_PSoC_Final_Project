/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#pragma once // include the header file only once

#include "SPI_Interface.h" // also include the SPI interface

#include "LIS3DH_Regs.h"    // include all register addresses

/*
 * LIS3DH status, extern allows us to share the var in other source codes
 * without having to include the whole header file
 * We just need to re-declare this variable in other source codes.
*/

 /*** ========= MACROS ========= ***/   

/** Instruction Set **/
#define SPI_LIS3DH_READ          0b10000000
#define SPI_LIS3DH_WRITE         0b00000000
#define SPI_LIS3DH_READ_MULTI    0b11000000
#define SPI_LIS3DH_WRITE_MULTI   0b01000000

/* LIS3DH FIFO_SRC_REG status */
#define SPI_LIS3DH_EMPTY_SHIFT        5
#define SPI_LIS3DH_OVRN_FIFO_SHIFT    6
#define SPI_LIS3DH_WTM_SHIFT          7

#define SPI_LIS3DH_EMPTY        ((uint8_t) 0x01u << SPI_LIS3DH_EMPTY_SHIFT)
#define SPI_LIS3DH_OVRN_FIFO    ((uint8_t) 0x01u << SPI_LIS3DH_OVRN_FIFO_SHIFT)
#define SPI_LIS3DH_WTM          ((uint8_t) 0x01u << SPI_LIS3DH_WTM_SHIFT)

#define SPI_LIS3DH_SIZE_BYTE    0xC0     // LIS3DH FIFO size in bytes (192 bytes)

#define MASK_FS                 0x03
#define MASK_FSR                0x0C
#define MASK_FSR_SHIFT          2
#define MASK_ANALOG             0x10
#define MASK_ANALOG_SHIFT       4
#define MASK_START_STOP         0x20
#define MASK_START_STOP_SHIFT   5

// LIS3DH variable settings
#define CTRL_REG1_NORM          0x07
#define CTRL_REG3_I1_OVERRUN    0x02
#define CTRL_REG4_NORM          0x00
#define CTRL_REG5_REBOOT        0x80
#define CTRL_REG5_FIFO_EN       0x48
#define FIFO_CTRL_REG_BYPASS    0x00
#define FIFO_CTRL_REG_STREAM    0x80

/*** ========= FUNCTION DECLARATIONS ========= ***/

/** ====== Setup Functions ====== **/

/*
* @brief Transform cfg_reg stored in the eeprom into usable settings
* 
* @param[in]: 8-bit register from EEEPROM
* @param[in]: three 8-bit registers to store configurations
*/
void LIS3DH_unzipConfig(uint8_t config_reg, volatile uint8_t* data);
uint8 LIS3DH_zipConfig(volatile uint8_t* data);

/*
* @brief Initialize the LIS3DH with default settings
*
*/
void LIS3DH_Init(void);

/*
* @brief Configure user-dependent registers
* 
* @param[in]: CTRL_REG1 and CTRL_REG4 settings
*/
void LIS3DH_setConfig(volatile uint8_t* data);

/** ====== Helper Functions ====== **/

/*
* @brief Read the STATUS registrer of the device.
*
* @return Status of the device (1 unsigned byte)
*/
uint8_t LIS3DH_readStatus();

/** ====== User-level Functions ====== **/

/*
* @brief Read a single byte @ addr.
*
* @param[in]: 8-bit memory address to read from.
* @return Read value (1 unsigned byte).
*/
uint8_t LIS3DH_readByte(uint8_t addr);

/*
* @brief Write a single byte with the value dataByte @ addr.
*
* @param[in]: 8-bit memory address to write to.
* @param[out]: 8-bit (unsigned byte) value to write in the memory location.
*/
void LIS3DH_writeByte(uint8_t addr, uint8_t dataByte);

/*
* @brief Read 1+ bytes from memory.
*
* @param[in]: 8-bit memory address to from read.
* @param[out]: uint8_t* pointer to the output data (cast operation required for different data type).
* @param[in]: Number of bytes to read.
*
*/
void LIS3DH_readPage(uint8_t addr, uint8_t* data, uint8_t nBytes);

/* [] END OF FILE */
