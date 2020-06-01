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

#include "25LC256.h"

uint8_t eeprom_Status;

void EEPROM_loadSettings(volatile uint8_t* config_reg, uint16_t* index_eeprom) {
    
    *config_reg = EEPROM_readByte(0x0002);
    EEPROM_readPage(0x0000, (uint8_t*) index_eeprom, 2);
    
}

/** ====== Helper Functions ====== **/

uint8_t EEPROM_readStatus() {
    
	return SPI_Interface_ReadByte_EEPROM(SPI_EEPROM_RDSR);
    
}

void EEPROM_writeEnable() {
    
	/* Send 1-byte Instruction */
	SPI_Interface_tradeByte_EEPROM(SPI_EEPROM_WREN);
    
}

void EEPROM_writeDisable() {
    
	/* Send 1-byte Instruction */
	SPI_Interface_tradeByte_EEPROM(SPI_EEPROM_WRDI);
    
}

/** ====== User-level Functions ====== **/

uint8_t EEPROM_readByte(uint16_t addr) {
    
	/* Prepare the TX data packet: instruction + address */
	uint8_t dataTX[3] = {SPI_EEPROM_READ, ((addr & 0xFF00) >> 8), (addr & 0x00FF)};
	
	/* Prepare the RX byte */
	uint8_t dataRX = 0;
	
	/* Read 1 byte from addr */
	SPI_Interface_Multi_RW_EEPROM(dataTX, 3, &dataRX, 1);
	
	return dataRX;
}

void EEPROM_writeByte(uint16_t addr, uint8_t dataByte) {
    
    /* Enable WRITE operations */
    EEPROM_writeEnable();
    
    CyDelayUs(1);
	
	/* Prepare the TX packet */
    uint8_t dataTX[4] = {SPI_EEPROM_WRITE, ((addr & 0xFF00) >> 8), (addr & 0x00FF), dataByte};
	/* Nothing to RX... */
	uint8_t temp = 0;
	
	/* Write 1 byte to addr */
	SPI_Interface_Multi_RW_EEPROM(dataTX, 4, &temp, 0);
	
}

void EEPROM_readPage(uint16_t addr, uint8_t* dataRX, uint8_t nBytes) {
    
	/* Prepare the TX data packet: instruction + address */
	uint8_t dataTX[3] = {SPI_EEPROM_READ, ((addr & 0xFF00) >> 8), (addr & 0x00FF)};
	
	/* Read the nBytes */
	SPI_Interface_Multi_RW_EEPROM(dataTX, 3, dataRX, nBytes);
    
}

void EEPROM_writePage(uint16_t addr, uint8_t* data, uint8_t nBytes) {
    
    /* Save current global interrupt enable and disable it */
    uint8 interruptState;
    interruptState = CyEnterCriticalSection();
    
    /* Enable WRITE operations */
    EEPROM_writeEnable();
	
    CyDelayUs(1);
    
	/* Prepare the TX packet of size nBytes+3 
       [ Write Instruction - Address MSB - Address LSB - +++data+++ ]
    */
    
	uint8_t dataTX[3+nBytes];
    dataTX[0] = SPI_EEPROM_WRITE;
    dataTX[1] = (addr & 0xFF00) >> 8;
    dataTX[2] = addr & 0x00FF;
    /* Copy the input data in the memory */
	memcpy(&dataTX[3], data, nBytes);
	
	/* Nothing to RX: point to a dummy variable */
	uint8_t temp = 0;
	
	SPI_Interface_Multi_RW_EEPROM(dataTX, 3+nBytes, &temp, 0);
    
    /* Restore global interrupt enable state */
    CyExitCriticalSection(interruptState);
	
}

void EEPROM_waitForWriteComplete() {
    
    while( EEPROM_readStatus() & SPI_EEPROM_WRITE_IN_PROGRESS );
    
}

/* [] END OF FILE */
