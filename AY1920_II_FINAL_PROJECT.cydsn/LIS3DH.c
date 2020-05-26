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

#include "LIS3DH.h"
#include "project.h"

const uint8_t lis3dh_dataRate_t[4]  = {0b00010000, 0b00100000, 0b00110000, 0b01000000};
const uint8_t lis3dh_dataRange_t[4] = {0b00000000, 0b00010000, 0b00100000, 0b00110000};

/** ====== Setup Functions ====== **/

void LIS3DH_unzipConfig(uint8_t config_reg, volatile uint8_t* data) {
    
    switch (config_reg & MASK_FS) { // FS
        case 0:     //  1Hz
            data[0] = lis3dh_dataRate_t[0] | CTRL_REG1_NORM;
            break;
        case 1:     // 10Hz
            data[0] = lis3dh_dataRate_t[1] | CTRL_REG1_NORM;
            break;
        case 2:     // 25Hz
            data[0] = lis3dh_dataRate_t[2] | CTRL_REG1_NORM;
            break;
        case 3:     // 50Hz
            data[0] = lis3dh_dataRate_t[3] | CTRL_REG1_NORM;
            break;
        default:
            break;
    }
    
    switch ((config_reg & MASK_FSR) >> 2) { // FSR
        case 0:     // +- 2g
            data[1] = lis3dh_dataRange_t[0] | CTRL_REG4_NORM;
            break;
        case 1:     // +- 4g
            data[1] = lis3dh_dataRange_t[1] | CTRL_REG4_NORM;
            break;
        case 2:     // +- 8g
            data[1] = lis3dh_dataRange_t[2] | CTRL_REG4_NORM;
            break;
        case 3:     // +-16g
            data[1] = lis3dh_dataRange_t[3] | CTRL_REG4_NORM;
            break;
        default:
            break;
    }
    
    switch ((config_reg & MASK_ANALOG) >> 4) { // ANALOG
        case 0:     // LDR
            data[2] = 0x00;
            break;
        case 1:     // POT
            data[2] = 0x01;
            break;
        default:
            break;
    }
    switch ((config_reg & MASK_START_STOP) >> 5) { // START/STOP
        case 0:     // STOP
            data[3] = 0x00;
            break;
        case 1:     // START
            data[3] = 0x80;
            break;
        default:
            break;
    }
}

uint8 LIS3DH_zipConfig(volatile uint8_t* data ) {
    uint8_t config_reg=0;
    switch (data[0]) { // FS
        case 0x17:     //  1Hz
            config_reg= 0;
            break;
        case 0x27:     // 10Hz
            config_reg= 1;
            break;
        case 0x37:     // 25Hz
            config_reg= 2;
            break;
        case 0x47:     // 50Hz
            config_reg= 3;
            break;
        default:
            break;
    }
    
    switch (data[1]) { // FSR
        case 0x00:     // +- 2g
            config_reg= config_reg | 0x00;
            break;
        case 0x10:     // +- 4g
            config_reg= config_reg | 0x04;
            break;
        case 0x20:     // +- 8g
            config_reg= config_reg | 0x08;
            break;
        case 0x30:     // +-16g
            config_reg= config_reg | 0x0C;
            break;
        default:
            break;
    }
    
    switch (data[2]) { // ANALOG
        case 0:     // LDR
            config_reg= config_reg | 0x00;
            break;
        case 1:     // POT
            config_reg= config_reg | 0x10;
            break;
        default:
            break;
    }
    switch (data[3]) { // START/STOP
        case 0x00:     // STOP
            config_reg= config_reg | 0x00;
            break;
        case 0x80:     // START
            config_reg= config_reg | 0x20;
            break;
        default:
            break;
    }
    return config_reg;
}

void LIS3DH_Init(void) {
    
    // Set CTRL_REG5 | Reboot
    LIS3DH_writeByte(CTRL_REG5, CTRL_REG5_REBOOT);
    // Clear LIS3DH FIFO
    LIS3DH_writeByte(FIFO_CTRL_REG, FIFO_CTRL_REG_BYPASS);
    
    // Set CTRL_REG3
    LIS3DH_writeByte(CTRL_REG3, CTRL_REG3_I1_OVERRUN);
    // Set CTRL_REG5
    LIS3DH_writeByte(CTRL_REG5, CTRL_REG5_FIFO_EN);
    // Set FIFO_CTRL_REG
    //LIS3DH_writeByte(FIFO_CTRL_REG, FIFO_CTRL_REG_BYPASS);
    UART_PutString("START\r\n");
}

void LIS3DH_Stop(void) {
    
    // Set CTRL_REG5 | Reboot
    //LIS3DH_writeByte(CTRL_REG5, CTRL_REG5_REBOOT);
    // Clear LIS3DH FIFO
    LIS3DH_writeByte(FIFO_CTRL_REG, FIFO_CTRL_REG_BYPASS);
}

void LIS3DH_setConfig(volatile uint8_t* data){
    
    // Set CTRL_REG1
    LIS3DH_writeByte(CTRL_REG1, data[0]);
    // Set CTRL_REG4
    LIS3DH_writeByte(CTRL_REG4, data[1]);
    //Set LIS3DH FIFO
    LIS3DH_writeByte(FIFO_CTRL_REG,data[3] );
    UART_PutString("SET CONFIG\r\n");
}

/** ====== Helper Functions ====== **/

uint8_t LIS3DH_readStatus() {
    
	return SPI_Interface_ReadByte_LIS3DH(SPI_LIS3DH_READ | FIFO_SRC_REG);
}

/** ====== User-level Functions ====== **/

uint8_t LIS3DH_readByte(uint8_t addr) {
    
	/* Prepare the TX data packet: instruction + address */
	uint8_t dataTX = {SPI_LIS3DH_READ | addr};
	
	/* Prepare the RX byte */
	uint8_t dataRX;
	
	/* Read 1 byte from addr */
	dataRX = SPI_Interface_ReadByte_LIS3DH(dataTX);
	//LIS3DH_waitForReadComplete();
	return dataRX;
}

/*
void LIS3DH_writeByte(uint8_t addr, uint8_t dataByte) {
	
	// Prepare the TX packet
    uint8_t dataTX[2] = {SPI_LIS3DH_WRITE | addr, dataByte};
	// Nothing to RX... 
	uint8_t temp = 0;
	
	// Write 1 byte to addr
	SPI_Interface_Multi_RW_LIS3DH(dataTX, 2, &temp, 0);
}
*/

void LIS3DH_writeByte(uint8_t addr, uint8_t dataByte) {
	
	// Prepare the TX packet
    uint8_t dataTX[2] = {SPI_LIS3DH_WRITE | addr, dataByte};
	
	// Write 1 byte to addr
	SPI_Interface_WriteByte_LIS3DH(dataTX);
}

void LIS3DH_readPage(uint8_t addr, uint8_t* dataRX, uint8_t nBytes) {
    
	/* Prepare the TX data packet: instruction + address */
	uint8_t dataTX = {SPI_LIS3DH_READ_MULTI | addr};
	
	/* Read the nBytes */
	SPI_Interface_Multi_RW_LIS3DH(&dataTX, 1, dataRX, nBytes);
}

void LIS3DH_writePage(uint8_t addr, uint8_t* data, uint8_t nBytes) {

	/* Prepare the TX packet of size nBytes+3 
       [ Write Instruction - Address MSB - Address LSB - +++data+++ ]
    */
    
	uint8_t dataTX[1+nBytes];
    dataTX[0] = (SPI_LIS3DH_WRITE_MULTI | addr);
    /* Copy the input data in the memory */
	memcpy(&dataTX[1], data, nBytes);
	
	/* Nothing to RX: point to a dummy variable */
	uint8_t temp = 0;
	
	SPI_Interface_Multi_RW_LIS3DH(dataTX, 1+nBytes, &temp, 0);
	
}
/*
void LIS3DH_waitForReadComplete() {
    
    while( LIS3DH_readStatus() & SPI_LIS3DH_EMPTY );
    
}
*/

/* [] END OF FILE */
