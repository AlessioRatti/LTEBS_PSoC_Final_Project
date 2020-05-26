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

/*											 
 * NOTE: in the .h Lib of the SPI Slave     		   *
 *       you need to define SLAVE_CS_Write  		   *
 *       with the name of the Chip Select    		   * 
 *       pin (as defined in the Top Design)  		   *
 * E.g.: #define SLAVE_CS_Write CS_1_Write (default)   *
*/

#include "SPI_Interface.h"


#ifndef SLAVE_CS_Write_1
    #define SLAVE_CS_1_Write CS_1_Write
#endif

#ifndef SLAVE_CS_Write_2
    #define SLAVE_CS_2_Write CS_2_Write
#endif

/*
* @brief Full-Duplex, Single-Operation 1-Byte Trade on the LIS3DH
*/
uint8_t SPI_Interface_tradeByte_LIS3DH(uint8_t byte) {
	
	/* Enable the Slave */
	SLAVE_CS_1_Write(0);
	
	/* Load the TX Buffer */
	SPIM_LIS3DH_WriteTxData(byte);
	
	/* Wait for TX */
    while(!(SPIM_LIS3DH_ReadTxStatus() & SPIM_LIS3DH_STS_BYTE_COMPLETE) );
	
	/* Read the RX Buffer */
	uint8_t data = SPIM_LIS3DH_ReadRxData();
	
	/* Disable the Slave */
	SLAVE_CS_1_Write(1);
	
	return data;
    
}

/*
* @brief Full-Duplex, Single-Operation 1-Byte Trade on the EEPROM
*/
uint8_t SPI_Interface_tradeByte_EEPROM(uint8_t byte) {
	
	/* Enable the Slave */
	SLAVE_CS_2_Write(0);
	
	/* Load the TX Buffer */
	SPIM_EEPROM_WriteTxData(byte);
	
	/* Wait for TX */
    while( !(SPIM_EEPROM_ReadTxStatus() & SPIM_EEPROM_STS_BYTE_COMPLETE) );
	
	/* Read the RX Buffer */
	uint8_t data = SPIM_EEPROM_ReadRxData();
	
	/* Disable the Slave */
	SLAVE_CS_2_Write(1);
	
	return data;
    
}

/*
* @brief RX-only, Dual-Operation 1-Byte READ for the LIS3DH
*/
uint8_t SPI_Interface_ReadByte_LIS3DH(uint8_t byteTX) {
    
	/* Enable the Slave */
    SLAVE_CS_1_Write(0);
    
    /* Load the TX Buffer */
    SPIM_LIS3DH_WriteTxData(byteTX);            // first: instruction byte
    
    /* SPI Dummy Byte */
    SPIM_LIS3DH_WriteTxData(SPI_DUMMY_BYTE);    // second: dummy byte
    
    /* Wait for TX/RX */
    while(!(SPIM_LIS3DH_ReadTxStatus() & SPIM_LIS3DH_STS_SPI_DONE));
    
    /* Read the RX Buffer */
    // we wrote two bytes so we need to read two bytes
    SPIM_LIS3DH_ReadRxData(); // nonsense
    uint8_t byteRX = SPIM_LIS3DH_ReadRxData(); // actual data
    
    /* Disable the Slave */
    SLAVE_CS_1_Write(1);
    
    return byteRX;
}

/*
* @brief RX-only, Dual-Operation 1-Byte READ for the EEPROM
*/
uint8_t SPI_Interface_ReadByte_EEPROM(uint8_t byteTX) {
    
	/* Enable the Slave */
    SLAVE_CS_2_Write(0);
    
    /* Load the TX Buffer */
    SPIM_EEPROM_WriteTxData(byteTX); // first: instruction byte
    	
    /* SPI Dummy Byte */
    SPIM_EEPROM_WriteTxData(SPI_DUMMY_BYTE); // second: dummy byte
    	
    /* Wait for TX/RX */
    while(!(SPIM_EEPROM_ReadTxStatus() & SPIM_EEPROM_STS_SPI_DONE));
    	
    /* Read the RX Buffer */
    // we wrote two bytes so we need to read two bytes
    SPIM_EEPROM_ReadRxData(); // nonsense
    uint8_t byteRX = SPIM_EEPROM_ReadRxData(); // actual data
    
    /* Disable the Slave */
    SLAVE_CS_2_Write(1);
    
    return byteRX;
}


/*
* @brief Full-Duplex, Multi-Byte Trade (RX/TX)
*/
/*
void SPI_Interface_Multi_Trade(uint8_t* dataTX, uint8_t* dataRX, uint8_t nBytes) {
    
    
}
*/



void SPI_Interface_WriteByte_LIS3DH(uint8_t* byteTX) {
    
	// Enable the Slave
    SLAVE_CS_1_Write(0);
    
    // Load the TX Buffer
    SPIM_LIS3DH_WriteTxData(byteTX[0]);     // first: instruction byte
    
    // SPI Dummy Byte
    SPIM_LIS3DH_WriteTxData(byteTX[1]);     // second: data byte
    
    // Wait for TX/RX
    while(!(SPIM_LIS3DH_ReadTxStatus() & SPIM_LIS3DH_STS_SPI_DONE));
    
    // Disable the Slave
    SLAVE_CS_1_Write(1);
    
    /* Clear */
    SPIM_LIS3DH_ClearFIFO();
    SPIM_LIS3DH_ClearRxBuffer();
    SPIM_LIS3DH_ClearTxBuffer();
    
}

/*
* @brief RX-only, Dual-Operation Multi-Byte READ/WRITE for the LIS3DH
*/
void SPI_Interface_Multi_RW_LIS3DH(uint8_t* dataTX, uint8_t bytesTX, uint8_t* dataRX, uint8_t bytesRX) {
    
    /* Enable the Slave */
    SLAVE_CS_1_Write(0);
    
    /* --------------- WRITE --------------- */
    
    int8_t count = bytesTX, index = 0;
    	
    /* Transmit Data */
    while ( count > 0 ) {
    	/* Load the TX buffer with Data*/
        SPIM_LIS3DH_PutArray(&dataTX[index*SPI_TxBufferSize], (count > SPI_TxBufferSize ? SPI_TxBufferSize : count));
        /* Wait for TX */
        while( !(SPIM_LIS3DH_ReadTxStatus() & SPIM_LIS3DH_STS_SPI_DONE) );
        
        /* Update count */
        count -= SPI_TxBufferSize;
        index++;
    }
        
    /* Clear the RX Buffer */
    SPIM_LIS3DH_ClearFIFO();
    SPIM_LIS3DH_ClearRxBuffer();
    
    /* --------------- READ --------------- */
    
    /* Init the Dummy TX Buffer */
    uint8_t dummyTX[SPI_RxBufferSize];
    memset(dummyTX, SPI_DUMMY_BYTE, SPI_RxBufferSize);
    
    /* Update count for RX */
    count = bytesRX;
    index = 0;
        
    /* Get the RX Data */
    while ( count > 0 ) {
        /* Load the TX buffer with Dummy Bytes*/
        SPIM_LIS3DH_PutArray(dummyTX, (count > SPI_TxBufferSize ? SPI_TxBufferSize : count));
        /* Wait for TX */
        while( !(SPIM_LIS3DH_ReadTxStatus() & SPIM_LIS3DH_STS_SPI_DONE) );
        /* Read the RX Buffer */
        for( uint8_t j = 0; j < (count > SPI_TxBufferSize ? SPI_TxBufferSize : count); j++ ) {
            dataRX[j + index*SPI_RxBufferSize] = SPIM_LIS3DH_ReadRxData();   
        }
        
        count -= SPI_RxBufferSize;
        index++;
    }
    
    /* Disable the Slave */
    SLAVE_CS_1_Write(1);
    
    /* Clear */
    SPIM_LIS3DH_ClearFIFO();
    SPIM_LIS3DH_ClearRxBuffer();
    SPIM_LIS3DH_ClearTxBuffer();
}

/*
* @brief RX-only, Dual-Operation Multi-Byte READ/WRITE for the EEPROM
*/
void SPI_Interface_Multi_RW_EEPROM(uint8_t* dataTX, uint8_t bytesTX, uint8_t* dataRX, uint8_t bytesRX) {
    
    /* Enable the Slave */
    SLAVE_CS_2_Write(0);
    
    int8_t count = bytesTX, index = 0;
    
    /* Transmit Data */
    while ( count > 0 ) {
    	/* Load the TX buffer with Data*/
        SPIM_EEPROM_PutArray(&dataTX[index*SPI_TxBufferSize], (count > SPI_TxBufferSize ? SPI_TxBufferSize : count));
        /* Wait for TX */
        while( !(SPIM_EEPROM_ReadTxStatus() & SPIM_EEPROM_STS_SPI_DONE) );
        
        /* Update count */
        count -= SPI_TxBufferSize;
        index++;
    }
    
    /* Clear the RX Buffer */
    SPIM_EEPROM_ClearFIFO();
    SPIM_EEPROM_ClearRxBuffer();
    
    /* Init the Dummy TX Buffer */
    uint8_t dummyTX[SPI_RxBufferSize];
    memset(dummyTX, SPI_DUMMY_BYTE, SPI_RxBufferSize);
    
    /* Update count for RX */
    count = bytesRX;
    index = 0;
    
    /* Get the RX Data */
    while ( count > 0 ) {
        /* Load the TX buffer with Dummy Bytes*/
        SPIM_EEPROM_PutArray(dummyTX, (count > SPI_TxBufferSize ? SPI_TxBufferSize : count));
        /* Wait for TX */
        while( !(SPIM_EEPROM_ReadTxStatus() & SPIM_EEPROM_STS_SPI_DONE) );
        /* Read the RX Buffer */
        for( uint8_t j = 0; j < (count > SPI_TxBufferSize ? SPI_TxBufferSize : count); j++ ) {
            dataRX[j + index*SPI_RxBufferSize] = SPIM_EEPROM_ReadRxData();
        }
        
        count -= SPI_RxBufferSize;
        index++;
    }
    
    /* Disable the Slave */
    SLAVE_CS_2_Write(1);
    
    /* Clear */
    SPIM_EEPROM_ClearFIFO();
    SPIM_EEPROM_ClearRxBuffer();
    SPIM_EEPROM_ClearTxBuffer();
    
}

/* [] END OF FILE */
