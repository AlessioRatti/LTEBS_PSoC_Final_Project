/* ========================================
 *
 * Copyright POLITECNICO DI MILANO, 2020
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 * 
 * PROJECT DESCRIPTION:
 * This project is about the development of
 * a firmware able to store both accelero-
 * meter and potentiometer/photoresistor
 * data in an external EEPROM.
 * The system allows to change the main
 * settings (e.g. frequency, FSR and other
 * parameters) thanks to a user-friendly
 * menu displayed using the CoolTerm inter-
 * face. Eventually, according to UART
 * communication protocol, the data is
 * sent to the Bridge Control Panel soft-
 * ware in order to be plotted.
 *
 * ========================================
*/
#include "project.h"

#include "InterruptRoutines.h"

/* -------- UART Communication --------- */
#include "stdio.h"
#include "string.h"

/* ------------ DMA library ------------ */
#include "DMARoutines.h"

/* ------ EEPROM 25LC256 Library ------- */
#include "25LC256.h"

/* -- Acceletrometer LIS3DH Libraries -- */
#include "LIS3DH.h"

/* ------ Size-related constants ------- */
#define BYTE_TO_SEND 8 // payload size during transmission in bytes
// Maximum number of bytes compressed in memory per interrupt
#define ACC_PLUS_ANALOG_SIZE 0xC0 // 128 (acc) + 64 (adc) bytes

/* ------- Bridge Control Panel -------- */
#define HEAD 0xA0
#define TAIL 0xC0
#define g_CONST 9.807       // physical constant (rounded at 3 decimal places)

/* -------------- STRUCTS -------------- */
/* union of int16 matched with 2 uint8_t
 * container for both physically meaninful
 * data and ready-to-send bytes to BCP
 */
union t_data{                   
    int16_t sample;
    uint8_t bytes[2];
};

/*----- Custom function prototypes ----- */
void CompressData(uint8_t* dataInAcc, uint8_t* dataInADC, uint8_t SizeBuffer,  uint8_t* dataOut);
void DeCompressData(uint8_t* dataIn, uint8_t* dataOut);
void SampledToBridge(uint8_t* dataIn, union t_data* dataOut);
void startSampling(volatile uint8_t* data);
void UART_Debug_BCP(const uint8 string[], uint8 byteCount, uint8 head, uint8 tail);

/*-------- Global vars in main.c ------- */
char bufferUART[200];
volatile uint8_t cfg_reg; // | - | - | ACQUISITION ON/OFF | ANALOG | FSR | FSR | FS | FS |
volatile uint8_t update_cfg_reg = 0;
uint8_t cfg_reg_old;
volatile uint8_t data_config[4];
volatile uint16_t indexSTREAM = DATA_START; // starting point for streaming data

// Period (ms) used in timer to acquire or stream data packets
const uint16_t Timer_Period[4] = {1000, 100, 40, 20}; // 1Hz, 10Hz, 25Hz, 50Hz

// From InterruptRoutines.c
extern volatile uint8_t dataReady_FIFO;          // Flag for Custom_ISR_OVRN
extern volatile uint8_t dataReady_DMA ;          // Flag for Custom_ISR_DMA
extern volatile uint8_t dataReady_STREAM;        // Flag for Custom_ISR_STREAM
extern uint8_t configSets[2];                    // (never written in different threads, no need for volatile)

int main(void)
{
    
    /*----------------------------------------------*/
    /*               Local variables                */
    /*----------------------------------------------*/
    uint8_t dataFIFO[SPI_LIS3DH_SIZE_BYTE]; // store data from readpage in LIS3DH
    union t_data dataSample[4]; // Container for data from ADC and LIS3DH
    // Indexes
    uint16_t indexEEPROM;
    uint16_t indexEEPROM_BCP = DATA_START; // copy of the index for streaming
    // Buffers
    uint8_t EEPROMBuffer[ACC_PLUS_ANALOG_SIZE]; // compressed data to be written in the EEPROM
    uint8_t STREAMBufferIn[6];  // data packet straight out of the EEPROM
    uint8_t STREAMBufferOut[8]; // same data decompressed
    // EEPROM indexing management vars
    uint8_t sizeEEPROMBuffer, indexEEPROMbuffer = 0;
    uint8_t toNextPage, nBytes;
    uint8_t  memoryFull = 0; // Flag
    uint8_t reg_value;  // support variable
    
    /*----------------------------------------------*/
    /*               Start components               */
    /*----------------------------------------------*/
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    /* Start UART */
    UART_Start();
    // Allow CoolTerm to exchange information at startup and dump the buffers
    CyDelay(1000);
    
    /*Start PWM*/
    PWM_BLINK_Start();
    
    /* Start Counter_RESET */
    Counter_INDEX_Start();
    Counter_RESET_Start();
    
    /* Start SPI Masters */
    SPIM_EEPROM_Start();
    CyDelay(10);
    SPIM_LIS3DH_Start();
    CyDelay(10);
    
    /*----------------------------------------------*/
    /*              Setup ADC and DMA               */
    /*----------------------------------------------*/
    /* MUX, ADC and DMA */
    AMux_Start();
    
    ADC_DelSig_Start();     // Start ADC
    ADC_DelSig_IRQ_Start(); // Enable ISR on EOC
    
    DMA_Config();           // Start DMA
    
    /*----------------------------------------------*/
    /*                Setup 25LC256                 */
    /*----------------------------------------------*/
    EEPROM_loadSettings(&cfg_reg, &indexEEPROM);
    
    sprintf(bufferUART, "** EEPROM Read\r\nconfig_reg = 0x%02x\r\nindexEEPROM = %d\r\n", cfg_reg, indexEEPROM);
    UART_PutString(bufferUART);
    
    UART_PutString("*************************************\r\n");
    
    /*----------------------------------------------*/
    /*                 Setup LIS3DH                 */
    /*----------------------------------------------*/
    LIS3DH_unzipConfig(cfg_reg, data_config);   // extract settings from single settings byte
    
    // Display current settings
    sprintf(bufferUART, "** LIS3DH Unzip\r\nFS     = 0x%02x\r\nFSR    = 0x%02x\r\nANALOG = 0x%02x\r\nSTATUS = 0x%02x\r\n", data_config[0], data_config[1], data_config[2], data_config[3]);
    UART_PutString(bufferUART);
    
    LIS3DH_Init(); // set minimum settings for LIS3DH
    LIS3DH_setConfig(data_config); // set user-dependent settings
    
    UART_PutString("** LIS3DH Read **\r\n");
    
    reg_value = LIS3DH_readByte(CTRL_REG3);
    sprintf(bufferUART, "CTRL_REG3     = 0x%02x\r\n", reg_value);
    UART_PutString(bufferUART);

    reg_value = LIS3DH_readByte(CTRL_REG5);
    sprintf(bufferUART, "CTRL_REG5     = 0x%02x\r\n", reg_value);
    UART_PutString(bufferUART);
    
    reg_value = LIS3DH_readByte(FIFO_CTRL_REG);
    sprintf(bufferUART, "FIFO_CTRL_REG = 0x%02x\r\n", reg_value);
    UART_PutString(bufferUART);
    
    reg_value = LIS3DH_readByte(CTRL_REG1);
    sprintf(bufferUART, "CTRL_REG1     = 0x%02x\r\n", reg_value);
    UART_PutString(bufferUART);
    
    reg_value = LIS3DH_readByte(CTRL_REG4);
    sprintf(bufferUART, "CTRL_REG4     = 0x%02x\r\n", reg_value);
    UART_PutString(bufferUART);
    
    UART_PutString("*************************************\r\n");
    
    /*----------------------------------------------*/
    /*              Setup ISR routines              */
    /*----------------------------------------------*/
    isr_RX_StartEx(Custom_ISR_RX);              // enable ISR from UART RX
    isr_TIMER_ADC_StartEx(Custom_ISR_TIMER);    // ISR on SOC
    isr_INT_StartEx(Custom_ISR_OVRN);           // enable ISR from LIS3DH in the µC
    isr_DMA_StartEx(Custom_ISR_DMA);            // ISR every 32 EOC
    isr_STREAM_StartEx(Custom_ISR_STREAM);      // enable ISR for streaming data
    isr_INDEX_StartEx(Custom_ISR_INDEX);        // ISR INDEX (5s)
    isr_RESET_StartEx(Custom_ISR_RESET);        // ISR RESET (10s)
    
    /* ---------------- END OF SETUP -------------- */
    
    if(((cfg_reg>>5) & 0x01)) {         // In case of acquisition from previous run
        // Start ISR from LIS3DH
        LIS3DH_setConfig(data_config);  // set user-dependent settings
        //Analog sensor
        AMux_Select(data_config[2]);    // select analog source
        // Start ADC and built-in LED
        startSampling(data_config);
    }
    
    //UART_PutChar(0x0C); // Clear terminal (uncomment in case you want to hide the registers config)
    ShowMenu(); // Begin by showing main interface
    
    while(UART_GetRxBufferSize()){UART_GetChar();} // Flush RX buffer
    
    for(;;)
    {
        while(UART_GetRxBufferSize()){UART_GetChar();} // Flush RX buffer
        
        /* ------------ Go into configuration mode ---------- */
        if (configSets[0] && configSets[1]) {
            
            cfg_reg_old = cfg_reg; // save old settings
            cfg_reg = LIS3DH_zipConfig(data_config); // generate new settings
            
            if (cfg_reg_old!=cfg_reg) { // compare old and new settings
                
                data_config[3] = 0x00; // Stop acquisition
                // Store new config in EEPROM
                cfg_reg = LIS3DH_zipConfig(data_config); // create new settings byte
                update_cfg_reg = 1; // ask to save settings
            }
            
            UART_PutChar(0x0C); // In case settings don't change
            ShowMenu();
            
            // Set RX default
            configSets[0]=0;
            configSets[1]=0;
        }
        
        /* -------------- Update configurations ------------- */
        if(update_cfg_reg & (!dataReady_FIFO) & (!dataReady_DMA)) { // activate save-settings only if ISRs are clear
            EEPROM_writeByte(CFG_REG, cfg_reg); // store settings in EEPROM
            EEPROM_waitForWriteComplete();
            
            LIS3DH_unzipConfig(cfg_reg, data_config); //in case of push button
            
            LIS3DH_Init();
            // Start/Stop ISR from LIS3DH
            LIS3DH_setConfig(data_config); // set user-dependent settings
            
            //Analog sensor
            AMux_Select(data_config[2]);
            
            if((((cfg_reg>>5) & 0x01)) && (!Timer_ADC_ReadControlRegister())) { // Start and avoid double call
                startSampling(data_config);
            }
            if((!((cfg_reg>>5) & 0x01)) && (Timer_ADC_ReadControlRegister())) { // Stop and avoid double call
                Timer_ADC_ReadStatusRegister();
                Timer_ADC_Stop();               //Stop ADC
                
                PWM_BLINK_WritePeriod(0);       //Turn off built-in led
                PWM_BLINK_WriteCompare(0);
                
                indexEEPROM_BCP = indexEEPROM;  // Copy index for stream
                indexEEPROM = DATA_START;       // bring index back to start
                
                // Save new index in EEPROM
                EEPROM_writePage(INDEX_REG, (uint8_t*) &indexEEPROM, 2);
                EEPROM_waitForWriteComplete();
                
                if (memoryFull) {   // Start fast blinking
                    PWM_BLINK_WritePeriod(49);
                    PWM_BLINK_WriteCompare(25);
                }
                else {              // Stop fast blinking
                    PWM_BLINK_WritePeriod(0);
                    PWM_BLINK_WriteCompare(0);
                }
                
                LIS3DH_Init();
                CyDelay(5);
                LIS3DH_readByte(INT1_SRC);
                Timer_ADC_ReadStatusRegister();
                isr_DMA_ClearPending();
                isr_INT_ClearPending();
                dataReady_DMA = 0;
                dataReady_FIFO = 0;
                memoryFull = 0;
            }
            
            // Clear terminal
            UART_PutChar(0x0C);
            ShowMenu();
            // Exit config mode
            update_cfg_reg = 0;
        }
        
        /* -------------- Visualize data in BCP -------------- */
        if(dataReady_STREAM & (!dataReady_FIFO) & (!dataReady_DMA)) {
            
            if(indexSTREAM >= indexEEPROM_BCP) { // send data in loop
                indexSTREAM  = DATA_START;
                // show beginning/end of acquisition (all values go to 10)
                uint8_t temp[8] = {0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0x10, 0x27};
                UART_Debug_BCP(temp, 8, HEAD, TAIL);
            }
            
            EEPROM_readPage(indexSTREAM, STREAMBufferIn, 6);    // read packet
            DeCompressData(STREAMBufferIn, STREAMBufferOut);    // de-compress packet
            SampledToBridge(STREAMBufferOut, dataSample);       // convert packet to physical quantity
            
            // In case analog sensor is slightly out of range (see ADC)
            if (dataSample[3].sample < MIN)    dataSample[3].sample = MIN;
            if (dataSample[3].sample > MAX)    dataSample[3].sample = MAX;
            
            // Send packet via UART
            UART_Debug_BCP(dataSample[0].bytes, BYTE_TO_SEND, HEAD, TAIL);
            
            indexSTREAM +=6;
            
            // Wait for next ISR
            dataReady_STREAM = 0;
        }
        
        /* ---------- Data flow from DMA and LIS3DH ---------- */
        if(dataReady_FIFO & dataReady_DMA & (!memoryFull)) {
            
            // Release INT line 
            LIS3DH_readByte(INT1_SRC);
            
            //How many bytes from FIFO
            reg_value = LIS3DH_readStatus();
            
            indexEEPROMbuffer = 0;
            // From LIS3DH application note (2/3 * 3/2)
            sizeEEPROMBuffer = ((reg_value & 0x5F) == 0x5F ? 0x20*6 : (reg_value & 0x1F)*6);
            
            // Data acquisition from FIFO
            LIS3DH_readPage(OUT_X_L, dataFIFO, 96);
            LIS3DH_readPage(OUT_X_L, &dataFIFO[96], 96);
            
            // Interlace FIFO and DMA, plus compress them
            CompressData(dataFIFO, ADCBuffer, sizeEEPROMBuffer, EEPROMBuffer);
            
            // Write compressed data on EEPROM
            while((indexEEPROMbuffer < sizeEEPROMBuffer) && (indexEEPROM <SPI_EEPROM_SIZE_BYTE)) {
                toNextPage = (indexEEPROM/SPI_EEPROM_PAGE_SIZE+1)*SPI_EEPROM_PAGE_SIZE - indexEEPROM;
                nBytes = (toNextPage > (sizeEEPROMBuffer-indexEEPROMbuffer) ? (sizeEEPROMBuffer-indexEEPROMbuffer) : toNextPage);
                EEPROM_waitForWriteComplete();
                EEPROM_writePage(indexEEPROM, (EEPROMBuffer+indexEEPROMbuffer), nBytes);
                EEPROM_waitForWriteComplete();
                
                indexEEPROMbuffer += nBytes;
                indexEEPROM += nBytes;
            }
            
            // Store new indexEEPROM after writing data
            EEPROM_writePage(INDEX_REG, (uint8_t*)&indexEEPROM, 2);
            EEPROM_waitForWriteComplete();
            
            if (indexEEPROM > SPI_EEPROM_SIZE_BYTE && (!memoryFull)){ // the last write should stop at 0x7FFF
                memoryFull = 1;
                data_config[3]=0x00; // bypass mode
                cfg_reg = LIS3DH_zipConfig(data_config);
                update_cfg_reg = 1; // write on EEPROM new config
            }
            
            dataReady_FIFO = 0; // acknowledge FIFO
            dataReady_DMA = 0;  // acknowledge DMA
            Timer_ADC_Start();
        }
    }
}

/* -- Compress data for minimizing storage used in the EEPROM -- */
void CompressData(uint8_t* dataInAcc, uint8_t* dataInADC, uint8_t SizeBuffer, uint8_t* dataOut) {
    // Compress data to required format
    for(uint8_t i=0; i<SizeBuffer; i+=6) {
        dataOut[i+0] = ((dataInAcc[i+1]  >>2));
        dataOut[i+1] = ((dataInAcc[i+1]  <<6) | (dataInAcc[i]>>2) | (dataInAcc[i+3]>>4));
        dataOut[i+2] = ((dataInAcc[i+3]<<4) | (dataInAcc[i+2]>>4) | (dataInAcc[i+5]>>6));
        dataOut[i+3] = ((dataInAcc[i+5]<<2) | (dataInAcc[i+4]>>6));
        // Append ADC value
        dataOut[i+4] = (dataInADC[i/3+0]);
        dataOut[i+5] = (dataInADC[i/3+1]);
    }
}

/* Decompress the data from the EEPROM and turn them into usable format */
void DeCompressData(uint8_t* dataIn, uint8_t* dataOut) {
    // Decompress data left adjusted for LIS3DH, right adjusted for ADC
    dataOut[0] = ((dataIn[1]<<2) & 0xC0);           // X-low
    dataOut[1] = ((dataIn[0]<<2) | (dataIn[1]>>6)); // X-high
    dataOut[2] = ((dataIn[2]<<4) & 0xC0);           // Y-low
    dataOut[3] = ((dataIn[1]<<4) | (dataIn[2]>>4)); // Y-high
    dataOut[4] = ((dataIn[3]<<6));                  // Z-low
    dataOut[5] = ((dataIn[2]<<6) | (dataIn[3]>>2)); // Z-high
    dataOut[6] = (dataIn[4]);                       // ADC-low
    dataOut[7] = (dataIn[5]);                       // ADC-high
}

/* Convert data from sampled to meaningful physical variables */
void SampledToBridge(uint8_t* dataIn, union t_data* dataOut){
    const uint8_t SampledTomg[4] = {4, 8, 16, 48};
    // Convert in m/s^2
    dataOut[0].sample = (float) (((int16_t) (dataIn[0] | (dataIn[1]<<8))>>6))*g_CONST*SampledTomg[cfg_reg>>2 & 0x03]/10;
    dataOut[1].sample = (float) (((int16_t) (dataIn[2] | (dataIn[3]<<8))>>6))*g_CONST*SampledTomg[cfg_reg>>2 & 0x03]/10;
    dataOut[2].sample = (float) (((int16_t) (dataIn[4] | (dataIn[5]<<8))>>6))*g_CONST*SampledTomg[cfg_reg>>2 & 0x03]/10;
    // Convert in V
    dataOut[3].sample = ADC_DelSig_CountsTo_mVolts((int32_t) ((dataIn[6] | (dataIn[7]<<8))));
}

// Custom version of the PutArray function
// it avoids having to deal with head and tail (I can place the data in a matrix and pass it as an array)
void UART_Debug_BCP(const uint8 string[], uint8 byteCount, uint8 head, uint8 tail)
{
    uint8 bufIndex = 0u;

    /* If not Initialized then skip this function */
    if(UART_initVar != 0u)
    {
        UART_PutChar(head);   // tranmit head
        while(bufIndex < byteCount) // transmit body
        {
            UART_PutChar(string[bufIndex]);
            bufIndex++;
        }
        UART_PutChar(tail);   // transmit tail
    }
}

void startSampling(volatile uint8_t* data) {
    // frequency 0 to 3
    uint8_t frequency_ADC = ((*data>>4)&0x0F)-1;
    // Start Timer_ADC
    Timer_ADC_Start();
    Timer_ADC_WritePeriod(Timer_Period[frequency_ADC]); // select frequency in const array
    
    // Turn on built-in LED
    PWM_BLINK_WritePeriod(199);     // T 1s
    PWM_BLINK_WriteCompare(100);    // 50% DC
}

/* [] END OF FILE */
