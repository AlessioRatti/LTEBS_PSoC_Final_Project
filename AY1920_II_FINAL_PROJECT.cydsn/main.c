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
#include "project.h"

#include "stdio.h"
#include "string.h"

#include "LIS3DH_Regs.h"
#include "InterruptRoutines.h"
#include "DMARoutines.h"

/* EEPROM 25LC256 Library */
#include "25LC256.h"
/* Acceletrometer LIS3DH Library */
#include "LIS3DH.h"

#define DATA_CONFIG_SIZE    3
#define DATA_COMP_SIZE      128 // 4 byte x 32
#define ACC_PLUS_ANALOG_SIZE 0xC0 // 128 (acc) + 64 (adc) bytes

// Bridge Control Panel
#define HEAD 0xA0
#define TAIL 0xC0
#define UART_BYTES_PER_AXIS 4   // bytes needed for transmission
#define g_CONST 9.807           // physical constant (rounded at 3 decimal places)

//uint8_t configArr[10];

union t_data{                                     // union of float matched with 4 uint8_t
        int16_t sample;
        uint8_t bytes[2];
    };

// Custom functions prototipes
void CompressData(uint8_t* dataInAcc, uint8_t* dataInADC, uint8_t* dataOut);
void DeCompressData(uint8_t* dataIn, uint8_t* dataOut);
void SampledToBridge(uint8_t* dataIn, union t_data* dataOut);

void UART_Debug_BCP(const uint8 string[], uint8 byteCount, uint8 head, uint8 tail);
//void exitSettingsMenu(void);

char bufferUART[200];

// From InterruptRoutines.c
extern uint8_t dataReady_FIFO;
extern uint8_t dataReady_STREAM;
extern uint8_t cfg_reg;
// | - | - | ACQUISITION ON/OFF | ANALOG | FSR | FSR | FS | FS |
extern uint8_t cfg_reg_old;
extern uint8_t data_config[4];
extern uint8_t configSets[2];
extern uint8_t update_cfg_reg;

extern uint8_t ResetButton;

const uint16_t Timer_Period[4] = {1000, 100, 40, 20};


// Global in main.c
extern uint16_t indexEEPROM;
uint8_t dataFIFO[SPI_LIS3DH_SIZE_BYTE]; // store data from readpage in LIS3DH
uint8_t reg_value;  // temp
uint16_t indexSTREAM = DATA_START;
uint8_t  memoryFull = 0;

uint8_t dataReady_DMA = 0; // extern in InterruptRoutines.h

//stream
extern uint8_t Startstream;
extern uint8_t Stopstream;
uint8_t frequency;
int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    uint8_t     EEPROMBuffer[ACC_PLUS_ANALOG_SIZE];
    
    uint8_t     STREAMBufferIn[6]; 
    uint8_t     STREAMBufferOut[8];
    
    // EEPROM indexing management vars
    uint8_t sizeEEPROMBuffer, indexEEPROMbuffer = 0;
    uint8_t toNextPage, nBytes;
    
    // Acceleration support variables
    //int16_t OutAcc[96];     // right adjusted values
    
    union t_data dataSample[4];
    
    /* Start UART */
    UART_Start();
    
    /*Start PWM*/
    blink_clock_Start();
    PWM_BLINK_Start();
    
    /* Start SPI Masters */
    SPIM_EEPROM_Start();
    CyDelay(10);
    SPIM_LIS3DH_Start();
    CyDelay(10);
    
    /*----------------------------------------------*/
    /*                Setup 25LC256                 */
    /*----------------------------------------------*/
    EEPROM_loadSettings(&cfg_reg, &indexEEPROM);
    indexEEPROM = DATA_START;
    
    sprintf(bufferUART, "** EEPROM Read\r\nconfig_reg = 0x%02x\r\n", cfg_reg);
    UART_PutString(bufferUART);
    
    UART_PutString("*************************************\r\n");
    
    /*----------------------------------------------*/
    /*                 Setup LIS3DH                 */
    /*----------------------------------------------*/
    LIS3DH_unzipConfig(cfg_reg, data_config);
    
    
    // Display current settings
    sprintf(bufferUART, "** LIS3DH Unzip\r\nFS     = 0x%02x\r\nFSR    = 0x%02x\r\nANALOG = 0x%02x\r\n", data_config[0], data_config[1], data_config[2]);
    UART_PutString(bufferUART);
    
    LIS3DH_Init(); // set minimum settings
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
    /*              Setup ADC and DMA               */
    /*----------------------------------------------*/
    
    /* MUX, ADC and DMA */
    AMux_Start();
    AMux_Select((cfg_reg>>4)&0x01); // select analog source
    
    ADC_DelSig_Start();     // Start ADC
    ADC_DelSig_IRQ_Start(); // Enable ISR on EOC
    
    DMA_Config();           // Start DMA
    
    /*----------------------------------------------*/
    /*              Setup ISR routines              */
    /*----------------------------------------------*/
    isr_STREAM_StartEx(Custom_ISR_STREAM);      // enable ISR for streaming data
    isr_INT_StartEx(Custom_ISR_OVRN);           // enable ISR from LIS3DH in the µC
    isr_RX_StartEx(Custom_ISR_RX);              // enable ISR from UART RX
    isr_DMA_StartEx(Custom_ISR_DMA);            // ISR every 32 EOC
    isr_TIMER_ADC_StartEx(Custom_ISR_TIMER);    // ISR on SOC
    SW1_ISR_StartEx(Custome_ISR_DEBOUNCE);      // ISR Reset button
    
    // Begin by showing main interface
    ShowMenu();

  
    for(;;)
    {
        if(ResetButton==1){
            cfg_reg=0x00;
            LIS3DH_unzipConfig(cfg_reg, data_config);
            LIS3DH_setConfig(data_config);
            
            indexEEPROM=DATA_START;
            EEPROM_writePage(0x0000, (uint8_t*)&indexEEPROM, 2);
            EEPROM_waitForWriteComplete();
            indexSTREAM=DATA_START;
            
            memoryFull=0;
            
            PWM_BLINK_WritePeriod(0);
            PWM_BLINK_WriteCompare(0);
            Pin_LED_ext_Write(LED_OFF);
            
            Timer_ADC_Stop();
            Timer_STREAM_Stop();
            
            sprintf(bufferUART, "** EEPROM Read\r\nconfig_reg = 0x%02x\r\n", cfg_reg);
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
            
            sprintf(bufferUART, "** EEPROM Read\r\nconfig_reg = 0x%02x\r\n", cfg_reg);
            UART_PutString(bufferUART);
            
            sprintf(bufferUART, "** IndexEEPROM= 0x%02x, IndexSTREAM= 0x%02x\r\n", indexEEPROM, indexSTREAM);
            UART_PutString(bufferUART);
            
            ResetButton=0;
            
        }
        
        if(update_cfg_reg) {
            
            EEPROM_writeByte(CFG_REG, cfg_reg);
            EEPROM_waitForWriteComplete();
            reg_value = EEPROM_readByte(CFG_REG);
            sprintf(bufferUART, "** UPDATED config_reg = 0x%02x, 0x%02x\r\n", reg_value, cfg_reg);
            UART_PutString(bufferUART);
            
            // Start/Stop ISR from LIS3DH
            LIS3DH_setConfig(data_config); // set user-dependent settings
            
            // Start/Stop Timer_ADC
            frequency=((data_config[0]>>4)&0x0F)-1;
            
            //(((cfg_reg>>5)&0x01) ==1 ? Timer_ADC_Start() : Timer_ADC_Stop());
            if(((cfg_reg>>5) & 0x01) ==1) {
                Timer_ADC_Start();
            }
            else {
                Timer_ADC_Stop();
            }
            
            Timer_ADC_WritePeriod(Timer_Period[frequency]); //cfg_reg&MASK_FS
            
            //sprintf(bufferUART, "DATACONFIG ADC = %d\r\n", Timer_Period[frequency]);
            //UART_PutString(bufferUART);
            
            update_cfg_reg = 0;
            
        }
        
        // go into configuration mode
        if ((configSets[0]!=0) && (configSets[1]<5) ) {
            
            cfg_reg_old = cfg_reg;
            
            sprintf(bufferUART, "cfg_rg OLD = 0x%02x\r\n", cfg_reg_old);
            UART_PutString(bufferUART);
            
            cfg_reg = LIS3DH_zipConfig(data_config);
            
            sprintf(bufferUART, "cfg_reg NEW = 0x%02x\r\n", cfg_reg);
            UART_PutString(bufferUART);
            
            if (cfg_reg_old!=cfg_reg) {
                PWM_BLINK_WritePeriod(0);
                PWM_BLINK_WriteCompare(0);
                indexEEPROM = DATA_START; // bring index back to start
                memoryFull = 0;
                
                data_config[3] = 0x00; // Stop acquisition
                LIS3DH_setConfig(data_config);
                Timer_ADC_Stop();
                
                reg_value = LIS3DH_readByte(FIFO_CTRL_REG);
                sprintf(bufferUART, "FIFO control after stop = %02x\r\n", reg_value);
                UART_PutString(bufferUART);
                
                // Store new config in EEPROM
                cfg_reg = LIS3DH_zipConfig(data_config);
                EEPROM_writeByte(CFG_REG, cfg_reg);
                EEPROM_waitForWriteComplete();
            }
            // Set RX default
            configSets[0]=0;
            configSets[1]=5;  
        }
        
        if(Startstream==1) {
            sprintf(bufferUART, "DATACONFIG = 0x%02x\r\n", data_config[0]);
            UART_PutString(bufferUART);
            
            frequency=((data_config[0]>>4)&0x0F)-1;
            
            sprintf(bufferUART, "DATACONFIG = 0x%02x\r\n", frequency);
            UART_PutString(bufferUART);
            
            // if b then v
            //EEPROM_writePage(0x0000, (uint8_t*)&indexEEPROM, 2);
            //EEPROM_waitForWriteComplete();
            
            Timer_STREAM_Start();
            Timer_STREAM_WritePeriod(Timer_Period[frequency]); //cfg_reg&MASK_FS
            
            Startstream=0;
        }
        
        if(Stopstream==1) {
            indexSTREAM  = DATA_START;
            indexEEPROM = DATA_START;
            memoryFull = 0;
            
            
            
            Stopstream=0;
        }
        // Visualize data in BCP
        if(dataReady_STREAM & !dataReady_FIFO) {
            
            /*
            reg_value = EEPROM_readStatus();
            sprintf(bufferUART, "STATUS REG STR = 0x%02x\r\n", reg_value);
            UART_PutString(bufferUART);
            */
            if(indexSTREAM >= indexEEPROM) {
                indexSTREAM  = DATA_START;
            }
            
            EEPROM_readPage(indexSTREAM, STREAMBufferIn, 6);

            DeCompressData(STREAMBufferIn, STREAMBufferOut);
            
            SampledToBridge(STREAMBufferOut, dataSample);
            
            if (dataSample[3].sample < MIN)    dataSample[3].sample = MIN;
            if (dataSample[3].sample > MAX)    dataSample[3].sample = MAX;
            
            
            
            UART_Debug_BCP(dataSample[0].bytes, 8, HEAD, TAIL);
            
            indexSTREAM +=6;

            dataReady_STREAM = 0;
        }
        
        if(dataReady_FIFO & dataReady_DMA & (!memoryFull)) {
            
            sprintf(bufferUART, "config_reg = 0x%02x\r\n", cfg_reg);
            //UART_PutString(bufferUART);
            
            /* Release INT line */
            reg_value = LIS3DH_readByte(INT1_SRC);
            //sprintf(bufferUART, "INT1 src loop = %02x\r\n", reg_value);
            //UART_PutString(bufferUART);
            
            //reg_value = LIS3DH_readByte(FIFO_CTRL_REG);
            //sprintf(bufferUART, "FIFO src before read = %02x\r\n", reg_value);
            //UART_PutString(bufferUART);
            
            reg_value = LIS3DH_readByte(FIFO_SRC_REG);
            //sprintf(bufferUART, "FIFO src before read = %02x\r\n", reg_value);
            //UART_PutString(bufferUART);
            
            LIS3DH_readPage(OUT_X_L, dataFIFO, 96);
            while(LIS3DH_readStatus() & SPI_LIS3DH_OVRN_FIFO);
            LIS3DH_readPage(OUT_X_L, &dataFIFO[96], 96);
            while(LIS3DH_readStatus() & SPI_LIS3DH_OVRN_FIFO);
            
            CompressData(dataFIFO, ADCBuffer, EEPROMBuffer); // TODO add size of dataFIFO if variable
            
            sprintf(bufferUART, "indexEEPROM = %d\r\n", indexEEPROM);
            //UART_PutString(bufferUART);
            
            /* Write compressed data on EEPROM */
            
            indexEEPROMbuffer = 0;
            // From LIS3DH application note (2/3 * 3/2)
            sizeEEPROMBuffer = ((reg_value & 0x5F) == 0x5F ? 0x20*6 : (reg_value & 0x1F)*6);
            
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
            EEPROM_writePage(0x0000, (uint8_t*)&indexEEPROM, 2);
            EEPROM_waitForWriteComplete();
            
            if (indexEEPROM > SPI_EEPROM_SIZE_BYTE){ // the last write should stop at 0x7FFF
                memoryFull = 1;
                data_config[3]=0x00; // bypass mode
                cfg_reg = LIS3DH_zipConfig(data_config);
                update_cfg_reg = 1; // write on EEPROM new config
                
                PWM_BLINK_WritePeriod(49);
                PWM_BLINK_WriteCompare(25);
            }
            
            dataReady_FIFO = 0; // acknowledge FIFO
            dataReady_DMA = 0;  // acknowledge DMA
        }
    }
}

void CompressData(uint8_t* dataInAcc, uint8_t* dataInADC, uint8_t* dataOut) {
    // Compress data to required format
    for(uint8_t i=0; i<ACC_PLUS_ANALOG_SIZE; i+=6) {
        dataOut[i+0] = ((dataInAcc[i+1]  >>2));
        dataOut[i+1] = ((dataInAcc[i+1]  <<6) | (dataInAcc[i]>>2) | (dataInAcc[i+3]>>4));
        dataOut[i+2] = ((dataInAcc[i+3]<<4) | (dataInAcc[i+2]>>4) | (dataInAcc[i+5]>>6));
        dataOut[i+3] = ((dataInAcc[i+5]<<2) | (dataInAcc[i+4]>>6));
        dataOut[i+4] = (dataInADC[i/3+0]);
        dataOut[i+5] = (dataInADC[i/3+1]);
    }
}

void DeCompressData(uint8_t* dataIn, uint8_t* dataOut) {
    // Decompress data left adjusted
    dataOut[0] = ((dataIn[1]<<2) & 0xC0);           // X-low
    dataOut[1] = ((dataIn[0]<<2) | (dataIn[1]>>6)); // X-high
    dataOut[2] = ((dataIn[2]<<4) & 0xC0);           // Y-low
    dataOut[3] = ((dataIn[1]<<4) | (dataIn[2]>>4)); // Y-high
    dataOut[4] = ((dataIn[3]<<6));                  // Z-low
    dataOut[5] = ((dataIn[2]<<6) | (dataIn[3]>>2)); // Z-high
    dataOut[6] = (dataIn[4]);                       // ADC-low
    dataOut[7] = (dataIn[5]);                       // ADC-high
}

/*void SampledToBridge(uint8_t* dataIn, union t_data* dataOut){
    dataOut[0].sample = (float) (((int16_t) (dataIn[0] | (dataIn[1]<<8))>>6)*g_CONST*100/(0x100>>(cfg_reg>>2 & 0x03)));
    dataOut[1].sample = (float) (((int16_t) (dataIn[2] | (dataIn[3]<<8))>>6)*g_CONST*100/(0x100>>(cfg_reg>>2 & 0x03)));
    dataOut[2].sample = (float) (((int16_t) (dataIn[4] | (dataIn[5]<<8))>>6)*g_CONST*100/(0x100>>(cfg_reg>>2 & 0x03)));
    dataOut[3].sample =          ((int16_t) (dataIn[6] | (dataIn[7]<<8)));
}*/

void SampledToBridge(uint8_t* dataIn, union t_data* dataOut){
    const uint8_t SampledTomg[4] = {4, 8, 16, 48};
    dataOut[0].sample = (float) (((int16_t) (dataIn[0] | (dataIn[1]<<8))>>6))*g_CONST*SampledTomg[cfg_reg>>2 & 0x03]/10;
    dataOut[1].sample = (float) (((int16_t) (dataIn[2] | (dataIn[3]<<8))>>6))*g_CONST*SampledTomg[cfg_reg>>2 & 0x03]/10;
    dataOut[2].sample = (float) (((int16_t) (dataIn[4] | (dataIn[5]<<8))>>6))*g_CONST*SampledTomg[cfg_reg>>2 & 0x03]/10;
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



/* [] END OF FILE */
