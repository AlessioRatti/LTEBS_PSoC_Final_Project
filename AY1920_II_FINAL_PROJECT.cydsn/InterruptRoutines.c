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
*   \brief Source code for Interrupt Routines.
*/

// Include header
#include "InterruptRoutines.h"

// Variables declaration
//int16 value_digit_analog;   //Value from the analog sensor

uint8_t ch_receveid;//Character to switch on/off the UART transmission
uint8_t ControlFlag;//Control the on/off of the led

// Extern defined here
uint8_t dataReady_FIFO = 0;
uint8_t dataReady_STREAM = 0;
uint8_t configSets[2] = {0, 0};
uint8_t data_config[4];
uint8_t cfg_reg;
uint8_t cfg_reg_old;
uint8_t update_cfg_reg;
uint8_t frequency;


char bufferUART[200];

uint8_t reg_value;

// From LIS3DH.c
extern const uint8_t lis3dh_dataRate_t[4];
extern const uint8_t lis3dh_dataRange_t[4];

// From main.c
extern uint16_t indexSTREAM;
extern uint16_t indexEEPROM;
extern uint16_t indexEEPROM_BCP;
extern uint8_t memoryFull;
extern uint8_t dataReady_DMA;
extern const uint16_t Timer_Period[4];

// From DMARoutines.c
extern uint8 DMA_Chan;


/*----------------------------------------------*/
/*                  ISR on RX                   */
/*----------------------------------------------*/

CY_ISR(Custom_ISR_RX)
{
    //Non-blocking call to get the latest data recieved  
    ch_receveid = UART_GetChar();
    sprintf(bufferUART, "ch_received = %c\r\n", ch_receveid);
    UART_PutString(bufferUART);
    sprintf(bufferUART, "configSets = %d - %d\r\n", configSets[0], configSets[1]);
    UART_PutString(bufferUART);
    
    //Switch on/off the UART comunication
    if(!configSets[0]){
        switch(ch_receveid)
        {
            case 'B':
            case 'b':
                // Go back to state 0
                configSets[0]=0;
                
                // Begin acquisition
                data_config[3]=0x80;
                
                // Compress new settings (begin acquisition)
                cfg_reg = LIS3DH_zipConfig(data_config);
                
                update_cfg_reg = 1;
                break;
            case 'S':
            case 's':
                // Go back to state 0
                configSets[0] = 0;
                
                // Stop acquisition
                data_config[3]=0x00; // bypass mode
                
                CyDmaChSetRequest(DMA_Chan, CY_DMA_CPU_TERM_TD); // Force release of DMA transaction
                
                // Compress new settings (stop acquisition)
                cfg_reg = LIS3DH_zipConfig(data_config);
                
                //Call isr in case of forced stop
                isr_INT_SetPending();
                isr_DMA_SetPending();
                
                update_cfg_reg = 1;
                
                // Show the main menu
                ShowMenu();
                break;
            case 'F':
            case 'f':
                UART_PutString("|Character | Full Scale Range         |\n| 1        | +- 2g                    |\n| 2        | +- 4g                    |\n| 3        | +- 8g                    |\n| 4        | +- 16g                   |\n\r");
                configSets[0] = ch_receveid;//Store submenu selection
                
                break;
            case 'P':
            case 'p':
                UART_PutString("|Character | Sampling Frequency       |\n| 1        |  1Hz                     |\n| 2        | 10Hz                     |\n| 3        | 25Hz                     |\n| 4        | 50Hz                     |\n\r");
                configSets[0] = ch_receveid;//Store submenu selection
               
                break;
            case 'L':
            case 'l':
                UART_PutString("|Character | Sensor                   |\n| 0        | Store luminosity data    |\n| 1        | Store potentiometer data |\n\r");
                configSets[0] = ch_receveid;//Store submenu selection
                break;
            
            default:
                break;
        }
    }
    else{
        switch(configSets[0]) {
            case 'f':
            case 'F':
                switch(ch_receveid) {
                    case '1':
                        data_config[1]=lis3dh_dataRange_t[0] | CTRL_REG4_NORM;//+-2g
                        configSets[1]=1;
                        break;
                    case '2':
                        data_config[1]=lis3dh_dataRange_t[1] | CTRL_REG4_NORM;//+-4g
                        configSets[1]=1;
                        break;
                    case '3':
                        data_config[1]=lis3dh_dataRange_t[2] | CTRL_REG4_NORM;//+-8g
                        configSets[1]=1;
                        break;
                    case '4':
                        data_config[1]=lis3dh_dataRange_t[3] | CTRL_REG4_NORM;//+-16g
                        configSets[1]=1;
                        break;
                    case '?':
                        ShowMenu();
                        break;
                    default:
                        configSets[1]=0;
                        
                        break;
                }
                break;
                
            case 'p':
            case 'P':
                switch(ch_receveid) {
                    case '1':
                        data_config[0]=lis3dh_dataRate_t[0]| CTRL_REG1_NORM;
                        configSets[1]=1;
                        break;
                    case '2':
                        data_config[0]=lis3dh_dataRate_t[1]| CTRL_REG1_NORM;
                        configSets[1]=1;
                        break;
                    case '3':
                        data_config[0]=lis3dh_dataRate_t[2]| CTRL_REG1_NORM;
                        configSets[1]=1;
                        break;
                    case '4':
                        data_config[0]=lis3dh_dataRate_t[3]| CTRL_REG1_NORM;
                        configSets[1]=1;
                        break;
                    case '?':
                        ShowMenu();
                        break;
                    default:
                        configSets[1]=0;
                        break;
                }
                break;
                
            case 'l':
            case 'L':
                switch(ch_receveid) {
                    case '0':
                        configSets[1] = 1;
                        UART_PutString("--> Photoresistor\n\r");
                        data_config[2]=0;//selection
                        break;
                    case '1':
                        configSets[1] = 1;
                        UART_PutString("--> Potentiometer\n\r");
                        data_config[2]=1;//selection
                        break;
                    case '?':
                        ShowMenu();
                        break;
                    default:
                        configSets[1]=0;
                        break;
                }
                break;
            
            default:
                break;
        }   
    }
    if(data_config[3] == 0x00 && (ch_receveid == 'V' || ch_receveid == 'v')) {
        //Store submenu selection
        configSets[0] = ch_receveid;
    
        //Stream frequency
        frequency=((data_config[0]>>4)&0x0F)-1;
        Timer_STREAM_Start();
        Timer_STREAM_WritePeriod(Timer_Period[frequency]); //cfg_reg&MASK_FS

        //Turn off external led
        Pin_LED_ext_Write(LED_ON);
        
    }
    // after V
    else if((configSets[0]=='V')||(configSets[0]=='v')){
        switch(ch_receveid){
            case 'U':
            case 'u':
            case '?':
                //stop timer stream
                Timer_STREAM_Stop();
                //reset indexstream
                indexSTREAM=DATA_START;
                //turn off led
                Pin_LED_ext_Write(LED_OFF);
                //Menu
                UART_PutChar(0x0C); // Clear CoolTerm
                ShowMenu();
                break;
        }
    }
}


/*----------------------------------------------*/
/*                 ISR on OVRN                  */
/*----------------------------------------------*/

CY_ISR(Custom_ISR_OVRN) {
    
    //UART_PutString("INTERRUPT OVRN\r\n");
    // Raise flag
    dataReady_FIFO = 1;
    //UART_PutString("LIS3DH\r\n");
}


/*----------------------------------------------*/
/*               ISR for STREAM                 */
/*----------------------------------------------*/

CY_ISR(Custom_ISR_STREAM) {
    // Raise flag
    dataReady_STREAM = 1;
}
/*
void ShowMenu(void) {
    configSets[0] = 0;
    configSets[1] = 0;
    UART_PutString("\n\r                *|WELCOME IN THE FINAL PROJECT|*                \r\nPlease select your personal settings\r\n");
    UART_PutString("    Legend:\r\n");
    UART_PutString("        B/b-->Start Data Acquisition\r\n        F/f-->Select Full Scale Range\r\n"
                   "        P/p-->Select Sampling Frequency\r\n        L/l-->Select Analog Sensor\r\n"
                   "        S/s-->Stop data acquisition\r\n");
}
*/

/*----------------------------------------------*/
/*               ISR for STREAM                 */
/*----------------------------------------------*/
CY_ISR (Custom_ISR_DMA) {
    dataReady_DMA = 1;
    //Pin_DEBUG_Write(!Pin_DEBUG_Read());
    //UART_PutString("DMA\r\n");
    /*
    uint8_t UARTarray[4];
    UARTarray[0] = 0xA0;
    UARTarray[1] = (ADCBuffer[0]);
    UARTarray[2] = (ADCBuffer[1]);
    UARTarray[3] = 0xC0;
    UART_PutArray(UARTarray, 4);
    */
}

/*----------------------------------------------*/
/*                ISR for SOC                   */
/*----------------------------------------------*/
CY_ISR (Custom_ISR_TIMER) {
    // Read Timer status register to bring interrupt line low
    Timer_ADC_ReadStatusRegister();
    //UART_PutString("TIMER\r\n");
    
}

/*----------------------------------------------*/
/*        ISRs for On-board Push Button         */
/*----------------------------------------------*/

CY_ISR(Custom_ISR_INDEX) { // ISR on 5s
    
    // In case visualization is taking place
    Pin_LED_ext_Write(LED_OFF); // LED STREAM OFF
    
    //stop timer stream
    Timer_STREAM_Stop();
    
    //reset indexstream
    indexSTREAM = DATA_START;
    
    // In case acquisition is taking place
    data_config[3]=0x00; // bypass mode
    cfg_reg = LIS3DH_zipConfig(data_config);
    
    // Reset Index EEPROM
    //indexEEPROM = DATA_START; // bring index back to start
    //EEPROM_writePage(0x0000, (uint8_t*) &indexEEPROM, 2);
    //EEPROM_waitForWriteComplete();
    
    isr_INT_SetPending();
    isr_DMA_SetPending();
    
    update_cfg_reg = 1;
    
    UART_PutString("Data memory cleared!\n\r");
    
}

CY_ISR(Custom_ISR_RESET) { // ISR on 10s
    
    cfg_reg = 0x00; // Reset configs
    
    update_cfg_reg = 1;
    
    UART_PutChar(0x0C); // Clear CoolTerm
    
    UART_PutString("SYSTEM RESET!\n\r");
    ShowMenu();
    
}

/* [] END OF FILE */
