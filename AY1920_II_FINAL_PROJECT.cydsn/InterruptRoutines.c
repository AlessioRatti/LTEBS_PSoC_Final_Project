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
int16 value_digit_analog;   //Value from the analog sensor

uint8_t ch_receveid;//Character to switch on/off the UART transmission
uint8_t ControlFlag;//Control the on/off of the led

// Extern defined here
uint8_t dataReady_FIFO = 0;
uint8_t dataReady_STREAM = 0;
uint8_t configSets[2] = {0, 5};
uint8_t data_config[4];
uint8_t cfg_reg;
uint8_t cfg_reg_old;
uint8_t update_cfg_reg;


char bufferUART[200];


uint8_t FlagFSR=0;
uint8_t reg_value;

// From LIS3DH.c
extern const uint8_t lis3dh_dataRate_t[4];
extern const uint8_t lis3dh_dataRange_t[4];

// From main.c
extern uint16_t indexSTREAM;
extern uint16_t indexEEPROM;
extern uint8_t memoryFull;
extern uint8_t dataReady_DMA;
extern const uint16_t Timer_Period[4];

CY_ISR(Custom_ISR_ADC) // ISR on EOC
{
    
    
    //Pin_DEBUG_Write(!Pin_DEBUG_Read());
    
    //Save the value of the selected analog sensor. We use this function because
    //when the ADC is configured for 16-bit single ended mode, the ADC_Read32() function should be used.
    
    //value_digit_analog = ADC_DelSig_GetResult16();
    
    //sprintf(bufferUART, "ADC val = %d\r\n", value_digit_analog);
    /*
    uint8_t UARTarray[4];
    UARTarray[0] = 0xA0;
    UARTarray[1] = (value_digit_analog & 0xFF);
    UARTarray[2] = (value_digit_analog>>8);
    UARTarray[3] = 0xC0;
    UART_PutArray(UARTarray, 4);
    */
    /*
    // Write bytes in buffer, in the position of the photoresistor
    DataBuffer[3] = value_digit_ldr >> 8;
    DataBuffer[4] = value_digit_ldr & 0xFF;
    
    //Switch between photoresistor and potentiometer
    if (value_digit_ldr<THRESHOLD)
    {
        
        AMux_Select(POTENTIOMETER);//Selcet the channel of the potentiometer
        value_digit_pot=ADC_DelSig_Read32();//Save the value of the potentiometer
        //Set the limitations
        if (value_digit_pot < MIN)    value_digit_pot = MIN;
        if (value_digit_pot > MAX)    value_digit_pot = MAX;

        // Write bytes in buffer
        DataBuffer[1] = value_digit_pot >> 8;
        DataBuffer[2] = value_digit_pot & 0xFF;
        
        PWM_WriteCompare(value_digit_pot);//Change the intensity of the LED light with the values from the potentiometer
        AMux_Select(PHOTORESISTOR);//Select the channel of the photoresistor
        ControlFlag=1;
    }
    //Switch off the LED
    if (value_digit_ldr>THRESHOLD && ControlFlag==1) 
    {
        PWM_WriteCompare(LED_OFF);
        ControlFlag=0;
    }
    */
    // Enable to send data based on UART command
    PacketReadyFlag=1;
    
    
}

/*----------------------------------------------*/
/*                  ISR on RX                   */
/*----------------------------------------------*/

uint8_t frequency;
uint8_t Startstream;
uint8_t Stopstream;
uint16_t indexEEPROM;
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
                PWM_BLINK_WritePeriod(199);
                PWM_BLINK_WriteCompare(100);
                //Pin_LED_Write(LED_ON);//Switch on the PSoC Blue LED --> the UART comunication is enabled
                //Timer_Start();//Sets the initVar variable, calls the Timer_Init() function, and then calls the Enable function.
                //AMux_Select(PHOTORESISTOR);//At the beginning set the channel of the photoresistor
                //PWM_WriteCompare(LED_OFF);//At the beginning the LED is off
                //ControlFlag=0;
                
                // Go back to state 0
                configSets[0]=0;
                
                // Begin acquisition
                data_config[3]=0x80;
                
                // Compress new settings (begin acquisition)
                cfg_reg = LIS3DH_zipConfig(data_config);
                
                // Save new settings in EEPROM
                /*EEPROM_writeByte(CFG_REG, cfg_reg);
                EEPROM_waitForWriteComplete();
                reg_value = EEPROM_readByte(CFG_REG);
                sprintf(bufferUART, "** UPDATED config_reg = 0x%02x\r\n", reg_value);
                UART_PutString(bufferUART);*/
                memoryFull = 0;
                indexEEPROM = DATA_START; // bring index back to start
                update_cfg_reg = 1;
                break;
            case 'S':
            case 's':
                PWM_BLINK_WritePeriod(0);
                PWM_BLINK_WriteCompare(0);
                //Pin_LED_Write(LED_OFF);//Switch off the PSoC Blue LED --> the UART comunication is disabled
                //Timer_Stop();//Disables the Timer
                //PWM_WriteCompare(LED_OFF);//At the end the LED is off
                
                // Go back to state 0
                configSets[0] = 0;
                
                // Stop acquisition
                data_config[3]=0x00; // bypass mode
                //LIS3DH_setConfig(data_config); // set user-dependent settings
                
                // Compress new settings (stop acquisition)
                cfg_reg = LIS3DH_zipConfig(data_config);
                
                // Save new settings in EEPROM
                /**EEPROM_writeByte(CFG_REG, cfg_reg);
                EEPROM_waitForWriteComplete();
                reg_value = EEPROM_readByte(CFG_REG);
                sprintf(bufferUART, "** UPDATED config_reg = 0x%02x\r\n", reg_value);
                UART_PutString(bufferUART);*/
                //reg_value = EEPROM_readByte(CFG_REG);
                //sprintf(bufferUART, "** UPDATED config_reg = 0x%02x\r\n", reg_value);
                //UART_PutString(bufferUART);
                
                update_cfg_reg = 1;
                
                // Show the main menu
                ShowMenu();
                break;
            case 'F':
            case 'f':
                Timer_STREAM_Stop();
                UART_PutString("|Character | Full Scale Range         |\n| 1        | +- 2g                    |\n| 2        | +- 4g                    |\n| 3        | +- 8g                    |\n| 4        | +- 16g                   |\n\r");
                configSets[0] = ch_receveid;
                
                break;
            case 'P':
            case 'p':
                Timer_STREAM_Stop();
                UART_PutString("|Character | Sampling Frequency       |\n| 1        |  1Hz                     |\n| 2        | 10Hz                     |\n| 3        | 25Hz                     |\n| 4        | 50Hz                     |\n\r");
                configSets[0] = ch_receveid;
               
                break;
            case 'L':
            case 'l':
                Timer_STREAM_Stop();
                UART_PutString("|Character | Sensor                   |\n| 0        | Store luminosity data    |\n| 1        | Store potentiometer data |\n\r");
                configSets[0] = ch_receveid;
                break;
            case 'V':
            case 'v':
                PWM_BLINK_WritePeriod(0);
                PWM_BLINK_WriteCompare(0);
                
                data_config[3]=0x00;
                LIS3DH_setConfig(data_config);// set user-dependent settings
                configSets[0] = ch_receveid;
                //configSets[1] = 5;
                /*
                IndexEEPROMStart = (uint8_t)DATA_START;
                EEPROM_writePage(0x0000, &IndexEEPROMStart, 2);
                EEPROM_waitForWriteComplete();
                */
                sprintf(bufferUART, "config_reg = 0x%02x\r\n", cfg_reg_old);
                UART_PutString(bufferUART);
                
                cfg_reg = LIS3DH_zipConfig(data_config);
                
                sprintf(bufferUART, "config_reg = 0x%02x\r\n", cfg_reg);
                UART_PutString(bufferUART);
                
                Pin_LED_ext_Write(LED_ON);
                
                /*
                sprintf(bufferUART, "DATACONFIG = 0x%02x\r\n", data_config[0]);
                UART_PutString(bufferUART);
                frequency=((data_config[0]>>4)&0x0F)-1;
                sprintf(bufferUART, "DATACONFIG = 0x%02x\r\n", frequency);
                UART_PutString(bufferUART);
                */
                //Timer_STREAM_WritePeriod(Timer_Period[frequency]); //cfg_reg&MASK_FS
                //Timer_STREAM_Start();
                Startstream=1;
                break;
            default:
                break;
        }
    }
    else if((configSets[0]=='V')||(configSets[0]=='v')){
        switch(ch_receveid){
            case 'U':
            case 'u':
            case '?':
                Timer_STREAM_Stop();
                Stopstream=1;
                Pin_LED_ext_Write(LED_OFF);
                ShowMenu();
                break;
                }
    }
    else{
        switch(configSets[0]) {
            case 'f':
            case 'F':
                switch(ch_receveid) {
                    case '1':
                        data_config[1]=lis3dh_dataRange_t[0] | CTRL_REG4_NORM;
                        configSets[1]=1;
                        break;
                    case '2':
                        data_config[1]=lis3dh_dataRange_t[1] | CTRL_REG4_NORM;
                        configSets[1]=2;
                        break;
                    case '3':
                        data_config[1]=lis3dh_dataRange_t[2] | CTRL_REG4_NORM;
                        configSets[1]=3;
                        
                        break;
                    case '4':
                        data_config[1]=lis3dh_dataRange_t[3] | CTRL_REG4_NORM;
                        configSets[1]=4;
                        break;
                    case '?':
                        ShowMenu();
                        break;
                    default:
                        configSets[1]=5;
                        
                        break;
                }
                break;
                
            case 'p':
            case 'P':
                switch(ch_receveid) {
                    case '1':
                        data_config[0]=lis3dh_dataRate_t[0]| CTRL_REG1_NORM;
                        sprintf(bufferUART, "dataconfig = 0x%02x\r\n", data_config[0]);
                        UART_PutString(bufferUART);
                        configSets[1]=1;
                        break;
                    case '2':
                        data_config[0]=lis3dh_dataRate_t[1]| CTRL_REG1_NORM;
                        configSets[1]=2;
                        sprintf(bufferUART, "dataconfig = 0x%02x\r\n", data_config[0]);
                        UART_PutString(bufferUART);
                        break;
                    case '3':
                        data_config[0]=lis3dh_dataRate_t[2]| CTRL_REG1_NORM;
                        configSets[1]=3;
                        sprintf(bufferUART, "dataconfig = 0x%02x\r\n", data_config[0]);
                        UART_PutString(bufferUART);
                        break;
                    case '4':
                        data_config[0]=lis3dh_dataRate_t[3]| CTRL_REG1_NORM;
                        configSets[1]=4;
                        sprintf(bufferUART, "dataconfig = 0x%02x\r\n", data_config[0]);
                        UART_PutString(bufferUART);
                        break;
                    case '?':
                        ShowMenu();
                        break;
                    default:
                        configSets[1]=5;
                        break;
                }
                break;
                
            case 'l':
            case 'L':
                switch(ch_receveid) {
                    case '0':
                        AMux_Select(PHOTORESISTOR);
                        configSets[1] = 0;
                        UART_PutString("--> Photoresistor\n\r");
                        data_config[2]=0;
                        break;
                    case '1':
                        AMux_Select(POTENTIOMETER);
                        configSets[1] = 1;
                        UART_PutString("--> Potentiometer\n\r");
                        data_config[2]=1;
                        break;
                    case '?':
                        ShowMenu();
                        break;
                    default:
                        configSets[1]=5;
                        break;
                }
                break;
            
            default:
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
    //UART_PutString("INTERRUPT STREAM\r\n");
    // Raise flag
    dataReady_STREAM = 1;
}

void ShowMenu(void) {
    configSets[0] = 0;
    configSets[1] = 5;
    UART_PutString("\n\r                *|WELCOME IN THE FINAL PROJECT|*                \r\nPlease select your personal settings\r\n");
    UART_PutString("    Legend:\r\n");
    UART_PutString("        B/b-->Start Data Acquisition\r\n        F/f-->Select Full Scale Range\r\n"
                   "        P/p-->Select Sampling Frequency\r\n        L/l-->Select Analog Sensor\r\n"
                   "        S/s-->Stop data acquisition\r\n");
}

/*----------------------------------------------*/
/*               ISR for STREAM                 */
/*----------------------------------------------*/
CY_ISR (Custom_ISR_DMA) {
    dataReady_DMA = 1;
    
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
/*        ISR for On-board Push Button          */
/*----------------------------------------------*/

CY_ISR(Custome_ISR_DEBOUNCE){
    ResetButton=1;
    UART_PutString("HAI PREMUTO IL PULSANTE DI RESET\r\n");
}
/* [] END OF FILE */
