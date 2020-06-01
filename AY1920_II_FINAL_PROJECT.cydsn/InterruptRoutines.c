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

// Global in InterruptRoutines.c
char bufferUART[200];
uint8_t configSets[2] = {0, 0};

// Extern in main.c
volatile uint8_t dataReady_FIFO   = 0;   // Flag for Custom_ISR_OVRN
volatile uint8_t dataReady_STREAM = 0;   // Flag for Custom_ISR_DMA
volatile uint8_t dataReady_DMA    = 0;   // Flag for Custom_ISR_STREAM

// From LIS3DH.c
extern const uint8_t lis3dh_dataRate_t[4];
extern const uint8_t lis3dh_dataRange_t[4];

// From DMARoutines.c
extern uint8 DMA_Chan; // Handle

// From main.c
extern volatile uint8_t cfg_reg;
extern volatile uint8_t update_cfg_reg;
extern volatile uint8_t data_config[4];
extern volatile uint16_t indexSTREAM;
extern const uint16_t Timer_Period[4];

/*----------------------------------------------*/
/*                  ISR on RX                   */
/*----------------------------------------------*/
CY_ISR(Custom_ISR_RX)
{
    // Non-blocking call to get the latest data recieved  
    uint8_t ch_receveid = UART_GetChar();
    
    // Switch cases to navigate the menu and submenus
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
                // Save new settings
                update_cfg_reg = 1;
                break;
                
            case 'S':
            case 's':
                // Go back to state 0
                configSets[0] = 0;
                // Stop acquisition
                data_config[3]=0x00; // bypass mode
                // Force release of DMA transaction
                CyDmaChSetRequest(DMA_Chan, CY_DMA_CPU_TERM_TD);
                // Compress new settings
                cfg_reg = LIS3DH_zipConfig(data_config);
                //Call isr in case of forced stop
                isr_INT_SetPending();
                isr_DMA_SetPending();
                // Save new settings
                update_cfg_reg = 1;
                break;
                
            case 'F':
            case 'f':
                UART_PutString("\n\r|Full Scale Range                     |\n| 1        | +- 2g                    |\n| 2        | +- 4g                    |\n| 3        | +- 8g                    |\n| 4        | +- 16g                   |\n\r");
                configSets[0] = ch_receveid;//Store submenu selection
                break;
                
            case 'P':
            case 'p':
                UART_PutString("\n\r| Sampling Frequency                  |\n| 1        |  1Hz                     |\n| 2        | 10Hz                     |\n| 3        | 25Hz                     |\n| 4        | 50Hz                     |\n\r");
                configSets[0] = ch_receveid;//Store submenu selection
                break;
                
            case 'L':
            case 'l':
                UART_PutString("\n\r| Sensor                              |\n| 0        | Store luminosity data    |\n| 1        | Store potentiometer data |\n\r");
                configSets[0] = ch_receveid;//Store submenu selection
                break;
            
            default:
                break;
        }
    }
    else{
        switch(configSets[0]) {
            // In case previous reception was an F/f
            case 'f':
            case 'F':
                switch(ch_receveid) {
                    case '1':
                        data_config[1]=lis3dh_dataRange_t[0] | CTRL_REG4_NORM; //+-2g
                        configSets[1]=1;
                        break;
                    case '2':
                        data_config[1]=lis3dh_dataRange_t[1] | CTRL_REG4_NORM; //+-4g
                        configSets[1]=1;
                        break;
                    case '3':
                        data_config[1]=lis3dh_dataRange_t[2] | CTRL_REG4_NORM; //+-8g
                        configSets[1]=1;
                        break;
                    case '4':
                        data_config[1]=lis3dh_dataRange_t[3] | CTRL_REG4_NORM; //+-16g
                        configSets[1]=1;
                        break;
                    case '?': // go back to main menu
                        UART_PutChar(0x0C);
                        ShowMenu();
                        break;
                    default: // invalid character received
                        configSets[1]=0;
                        break;
                }
                break;
                
            // In case previous reception was a P/p
            case 'p':
            case 'P':
                switch(ch_receveid) {
                    case '1':
                        data_config[0]=lis3dh_dataRate_t[0]| CTRL_REG1_NORM; // 1Hz
                        configSets[1]=1;
                        break;
                    case '2':
                        data_config[0]=lis3dh_dataRate_t[1]| CTRL_REG1_NORM; // 10Hz
                        configSets[1]=1;
                        break;
                    case '3':
                        data_config[0]=lis3dh_dataRate_t[2]| CTRL_REG1_NORM; // 25Hz
                        configSets[1]=1;
                        break;
                    case '4':
                        data_config[0]=lis3dh_dataRate_t[3]| CTRL_REG1_NORM; // 50Hz
                        configSets[1]=1;
                        break;
                    case '?': // go back to main menu
                        UART_PutChar(0x0C);
                        ShowMenu();
                        break;
                    default: // invalid character received
                        configSets[1] = 0;
                        break;
                }
                break;
            
            // In case previous reception was an L/l
            case 'l':
            case 'L':
                switch(ch_receveid) {
                    case '0':
                        configSets[1] = 1;
                        UART_PutString("--> Photoresistor\n\r");
                        data_config[2] = 0; //selection
                        break;
                    case '1':
                        configSets[1] = 1;
                        UART_PutString("--> Potentiometer\n\r");
                        data_config[2] = 1; //selection
                        break;
                    case '?': // go back to main menu
                        UART_PutChar(0x0C);
                        ShowMenu();
                        break;
                    default: // invalid character received
                        configSets[1] = 0;
                        break;
                }
                break;
            
            default:
                break;
        }   
    }
    
    // Manage visualization
    if(!data_config[3] && (ch_receveid == 'V' || ch_receveid == 'v')) {
        //Store submenu selection
        configSets[0] = ch_receveid;
    
        //Stream frequency
        // Align frequency of transmission with sampling frequency
        uint8_t frequency_STREAM = ((data_config[0]>>4)&0x0F)-1;
        Timer_STREAM_Start();
        Timer_STREAM_WritePeriod(Timer_Period[frequency_STREAM]);

        //Turn on external LED
        Pin_LED_ext_Write(LED_ON);
        
    }
    // after V
    else if((configSets[0]=='V')||(configSets[0]=='v')){
        switch(ch_receveid){
            case 'U':
            case 'u':
            case '?':
                // stop timer stream
                Timer_STREAM_Stop();
                // reset indexstream
                indexSTREAM = DATA_START;
                // turn off LED
                Pin_LED_ext_Write(LED_OFF);
                // Clear terminal and show Menu
                UART_PutChar(0x0C);
                ShowMenu();
                break;
        }
    }
}

/*----------------------------------------------*/
/*                 ISR on OVRN                  */
/*----------------------------------------------*/
CY_ISR(Custom_ISR_OVRN) {
    
    // Raise flag
    dataReady_FIFO = 1;
    
}

/*----------------------------------------------*/
/*               ISR for STREAM                 */
/*----------------------------------------------*/
CY_ISR(Custom_ISR_STREAM) {
    
    // Raise flag
    dataReady_STREAM = 1;
    
}

/*----------------------------------------------*/
/*                ISR for DMA                   */
/*----------------------------------------------*/
CY_ISR (Custom_ISR_DMA) {
    
    // Raise flag
    dataReady_DMA = 1;
    Timer_ADC_Stop();
    
}

/*----------------------------------------------*/
/*                ISR for SOC                   */
/*----------------------------------------------*/
CY_ISR (Custom_ISR_TIMER) {
    
    // Read Timer status register to bring interrupt line low
    Timer_ADC_ReadStatusRegister();
    
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
    data_config[3] = 0x00; // bypass mode
    cfg_reg = LIS3DH_zipConfig(data_config);
    
    // Make sure to read the last packets
    isr_INT_SetPending();
    isr_DMA_SetPending();
    
    // Stop fast blinking in case of memory full
    PWM_BLINK_WritePeriod(0);
    PWM_BLINK_WriteCompare(0);
    
    // apply and save new settings
    update_cfg_reg = 1;
    
    UART_PutString("\r\n************** Data memory cleared! **************\n\r");
    
}

CY_ISR(Custom_ISR_RESET) { // ISR on 10s
    
    cfg_reg = 0x00; // Reset configs
    // apply and save new settings
    update_cfg_reg = 1;
    
    UART_PutString("\r\n**************     SYSTEM RESET!    **************\n\r");
    
}

// Aggregate all information to show the user into a single function
void ShowMenu(void) {
    configSets[0] = 0;
    configSets[1] = 0;
    UART_PutString("                        <<--- PORTABLE DATA ACQUISITION BOARD --->>                \r\nPlease select your personal settings\r\n");
    UART_PutString("    Legend:\r\n");
    UART_PutString("        B/b --> Start Data Acquisition\r\n"
                   "        F/f --> Select Full Scale Range\r\n"
                   "        P/p --> Select Sampling Frequency\r\n"
                   "        L/l --> Select Analog Sensor\r\n"
                   "        S/s --> Stop data acquisition\r\n"
                   "        V/v --> Visualize data in BCP\r\n"
                   "        U/u --> Exit data visualization\r\n"
                   "         ?  --> Main Menu\r\n");
    const uint8_t freqz[4] = {1, 10, 25, 50};
    const uint8_t FSR[4]   = {2, 4, 8, 16};
    const char *sensor[2];
    sensor[0] = "Photoresistor";
    sensor[1] = "Potentiometer";
    const char *state[2];
    state[0] = "Idling";
    state[1] = "Acquiring";
    sprintf(bufferUART, "Sampling frequency = %d Hz | ", freqz[(cfg_reg&0x03)]);
    UART_PutString(bufferUART);
    sprintf(bufferUART, "Full scale range = +-%d g\r\n", FSR[((cfg_reg>>2)&0x03)]);
    UART_PutString(bufferUART);
    sprintf(bufferUART, "Analog sensor = %s | ", sensor[((cfg_reg>>4)&0x01)]);
    UART_PutString(bufferUART);
    sprintf(bufferUART, "State = %s \r\n", state[((cfg_reg>>5)&0x01)]);
    UART_PutString(bufferUART);
}

/* [] END OF FILE */
