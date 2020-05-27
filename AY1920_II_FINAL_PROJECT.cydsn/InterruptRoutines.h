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

#ifndef __INTERRUPT_ROUTINES_H
    //header guard
    #define __INTERRUPT_ROUTINES_H
    
    //Include header 
    #include "cytypes.h"
    #include "stdio.h"
    #include "LIS3DH.h"
    #include "25LC256.h"
    #include "project.h"
    //#include "LIS3DH_Regs.h"
    
    //Define some constant
    #define BYTE_TO_SEND 4 //Information about the photoresist and the potentiometer
    #define TRANSMIT_BUFFER_SIZE 1+BYTE_TO_SEND+1 //Buffer size
    #define LED_OFF 0 //Switch off the LED
    #define LED_ON  1 //Switch on the LED
    #define MAX 1023 //Maximum value for the ADC
    #define MIN 0 //Minimum value for the ADC
    #define THRESHOLD 25000 //Set arbitrary threshold for the light intensity
    #define PHOTORESISTOR 0 //Photoresist's channel into the AMux
    #define POTENTIOMETER 1 //Potentiometer's channel into the AMux
    
    #define CFG_REG 0x0002 // location where permanent settings are saved

    
    /*
    *   \brief ISR Code.
    */
    CY_ISR_PROTO (Custom_ISR_ADC);
    CY_ISR_PROTO (Custom_ISR_RX);
    CY_ISR_PROTO (Custom_ISR_OVRN);
    CY_ISR_PROTO (Custom_ISR_STREAM);
    CY_ISR_PROTO (Custom_ISR_DMA);
    CY_ISR_PROTO (Custom_ISR_TIMER);
    
    /* Functions */
    void ShowMenu(void);
    
    //Set variables
    uint8_t DataBuffer[TRANSMIT_BUFFER_SIZE];   //Memory buffer
    volatile uint8 PacketReadyFlag;             //Flag to enable the transmission
    
    // Bring out extern vars
    extern uint8_t data_config[4];
    extern uint8_t cfg_reg;
    extern uint8_t cfg_reg_old;
    extern uint8_t dataReady_FIFO;
    extern uint8_t dataReady_STREAM;
    extern uint8_t configSets[2];
    extern uint8_t update_cfg_reg;
    
    
#endif

/* [] END OF FILE */
