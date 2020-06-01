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
    #include "project.h"
    #include "cytypes.h"
    #include "stdio.h"
    #include "string.h"
    #include "LIS3DH.h"
    #include "25LC256.h"
    #include "DMARoutines.h"
    
    //Define some constans
    #define LED_OFF 0           // Switch off the LED
    #define LED_ON  1           // Switch on the LED
    #define MAX 5000            // Maximum value for the ADC
    #define MIN 0               // Minimum value for the ADC
    #define PHOTORESISTOR 0     // Photoresist's channel into the AMux
    #define POTENTIOMETER 1     // Potentiometer's channel into the AMux
    
    #define CFG_REG 0x0002      // location where permanent settings are saved
    #define INDEX_REG 0x0000    // location where permanent EEPROMIndex is saved
    
    /*
    *   \brief ISR Code.
    */
    CY_ISR_PROTO (Custom_ISR_RX);
    CY_ISR_PROTO (Custom_ISR_TIMER);
    CY_ISR_PROTO (Custom_ISR_OVRN);
    CY_ISR_PROTO (Custom_ISR_DMA);
    CY_ISR_PROTO (Custom_ISR_STREAM);
    CY_ISR_PROTO (Custom_ISR_INDEX);
    CY_ISR_PROTO (Custom_ISR_RESET);
    
    /* Functions */
    void ShowMenu(void);
    
#endif

/* [] END OF FILE */
