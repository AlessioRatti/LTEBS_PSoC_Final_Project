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

#ifndef __DMA_ROUTINES_H
    #define __DMA_ROUTINES_H
    
        #include "project.h"
        #include "stdio.h"
    
    /* Defines for DMA */
    #define DMA_BYTES_PER_BURST 2
    #define DMA_REQUEST_PER_BURST 1
    #define DMA_SRC_BASE (CYDEV_PERIPH_BASE)
    #define DMA_DST_BASE (CYDEV_SRAM_BASE)
    
    #define BYTE_IN_DMA 64
    
    void DMA_Config(void);
    
    uint8_t ADCBuffer[BYTE_IN_DMA];
    
#endif

/* [] END OF FILE */


