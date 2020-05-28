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

#include "DMARoutines.h"

// Used in InterruptRoutines.c
uint8 DMA_Chan;

void DMA_Config(void){
    
    /* Variable declarations for DMA */
    /* Move these variable declarations to the top of the function */
    uint8 DMA_Chan;
    uint8 DMA_TD[1];

    /* DMA Configuration for DMA */
    DMA_Chan = DMA_DmaInitialize(DMA_BYTES_PER_BURST, DMA_REQUEST_PER_BURST, 
        HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    DMA_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_TD[0], 64, DMA_TD[0], DMA__TD_TERMOUT_EN | CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(DMA_TD[0], LO16((uint32)ADC_DelSig_DEC_SAMP_PTR), LO16((uint32)ADCBuffer));
    CyDmaChSetInitialTd(DMA_Chan, DMA_TD[0]);
    CyDmaChEnable(DMA_Chan, 1);
}

/* [] END OF FILE */
