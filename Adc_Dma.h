/**
 * \author Mr.Nobody
 * \file Adc_Dma.h
 * \ingroup Adc
 * \brief Adc module DMA data transfer handler (private)
 *
 * ADC_TRANSFER_MODE_DMA - regular results are moved by a GPDMA channel. Private to the
 * Adc library.
 *
 */

#ifndef ADC_ADC_DMA_H
#define ADC_ADC_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Adc_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

adc_RequestState_t Adc_Dma_Check_Config ( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig );

adc_RequestState_t Adc_Dma_Init         ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Dma_Deinit       ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Dma_Start        ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Dma_Stop         ( adc_PeriphId_t periphId );

#ifdef __cplusplus
}
#endif

#endif /* ADC_ADC_DMA_H */
