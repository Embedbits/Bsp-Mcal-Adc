/**
 * \author Mr.Nobody
 * \file Adc_Isr.h
 * \ingroup Adc
 * \brief Adc module ADC interrupt data transfer handler (private)
 *
 * ADC interrupt handling used by ADC_TRANSFER_MODE_ISR (EOC, OVR, JEOC) and by
 * ADC_TRANSFER_MODE_DMA (OVR, JEOC). Private to the Adc library.
 *
 */

#ifndef ADC_ADC_ISR_H
#define ADC_ADC_ISR_H

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

adc_RequestState_t Adc_Isr_Check_Config    ( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig );

adc_RequestState_t Adc_Isr_Init            ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Isr_Deinit          ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Isr_Start           ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Isr_Stop            ( adc_PeriphId_t periphId );

adc_RequestState_t Adc_Isr_Set_IrqActive   ( adc_PeriphId_t periphId, uint32_t llItMask );
adc_RequestState_t Adc_Isr_Set_IrqInactive ( adc_PeriphId_t periphId, uint32_t llItMask );

#ifdef __cplusplus
}
#endif

#endif /* ADC_ADC_ISR_H */
