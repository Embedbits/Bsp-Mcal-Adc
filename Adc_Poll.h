/**
 * \author Mr.Nobody
 * \file Adc_Poll.h
 * \ingroup Adc
 * \brief Adc module polling data transfer handler (private)
 *
 * ADC_TRANSFER_MODE_POLL - ADC flags (EOC, OVR, JEOS) are polled from Adc_Task().
 * Private to the Adc library.
 *
 */

#ifndef ADC_ADC_POLL_H
#define ADC_ADC_POLL_H

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

adc_RequestState_t Adc_Poll_Check_Config ( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig );

adc_RequestState_t Adc_Poll_Init         ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Poll_Deinit       ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Poll_Start        ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Poll_Stop         ( adc_PeriphId_t periphId );

adc_RequestState_t Adc_Poll_Task         ( adc_PeriphId_t periphId );

#ifdef __cplusplus
}
#endif

#endif /* ADC_ADC_POLL_H */
