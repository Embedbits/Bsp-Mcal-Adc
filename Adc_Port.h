/**
 * \author Mr.Nobody
 * \file Adc_Port.h
 * \ingroup Adc
 * \brief Analog-to-Digital Converter (ADC) module public functionality
 *
 * This file contains all available public functionality, any other files shall 
 * not used outside of the module.
 *
 */

#ifndef ADC_ADC_PORT_H
#define ADC_ADC_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Adc_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

adc_ModuleVersion_t         Adc_Get_ModuleVersion           ( void );

adc_RequestState_t          Adc_Init                        ( adc_Config_t * const adcConfig );

adc_RequestState_t          Adc_Deinit                      ( adc_PeriphId_t periphId );
void                        Adc_Task                        ( void );

/* -------------------------------------------------------------------------- */
/* -------------------------- Clock configuration --------------------------- */
/* -------------------------------------------------------------------------- */

adc_RequestState_t          Adc_Set_ClockSource             ( adc_ClkSrc_t clkSource );
adc_RequestState_t          Adc_Get_ClockSource             ( adc_ClkSrc_t * const clkSource );

adc_RequestState_t          Adc_Set_ClockDivider            ( adc_ClkDiv_t clkDiv );
adc_RequestState_t          Adc_Get_ClockDivider            ( adc_ClkDiv_t * const clkDiv );

/* -------------------------------------------------------------------------- */
/* ------------------------ Peripheral configuration ------------------------ */
/* -------------------------------------------------------------------------- */

adc_RequestState_t          Adc_PeriphInit                  ( adc_PeriphConfig_t * const adcConfig );

adc_RequestState_t          Adc_Set_TriggerSrc              ( adc_PeriphId_t periphId, adc_RegTriggerId_t triggerSrc );
adc_RequestState_t          Adc_Get_TriggerSrc              ( adc_PeriphId_t periphId, adc_RegTriggerId_t * const triggerSrc );

adc_RequestState_t          Adc_Set_TriggerMode             ( adc_PeriphId_t periphId, adc_RegTriggerMode_t triggerMode );
adc_RequestState_t          Adc_Get_TriggerMode             ( adc_PeriphId_t periphId, adc_RegTriggerMode_t * const triggerMode );

adc_RequestState_t          Adc_Set_TriggerEdge             ( adc_PeriphId_t periphId, adc_TriggerEdge_t triggerEdge );
adc_RequestState_t          Adc_Get_TriggerEdge             ( adc_PeriphId_t periphId, adc_TriggerEdge_t * const triggerEdge );

/* -------------------------------------------------------------------------- */
/* -------------------------- Peripheral control ---------------------------- */
/* -------------------------------------------------------------------------- */

adc_RequestState_t          Adc_Set_PeriphActive            ( adc_PeriphId_t periphId );
adc_RequestState_t          Adc_Set_PeriphInactive          ( adc_PeriphId_t periphId );

adc_RequestState_t          Adc_Set_RegStart                ( adc_PeriphId_t periphId );
adc_RequestState_t          Adc_Set_RegStop                 ( adc_PeriphId_t periphId );

adc_RequestState_t          Adc_Set_InjStart                ( adc_PeriphId_t periphId );
adc_RequestState_t          Adc_Set_InjStop                 ( adc_PeriphId_t periphId );

/* -------------------------------------------------------------------------- */
/* ---------------------------- Conversion data ----------------------------- */
/* -------------------------------------------------------------------------- */

adc_RequestState_t          Adc_Set_DataConfig              ( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig );
adc_RequestState_t          Adc_Get_DataConfig              ( adc_PeriphId_t periphId, adc_DataConfig_t * const dataConfig );

adc_RequestState_t          Adc_Get_RegData                 ( adc_PeriphId_t periphId, adc_Data_t * const data );
adc_RequestState_t          Adc_Get_InjData                 ( adc_PeriphId_t periphId, adc_InjSequenceId_t rankId, adc_Data_t * const data );

adc_RequestState_t          Adc_Get_Flag                    ( adc_PeriphId_t periphId, adc_FlagId_t flagId, adc_FlagState_t * const flagState );
adc_RequestState_t          Adc_Clear_Flag                  ( adc_PeriphId_t periphId, adc_FlagId_t flagId );

/* -------------------------------------------------------------------------- */
/* -------------------------- Channel configuration ------------------------- */
/* -------------------------------------------------------------------------- */

adc_RequestState_t          Adc_ChannelInit                 ( adc_PeriphId_t periphId, adc_ChannelConfig_t * const channelConfig );

adc_RequestState_t          Adc_Set_Resolution              ( adc_PeriphId_t periphId, adc_Resolution_t channelRes );
adc_RequestState_t          Adc_Get_Resolution              ( adc_PeriphId_t periphId, adc_Resolution_t * const channelRes );

adc_RequestState_t          Adc_Set_SamplingTime            ( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelSampling_t samplingTime );
adc_RequestState_t          Adc_Get_SamplingTime            ( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelSampling_t * const samplingTime );

adc_RequestState_t          Adc_Set_ChannelInput            ( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput );
adc_RequestState_t          Adc_Get_ChannelInput            ( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelInput_t * const channelInput );

/* -------------------------------------------------------------------------- */
/* --------------------- Analog Watch-Dog configuration --------------------- */
/* -------------------------------------------------------------------------- */

adc_RequestState_t          Adc_AwdInit                     ( adc_PeriphId_t periphId, adc_AwdConfig_t * const awdConfig );

adc_RequestState_t          Adc_Set_AwdThresholds           ( adc_PeriphId_t periphId, adc_AwdId_t awdId, adc_AwdThreshold_t lowThreshold, adc_AwdThreshold_t highThreshold);
adc_RequestState_t          Adc_Get_AwdThresholds           ( adc_PeriphId_t periphId, adc_AwdId_t awdId, adc_AwdThreshold_t * const lowThreshold, adc_AwdThreshold_t * const highThreshold);

adc_RequestState_t          Adc_Set_AwdFilter               ( adc_PeriphId_t periphId, adc_AwdId_t awdId, adc_AwdFilter_t awdFilter );
adc_RequestState_t          Adc_Get_AwdFilter               ( adc_PeriphId_t periphId, adc_AwdId_t awdId, adc_AwdFilter_t * const awdFilter );

#ifdef __cplusplus
}
#endif

#endif /* ADC_ADC_PORT_H */

