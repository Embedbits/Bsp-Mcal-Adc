/**
 * \author Mr.Nobody
 * \file Adc.h
 * \ingroup Adc
 * \brief Adc module private interface shared between Adc.c and data transfer handlers
 *
 * This file is private to the Adc library (not exported through public headers). It
 * connects the module root (Adc.c) with data transfer mode handlers (Adc_Dma.c, Adc_Isr.c,
 * Adc_Poll.c). Buffer handling and user callbacks are implemented once in Adc.c, the mode
 * handlers only move the data and report events.
 *
 */

#ifndef ADC_ADC_H
#define ADC_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Adc_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief Type representing iteration count of busy-wait loops (register read-back, HW flags) */
typedef uint32_t adc_TimeoutCnt_t;


/** \brief Type representing frequency in Hz */
typedef uint32_t adc_FreqHz_t;


/** \brief Type representing time in microseconds */
typedef uint32_t adc_TimeUs_t;


/** \brief Type representing time in nanoseconds */
typedef uint32_t adc_TimeNs_t;


/** \brief Runtime context of regular group data transfer (one per ADC peripheral) */
typedef struct
{
    adc_DataConfig_t             Config;    /**< Copy of user data transfer configuration              */
    volatile adc_BufferSize_t    BufferIdx; /**< Index of the next buffer item to be written            */
    volatile adc_FunctionState_t XferState; /**< Buffer is being filled (between start and full / stop) */
    adc_FunctionState_t          InitState; /**< Data transfer handler is initialized                  */
}   adc_XferContext_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Busy-wait iteration budget used while polling a HW ready/state-machine flag or register
 *  read-back (common for all Adc module files). Same order of magnitude and role as
 *  RCC_OSC_TIMEOUT_RAW / TIM_TIMEOUT_RAW used by the sibling Rcc/Tim modules. */
#define ADC_TIMEOUT_RAW              ( (adc_TimeoutCnt_t)0x84FCBu )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* Implemented in Adc.c - common services for data transfer handlers */
adc_RequestState_t Adc_Get_PeriphReg   ( adc_PeriphId_t periphId, ADC_TypeDef ** const periphReg );
adc_RequestState_t Adc_Get_XferContext ( adc_PeriphId_t periphId, adc_XferContext_t ** const xferContext );

adc_RequestState_t Adc_Set_XferData    ( adc_PeriphId_t periphId, adc_Data_t data );
adc_RequestState_t Adc_Set_XferHalf    ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Set_XferDone    ( adc_PeriphId_t periphId );
adc_RequestState_t Adc_Set_XferError   ( adc_PeriphId_t periphId, adc_ErrorId_t errorId );
adc_RequestState_t Adc_Set_XferInjDone ( adc_PeriphId_t periphId );

#ifdef __cplusplus
}
#endif

#endif /* ADC_ADC_H */
