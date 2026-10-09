/**
 * \author Mr.Nobody
 * \file Adc_Poll.c
 * \ingroup Adc
 * \brief Adc module polling data transfer handler
 *
 * ADC_TRANSFER_MODE_POLL - no interrupt is used. Adc_Task() calls Adc_Poll_Task() which
 * polls EOC (regular result), OVR (overrun) and JEOS (injected end of sequence) flags and
 * reports the events to Adc.c (Adc_Set_Xfer* services), where buffer handling and user
 * callbacks are implemented.
 *
 * If DataBuffer is NULL, regular results are not collected - the user reads them with
 * Adc_Get_RegData() (manual polling).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Adc_Poll.h"                       /* Self include                   */
#include "Adc.h"                            /* Module private interface       */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks polling related part of the data transfer configuration
 *
 * \note  Polling mode has no mode specific resources - every peripheral is supported.
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param dataConfig [in]: Pointer to data transfer configuration. Must not be NULL.
 *
 * \return Returns \ref ADC_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Poll_Check_Config( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT  > periphId   ) &&
        ( ADC_NULL_PTR   != dataConfig )    )
    {
        retState = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes polling data transfer (no HW resource is needed)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Poll_Init( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState = ADC_REQUEST_ERROR;
    adc_XferContext_t * xferCtx  = ADC_NULL_PTR;

    retState = Adc_Get_XferContext( periphId, &xferCtx );

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Poll_Check_Config( periphId, &xferCtx->Config );
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes polling data transfer (no HW resource is used)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Poll_Deinit( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        retState = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts polling data transfer - stale EOC / OVR events are cleared
 *
 * \note  Event flags are set by HW at any time, the clear is not verified by read-back.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Poll_Start( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *      periphReg = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ADC_REQUEST_OK == retState )
    {
        WRITE_REG( periphReg->ISR, LL_ADC_FLAG_EOC | LL_ADC_FLAG_OVR );
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops polling data transfer (Adc_Task() stops collecting results when the transfer
 *        state in Adc.c is inactive, no HW resource is used)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Poll_Stop( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        retState = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Polls ADC flags of one peripheral and reports the events (called from Adc_Task())
 *
 * - EOC:  only while the regular transfer is running and DataBuffer is configured
 * - OVR:  always (reported through ErrorCallback)
 * - JEOS: only if InjCompleteCallback is configured
 *
 * \note  Only one regular result is read per call - Adc_Task() has to be called at least once
 *        per regular conversion, otherwise an overrun is reported.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Poll_Task( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *       periphReg = ADC_NULL_PTR;
    adc_XferContext_t * xferCtx   = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Get_XferContext( periphId, &xferCtx );
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    if( ADC_REQUEST_OK == retState )
    {
        const uint32_t isrReg = READ_REG( periphReg->ISR );

        /* Regular conversion result (reading of DR clears EOC) */
        if( ( ADC_FUNCTION_ACTIVE == xferCtx->XferState           ) &&
            ( ADC_NULL_PTR        != xferCtx->Config.DataBuffer   ) &&
            ( 0u                  != ( isrReg & LL_ADC_FLAG_EOC ) )    )
        {
            const adc_Data_t data = (adc_Data_t)LL_ADC_REG_ReadConversionData32( periphReg );

            retState = Adc_Set_XferData( periphId, data );
        }
        else
        {
            /* No regular result to be collected */
        }

        /* Regular group overrun */
        if( 0u != ( isrReg & LL_ADC_FLAG_OVR ) )
        {
            WRITE_REG( periphReg->ISR, LL_ADC_FLAG_OVR );
            retState = Adc_Set_XferError( periphId, ADC_ERROR_OVERRUN );
        }
        else
        {
            /* No overrun */
        }

        /* Injected end of sequence (unitary JEOC flags are cleared as well) */
        if( ( ADC_NULL_PTR != xferCtx->Config.InjCompleteCallback ) &&
            ( 0u           != ( isrReg & LL_ADC_FLAG_JEOS )       )    )
        {
            WRITE_REG( periphReg->ISR, LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JEOC );
            retState = Adc_Set_XferInjDone( periphId );
        }
        else
        {
            /* No injected end of sequence to be reported */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
