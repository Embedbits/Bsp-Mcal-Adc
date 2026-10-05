/**
 * \author Mr.Nobody
 * \file Adc_Isr.c
 * \ingroup Adc
 * \brief Adc module ADC interrupt data transfer handler
 *
 * - ADC_TRANSFER_MODE_ISR: regular results are read in EOC interrupt, overrun (OVR) and
 *   injected end of sequence (JEOC) are reported from the same interrupt.
 * - ADC_TRANSFER_MODE_DMA: only OVR and JEOC interrupts are used (data are moved by DMA).
 *   Overrun blocks the ADC DMA requests on STM32F4 - the regular conversion and the data
 *   transfer are stopped (Adc_Set_RegStop()), Adc_Set_RegStart() restarts them.
 *
 * STM32F4 ADC1 / ADC2 / ADC3 share one interrupt - the handler services every peripheral
 * using the interrupt and the interrupt is disabled in NVIC when the last user is released.
 * Interrupt priority is common for all ADC peripherals (the last configured one is applied).
 *
 * Buffer handling and user callbacks are implemented in Adc.c (Adc_Set_Xfer* services).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Adc_Isr.h"                        /* Self include                   */
#include "Adc.h"                            /* Module private interface       */
#include "Adc_Port.h"                       /* Regular group stop on overrun  */
#include "Nvic_Port.h"                      /* NVIC Mcal layer include        */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

static void               Adc_Isr_IrqHandler      ( void );
static adc_RequestState_t Adc_Isr_Handler         ( adc_PeriphId_t periphId );
static uint32_t           Adc_Isr_Get_FlagMask    ( uint32_t llItMask );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** ADC interrupts used by the handler (all of them are disabled on deinitialization) */
#define ADC_ISR_IT_ALL               ( LL_ADC_IT_EOCS | LL_ADC_IT_OVR | LL_ADC_IT_JEOS )

/** Common interrupt of all ADC peripherals */
#define ADC_ISR_IRQ_ID               ( NVIC_PERIPH_IRQ_ADC )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** Peripherals using the common ADC interrupt (handler initialized by Adc_Isr_Init()) */
static volatile adc_FunctionState_t adc_IsrUsers[ ADC_PERIPH_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks ISR related part of the data transfer configuration
 *
 * \note  ADC interrupt is available for every STM32F4 ADC peripheral.
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param dataConfig [in]: Pointer to data transfer configuration. Must not be NULL.
 *
 * \return Returns \ref ADC_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Check_Config( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId   ) &&
        ( ADC_NULL_PTR  != dataConfig )    )
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
 * \brief Connects the common ADC interrupt to NVIC (handler, priority, enable) and enables
 *        injected end of sequence interrupt if InjCompleteCallback is configured
 *
 * \pre   Transfer context of the peripheral contains the data transfer configuration.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Init( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    nvic_RequestState_t nvicState = NVIC_REQUEST_ERROR;
    adc_XferContext_t * xferCtx   = ADC_NULL_PTR;

    retState = Adc_Get_XferContext( periphId, &xferCtx );

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Isr_Check_Config( periphId, &xferCtx->Config );
    }

    if( ADC_REQUEST_OK == retState )
    {
        /* Peripheral is registered before the interrupt is enabled */
        adc_IsrUsers[ periphId ] = ADC_FUNCTION_ACTIVE;

        nvicState = Nvic_Set_PeriphIrq_Handler( ADC_ISR_IRQ_ID, Adc_Isr_IrqHandler );

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Prio( ADC_ISR_IRQ_ID, (nvic_IrqPrio_t)xferCtx->Config.IrqPriority );
        }
        else
        {
            /* Handler registration failed, priority is not configured */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Active( ADC_ISR_IRQ_ID );
        }
        else
        {
            /* Priority configuration failed, interrupt is not enabled */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            retState = ADC_REQUEST_ERROR;
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    /* Injected end of sequence is reported independently of regular data transfer */
    if( ( ADC_REQUEST_OK == retState ) && ( ADC_NULL_PTR != xferCtx->Config.InjCompleteCallback ) )
    {
        retState = Adc_Isr_Set_IrqActive( periphId, LL_ADC_IT_JEOS );
    }
    else
    {
        /* Injected end of sequence is not reported */
    }

    return ( retState );
}


/**
 * \brief Disables all ADC interrupts of the peripheral used by the handler, the common ADC
 *        interrupt is disabled in NVIC if no other peripheral uses it
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Deinit( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    retState = Adc_Isr_Set_IrqInactive( periphId, ADC_ISR_IT_ALL );

    if( ADC_REQUEST_OK == retState )
    {
        adc_FunctionState_t irqUsed = ADC_FUNCTION_INACTIVE;

        adc_IsrUsers[ periphId ] = ADC_FUNCTION_INACTIVE;

        for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx ++ )
        {
            if( ADC_FUNCTION_ACTIVE == adc_IsrUsers[ periphIdx ] )
            {
                irqUsed = ADC_FUNCTION_ACTIVE;
            }
            else
            {
                /* Peripheral does not use the interrupt */
            }
        }

        if( ADC_FUNCTION_INACTIVE == irqUsed )
        {
            const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( ADC_ISR_IRQ_ID );

            if( NVIC_REQUEST_OK == nvicState )
            {
                retState = ADC_REQUEST_OK;
            }
            else
            {
                retState = ADC_REQUEST_ERROR;
            }
        }
        else
        {
            /* Interrupt is still used by another ADC peripheral */
        }
    }
    else
    {
        /* Interrupts could not be disabled */
    }

    return ( retState );
}


/**
 * \brief Starts regular data transfer in ISR mode (EOC and OVR interrupts)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Start( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    retState = Adc_Isr_Set_IrqActive( periphId, LL_ADC_IT_EOCS | LL_ADC_IT_OVR );

    return ( retState );
}


/**
 * \brief Stops regular data transfer in ISR mode (EOC and OVR interrupts)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Stop( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    retState = Adc_Isr_Set_IrqInactive( periphId, LL_ADC_IT_EOCS | LL_ADC_IT_OVR );

    return ( retState );
}


/**
 * \brief Clears pending flags and enables ADC interrupts
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param llItMask [in]: Combination of LL_ADC_IT_EOCS / LL_ADC_IT_OVR / LL_ADC_IT_JEOS
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Set_IrqActive( adc_PeriphId_t periphId, uint32_t llItMask )
{
    adc_RequestState_t retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *      periphReg = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ( ADC_REQUEST_OK == retState ) && ( llItMask == ( llItMask & ADC_ISR_IT_ALL ) ) )
    {
        /* Stale events are cleared (SR bits are cleared by writing 0) */
        WRITE_REG( periphReg->SR, ~Adc_Isr_Get_FlagMask( llItMask ) );
        SET_BIT( periphReg->CR1, llItMask );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t itReg = READ_BIT( periphReg->CR1, llItMask );

            if( llItMask == itReg )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* Interrupt enable has not yet been applied, keep return state as error */
                retState = ADC_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables ADC interrupts
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param llItMask [in]: Combination of LL_ADC_IT_EOCS / LL_ADC_IT_OVR / LL_ADC_IT_JEOS
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Set_IrqInactive( adc_PeriphId_t periphId, uint32_t llItMask )
{
    adc_RequestState_t retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *      periphReg = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ( ADC_REQUEST_OK == retState ) && ( llItMask == ( llItMask & ADC_ISR_IT_ALL ) ) )
    {
        CLEAR_BIT( periphReg->CR1, llItMask );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t itReg = READ_BIT( periphReg->CR1, llItMask );

            if( 0u == itReg )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* Interrupt disable has not yet been applied, keep return state as error */
                retState = ADC_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Translates ADC interrupt enable bits (CR1) to the related event flags (SR)
 *
 * \param llItMask [in]: Combination of LL_ADC_IT_EOCS / LL_ADC_IT_OVR / LL_ADC_IT_JEOS
 *
 * \return Combination of ADC_SR_EOC / ADC_SR_OVR / ADC_SR_JEOC
 */
static uint32_t Adc_Isr_Get_FlagMask( uint32_t llItMask )
{
    uint32_t flagMask = 0u;

    if( 0u != ( llItMask & LL_ADC_IT_EOCS ) )
    {
        flagMask |= LL_ADC_FLAG_EOCS;
    }

    if( 0u != ( llItMask & LL_ADC_IT_OVR ) )
    {
        flagMask |= LL_ADC_FLAG_OVR;
    }

    if( 0u != ( llItMask & LL_ADC_IT_JEOS ) )
    {
        flagMask |= LL_ADC_FLAG_JEOS;
    }

    return ( flagMask );
}


/**
 * \brief ADC interrupt handler of one peripheral - processes enabled and pending EOC, OVR and
 *        JEOC events
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Isr_Handler( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *       periphReg = ADC_NULL_PTR;
    adc_XferContext_t * xferCtx   = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Get_XferContext( periphId, &xferCtx );
    }

    if( ADC_REQUEST_OK == retState )
    {
        const uint32_t pendingIt = Adc_Isr_Get_FlagMask( READ_REG( periphReg->CR1 ) ) & READ_REG( periphReg->SR );

        /* Regular conversion result (reading of DR clears EOC) */
        if( 0u != ( pendingIt & LL_ADC_FLAG_EOCS ) )
        {
            const adc_Data_t data = (adc_Data_t)LL_ADC_REG_ReadConversionData32( periphReg );

            retState = Adc_Set_XferData( periphId, data );
        }
        else
        {
            /* No regular conversion result pending */
        }

        /* Regular group overrun */
        if( 0u != ( pendingIt & LL_ADC_FLAG_OVR ) )
        {
            WRITE_REG( periphReg->SR, ~LL_ADC_FLAG_OVR );

            if( ADC_TRANSFER_MODE_DMA == xferCtx->Config.TransferMode )
            {
                /* DMA requests are blocked by the overrun - regular group and DMA are stopped */
                (void)Adc_Set_RegStop( periphId );
            }
            else
            {
                /* Conversions continue, the lost result is only reported */
            }

            retState = Adc_Set_XferError( periphId, ADC_ERROR_OVERRUN );
        }
        else
        {
            /* No overrun pending */
        }

        /* Injected end of sequence */
        if( 0u != ( pendingIt & LL_ADC_FLAG_JEOS ) )
        {
            WRITE_REG( periphReg->SR, ~( LL_ADC_FLAG_JEOS | ADC_SR_JSTRT ) );
            retState = Adc_Set_XferInjDone( periphId );
        }
        else
        {
            /* No injected end of sequence pending */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/** \brief Common ADC1 / ADC2 / ADC3 interrupt handler (registered in NVIC by Adc_Isr_Init()) */
static void Adc_Isr_IrqHandler( void )
{
    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx ++ )
    {
        if( ADC_FUNCTION_ACTIVE == adc_IsrUsers[ periphIdx ] )
        {
            (void)Adc_Isr_Handler( periphIdx );
        }
        else
        {
            /* Peripheral does not use the interrupt */
        }
    }
}

/* ================================ TASKS =================================== */
