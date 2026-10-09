/**
 * \author Mr.Nobody
 * \file Adc_Isr.c
 * \ingroup Adc
 * \brief Adc module ADC interrupt data transfer handler
 *
 * - ADC_TRANSFER_MODE_ISR: regular results are read in EOC interrupt, overrun (OVR) and
 *   injected end of sequence (JEOS) are reported from the same interrupt.
 * - ADC_TRANSFER_MODE_DMA: only OVR and JEOS interrupts are used (data are moved by DMA).
 *
 * Buffer handling and user callbacks are implemented in Adc.c (Adc_Set_Xfer* services).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Adc_Isr.h"                        /* Self include                   */
#include "Adc.h"                            /* Module private interface       */
#include "Nvic_Port.h"                      /* NVIC Mcal layer include        */
/* ============================== TYPEDEFS ================================== */

/** ADC interrupt connection to NVIC */
typedef struct
{
    adc_FunctionState_t  Available; /**< ADC interrupt is available in NVIC module                   */
    nvic_PeriphIrqList_t IrqId;     /**< NVIC interrupt identification                                */
    nvic_IsrCallback_t   Handler;   /**< Interrupt handler of the peripheral                          */
}   adc_IsrIrqConfig_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static void               Adc_Isr_Adc1Handler ( void );
static adc_RequestState_t Adc_Isr_Handler     ( adc_PeriphId_t periphId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** ADC interrupts used by the handler (all of them are disabled on deinitialization) */
#define ADC_ISR_IT_ALL               ( LL_ADC_IT_EOC | LL_ADC_IT_OVR | LL_ADC_IT_JEOS )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** adc_PeriphId_t -> ADC interrupt. STM32U5 ADC1 and ADC2 share one interrupt (ADC1_2), the common
 *  handler services both peripherals (only enabled interrupt sources are processed). */
static const adc_IsrIrqConfig_t adc_IsrIrqConfig[ ] =
{
    { .Available = ADC_FUNCTION_ACTIVE,   .IrqId = NVIC_PERIPH_IRQ_ADC1, .Handler = Adc_Isr_Adc1Handler },
#if defined (ADC2)
    { .Available = ADC_FUNCTION_ACTIVE,   .IrqId = NVIC_PERIPH_IRQ_ADC1, .Handler = Adc_Isr_Adc1Handler },
#endif /* ADC2 */
};

_Static_assert( ADC_PERIPH_CNT == ( sizeof(adc_IsrIrqConfig) / sizeof(adc_IsrIrqConfig_t) ), "Adc: adc_IsrIrqConfig has incorrect size." );

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks ISR related part of the data transfer configuration
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param dataConfig [in]: Pointer to data transfer configuration. Must not be NULL.
 *
 * \return Returns \ref ADC_REQUEST_OK if the ADC interrupt is available for the peripheral.
 *         Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Check_Config( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId   ) &&
        ( ADC_NULL_PTR  != dataConfig )    )
    {
        if( ADC_FUNCTION_ACTIVE == adc_IsrIrqConfig[ periphId ].Available )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* ADC interrupt of the peripheral is not available */
            retState = ADC_REQUEST_ERROR;
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Connects the ADC interrupt to NVIC (handler, priority, enable) and enables injected
 *        end of sequence interrupt if InjCompleteCallback is configured
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
    else
    {
        /* Previous step failed, error state is kept */
    }

    if( ADC_REQUEST_OK == retState )
    {
        const adc_IsrIrqConfig_t * const irqConfig = &adc_IsrIrqConfig[ periphId ];

        nvicState = Nvic_Set_PeriphIrq_Handler( irqConfig->IrqId, irqConfig->Handler );

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Prio( irqConfig->IrqId, (nvic_IrqPrio_t)xferCtx->Config.IrqPriority );
        }
        else
        {
            /* Handler registration failed, priority is not configured */
        }

        if( NVIC_REQUEST_OK == nvicState )
        {
            nvicState = Nvic_Set_PeriphIrq_Active( irqConfig->IrqId );
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
    if( ( ADC_REQUEST_OK == retState ) &&
        ( ADC_NULL_PTR != xferCtx->Config.InjCompleteCallback )    )
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
 * \brief Disables all ADC interrupts used by the handler and the ADC interrupt in NVIC
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

    if( ( ADC_REQUEST_OK == retState ) &&
        ( ADC_FUNCTION_ACTIVE == adc_IsrIrqConfig[ periphId ].Available )    )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( adc_IsrIrqConfig[ periphId ].IrqId );

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
        /* Interrupts could not be disabled or NVIC interrupt is not available */
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

    retState = Adc_Isr_Set_IrqActive( periphId, LL_ADC_IT_EOC | LL_ADC_IT_OVR );

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

    retState = Adc_Isr_Set_IrqInactive( periphId, LL_ADC_IT_EOC | LL_ADC_IT_OVR );

    return ( retState );
}


/**
 * \brief Clears pending flags and enables ADC interrupts
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param llItMask [in]: Combination of LL_ADC_IT_EOC / LL_ADC_IT_OVR / LL_ADC_IT_JEOS
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Set_IrqActive( adc_PeriphId_t periphId, uint32_t llItMask )
{
    adc_RequestState_t retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *      periphReg = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ( ADC_REQUEST_OK == retState ) &&
        ( llItMask == ( llItMask & ADC_ISR_IT_ALL ) )    )
    {
        /* Stale events are cleared, interrupt flags use the same bit positions as enable bits */
        WRITE_REG( periphReg->ISR, llItMask );
        SET_BIT( periphReg->IER, llItMask );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t ierReg = READ_BIT( periphReg->IER, llItMask );

            if( llItMask == ierReg )
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
 * \param llItMask [in]: Combination of LL_ADC_IT_EOC / LL_ADC_IT_OVR / LL_ADC_IT_JEOS
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Isr_Set_IrqInactive( adc_PeriphId_t periphId, uint32_t llItMask )
{
    adc_RequestState_t retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *      periphReg = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ( ADC_REQUEST_OK == retState ) &&
        ( llItMask == ( llItMask & ADC_ISR_IT_ALL ) )    )
    {
        CLEAR_BIT( periphReg->IER, llItMask );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t ierReg = READ_BIT( periphReg->IER, llItMask );

            if( 0u == ierReg )
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
 * \brief Common ADC interrupt handler - processes enabled and pending EOC, OVR and JEOS events
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Isr_Handler( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *      periphReg = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ADC_REQUEST_OK == retState )
    {
        const uint32_t pendingIt = READ_REG( periphReg->ISR ) & READ_REG( periphReg->IER );

        /* Regular conversion result (reading of DR clears EOC) */
        if( 0u != ( pendingIt & LL_ADC_IT_EOC ) )
        {
            const adc_Data_t data = (adc_Data_t)LL_ADC_REG_ReadConversionData32( periphReg );

            retState = Adc_Set_XferData( periphId, data );
        }
        else
        {
            /* No regular conversion result pending */
        }

        /* Regular group overrun */
        if( 0u != ( pendingIt & LL_ADC_IT_OVR ) )
        {
            WRITE_REG( periphReg->ISR, LL_ADC_FLAG_OVR );
            retState = Adc_Set_XferError( periphId, ADC_ERROR_OVERRUN );
        }
        else
        {
            /* No overrun pending */
        }

        /* Injected end of sequence (unitary JEOC flags are cleared as well) */
        if( 0u != ( pendingIt & LL_ADC_IT_JEOS ) )
        {
            WRITE_REG( periphReg->ISR, LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JEOC );
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

/** \brief ADC1 interrupt handler (registered in NVIC by Adc_Isr_Init()) */
static void Adc_Isr_Adc1Handler( void )
{
    (void)Adc_Isr_Handler( ADC_PERIPH_1 );
#if defined (ADC2)
    (void)Adc_Isr_Handler( ADC_PERIPH_2 );
#endif /* ADC2 */
}


/* ================================ TASKS =================================== */
