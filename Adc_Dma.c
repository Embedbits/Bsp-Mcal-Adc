/**
 * \author Mr.Nobody
 * \file Adc_Dma.c
 * \ingroup Adc
 * \brief Adc module DMA data transfer handler
 *
 * ADC_TRANSFER_MODE_DMA - regular results are moved from ADC_DR to the user buffer by a DMA
 * channel (peripheral to memory, 16-bit, BufferSize items).
 * - The ADC request (DMA_REQ_ADCx) is routed by DMAMUX1 to the DMA1 / DMA2 channel selected by
 *   DmaPeriphId / DmaChannelId of the data transfer configuration.
 * - ADC_BUFFER_MODE_CIRCULAR: DMA channel in circular mode, ADC DMA requests are kept after the
 *   last transfer (DMACFG = 1). ADC_BUFFER_MODE_ONE_SHOT: normal mode, DMACFG = 0.
 * - Half transfer / transfer complete / DMA transfer errors are reported from DMA interrupt.
 *   The DMA channel stays enabled after a normal mode transfer - it is disabled by the stop of
 *   the transfer (Adc_Dma_Stop()) and reprogrammed by every start.
 * - Overrun (OVR) and injected end of sequence (JEOS) are reported from ADC interrupt
 *   (see Adc_Isr.c). DMA channel interrupt priority is handled by Dma module, IrqPriority is
 *   applied to the ADC interrupt.
 *
 * Buffer handling and user callbacks are implemented in Adc.c (Adc_Set_Xfer* services).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Adc_Dma.h"                        /* Self include                   */
#include "Adc_Isr.h"                        /* ADC interrupt handler          */
#include "Adc.h"                            /* Module private interface       */
#include "Dma_Port.h"                       /* DMA Mcal layer include         */
/* ============================== TYPEDEFS ================================== */

/** ADC DMA request (DMAMUX) and DMA handlers of the peripheral */
typedef struct
{
    dma_PeriphReqId_t Request;       /**< DMAMUX request of the ADC                   */
    dma_IsrCallback   XferCpltIsr;   /**< Transfer complete handler of the peripheral */
    dma_IsrCallback   HalfXferIsr;   /**< Half transfer handler of the peripheral     */
    dma_IsrCallback   ErrorIsr;      /**< Transfer error handler of the peripheral    */
}   adc_DmaReqConfig_t;


/** DMA channel ownership of a peripheral */
typedef struct
{
    adc_FunctionState_t Initialized; /**< DMA channel was initialized for the peripheral */
    adc_DmaPeriphId_t   PeriphId;    /**< Initialized DMA peripheral                     */
    adc_DmaChannelId_t  ChannelId;   /**< Initialized DMA channel                        */
}   adc_DmaChannelState_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static adc_RequestState_t Adc_Dma_Check_Channel ( adc_DmaPeriphId_t dmaPeriphId, adc_DmaChannelId_t dmaChannelId );
static adc_RequestState_t Adc_Dma_Set_Request   ( adc_PeriphId_t periphId, uint32_t llDmaMode );

static void               Adc_Dma_Adc1XferCplt  ( void );
static void               Adc_Dma_Adc1HalfXfer  ( void );
static void               Adc_Dma_Adc1Error     ( void );
#if defined (ADC2)
static void               Adc_Dma_Adc2XferCplt  ( void );
static void               Adc_Dma_Adc2HalfXfer  ( void );
static void               Adc_Dma_Adc2Error     ( void );
#endif /* ADC2 */
#if defined (ADC3)
static void               Adc_Dma_Adc3XferCplt  ( void );
static void               Adc_Dma_Adc3HalfXfer  ( void );
static void               Adc_Dma_Adc3Error     ( void );
#endif /* ADC3 */
#if defined (ADC4)
static void               Adc_Dma_Adc4XferCplt  ( void );
static void               Adc_Dma_Adc4HalfXfer  ( void );
static void               Adc_Dma_Adc4Error     ( void );
#endif /* ADC4 */
#if defined (ADC5)
static void               Adc_Dma_Adc5XferCplt  ( void );
static void               Adc_Dma_Adc5HalfXfer  ( void );
static void               Adc_Dma_Adc5Error     ( void );
#endif /* ADC5 */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** adc_PeriphId_t -> DMAMUX request and DMA handlers */
static const adc_DmaReqConfig_t adc_DmaReqConfig[ ] =
{
    { .Request     = DMA_REQ_ADC1,
      .XferCpltIsr = Adc_Dma_Adc1XferCplt, .HalfXferIsr = Adc_Dma_Adc1HalfXfer, .ErrorIsr = Adc_Dma_Adc1Error },
#if defined (ADC2)
    { .Request     = DMA_REQ_ADC2,
      .XferCpltIsr = Adc_Dma_Adc2XferCplt, .HalfXferIsr = Adc_Dma_Adc2HalfXfer, .ErrorIsr = Adc_Dma_Adc2Error },
#endif /* ADC2 */
#if defined (ADC3)
    { .Request     = DMA_REQ_ADC3,
      .XferCpltIsr = Adc_Dma_Adc3XferCplt, .HalfXferIsr = Adc_Dma_Adc3HalfXfer, .ErrorIsr = Adc_Dma_Adc3Error },
#endif /* ADC3 */
#if defined (ADC4)
    { .Request     = DMA_REQ_ADC4,
      .XferCpltIsr = Adc_Dma_Adc4XferCplt, .HalfXferIsr = Adc_Dma_Adc4HalfXfer, .ErrorIsr = Adc_Dma_Adc4Error },
#endif /* ADC4 */
#if defined (ADC5)
    { .Request     = DMA_REQ_ADC5,
      .XferCpltIsr = Adc_Dma_Adc5XferCplt, .HalfXferIsr = Adc_Dma_Adc5HalfXfer, .ErrorIsr = Adc_Dma_Adc5Error },
#endif /* ADC5 */
};

_Static_assert( ADC_PERIPH_CNT == ( sizeof(adc_DmaReqConfig) / sizeof(adc_DmaReqConfig_t) ), "Adc: adc_DmaReqConfig has incorrect size." );

/* Every buffer size fits the 16 bit DMA data counter (items of peripheral size) */
_Static_assert( UINT16_MAX <= DMA_DATA_COUNT_MAX, "Adc: adc_BufferSize_t exceeds DMA data counter." );


/** DMA channel ownership per peripheral */
static adc_DmaChannelState_t adc_DmaChannelState[ ADC_PERIPH_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the data transfer configuration
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param dataConfig [in]: Pointer to data transfer configuration. Must not be NULL.
 *
 * \return Returns \ref ADC_REQUEST_OK if the ADC interrupt is available, DMA channel exists,
 *         DMA priority is valid, DataBuffer is set and BufferSize is not 0. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Dma_Check_Config( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT  > periphId   ) &&
        ( ADC_NULL_PTR   != dataConfig )    )
    {
        const adc_RequestState_t isrState     = Adc_Isr_Check_Config( periphId, dataConfig );
        const adc_RequestState_t channelState = Adc_Dma_Check_Channel( dataConfig->DmaPeriphId, dataConfig->DmaChannelId );

        if( ( ADC_REQUEST_OK             == isrState                          ) &&
            ( ADC_REQUEST_OK             == channelState                      ) &&
            ( ADC_NULL_PTR               != dataConfig->DataBuffer            ) &&
            ( 0u                          < dataConfig->BufferSize            ) &&
            ( (uint32_t)DMA_PRIORITY_CNT  > (uint32_t)dataConfig->DmaPriority )    )
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

    return ( retState );
}


/**
 * \brief Initializes DMA data transfer: ADC DMA requests (DMAEN, DMACFG), DMA channel (DMAMUX
 *        request, direction, mode, sizes, callbacks, interrupts) and ADC interrupt for OVR / JEOS
 *
 * \pre   Transfer context of the peripheral contains the data transfer configuration.
 *        No regular conversion is ongoing (called from Adc_PeriphInit() / Adc_Set_DataConfig()).
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Dma_Init( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *       periphReg = ADC_NULL_PTR;
    adc_XferContext_t * xferCtx   = ADC_NULL_PTR;
    uint32_t            llDmaMode = LL_ADC_REG_DMA_TRANSFER_LIMITED;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Get_XferContext( periphId, &xferCtx );
    }

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Dma_Check_Config( periphId, &xferCtx->Config );
    }

    /* --- ADC DMA requests: circular buffer keeps requesting after the last transfer --- */
    if( ADC_REQUEST_OK == retState )
    {
        if( ADC_BUFFER_MODE_CIRCULAR == xferCtx->Config.BufferMode )
        {
            llDmaMode = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
        }
        else
        {
            llDmaMode = LL_ADC_REG_DMA_TRANSFER_LIMITED;
        }

        retState = Adc_Dma_Set_Request( periphId, llDmaMode );
    }

    /* --- DMA channel --- */
    if( ADC_REQUEST_OK == retState )
    {
        adc_DmaChannelState_t * const    chState   = &adc_DmaChannelState[ periphId ];
        const adc_DmaReqConfig_t * const reqConfig = &adc_DmaReqConfig[ periphId ];
        const dma_PeriphId_t             dmaPeriph = (dma_PeriphId_t)xferCtx->Config.DmaPeriphId;
        const dma_ChannelId_t            dmaChannel = (dma_ChannelId_t)xferCtx->Config.DmaChannelId;
        dma_ConfigStruct_t               dmaConfig;
        dma_RequestState_t               dmaState  = Dma_Get_DefaultConfig( &dmaConfig );

        dmaConfig.DmaPeriphId              = dmaPeriph;
        dmaConfig.DmaChannel               = dmaChannel;
        dmaConfig.PeripheralReqId          = reqConfig->Request;
        dmaConfig.Direction                = DMA_DIR_PERIPH_TO_MEMORY;
        dmaConfig.PeriphAddress            = (dma_PeriphAddr_t)LL_ADC_DMA_GetRegAddr( periphReg, LL_ADC_DMA_REG_REGULAR_DATA );
        dmaConfig.MemoryAddress            = (dma_MemoryAddr_t)xferCtx->Config.DataBuffer;
        dmaConfig.PeriphAddrIncrement      = DMA_PERIPH_ADDR_STATIC;
        dmaConfig.MemoryAddrIncrement      = DMA_MEMORY_ADDR_INCREMENT;
        dmaConfig.PeriphTransferSize       = DMA_TRANSFER_SIZE_16BIT;
        dmaConfig.MemoryTransferSize       = DMA_TRANSFER_SIZE_16BIT;
        dmaConfig.DataCount                = (dma_DataCount_t)xferCtx->Config.BufferSize;
        dmaConfig.Priority                 = (dma_Priority_t)xferCtx->Config.DmaPriority;
        dmaConfig.TransferCompleteCallback = reqConfig->XferCpltIsr;
        dmaConfig.HalfTransferCallback     = reqConfig->HalfXferIsr;
        dmaConfig.TransferErrorCallback    = reqConfig->ErrorIsr;

        if( ADC_BUFFER_MODE_CIRCULAR == xferCtx->Config.BufferMode )
        {
            dmaConfig.TransferMode = DMA_TRANSFER_MODE_CIRCULAR;
        }
        else
        {
            dmaConfig.TransferMode = DMA_TRANSFER_MODE_NORMAL;
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Init( &dmaConfig );
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            chState->Initialized = ADC_FUNCTION_ACTIVE;
            chState->PeriphId    = xferCtx->Config.DmaPeriphId;
            chState->ChannelId   = xferCtx->Config.DmaChannelId;

            dmaState = Dma_Set_TransferCompleteIrqActive( dmaPeriph, dmaChannel );
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferErrorIrqActive( dmaPeriph, dmaChannel );
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            if( ADC_NULL_PTR != xferCtx->Config.HalfTransferCallback )
            {
                dmaState = Dma_Set_HalfTransferIrqActive( dmaPeriph, dmaChannel );
            }
            else
            {
                dmaState = Dma_Set_HalfTransferIrqInactive( dmaPeriph, dmaChannel );
            }
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_InterruptActive( dmaPeriph, dmaChannel );
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            retState = ADC_REQUEST_ERROR;
        }
    }

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Isr_Init( periphId );
    }

    return ( retState );
}


/**
 * \brief Deinitializes DMA data transfer: DMA channel and its interrupts disabled, ADC DMA requests
 *        disabled, ADC interrupt disabled
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Dma_Deinit( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        adc_DmaChannelState_t * const chState = &adc_DmaChannelState[ periphId ];

        retState = ADC_REQUEST_OK;

        if( ADC_FUNCTION_ACTIVE == chState->Initialized )
        {
            const dma_PeriphId_t  dmaPeriph = (dma_PeriphId_t)chState->PeriphId;
            const dma_ChannelId_t dmaChannel = (dma_ChannelId_t)chState->ChannelId;
            dma_RequestState_t    dmaState  = Dma_Set_TransferInactive( dmaPeriph, dmaChannel );

            if( DMA_REQUEST_OK == dmaState )
            {
                dmaState = Dma_Set_TransferCompleteIrqInactive( dmaPeriph, dmaChannel );
            }

            if( DMA_REQUEST_OK == dmaState )
            {
                dmaState = Dma_Set_HalfTransferIrqInactive( dmaPeriph, dmaChannel );
            }

            if( DMA_REQUEST_OK == dmaState )
            {
                dmaState = Dma_Set_TransferErrorIrqInactive( dmaPeriph, dmaChannel );
            }

            if( DMA_REQUEST_OK == dmaState )
            {
                dmaState = Dma_Set_InterruptInactive( dmaPeriph, dmaChannel );
            }

            if( DMA_REQUEST_OK == dmaState )
            {
                chState->Initialized = ADC_FUNCTION_INACTIVE;
            }
            else
            {
                retState = ADC_REQUEST_ERROR;
            }
        }
        else
        {
            /* DMA channel was not initialized */
        }

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Dma_Set_Request( periphId, LL_ADC_REG_DMA_TRANSFER_NONE );
        }

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Isr_Deinit( periphId );
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts DMA data transfer - ADC DMA requests are re-enabled (DMAEN toggled, required
 *        after the last transfer of a one shot buffer or after an overrun), the DMA channel is armed
 *        for the whole buffer and the overrun interrupt is enabled
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Dma_Start( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *       periphReg = ADC_NULL_PTR;
    adc_XferContext_t * xferCtx   = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Get_XferContext( periphId, &xferCtx );
    }

    if( ( ADC_REQUEST_OK      == retState                                    ) &&
        ( ADC_FUNCTION_ACTIVE == adc_DmaChannelState[ periphId ].Initialized )    )
    {
        const uint32_t        llDmaMode = LL_ADC_REG_GetDMATransfer( periphReg );
        const dma_PeriphId_t  dmaPeriph = (dma_PeriphId_t)adc_DmaChannelState[ periphId ].PeriphId;
        const dma_ChannelId_t dmaChannel = (dma_ChannelId_t)adc_DmaChannelState[ periphId ].ChannelId;
        dma_RequestState_t    dmaState  = DMA_REQUEST_ERROR;

        /* Overrun interrupt is enabled first - stale OVR flag is cleared, so the ADC accepts triggers */
        retState = Adc_Isr_Set_IrqActive( periphId, LL_ADC_IT_OVR );

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Dma_Set_Request( periphId, LL_ADC_REG_DMA_TRANSFER_NONE );
        }

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Dma_Set_Request( periphId, llDmaMode );
        }

        if( ADC_REQUEST_OK == retState )
        {
            dmaState = Dma_Set_TransferInactive( dmaPeriph, dmaChannel );

            if( DMA_REQUEST_OK == dmaState )
            {
                dmaState = Dma_Set_MemoryAddr( dmaPeriph, dmaChannel, (dma_MemoryAddr_t)xferCtx->Config.DataBuffer );
            }

            if( DMA_REQUEST_OK == dmaState )
            {
                dmaState = Dma_Set_DataCount( dmaPeriph, dmaChannel, (dma_DataCount_t)xferCtx->Config.BufferSize );
            }

            if( DMA_REQUEST_OK == dmaState )
            {
                dmaState = Dma_Set_TransferActive( dmaPeriph, dmaChannel );
            }

            if( DMA_REQUEST_OK == dmaState )
            {
                retState = ADC_REQUEST_OK;
            }
            else
            {
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
 * \brief Stops DMA data transfer - DMA channel and overrun interrupt are disabled
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Dma_Stop( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT       > periphId                                    ) &&
        ( ADC_FUNCTION_ACTIVE == adc_DmaChannelState[ periphId ].Initialized )    )
    {
        const dma_RequestState_t dmaState = Dma_Set_TransferInactive( (dma_PeriphId_t)adc_DmaChannelState[ periphId ].PeriphId,
                                                                      (dma_ChannelId_t)adc_DmaChannelState[ periphId ].ChannelId );

        if( DMA_REQUEST_OK == dmaState )
        {
            retState = Adc_Isr_Set_IrqInactive( periphId, LL_ADC_IT_OVR );
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

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks that the DMA channel exists (any DMA1 / DMA2 channel can serve the ADC request
 *        through DMAMUX1)
 *
 * \param dmaPeriphId  [in]: DMA peripheral, value from \ref adc_DmaPeriphId_t
 * \param dmaChannelId [in]: DMA channel, value from \ref adc_DmaChannelId_t
 *
 * \return Returns \ref ADC_REQUEST_OK if the DMA channel exists. Otherwise returns
 *         \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Dma_Check_Channel( adc_DmaPeriphId_t dmaPeriphId, adc_DmaChannelId_t dmaChannelId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_DMA_PERIPH_CNT  > dmaPeriphId  ) &&
        ( ADC_DMA_CHANNEL_CNT > dmaChannelId )    )
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
 * \brief Configures ADC DMA requests (ADC_CFGR DMAEN / DMACFG)
 *
 * \param periphId  [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param llDmaMode [in]: LL_ADC_REG_DMA_TRANSFER_NONE / _LIMITED / _UNLIMITED
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Dma_Set_Request( adc_PeriphId_t periphId, uint32_t llDmaMode )
{
    adc_RequestState_t retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *      periphReg = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ADC_REQUEST_OK == retState )
    {
        LL_ADC_REG_SetDMATransfer( periphReg, llDmaMode );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_ADC_REG_GetDMATransfer( periphReg );

            if( llDmaMode == regValue )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* DMA request mode has not yet been applied, keep return state as error */
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

/* =========================== INTERRUPT HANDLERS =========================== */

/** \brief ADC1 DMA transfer complete handler (registered in DMA module) */
static void Adc_Dma_Adc1XferCplt( void )
{
    (void)Adc_Set_XferDone( ADC_PERIPH_1 );
}


/** \brief ADC1 DMA half transfer handler (registered in DMA module) */
static void Adc_Dma_Adc1HalfXfer( void )
{
    (void)Adc_Set_XferHalf( ADC_PERIPH_1 );
}


/** \brief ADC1 DMA transfer error handler (registered in DMA module) */
static void Adc_Dma_Adc1Error( void )
{
    (void)Adc_Set_XferError( ADC_PERIPH_1, ADC_ERROR_DMA_TRANSFER );
}

#if defined (ADC2)
/** \brief ADC2 DMA transfer complete handler (registered in DMA module) */
static void Adc_Dma_Adc2XferCplt( void )
{
    (void)Adc_Set_XferDone( ADC_PERIPH_2 );
}


/** \brief ADC2 DMA half transfer handler (registered in DMA module) */
static void Adc_Dma_Adc2HalfXfer( void )
{
    (void)Adc_Set_XferHalf( ADC_PERIPH_2 );
}


/** \brief ADC2 DMA transfer error handler (registered in DMA module) */
static void Adc_Dma_Adc2Error( void )
{
    (void)Adc_Set_XferError( ADC_PERIPH_2, ADC_ERROR_DMA_TRANSFER );
}
#endif /* ADC2 */

#if defined (ADC3)
/** \brief ADC3 DMA transfer complete handler (registered in DMA module) */
static void Adc_Dma_Adc3XferCplt( void )
{
    (void)Adc_Set_XferDone( ADC_PERIPH_3 );
}


/** \brief ADC3 DMA half transfer handler (registered in DMA module) */
static void Adc_Dma_Adc3HalfXfer( void )
{
    (void)Adc_Set_XferHalf( ADC_PERIPH_3 );
}


/** \brief ADC3 DMA transfer error handler (registered in DMA module) */
static void Adc_Dma_Adc3Error( void )
{
    (void)Adc_Set_XferError( ADC_PERIPH_3, ADC_ERROR_DMA_TRANSFER );
}
#endif /* ADC3 */

#if defined (ADC4)
/** \brief ADC4 DMA transfer complete handler (registered in DMA module) */
static void Adc_Dma_Adc4XferCplt( void )
{
    (void)Adc_Set_XferDone( ADC_PERIPH_4 );
}


/** \brief ADC4 DMA half transfer handler (registered in DMA module) */
static void Adc_Dma_Adc4HalfXfer( void )
{
    (void)Adc_Set_XferHalf( ADC_PERIPH_4 );
}


/** \brief ADC4 DMA transfer error handler (registered in DMA module) */
static void Adc_Dma_Adc4Error( void )
{
    (void)Adc_Set_XferError( ADC_PERIPH_4, ADC_ERROR_DMA_TRANSFER );
}
#endif /* ADC4 */

#if defined (ADC5)
/** \brief ADC5 DMA transfer complete handler (registered in DMA module) */
static void Adc_Dma_Adc5XferCplt( void )
{
    (void)Adc_Set_XferDone( ADC_PERIPH_5 );
}


/** \brief ADC5 DMA half transfer handler (registered in DMA module) */
static void Adc_Dma_Adc5HalfXfer( void )
{
    (void)Adc_Set_XferHalf( ADC_PERIPH_5 );
}


/** \brief ADC5 DMA transfer error handler (registered in DMA module) */
static void Adc_Dma_Adc5Error( void )
{
    (void)Adc_Set_XferError( ADC_PERIPH_5, ADC_ERROR_DMA_TRANSFER );
}
#endif /* ADC5 */

/* ================================ TASKS =================================== */
