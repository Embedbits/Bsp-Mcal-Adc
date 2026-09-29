/**
 * \author Mr.Nobody
 * \file Adc_Dma.c
 * \ingroup Adc
 * \brief Adc module DMA data transfer handler
 *
 * ADC_TRANSFER_MODE_DMA - regular results are moved from ADC_DR to the user buffer by a GPDMA
 * channel (peripheral to memory, 16-bit, one block of BufferSize items).
 * - Half transfer / transfer complete / DMA errors are reported from GPDMA interrupt.
 * - Overrun (OVR) and injected end of sequence (JEOS) are reported from ADC interrupt
 *   (see Adc_Isr.c).
 * - ADC_BUFFER_MODE_CIRCULAR: GPDMA module does not support cyclic transfer lists yet, so the
 *   channel is re-armed in the transfer complete interrupt (ADC DMA requests are kept by
 *   DMACFG = 1 while the channel is re-armed).
 *
 * Buffer handling and user callbacks are implemented in Adc.c (Adc_Set_Xfer* services).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Adc_Dma.h"                        /* Self include                   */
#include "Adc_Isr.h"                        /* ADC interrupt handler          */
#include "Adc.h"                            /* Module private interface       */
#include "Gpdma_Port.h"                     /* GPDMA Mcal layer include       */
/* ============================== TYPEDEFS ================================== */

/** ADC DMA request connection to GPDMA */
typedef struct
{
    adc_FunctionState_t   Available;   /**< DMA request is available in GPDMA module     */
    gpdma_PeriphReqId_t   Request;     /**< GPDMA request identification                  */
    gpdma_IsrCallback    *XferCpltIsr; /**< Transfer complete handler of the peripheral   */
    gpdma_IsrCallback    *HalfXferIsr; /**< Half transfer handler of the peripheral       */
    gpdma_IsrErrCallback *ErrorIsr;    /**< Transfer error handler of the peripheral      */
}   adc_DmaReqConfig_t;


/** GPDMA channel ownership of a peripheral (GPDMA channel can not be de-initialized separately) */
typedef struct
{
    adc_FunctionState_t Initialized; /**< GPDMA channel was initialized for the peripheral */
    adc_DmaPeriphId_t   PeriphId;    /**< Initialized GPDMA peripheral                     */
    adc_DmaChannelId_t  ChannelId;   /**< Initialized GPDMA channel                        */
}   adc_DmaChannelState_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static adc_RequestState_t Adc_Dma_Set_Transfer  ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Dma_XferCplt      ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Dma_XferError     ( adc_PeriphId_t periphId, gpdma_ErrorMaskId_t errorMask );

static void               Adc_Dma_Adc1XferCplt  ( void );
static void               Adc_Dma_Adc1HalfXfer  ( void );
static void               Adc_Dma_Adc1Error     ( gpdma_ErrorMaskId_t errorMask );
#if defined (ADC2)
static void               Adc_Dma_Adc2XferCplt  ( void );
static void               Adc_Dma_Adc2HalfXfer  ( void );
static void               Adc_Dma_Adc2Error     ( gpdma_ErrorMaskId_t errorMask );
#endif /* ADC2 */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Count of data items transferred per DMA request (single transfer) */
#define ADC_DMA_BURST_LEN            ( 1u )

/** Count of transfers in GPDMA transfer list (one block of BufferSize items) */
#define ADC_DMA_TRANSFERS_CNT        ( 1u )

/** Maximum buffer size in adc_Data_t items - GPDMA block size is limited to 16 bits (bytes) */
#define ADC_DMA_BUFFER_SIZE_MAX      ( (uint32_t)UINT16_MAX / (uint32_t)sizeof( adc_Data_t ) )

/** GPDMA errors reported to the user */
#define ADC_DMA_ERROR_MASK           ( GPDMA_ERROR_TRANSFER      | \
                                       GPDMA_ERROR_CONFIG_UPDATE | \
                                       GPDMA_ERROR_CONFIG_ERROR  | \
                                       GPDMA_ERROR_TRIG_OVERRUN    )

/** GPDMA error bit of ADC errors which are not reported by GPDMA (e.g. ADC overrun) */
#define ADC_DMA_ERROR_NONE           ( 0u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** adc_PeriphId_t -> GPDMA request and handlers. ADC3 request is not provided by the GPDMA module yet. */
static const adc_DmaReqConfig_t adc_DmaReqConfig[ ] =
{
    { .Available = ADC_FUNCTION_ACTIVE,   .Request = GPDMA_REQ_ADC1, .XferCpltIsr = Adc_Dma_Adc1XferCplt, .HalfXferIsr = Adc_Dma_Adc1HalfXfer, .ErrorIsr = Adc_Dma_Adc1Error },
#if defined (ADC2)
    { .Available = ADC_FUNCTION_ACTIVE,   .Request = GPDMA_REQ_ADC2, .XferCpltIsr = Adc_Dma_Adc2XferCplt, .HalfXferIsr = Adc_Dma_Adc2HalfXfer, .ErrorIsr = Adc_Dma_Adc2Error },
#endif /* ADC2 */
#if defined (ADC3)
    { .Available = ADC_FUNCTION_INACTIVE, .Request = GPDMA_REQ_ADC1, .XferCpltIsr = GPDMA_NULL_PTR,       .HalfXferIsr = GPDMA_NULL_PTR,       .ErrorIsr = GPDMA_NULL_PTR    },
#endif /* ADC3 */
};

_Static_assert( ADC_PERIPH_CNT == ( sizeof(adc_DmaReqConfig) / sizeof(adc_DmaReqConfig_t) ), "Adc: adc_DmaReqConfig has incorrect size." );


/** adc_ErrorId_t -> GPDMA error bit reporting the error */
static const uint32_t adc_DmaErrorMaskLut[ ADC_ERROR_CNT ] =
{
    [ADC_ERROR_OVERRUN]             = ADC_DMA_ERROR_NONE,
    [ADC_ERROR_DMA_TRANSFER]        = GPDMA_ERROR_TRANSFER,
    [ADC_ERROR_DMA_CONFIG]          = GPDMA_ERROR_CONFIG_ERROR,
    [ADC_ERROR_DMA_CONFIG_UPDATE]   = GPDMA_ERROR_CONFIG_UPDATE,
    [ADC_ERROR_DMA_TRIGGER_OVERRUN] = GPDMA_ERROR_TRIG_OVERRUN,
};


/** GPDMA transfer lists (must be static, used by GPDMA HW) */
static gpdma_XferList_t adc_DmaXferList[ ADC_PERIPH_CNT ];


/** GPDMA channel ownership per peripheral */
static adc_DmaChannelState_t adc_DmaChannelState[ ADC_PERIPH_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the data transfer configuration
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param dataConfig [in]: Pointer to data transfer configuration. Must not be NULL.
 *
 * \return Returns \ref ADC_REQUEST_OK if DMA request and ADC interrupt are available for the
 *         peripheral, DMA identifications are valid, DataBuffer is set and BufferSize is within
 *         1 - ADC_DMA_BUFFER_SIZE_MAX. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Dma_Check_Config( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId   ) &&
        ( ADC_NULL_PTR  != dataConfig )    )
    {
        const adc_RequestState_t isrState = Adc_Isr_Check_Config( periphId, dataConfig );

        if( ( ADC_FUNCTION_ACTIVE         == adc_DmaReqConfig[ periphId ].Available  ) &&
            ( ADC_REQUEST_OK              == isrState                                ) &&
            ( ADC_NULL_PTR                != dataConfig->DataBuffer                  ) &&
            ( 0u                           < dataConfig->BufferSize                  ) &&
            ( ADC_DMA_BUFFER_SIZE_MAX     >= (uint32_t)dataConfig->BufferSize        ) &&
            ( ADC_DMA_PERIPH_CNT           > dataConfig->DmaPeriphId                 ) &&
            ( ADC_DMA_CHANNEL_CNT          > dataConfig->DmaChannelId                ) &&
            ( (uint32_t)GPDMA_PRIORITY_CNT > (uint32_t)dataConfig->DmaPriority       )    )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* DMA resources are not available or DMA configuration is invalid */
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
 * \brief Initializes DMA data transfer: ADC DMA requests (DMAEN, DMACFG), GPDMA channel and
 *        ADC interrupt for OVR / JEOS
 *
 * \note  A GPDMA channel already initialized for the peripheral by a previous initialization is
 *        reused (GPDMA module does not support de-initialization of a single channel) - only
 *        its priority and half transfer interrupt are updated.
 *
 * \pre   Transfer context of the peripheral contains the data transfer configuration.
 *        No regular conversion is ongoing (called from Adc_PeriphInit() only).
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

    /* --- ADC DMA requests: limited (stop after the last DMA transfer) / unlimited (circular) --- */
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
                /* DMA transfer mode has not yet been applied, keep return state as error */
                retState = ADC_REQUEST_ERROR;
            }
        }
    }

    /* --- GPDMA channel --- */
    if( ADC_REQUEST_OK == retState )
    {
        adc_DmaChannelState_t * const    chState   = &adc_DmaChannelState[ periphId ];
        const adc_DmaReqConfig_t * const reqConfig = &adc_DmaReqConfig[ periphId ];
        const gpdma_PeriphId_t           dmaPeriph = (gpdma_PeriphId_t)xferCtx->Config.DmaPeriphId;
        const gpdma_ChannelId_t          dmaChan   = (gpdma_ChannelId_t)xferCtx->Config.DmaChannelId;
        gpdma_RequestState_t             dmaState  = GPDMA_REQUEST_ERROR;

        if( ( ADC_FUNCTION_ACTIVE         == chState->Initialized ) &&
            ( xferCtx->Config.DmaPeriphId == chState->PeriphId    ) &&
            ( xferCtx->Config.DmaChannelId == chState->ChannelId  )    )
        {
            /* Channel is already configured for this peripheral - priority and half transfer are updated */
            dmaState = Gpdma_Set_Priority( dmaPeriph, dmaChan, (gpdma_Priority_t)xferCtx->Config.DmaPriority );

            if( ( GPDMA_REQUEST_OK == dmaState ) && ( ADC_NULL_PTR != xferCtx->Config.HalfTransferCallback ) )
            {
                dmaState = Gpdma_Set_HalfTransferIsrHandler( dmaPeriph, dmaChan, reqConfig->HalfXferIsr );

                if( GPDMA_REQUEST_OK == dmaState )
                {
                    dmaState = Gpdma_Set_HalfTransferIrqActive( dmaPeriph, dmaChan );
                }
                else
                {
                    /* Handler registration failed, interrupt is not enabled */
                }
            }
            else if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_HalfTransferIrqInactive( dmaPeriph, dmaChan );
            }
            else
            {
                /* Priority configuration failed */
            }
        }
        else
        {
            gpdma_ConfigStruct_t   dmaConfig   = { 0u };
            gpdma_TransferConfig_t xferConfig  = { 0u };

            xferConfig.Direction                   = GPDMA_DIR_PERIPH_TO_MEMORY;
            xferConfig.EventMode                   = GPDMA_TRANSFER_EVENT_BLOCK;

            xferConfig.TriggerType                 = GPDMA_TRG_NOT_USED;
            xferConfig.TriggerMode                 = GPDMA_TRIGGER_BLOCK;

            xferConfig.RequestSource               = reqConfig->Request;
            xferConfig.RequestMode                 = GPDMA_PERIPH_REQ_SINGLE;

            xferConfig.BlockSize                   = (gpdma_BlockSize_t)( (uint32_t)xferCtx->Config.BufferSize * (uint32_t)sizeof( adc_Data_t ) );
            xferConfig.BlockRepetitionCount        = 0u;

            xferConfig.SourceAddr                  = (gpdma_SrcAddr_t)LL_ADC_DMA_GetRegAddr( periphReg, LL_ADC_DMA_REG_REGULAR_DATA );
            xferConfig.SourceDataSize              = GPDMA_DATA_SIZE_16BITS;
            xferConfig.SourceBurstLength           = ADC_DMA_BURST_LEN;
            xferConfig.SourceAddrMode              = GPDMA_ADDR_STATIC;
            xferConfig.SourcePortId                = GPDMA_PORT_DEFAULT;
            xferConfig.SourceDataOp                = GPDMA_SRC_DATA_PRESERVE;

            xferConfig.DestinationAddr             = (gpdma_DstAddr_t)xferCtx->Config.DataBuffer;
            xferConfig.DestinationDataSize         = GPDMA_DATA_SIZE_16BITS;
            xferConfig.DestinationBurstLength      = ADC_DMA_BURST_LEN;
            xferConfig.DestinationAddrMode         = GPDMA_ADDR_INCREMENT;
            xferConfig.DestinationPortId           = GPDMA_PORT_DEFAULT;
            xferConfig.DestinationDataOp           = GPDMA_DEST_DATA_PRESERVE;

            dmaState = Gpdma_Get_DefaultConfig( &dmaConfig );

            dmaConfig.PeriphId            = dmaPeriph;
            dmaConfig.ChannelId           = dmaChan;
            dmaConfig.ChannelPrio         = (gpdma_Priority_t)xferCtx->Config.DmaPriority;
            dmaConfig.TransferExecMode    = GPDMA_XFER_EXEC_CONTINUOUS;
            dmaConfig.TransferConfig      = &xferConfig;
            dmaConfig.TransfersCount      = ADC_DMA_TRANSFERS_CNT;
            dmaConfig.XferListAccessMode  = GPDMA_TRANSFER_LIST_ACCESS_SINGLE;
            dmaConfig.XferList            = &adc_DmaXferList[ periphId ];
            dmaConfig.TransferLockState   = GPDMA_TRANSFER_LIST_LOCKED;

            /* Transfer complete is always needed (one shot stop / circular re-arm) */
            dmaConfig.TransferCompleteIsr = reqConfig->XferCpltIsr;
            dmaConfig.ErrorIsr            = reqConfig->ErrorIsr;
            dmaConfig.ErrorMask           = ADC_DMA_ERROR_MASK;

            if( ADC_NULL_PTR != xferCtx->Config.HalfTransferCallback )
            {
                dmaConfig.HalfTransferIsr = reqConfig->HalfXferIsr;
            }
            else
            {
                dmaConfig.HalfTransferIsr = GPDMA_NULL_PTR;
            }

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Init( &dmaConfig );
            }
            else
            {
                /* Default configuration is not available */
            }

            if( GPDMA_REQUEST_OK == dmaState )
            {
                chState->Initialized = ADC_FUNCTION_ACTIVE;
                chState->PeriphId    = xferCtx->Config.DmaPeriphId;
                chState->ChannelId   = xferCtx->Config.DmaChannelId;
            }
            else
            {
                /* GPDMA channel initialization failed */
            }
        }

        /* GPDMA module does not enable the channel interrupt in NVIC by itself */
        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_InterruptActive( dmaPeriph, dmaChan );
        }
        else
        {
            /* GPDMA channel configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            retState = ADC_REQUEST_ERROR;
        }
    }

    /* --- ADC interrupt for overrun and injected end of sequence --- */
    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Isr_Init( periphId );
    }

    return ( retState );
}


/**
 * \brief Deinitializes DMA data transfer: GPDMA channel disabled, ADC DMA requests disabled,
 *        ADC interrupt disabled
 *
 * \note  GPDMA channel configuration is kept (see Adc_Dma_Init()).
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Dma_Deinit( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef *       periphReg = ADC_NULL_PTR;

    retState = Adc_Get_PeriphReg( periphId, &periphReg );

    if( ( ADC_REQUEST_OK == retState ) && ( ADC_FUNCTION_ACTIVE == adc_DmaChannelState[ periphId ].Initialized ) )
    {
        const gpdma_PeriphId_t  dmaPeriph = (gpdma_PeriphId_t)adc_DmaChannelState[ periphId ].PeriphId;
        const gpdma_ChannelId_t dmaChan   = (gpdma_ChannelId_t)adc_DmaChannelState[ periphId ].ChannelId;
        gpdma_RequestState_t    dmaState  = Gpdma_Set_ChannelInactive( dmaPeriph, dmaChan );

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_InterruptInactive( dmaPeriph, dmaChan );
        }
        else
        {
            /* Channel could not be disabled */
        }

        if( GPDMA_REQUEST_OK == dmaState )
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
        /* No GPDMA channel was initialized for the peripheral */
    }

    if( ADC_REQUEST_OK == retState )
    {
        LL_ADC_REG_SetDMATransfer( periphReg, LL_ADC_REG_DMA_TRANSFER_NONE );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_ADC_REG_GetDMATransfer( periphReg );

            if( LL_ADC_REG_DMA_TRANSFER_NONE == regValue )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* DMA transfer mode has not yet been applied, keep return state as error */
                retState = ADC_REQUEST_ERROR;
            }
        }
    }

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Isr_Deinit( periphId );
    }

    return ( retState );
}


/**
 * \brief Starts DMA data transfer - GPDMA channel is armed for the whole buffer, overrun
 *        interrupt is enabled
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Dma_Start( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    retState = Adc_Dma_Set_Transfer( periphId );

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Isr_Set_IrqActive( periphId, LL_ADC_IT_OVR );
    }
    else
    {
        /* DMA channel could not be armed */
    }

    return ( retState );
}


/**
 * \brief Stops DMA data transfer - GPDMA channel and overrun interrupt are disabled
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Dma_Stop( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) && ( ADC_FUNCTION_ACTIVE == adc_DmaChannelState[ periphId ].Initialized ) )
    {
        const gpdma_RequestState_t dmaState = Gpdma_Set_ChannelInactive( (gpdma_PeriphId_t)adc_DmaChannelState[ periphId ].PeriphId,
                                                                         (gpdma_ChannelId_t)adc_DmaChannelState[ periphId ].ChannelId );

        if( GPDMA_REQUEST_OK == dmaState )
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
 * \brief Arms the GPDMA channel for the whole buffer (block size, destination address, enable)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Dma_Set_Transfer( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState = ADC_REQUEST_ERROR;
    adc_XferContext_t * xferCtx  = ADC_NULL_PTR;

    retState = Adc_Get_XferContext( periphId, &xferCtx );

    if( ( ADC_REQUEST_OK == retState ) && ( ADC_FUNCTION_ACTIVE == adc_DmaChannelState[ periphId ].Initialized ) )
    {
        const gpdma_PeriphId_t  dmaPeriph = (gpdma_PeriphId_t)adc_DmaChannelState[ periphId ].PeriphId;
        const gpdma_ChannelId_t dmaChan   = (gpdma_ChannelId_t)adc_DmaChannelState[ periphId ].ChannelId;
        const gpdma_BlockSize_t blockSize = (gpdma_BlockSize_t)( (uint32_t)xferCtx->Config.BufferSize * (uint32_t)sizeof( adc_Data_t ) );
        gpdma_RequestState_t    dmaState  = Gpdma_Set_BlockSize( dmaPeriph, dmaChan, blockSize );

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_DestinationAddr( dmaPeriph, dmaChan, (gpdma_DstAddr_t)xferCtx->Config.DataBuffer );
        }
        else
        {
            /* Block size configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_ChannelActive( dmaPeriph, dmaChan );
        }
        else
        {
            /* Destination address configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
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
 * \brief Transfer complete processing: circular buffer is re-armed, event is reported to Adc.c
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Dma_XferCplt( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState = ADC_REQUEST_ERROR;
    adc_XferContext_t * xferCtx  = ADC_NULL_PTR;

    retState = Adc_Get_XferContext( periphId, &xferCtx );

    if( ( ADC_REQUEST_OK == retState ) && ( ADC_BUFFER_MODE_CIRCULAR == xferCtx->Config.BufferMode ) )
    {
        /* Channel is disabled by HW at the end of the block - it is armed again immediately */
        retState = Adc_Dma_Set_Transfer( periphId );
    }
    else
    {
        /* One shot buffer - transfer is stopped by Adc_Set_XferDone() */
    }

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Set_XferDone( periphId );
    }
    else
    {
        /* Re-arm failed, reported as DMA transfer error */
        (void)Adc_Set_XferError( periphId, ADC_ERROR_DMA_TRANSFER );
    }

    return ( retState );
}


/**
 * \brief DMA error processing: every reported GPDMA error bit is forwarded as ADC error
 *
 * \param periphId  [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param errorMask [in]: GPDMA error bit mask
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Dma_XferError( adc_PeriphId_t periphId, gpdma_ErrorMaskId_t errorMask )
{
    adc_RequestState_t retState = ADC_REQUEST_OK;

    for( adc_ErrorId_t errorId = ADC_ERROR_OVERRUN; ADC_ERROR_CNT > errorId; errorId ++ )
    {
        if( 0u != ( (uint32_t)errorMask & adc_DmaErrorMaskLut[ errorId ] ) )
        {
            retState = Adc_Set_XferError( periphId, errorId );
        }
        else
        {
            /* Error bit is not reported */
        }
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/** \brief ADC1 DMA transfer complete handler (registered in GPDMA module) */
static void Adc_Dma_Adc1XferCplt( void )
{
    (void)Adc_Dma_XferCplt( ADC_PERIPH_1 );
}


/** \brief ADC1 DMA half transfer handler (registered in GPDMA module) */
static void Adc_Dma_Adc1HalfXfer( void )
{
    (void)Adc_Set_XferHalf( ADC_PERIPH_1 );
}


/**
 * \brief ADC1 DMA error handler (registered in GPDMA module)
 *
 * \param errorMask [in]: Mask of GPDMA errors reported by the channel.
 */
static void Adc_Dma_Adc1Error( gpdma_ErrorMaskId_t errorMask )
{
    (void)Adc_Dma_XferError( ADC_PERIPH_1, errorMask );
}


#if defined (ADC2)
/** \brief ADC2 DMA transfer complete handler (registered in GPDMA module) */
static void Adc_Dma_Adc2XferCplt( void )
{
    (void)Adc_Dma_XferCplt( ADC_PERIPH_2 );
}


/** \brief ADC2 DMA half transfer handler (registered in GPDMA module) */
static void Adc_Dma_Adc2HalfXfer( void )
{
    (void)Adc_Set_XferHalf( ADC_PERIPH_2 );
}


/**
 * \brief ADC2 DMA error handler (registered in GPDMA module)
 *
 * \param errorMask [in]: Mask of GPDMA errors reported by the channel.
 */
static void Adc_Dma_Adc2Error( gpdma_ErrorMaskId_t errorMask )
{
    (void)Adc_Dma_XferError( ADC_PERIPH_2, errorMask );
}
#endif /* ADC2 */

/* ================================ TASKS =================================== */
