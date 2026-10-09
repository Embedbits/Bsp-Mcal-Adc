/**
 * \author Mr.Nobody
 * \file Adc.c
 * \ingroup Adc
 * \brief Adc module common functionality
 *
 */
/* ============================== INCLUDES ================================== */
#include "Adc.h"                            /* Self include                   */
#include "Adc_Port.h"                       /* Own port file include          */
#include "Adc_Types.h"                      /* Module types definitions       */
#include "Adc_Dma.h"                        /* DMA data transfer handler      */
#include "Adc_Isr.h"                        /* ISR data transfer handler      */
#include "Adc_Poll.h"                       /* Polling data transfer handler  */
#include "Rcc_Port.h"                       /* RCC Mcal layer include         */
#include "Gpio_Port.h"                      /* GPIO Mcal layer include        */
#include "Nvic_Port.h"                      /* NVIC Mcal layer include        */
#include "Gpdma_Port.h"                     /* GPDMA Mcal layer include       */
#include "Stm32.h"                          /* MCU common functionality (ADC1/ADC2 instance pointers) */
/* ============================== TYPEDEFS ================================== */

typedef struct
{
    gpio_PortId_t PortId;
    gpio_PinId_t  PinId;
}   adc_GpioConfig_t;

typedef struct
{
    adc_ChannelId_t  ChannelId;
    adc_GpioConfig_t ChannelInP;
    adc_GpioConfig_t ChannelInN;
}   adc_GpioChannelConfig_t;

typedef struct
{
    adc_PeriphId_t PeriphId;
    adc_GpioChannelConfig_t Channel[ ADC_CHANNEL_CNT ];
}   adc_GpioPeriphConfig_t;

typedef struct
{
    ADC_TypeDef        *PeriphReg; /**< CMSIS ADC instance register pointer               */
}   adc_PeriphConfigStruct_t;


typedef struct
{
    adc_PeriphId_t  PeriphId;
    adc_ChannelId_t ChannelId[ ADC_CHANNEL_INPUT_CNT ];
}   adc_InputConfigStruct_t;


/** Data transfer mode handler interface (implemented in Adc_Dma.c / Adc_Isr.c / Adc_Poll.c) */
typedef struct
{
    adc_RequestState_t ( *CheckConfig )( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig ); /**< Mode specific configuration check */
    adc_RequestState_t ( *Init        )( adc_PeriphId_t periphId );                                            /**< Mode resources initialization     */
    adc_RequestState_t ( *Deinit      )( adc_PeriphId_t periphId );                                            /**< Mode resources deinitialization   */
    adc_RequestState_t ( *Start       )( adc_PeriphId_t periphId );                                            /**< Regular data transfer start       */
    adc_RequestState_t ( *Stop        )( adc_PeriphId_t periphId );                                            /**< Regular data transfer stop        */
}   adc_XferModeIf_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static adc_RequestState_t Adc_Set_InternalInput ( adc_PeriphId_t periphId, adc_ChannelInput_t channelInput );
static adc_RequestState_t Adc_Check_ConversionStopped ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Check_PeriphDisabled    ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Check_AllPeriphsDisabled( void );
static adc_RequestState_t Adc_Check_ChannelInput      ( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput );
static adc_RequestState_t Adc_Check_ChannelSlot       ( adc_PeriphId_t periphId,
                                                        const adc_ChannelConfig_t * const channelSlot,
                                                        const adc_ChannelConfig_t ** const channelTable );
static adc_RequestState_t Adc_Check_PeriphConfig      ( const adc_PeriphConfig_t * const adcConfig );
static adc_RequestState_t Adc_Set_Calibration         ( adc_PeriphId_t periphId );
static adc_FunctionState_t Adc_Get_ExtendedCalibration( void );
static adc_RequestState_t Adc_Set_EnableWait          ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Set_ChannelPreselection ( adc_PeriphId_t periphId, adc_ChannelId_t channelId );
static adc_RequestState_t Adc_Set_PinAnalog           ( const adc_GpioConfig_t * const pinConfig );
static adc_RequestState_t Adc_Set_RegSequencer        ( const adc_PeriphConfig_t * const adcConfig );
static adc_RequestState_t Adc_Set_InjSequencer        ( const adc_PeriphConfig_t * const adcConfig );
static adc_RequestState_t Adc_Set_InjAutoMode         ( adc_PeriphId_t periphId, uint32_t llTrigAuto );
static adc_RequestState_t Adc_Set_InjDiscontMode      ( adc_PeriphId_t periphId, uint32_t llDiscont );
static adc_RequestState_t Adc_Set_Delay               ( adc_TimeUs_t delayUs );
static adc_RequestState_t Adc_Get_ClockFreq           ( adc_ClkSrc_t clkSource, adc_ClkDiv_t clkDiv, adc_FreqHz_t * const clkFreqHz );
static adc_RequestState_t Adc_Get_ActiveClockFreq     ( adc_FreqHz_t * const clkFreqHz );
static adc_RequestState_t Adc_Check_ClockFreq         ( adc_ClkSrc_t clkSource, adc_ClkDiv_t clkDiv );
static adc_RequestState_t Adc_Check_SamplingTime      ( adc_ChannelInput_t channelInput, adc_ChannelSampling_t samplingTime );
static adc_RequestState_t Adc_Check_DataConfig        ( adc_PeriphId_t periphId, adc_FunctionState_t regUsed, const adc_DataConfig_t * const dataConfig );
static adc_RequestState_t Adc_Set_XferInit            ( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig );
static adc_RequestState_t Adc_Set_XferStart           ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Set_XferStop            ( adc_PeriphId_t periphId );
static uint32_t           Adc_Get_AwdThresholdReg     ( adc_AwdId_t awdId, uint32_t resShift, uint32_t rawThreshold );
static uint32_t           Adc_Get_AwdThresholdRaw     ( adc_AwdId_t awdId, uint32_t resShift, uint32_t regThreshold );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define ADC_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define ADC_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define ADC_PATCH_VERSION           ( 0u )


/** Minimum ADC kernel clock frequency after divider (fADC min of ADC1 / ADC2, refer to device datasheet) */
#define ADC_CLK_FREQ_MIN_HZ          ( (adc_FreqHz_t)140000u )

/** Maximum ADC kernel clock frequency after divider (fADC max of ADC1 / ADC2, refer to device datasheet) */
#define ADC_CLK_FREQ_MAX_HZ          ( (adc_FreqHz_t)55000000u )


/** Minimum sampling time of temperature sensor channel in ns (tS_temp, refer to device datasheet) */
#define ADC_SAMPLING_MIN_TEMP_NS     ( 5000u )

/** Minimum sampling time of internal reference voltage channel in ns (tS_vrefint, refer to device datasheet) */
#define ADC_SAMPLING_MIN_VREF_NS     ( 4000u )

/** Minimum sampling time of VBAT/4 channel in ns (tS_vbat, refer to device datasheet) */
#define ADC_SAMPLING_MIN_VBAT_NS     ( 12000u )

/** Channel input without minimum sampling time requirement */
#define ADC_SAMPLING_MIN_NONE_NS     ( 0u )


/** Maximal threshold of analog watch-dogs (thresholds of all AWDs compare 14 bit values) */
#define ADC_AWD1_THRESHOLD_MAX       ( 0x3FFFu )

/** Shift of 14 bit value to the configured resolution (0 - 14 bit, 2 - 12 bit, 4 - 10 bit, 6 - 8 bit) */
#define ADC_AWD_RES_SHIFT( llRes )   ( ( (llRes) >> ADC_CFGR1_RES_Pos ) * 2u )

/** Calibration factor index of the extended calibration (RM0456, ST HAL HAL_ADCEx_Calibration_Start) */
#define ADC_CALIB_EXT_INDEX          ( 0x9u )

/** CALFACT2 bits written by the extended calibration */
#define ADC_CALIB_EXT_CALFACT2_MASK  ( 0xFFFFFF00u )

/** CALFACT2 value written by the extended calibration */
#define ADC_CALIB_EXT_CALFACT2_VALUE ( 0x03021100u )

/** Device IDs with extended calibration (STM32U535 / U545, STM32U5Fx / U5Gx) */
#define ADC_DEV_ID_U535_U545         ( 0x455u )
#define ADC_DEV_ID_U5F_U5G           ( 0x476u )

/** Device IDs with extended calibration from revision ADC_REV_ID_EXT_CALIB (STM32U575 / U585, STM32U59x / U5Ax) */
#define ADC_DEV_ID_U575_U585         ( 0x482u )
#define ADC_DEV_ID_U59X_U5AX         ( 0x481u )

/** First device revision of STM32U575 / U585 / U59x / U5Ax with extended calibration */
#define ADC_REV_ID_EXT_CALIB         ( 0x3000u )


/** Count of microseconds in one second */
#define ADC_US_PER_S                 ( 1000000u )

/** Count of nanoseconds in one second */
#define ADC_NS_PER_S                 ( 1000000000u )

/** Offset between sequence length and index of the length in adc_RegSeqLenLut / adc_InjSeqLenLut
 *  (sequence length 1 is stored at index 0) */
#define ADC_SEQ_LEN_IDX_OFFSET       ( 1u )

/** Divider of the buffer size giving the half transfer position */
#define ADC_BUFFER_HALF_DIVIDER      ( 2u )

/** Count of half clock cycles in one clock cycle (sampling times are defined in half cycles) */
#define ADC_HALF_CYCLES_PER_CYCLE    ( 2u )

/* =============================== MACROS =================================== */

/** Integer division rounded up (e.g. CPU cycles per microsecond, the delay is never shorter) */
#define ADC_DIV_ROUND_UP( dividend, divisor )    ( ( (dividend) + (divisor) - 1u ) / (divisor) )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

static const adc_GpioPeriphConfig_t adc_GpioPeriphConfig[ ADC_PERIPH_CNT ] =
{
 { .PeriphId = ADC_PERIPH_1,
   .Channel[ ADC_CHANNEL_0  ] = { .ChannelId  = ADC_CHANNEL_0 , .ChannelInP = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT },
                                                                .ChannelInN = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT }  },
   .Channel[ ADC_CHANNEL_1  ] = { .ChannelId  = ADC_CHANNEL_1 , .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_0   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_1   }  },
   .Channel[ ADC_CHANNEL_2  ] = { .ChannelId  = ADC_CHANNEL_2 , .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_1   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_2   }  },
   .Channel[ ADC_CHANNEL_3  ] = { .ChannelId  = ADC_CHANNEL_3 , .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_2   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_3   }  },
   .Channel[ ADC_CHANNEL_4  ] = { .ChannelId  = ADC_CHANNEL_4 , .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_3   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_0   }  },
   .Channel[ ADC_CHANNEL_5  ] = { .ChannelId  = ADC_CHANNEL_5 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_0   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_1   }  },
   .Channel[ ADC_CHANNEL_6  ] = { .ChannelId  = ADC_CHANNEL_6 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_1   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_2   }  },
   .Channel[ ADC_CHANNEL_7  ] = { .ChannelId  = ADC_CHANNEL_7 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_2   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_3   }  },
   .Channel[ ADC_CHANNEL_8  ] = { .ChannelId  = ADC_CHANNEL_8 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_3   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_4   }  },
   .Channel[ ADC_CHANNEL_9  ] = { .ChannelId  = ADC_CHANNEL_9 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_4   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_5   }  },
   .Channel[ ADC_CHANNEL_10 ] = { .ChannelId  = ADC_CHANNEL_10, .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_5   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_6   }  },
   .Channel[ ADC_CHANNEL_11 ] = { .ChannelId  = ADC_CHANNEL_11, .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_6   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_7   }  },
   .Channel[ ADC_CHANNEL_12 ] = { .ChannelId  = ADC_CHANNEL_12, .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_7   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_4   }  },
   .Channel[ ADC_CHANNEL_13 ] = { .ChannelId  = ADC_CHANNEL_13, .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_4   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_5   }  },
   .Channel[ ADC_CHANNEL_14 ] = { .ChannelId  = ADC_CHANNEL_14, .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_5   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_0   }  },
   .Channel[ ADC_CHANNEL_15 ] = { .ChannelId  = ADC_CHANNEL_15, .ChannelInP = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_0   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_1   }  },
   .Channel[ ADC_CHANNEL_16 ] = { .ChannelId  = ADC_CHANNEL_16, .ChannelInP = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_1   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_2   }  },
   .Channel[ ADC_CHANNEL_17 ] = { .ChannelId  = ADC_CHANNEL_17, .ChannelInP = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_2   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT }  },
   .Channel[ ADC_CHANNEL_18 ] = { .ChannelId  = ADC_CHANNEL_18, .ChannelInP = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT },
                                                                .ChannelInN = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT }  },
   .Channel[ ADC_CHANNEL_19 ] = { .ChannelId  = ADC_CHANNEL_19, .ChannelInP = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT },
                                                                .ChannelInN = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT }  },
 },
#if defined (ADC2)
 { .PeriphId = ADC_PERIPH_2,
   .Channel[ ADC_CHANNEL_0  ] = { .ChannelId  = ADC_CHANNEL_0 , .ChannelInP = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT },
                                                                .ChannelInN = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT }  },
   .Channel[ ADC_CHANNEL_1  ] = { .ChannelId  = ADC_CHANNEL_1 , .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_0   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_1   }  },
   .Channel[ ADC_CHANNEL_2  ] = { .ChannelId  = ADC_CHANNEL_2 , .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_1   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_2   }  },
   .Channel[ ADC_CHANNEL_3  ] = { .ChannelId  = ADC_CHANNEL_3 , .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_2   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_3   }  },
   .Channel[ ADC_CHANNEL_4  ] = { .ChannelId  = ADC_CHANNEL_4 , .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_3   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_0   }  },
   .Channel[ ADC_CHANNEL_5  ] = { .ChannelId  = ADC_CHANNEL_5 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_0   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_1   }  },
   .Channel[ ADC_CHANNEL_6  ] = { .ChannelId  = ADC_CHANNEL_6 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_1   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_2   }  },
   .Channel[ ADC_CHANNEL_7  ] = { .ChannelId  = ADC_CHANNEL_7 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_2   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_3   }  },
   .Channel[ ADC_CHANNEL_8  ] = { .ChannelId  = ADC_CHANNEL_8 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_3   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_4   }  },
   .Channel[ ADC_CHANNEL_9  ] = { .ChannelId  = ADC_CHANNEL_9 , .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_4   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_5   }  },
   .Channel[ ADC_CHANNEL_10 ] = { .ChannelId  = ADC_CHANNEL_10, .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_5   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_6   }  },
   .Channel[ ADC_CHANNEL_11 ] = { .ChannelId  = ADC_CHANNEL_11, .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_6   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_7   }  },
   .Channel[ ADC_CHANNEL_12 ] = { .ChannelId  = ADC_CHANNEL_12, .ChannelInP = { .PortId = GPIO_PORT_A  , .PinId = GPIO_PIN_ID_7   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_4   }  },
   .Channel[ ADC_CHANNEL_13 ] = { .ChannelId  = ADC_CHANNEL_13, .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_4   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_5   }  },
   .Channel[ ADC_CHANNEL_14 ] = { .ChannelId  = ADC_CHANNEL_14, .ChannelInP = { .PortId = GPIO_PORT_C  , .PinId = GPIO_PIN_ID_5   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_0   }  },
   .Channel[ ADC_CHANNEL_15 ] = { .ChannelId  = ADC_CHANNEL_15, .ChannelInP = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_0   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_1   }  },
   .Channel[ ADC_CHANNEL_16 ] = { .ChannelId  = ADC_CHANNEL_16, .ChannelInP = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_1   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_2   }  },
   .Channel[ ADC_CHANNEL_17 ] = { .ChannelId  = ADC_CHANNEL_17, .ChannelInP = { .PortId = GPIO_PORT_B  , .PinId = GPIO_PIN_ID_2   },
                                                                .ChannelInN = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT }  },
   .Channel[ ADC_CHANNEL_18 ] = { .ChannelId  = ADC_CHANNEL_18, .ChannelInP = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT },
                                                                .ChannelInN = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT }  },
   .Channel[ ADC_CHANNEL_19 ] = { .ChannelId  = ADC_CHANNEL_19, .ChannelInP = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT },
                                                                .ChannelInN = { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT }  },
 },
#endif /* ADC2 */
};

/** CMSIS instance lookup, indexed by adc_PeriphId_t (mirrors tim_PeriphConf in Tim.c).
 *  ADC1/ADC2 share the ADC12_COMMON register block (ADC4 is not handled by the module). */
static const adc_PeriphConfigStruct_t adc_PeriphConf[ ] =
{
    { .PeriphReg = ADC1 },
#if defined (ADC2)
    { .PeriphReg = ADC2 },
#endif /* ADC2 */
};

_Static_assert( ADC_PERIPH_CNT == ( sizeof(adc_PeriphConf) / sizeof(adc_PeriphConfigStruct_t) ), "Adc: adc_PeriphConf has incorrect size." );


/** Internal channel mapping per peripheral (ADC_CHANNEL_CNT == input not available), taken over
 *  from LL_ADC_CHANNEL_TEMPSENSOR / VREFINT / VBAT definitions. VDDCORE and DAC1 outputs are
 *  internal channels of ADC4 only (ADC4 is not handled by the module, the inputs are not in
 *  the list \ref adc_ChannelInput_t). */
static const adc_InputConfigStruct_t    adc_InputConfig[ ] =
{
 { .PeriphId = ADC_PERIPH_1,
   .ChannelId[ ADC_CHANNEL_INPUT_PIN_SINGLE ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_PIN_DIFF   ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_TEMP       ] = ADC_CHANNEL_19 ,
   .ChannelId[ ADC_CHANNEL_INPUT_VREF       ] = ADC_CHANNEL_0  ,
   .ChannelId[ ADC_CHANNEL_INPUT_VBAT       ] = ADC_CHANNEL_18 ,
 },
#if defined (ADC2)
 { .PeriphId = ADC_PERIPH_2,
   .ChannelId[ ADC_CHANNEL_INPUT_PIN_SINGLE ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_PIN_DIFF   ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_TEMP       ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_VREF       ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_VBAT       ] = ADC_CHANNEL_CNT,
 },
#endif /* ADC2 */
};

_Static_assert( ADC_PERIPH_CNT == ( sizeof(adc_InputConfig) / sizeof(adc_InputConfigStruct_t) ), "Adc: adc_InputConfig has incorrect size." );


/** adc_ClkSrc_t -> rcc_PeriphId_t. Rcc.c models each ADC/DAC kernel clock source,
 *  including HCLK, as its own rcc_PeriphId_t value that shares a single RCC_BLOCK_ADC
 *  bus-enable bit (see Rcc_Types.h / Rcc.c rcc_PeriphMap). */
static const rcc_PeriphId_t adc_ClkSrcRccLut[ ] =
{
    [ADC_CLK_SRC_HCLK]   = RCC_PERIPH_ADC_HCLK,
    [ADC_CLK_SRC_SYSCLK] = RCC_PERIPH_ADC_SYSCLK,
    [ADC_CLK_SRC_PLL2R]  = RCC_PERIPH_ADC_PLL2R,
    [ADC_CLK_SRC_HSE]    = RCC_PERIPH_ADC_HSE,
    [ADC_CLK_SRC_HSI]    = RCC_PERIPH_ADC_HSI,
    [ADC_CLK_SRC_MSIK]   = RCC_PERIPH_ADC_MSIK,
};

_Static_assert( ADC_CLK_SRC_CNT == ( sizeof(adc_ClkSrcRccLut) / sizeof(rcc_PeriphId_t) ), "Adc: adc_ClkSrcRccLut has incorrect size." );


/** adc_ClkDiv_t -> LL_ADC_CLOCK_ASYNC_DIVx (STM32U5 ADC1 / ADC2 are clocked only by the asynchronous kernel clock) */
static const uint32_t adc_ClkDivAsyncLut[ ADC_CLK_DIV_CNT ] =
{
    [ADC_CLK_DIV_1]   = LL_ADC_CLOCK_ASYNC_DIV1,
    [ADC_CLK_DIV_2]   = LL_ADC_CLOCK_ASYNC_DIV2,
    [ADC_CLK_DIV_4]   = LL_ADC_CLOCK_ASYNC_DIV4,
    [ADC_CLK_DIV_8]   = LL_ADC_CLOCK_ASYNC_DIV8,
    [ADC_CLK_DIV_16]  = LL_ADC_CLOCK_ASYNC_DIV16,
    [ADC_CLK_DIV_32]  = LL_ADC_CLOCK_ASYNC_DIV32,
    [ADC_CLK_DIV_64]  = LL_ADC_CLOCK_ASYNC_DIV64,
    [ADC_CLK_DIV_128] = LL_ADC_CLOCK_ASYNC_DIV128,
    [ADC_CLK_DIV_256] = LL_ADC_CLOCK_ASYNC_DIV256,
};


/** adc_RegTriggerId_t -> LL_ADC_REG_TRIG_*. Entries follow the exact order and \#if guarding
 *  of adc_RegTriggerId_t in Adc_Types.h, (STM32U5 ADC1 / ADC2 triggers, the same on all lines). */
static const uint32_t adc_RegTriggerSrcLut[ ] =
{
    [ADC_REG_TRIGGER_SOFTWARE]        = LL_ADC_REG_TRIG_SOFTWARE,
    [ADC_REG_TRIGGER_EXT_TIM1_CH1]    = LL_ADC_REG_TRIG_EXT_TIM1_CH1,
    [ADC_REG_TRIGGER_EXT_TIM1_CH2]    = LL_ADC_REG_TRIG_EXT_TIM1_CH2,
    [ADC_REG_TRIGGER_EXT_TIM1_CH3]    = LL_ADC_REG_TRIG_EXT_TIM1_CH3,
    [ADC_REG_TRIGGER_EXT_TIM2_CH2]    = LL_ADC_REG_TRIG_EXT_TIM2_CH2,
    [ADC_REG_TRIGGER_EXT_TIM3_TRGO]   = LL_ADC_REG_TRIG_EXT_TIM3_TRGO,
    [ADC_REG_TRIGGER_EXT_TIM4_CH4]    = LL_ADC_REG_TRIG_EXT_TIM4_CH4,
    [ADC_REG_TRIGGER_EXT_EXTI_LINE11] = LL_ADC_REG_TRIG_EXT_EXTI_LINE11,
    [ADC_REG_TRIGGER_EXT_TIM8_TRGO]   = LL_ADC_REG_TRIG_EXT_TIM8_TRGO,
    [ADC_REG_TRIGGER_EXT_TIM8_TRGO2]  = LL_ADC_REG_TRIG_EXT_TIM8_TRGO2,
    [ADC_REG_TRIGGER_EXT_TIM1_TRGO]   = LL_ADC_REG_TRIG_EXT_TIM1_TRGO,
    [ADC_REG_TRIGGER_EXT_TIM1_TRGO2]  = LL_ADC_REG_TRIG_EXT_TIM1_TRGO2,
    [ADC_REG_TRIGGER_EXT_TIM2_TRGO]   = LL_ADC_REG_TRIG_EXT_TIM2_TRGO,
    [ADC_REG_TRIGGER_EXT_TIM4_TRGO]   = LL_ADC_REG_TRIG_EXT_TIM4_TRGO,
    [ADC_REG_TRIGGER_EXT_TIM6_TRGO]   = LL_ADC_REG_TRIG_EXT_TIM6_TRGO,
    [ADC_REG_TRIGGER_EXT_TIM15_TRGO]  = LL_ADC_REG_TRIG_EXT_TIM15_TRGO,
    [ADC_REG_TRIGGER_EXT_TIM3_CH4]    = LL_ADC_REG_TRIG_EXT_TIM3_CH4,
    [ADC_REG_TRIGGER_EXT_EXTI_LINE15] = LL_ADC_REG_TRIG_EXT_EXTI_LINE15,
    [ADC_REG_TRIGGER_EXT_LPTIM1_CH1]  = LL_ADC_REG_TRIG_EXT_LPTIM1_CH1,
    [ADC_REG_TRIGGER_EXT_LPTIM2_CH1]  = LL_ADC_REG_TRIG_EXT_LPTIM2_CH1,
    [ADC_REG_TRIGGER_EXT_LPTIM3_CH1]  = LL_ADC_REG_TRIG_EXT_LPTIM3_CH1,
    [ADC_REG_TRIGGER_EXT_LPTIM4_OUT]  = LL_ADC_REG_TRIG_EXT_LPTIM4_OUT,
};

_Static_assert( ADC_REG_TRIGGER_CNT == ( sizeof(adc_RegTriggerSrcLut) / sizeof(uint32_t) ), "Adc: adc_RegTriggerSrcLut has incorrect size." );


/** adc_InjTriggerId_t -> LL_ADC_INJ_TRIG_*. Entries follow the exact order and \#if guarding
 *  of adc_InjTriggerId_t in Adc_Types.h. ADC_INJ_TRIGGER_AUTO has no own trigger source -
 *  auto-injected mode (JAUTO) requires the injected external trigger disabled (JEXTEN = 0). */
static const uint32_t adc_InjTriggerSrcLut[ ] =
{
    [ADC_INJ_TRIGGER_SOFTWARE]        = LL_ADC_INJ_TRIG_SOFTWARE,
    [ADC_INJ_TRIGGER_AUTO]            = LL_ADC_INJ_TRIG_SOFTWARE,
    [ADC_INJ_TRIGGER_EXT_TIM1_TRGO]   = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO,
    [ADC_INJ_TRIGGER_EXT_TIM1_CH4]    = LL_ADC_INJ_TRIG_EXT_TIM1_CH4,
    [ADC_INJ_TRIGGER_EXT_TIM2_TRGO]   = LL_ADC_INJ_TRIG_EXT_TIM2_TRGO,
    [ADC_INJ_TRIGGER_EXT_TIM2_CH1]    = LL_ADC_INJ_TRIG_EXT_TIM2_CH1,
    [ADC_INJ_TRIGGER_EXT_TIM3_CH4]    = LL_ADC_INJ_TRIG_EXT_TIM3_CH4,
    [ADC_INJ_TRIGGER_EXT_TIM4_TRGO]   = LL_ADC_INJ_TRIG_EXT_TIM4_TRGO,
    [ADC_INJ_TRIGGER_EXT_EXTI_LINE15] = LL_ADC_INJ_TRIG_EXT_EXTI_LINE15,
    [ADC_INJ_TRIGGER_EXT_TIM8_CH4]    = LL_ADC_INJ_TRIG_EXT_TIM8_CH4,
    [ADC_INJ_TRIGGER_EXT_TIM1_TRGO2]  = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2,
    [ADC_INJ_TRIGGER_EXT_TIM8_TRGO]   = LL_ADC_INJ_TRIG_EXT_TIM8_TRGO,
    [ADC_INJ_TRIGGER_EXT_TIM8_TRGO2]  = LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2,
    [ADC_INJ_TRIGGER_EXT_TIM3_CH3]    = LL_ADC_INJ_TRIG_EXT_TIM3_CH3,
    [ADC_INJ_TRIGGER_EXT_TIM3_TRGO]   = LL_ADC_INJ_TRIG_EXT_TIM3_TRGO,
    [ADC_INJ_TRIGGER_EXT_TIM3_CH1]    = LL_ADC_INJ_TRIG_EXT_TIM3_CH1,
    [ADC_INJ_TRIGGER_EXT_TIM6_TRGO]   = LL_ADC_INJ_TRIG_EXT_TIM6_TRGO,
    [ADC_INJ_TRIGGER_EXT_TIM15_TRGO]  = LL_ADC_INJ_TRIG_EXT_TIM15_TRGO,
    [ADC_INJ_TRIGGER_EXT_LPTIM1_CH2]  = LL_ADC_INJ_TRIG_EXT_LPTIM1_CH2,
    [ADC_INJ_TRIGGER_EXT_LPTIM2_CH2]  = LL_ADC_INJ_TRIG_EXT_LPTIM2_CH2,
    [ADC_INJ_TRIGGER_EXT_LPTIM3_CH1]  = LL_ADC_INJ_TRIG_EXT_LPTIM3_CH1,
    [ADC_INJ_TRIGGER_EXT_LPTIM4_OUT]  = LL_ADC_INJ_TRIG_EXT_LPTIM4_OUT,
};

_Static_assert( ADC_INJ_TRIGGER_CNT == ( sizeof(adc_InjTriggerSrcLut) / sizeof(uint32_t) ), "Adc: adc_InjTriggerSrcLut has incorrect size." );


/** adc_TriggerEdge_t -> LL_ADC_REG_TRIG_EXT_* edge selector (regular group) */
static const uint32_t adc_RegTriggerEdgeLut[ ADC_TRIGGER_EDGE_CNT ] =
{
    [ADC_TRIGGER_EDGE_RISING]  = LL_ADC_REG_TRIG_EXT_RISING,
    [ADC_TRIGGER_EDGE_FALLING] = LL_ADC_REG_TRIG_EXT_FALLING,
    [ADC_TRIGGER_EDGE_BOTH]    = LL_ADC_REG_TRIG_EXT_RISINGFALLING,
};


/** adc_TriggerEdge_t -> LL_ADC_INJ_TRIG_EXT_* edge selector (injected group) */
static const uint32_t adc_InjTriggerEdgeLut[ ADC_TRIGGER_EDGE_CNT ] =
{
    [ADC_TRIGGER_EDGE_RISING]  = LL_ADC_INJ_TRIG_EXT_RISING,
    [ADC_TRIGGER_EDGE_FALLING] = LL_ADC_INJ_TRIG_EXT_FALLING,
    [ADC_TRIGGER_EDGE_BOTH]    = LL_ADC_INJ_TRIG_EXT_RISINGFALLING,
};


/** adc_Resolution_t -> LL_ADC_RESOLUTION_x */
static const uint32_t adc_ResolutionLut[ ADC_RESOLUTION_CNT ] =
{
    [ADC_RESOLUTION_12BIT] = LL_ADC_RESOLUTION_12B,
    [ADC_RESOLUTION_10BIT] = LL_ADC_RESOLUTION_10B,
    [ADC_RESOLUTION_8BIT]  = LL_ADC_RESOLUTION_8B,
    [ADC_RESOLUTION_14BIT] = LL_ADC_RESOLUTION_14B,
};


/** adc_ChannelSampling_t -> LL_ADC_SAMPLINGTIME_x */
static const uint32_t adc_SamplingTimeLut[ ADC_CHANNEL_SAMPLING_CNT ] =
{
    [ADC_CHANNEL_SAMPLING_5_CYCLES]     = LL_ADC_SAMPLINGTIME_5CYCLES,
    [ADC_CHANNEL_SAMPLING_6_CYCLES]     = LL_ADC_SAMPLINGTIME_6CYCLES,
    [ADC_CHANNEL_SAMPLING_12_CYCLES]    = LL_ADC_SAMPLINGTIME_12CYCLES,
    [ADC_CHANNEL_SAMPLING_20_CYCLES]    = LL_ADC_SAMPLINGTIME_20CYCLES,
    [ADC_CHANNEL_SAMPLING_36_CYCLES]    = LL_ADC_SAMPLINGTIME_36CYCLES,
    [ADC_CHANNEL_SAMPLING_68_CYCLES]    = LL_ADC_SAMPLINGTIME_68CYCLES,
    [ADC_CHANNEL_SAMPLING_391_CYCLES]   = LL_ADC_SAMPLINGTIME_391CYCLES,
    [ADC_CHANNEL_SAMPLING_814_CYCLES]   = LL_ADC_SAMPLINGTIME_814CYCLES,
};


/** adc_AwdId_t -> LL_ADC_AWDx */
static const uint32_t adc_AwdIdLut[ ADC_AWD_CNT ] =
{
    [ADC_AWD_1] = LL_ADC_AWD1,
    [ADC_AWD_2] = LL_ADC_AWD2,
    [ADC_AWD_3] = LL_ADC_AWD3,
};

/** adc_AwdFilter_t -> LL_ADC_AWD_FILTERING_x */
static const uint32_t adc_AwdFilterLut[ ADC_AWD_FILTER_CNT ] =
{
    [ADC_AWD_FILTER_NONE] = LL_ADC_AWD_FILTERING_NONE,
    [ADC_AWD_FILTER_2]    = LL_ADC_AWD_FILTERING_2SAMPLES,
    [ADC_AWD_FILTER_3]    = LL_ADC_AWD_FILTERING_3SAMPLES,
    [ADC_AWD_FILTER_4]    = LL_ADC_AWD_FILTERING_4SAMPLES,
    [ADC_AWD_FILTER_5]    = LL_ADC_AWD_FILTERING_5SAMPLES,
    [ADC_AWD_FILTER_6]    = LL_ADC_AWD_FILTERING_6SAMPLES,
    [ADC_AWD_FILTER_7]    = LL_ADC_AWD_FILTERING_7SAMPLES,
    [ADC_AWD_FILTER_8]    = LL_ADC_AWD_FILTERING_8SAMPLES,
};


/** Regular rank (RegChannels[] slot index, ADC_REG_SEQUENCE_1 == rank 1) -> LL_ADC_REG_RANK_x */
static const uint32_t adc_RegSeqRankLut[ ADC_REG_SEQUENCE_CNT ] =
{
    LL_ADC_REG_RANK_1,
    LL_ADC_REG_RANK_2,
    LL_ADC_REG_RANK_3,
    LL_ADC_REG_RANK_4,
    LL_ADC_REG_RANK_5,
    LL_ADC_REG_RANK_6,
    LL_ADC_REG_RANK_7,
    LL_ADC_REG_RANK_8,
    LL_ADC_REG_RANK_9,
    LL_ADC_REG_RANK_10,
    LL_ADC_REG_RANK_11,
    LL_ADC_REG_RANK_12,
    LL_ADC_REG_RANK_13,
    LL_ADC_REG_RANK_14,
    LL_ADC_REG_RANK_15,
    LL_ADC_REG_RANK_16,
};


/** Regular sequence length (1..16, array index 0 == length 1) -> LL_ADC_REG_SEQ_SCAN_x */
static const uint32_t adc_RegSeqLenLut[ ADC_REG_SEQUENCE_CNT ] =
{
    LL_ADC_REG_SEQ_SCAN_DISABLE,
    LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_7RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_9RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_10RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_11RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_12RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_13RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_14RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_15RANKS,
    LL_ADC_REG_SEQ_SCAN_ENABLE_16RANKS,
};


/** Injected rank (InjChannels[] slot index, ADC_INJ_SEQUENCE_1 == rank 1) -> LL_ADC_INJ_RANK_x */
static const uint32_t adc_InjSeqRankLut[ ADC_INJ_SEQUENCE_CNT ] =
{
    LL_ADC_INJ_RANK_1,
    LL_ADC_INJ_RANK_2,
    LL_ADC_INJ_RANK_3,
    LL_ADC_INJ_RANK_4,
};


/** Injected sequence length (1..4, array index 0 == length 1) -> LL_ADC_INJ_SEQ_SCAN_x */
static const uint32_t adc_InjSeqLenLut[ ADC_INJ_SEQUENCE_CNT ] =
{
    LL_ADC_INJ_SEQ_SCAN_DISABLE,
    LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
    LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS,
    LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS,
};


/** adc_InjTriggerMode_t -> LL_ADC_INJ_SEQ_DISCONT_x (JDISCEN) */
static const uint32_t adc_InjTriggerModeLut[ ADC_INJ_TRIGGER_MODE_CNT ] =
{
    [ADC_INJ_TRIGGER_MODE_CONTINUOUS] = LL_ADC_INJ_SEQ_DISCONT_DISABLE, /* Trigger converts the sequence */
    [ADC_INJ_TRIGGER_MODE_SINGLE]     = LL_ADC_INJ_SEQ_DISCONT_1RANK,   /* Trigger converts one rank     */
};


/** adc_ClkDiv_t -> numeric value of the ADC kernel clock divider */
static const uint32_t adc_ClkDivValueLut[ ADC_CLK_DIV_CNT ] =
{
    [ADC_CLK_DIV_1]   = 1u,
    [ADC_CLK_DIV_2]   = 2u,
    [ADC_CLK_DIV_4]   = 4u,
    [ADC_CLK_DIV_8]   = 8u,
    [ADC_CLK_DIV_16]  = 16u,
    [ADC_CLK_DIV_32]  = 32u,
    [ADC_CLK_DIV_64]  = 64u,
    [ADC_CLK_DIV_128] = 128u,
    [ADC_CLK_DIV_256] = 256u,
};


/** adc_ChannelSampling_t -> sampling time in ADC clock half cycles (5 cycles == 10 half cycles) */
static const uint32_t adc_SamplingHalfCyclesLut[ ADC_CHANNEL_SAMPLING_CNT ] =
{
    [ADC_CHANNEL_SAMPLING_5_CYCLES]     = 10u,
    [ADC_CHANNEL_SAMPLING_6_CYCLES]     = 12u,
    [ADC_CHANNEL_SAMPLING_12_CYCLES]    = 24u,
    [ADC_CHANNEL_SAMPLING_20_CYCLES]    = 40u,
    [ADC_CHANNEL_SAMPLING_36_CYCLES]    = 72u,
    [ADC_CHANNEL_SAMPLING_68_CYCLES]    = 136u,
    [ADC_CHANNEL_SAMPLING_391_CYCLES]   = 782u,
    [ADC_CHANNEL_SAMPLING_814_CYCLES]   = 1628u,
};


/** adc_ChannelInput_t -> minimum sampling time in ns required by the connected signal */
static const adc_TimeNs_t adc_SamplingMinNsLut[ ADC_CHANNEL_INPUT_CNT ] =
{
    [ADC_CHANNEL_INPUT_PIN_SINGLE] = ADC_SAMPLING_MIN_NONE_NS,
    [ADC_CHANNEL_INPUT_PIN_DIFF]   = ADC_SAMPLING_MIN_NONE_NS,
    [ADC_CHANNEL_INPUT_TEMP]       = ADC_SAMPLING_MIN_TEMP_NS,
    [ADC_CHANNEL_INPUT_VREF]       = ADC_SAMPLING_MIN_VREF_NS,
    [ADC_CHANNEL_INPUT_VBAT]       = ADC_SAMPLING_MIN_VBAT_NS,
};


/** adc_TransferMode_t -> data transfer mode handler */
static const adc_XferModeIf_t adc_XferModeLut[ ADC_TRANSFER_MODE_CNT ] =
{
    [ADC_TRANSFER_MODE_DMA]  = { .CheckConfig = Adc_Dma_Check_Config,  .Init = Adc_Dma_Init,  .Deinit = Adc_Dma_Deinit,  .Start = Adc_Dma_Start,  .Stop = Adc_Dma_Stop  },
    [ADC_TRANSFER_MODE_ISR]  = { .CheckConfig = Adc_Isr_Check_Config,  .Init = Adc_Isr_Init,  .Deinit = Adc_Isr_Deinit,  .Start = Adc_Isr_Start,  .Stop = Adc_Isr_Stop  },
    [ADC_TRANSFER_MODE_POLL] = { .CheckConfig = Adc_Poll_Check_Config, .Init = Adc_Poll_Init, .Deinit = Adc_Poll_Deinit, .Start = Adc_Poll_Start, .Stop = Adc_Poll_Stop },
};


/** adc_TransferMode_t -> regular group overrun behavior. DMA keeps the unread result, so a lost
 *  sample is reported as overrun error; ISR / POLL keep the latest result in DR. */
static const uint32_t adc_OvrModeLut[ ADC_TRANSFER_MODE_CNT ] =
{
    [ADC_TRANSFER_MODE_DMA]  = LL_ADC_REG_OVR_DATA_PRESERVED,
    [ADC_TRANSFER_MODE_ISR]  = LL_ADC_REG_OVR_DATA_OVERWRITTEN,
    [ADC_TRANSFER_MODE_POLL] = LL_ADC_REG_OVR_DATA_OVERWRITTEN,
};


/** adc_FlagId_t -> LL_ADC_FLAG_x (ADC_ISR bit) */
static const uint32_t adc_FlagLut[ ADC_FLAG_CNT ] =
{
    [ADC_FLAG_REG_EOC] = LL_ADC_FLAG_EOC,
    [ADC_FLAG_REG_EOS] = LL_ADC_FLAG_EOS,
    [ADC_FLAG_REG_OVR] = LL_ADC_FLAG_OVR,
    [ADC_FLAG_INJ_EOC] = LL_ADC_FLAG_JEOC,
    [ADC_FLAG_INJ_EOS] = LL_ADC_FLAG_JEOS,
    [ADC_FLAG_AWD1]    = LL_ADC_FLAG_AWD1,
    [ADC_FLAG_AWD2]    = LL_ADC_FLAG_AWD2,
    [ADC_FLAG_AWD3]    = LL_ADC_FLAG_AWD3,
};


/** Shadow copy of the last clock source selected via Adc_Set_ClockSource(). No HW
 *  read-back path distinguishes between the 6 possible kernel clock sources through
 *  the Rcc abstraction used by this module (all 6 rcc_PeriphId_t values gate the same
 *  single RCC_BLOCK_ADC enable bit, so Rcc_Get_PeriphState() cannot tell them apart) -
 *  mirrors the shadow-state approach the sibling Tim module takes for the same kind of
 *  gap (see Tim_Get_ClockSource in Tim.c). */
static adc_ClkSrc_t adc_ShadowClkSrc = ADC_CLK_SRC_HCLK;


/** Shadow copy of the last input type selected via Adc_Set_ChannelInput(), per
 *  peripheral/channel. Distinguishing every possible adc_ChannelInput_t value purely
 *  from HW read-back (single/diff bit + internal-path enable bits + which physical
 *  channel number is in use) is not unambiguous in every case, so a shadow is kept
 *  here too, consistent with adc_ShadowClkSrc above. */
static adc_ChannelInput_t adc_ShadowChannelInput[ ADC_PERIPH_CNT ][ ADC_CHANNEL_CNT ];


/** Regular group data transfer runtime context per peripheral (shared with mode handlers
 *  through Adc_Get_XferContext()) */
static adc_XferContext_t adc_XferContext[ ADC_PERIPH_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
adc_ModuleVersion_t Adc_Get_ModuleVersion( void )
{
    adc_ModuleVersion_t retVersion;

    retVersion.Major = ADC_MAJOR_VERSION;
    retVersion.Minor = ADC_MINOR_VERSION;
    retVersion.Patch = ADC_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Initializes module Adc
 *
 * Configures the shared ADC kernel clock source and clock divider, then initializes every
 * peripheral from adcConfig->PeriphConfig[] that is in use (see \ref Adc_PeriphInit()).
 *
 * \note  adcConfig->PeriphConfig[] is indexed by \ref adc_PeriphId_t. A slot with
 *        RegChannelsCnt == 0 and InjChannelsCnt == 0 is treated as "peripheral not used" and
 *        is skipped. For a used slot, PeriphConfig[ i ].PeriphId must be equal to i, otherwise
 *        \ref ADC_REQUEST_ERROR is returned.
 *
 * \note  ADC kernel clock frequency (source frequency / divider) must be within
 *        ADC_CLK_FREQ_MIN_HZ - ADC_CLK_FREQ_MAX_HZ, it is checked before any register is modified.
 *
 * \note  If initialization of any peripheral fails, all peripherals initialized by this call
 *        are deinitialized (see \ref Adc_Deinit()) and \ref ADC_REQUEST_ERROR is returned.
 *
 * \pre   All ADC peripherals must be disabled (ADEN = 0), because the setting is shared
 *        through the ADC common register block(s). Otherwise \ref ADC_REQUEST_ERROR is
 *        returned and no register is modified.
 *        (checked by Adc_Set_ClockSource() / Adc_Set_ClockDivider())
 *
 * \param adcConfig [in]: Pointer to module configuration structure \ref adc_Config_t (clock source,
 *                        clock divider, per-peripheral configuration). Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Init( adc_Config_t * const adcConfig )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    adc_FunctionState_t clkConfOk = ADC_FUNCTION_INACTIVE;

    if( ADC_NULL_PTR != adcConfig )
    {
        /* ------------------------------------------------------------------ */
        /* ---------------------- Clock configuration ----------------------- */
        /* ------------------------------------------------------------------ */

        /* Resulting ADC kernel clock frequency is validated before any register is modified */
        retState = Adc_Check_ClockFreq( adcConfig->ClockSource, adcConfig->ClockDivider );

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_ClockSource( adcConfig->ClockSource );
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_ClockDivider( adcConfig->ClockDivider );
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        if( ADC_REQUEST_OK == retState )
        {
            /* All ADC peripherals were disabled (checked by clock configuration) - every
             * peripheral touched from now on is owned by this initialization */
            clkConfOk = ADC_FUNCTION_ACTIVE;
        }
        else
        {
            /* Clock configuration failed, peripherals are not initialized */
            clkConfOk = ADC_FUNCTION_INACTIVE;
        }

        /* ------------------------------------------------------------------ */
        /* -------------------- Peripherals configuration ------------------- */
        /* ------------------------------------------------------------------ */

        for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ( ADC_PERIPH_CNT > periphIdx ) && ( ADC_REQUEST_OK == retState ); periphIdx ++ )
        {
            adc_PeriphConfig_t * const periphCfg = &adcConfig->PeriphConfig[ periphIdx ];

            if( ( 0u == periphCfg->RegChannelsCnt ) &&
                ( 0u == periphCfg->InjChannelsCnt )    )
            {
                /* Peripheral is not used - nothing to configure */
                retState = ADC_REQUEST_OK;
            }
            else if( periphIdx != periphCfg->PeriphId )
            {
                /* PeriphConfig[] slot does not match its peripheral identification */
                retState = ADC_REQUEST_ERROR;
            }
            else
            {
                retState = Adc_PeriphInit( periphCfg );
            }
        }

        /* ------------------------------------------------------------------ */
        /* --------------------- Initialization result ---------------------- */
        /* ------------------------------------------------------------------ */

        if( ( ADC_REQUEST_OK != retState ) &&
            ( ADC_FUNCTION_ACTIVE == clkConfOk )    )
        {
            /* Initialization failed - all used peripherals are returned to the disabled state */
            for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx ++ )
            {
                const adc_PeriphConfig_t * const periphCfg = &adcConfig->PeriphConfig[ periphIdx ];

                if( ( 0u != periphCfg->RegChannelsCnt ) ||
                    ( 0u != periphCfg->InjChannelsCnt )    )
                {
                    /* Result is intentionally not evaluated, initialization is already reported as failed */
                    (void)Adc_Deinit( periphIdx );
                }
                else
                {
                    /* Peripheral is not used - nothing to deinitialize */
                }
            }
        }
        else
        {
            /* Initialization succeeded or no peripheral was touched */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes a single Adc channel (channel input and per-channel sampling time)
 *
 * \note  ADC resolution is an instance-wide HW setting (one RES field per ADC
 *        instance); it is modeled in adc_PeriphConfig_t.Resolution and applied
 *        once per peripheral by Adc_PeriphInit() (via Adc_Set_Resolution()), not
 *        here.
 *
 * \note  Channel input is applied first (see Adc_Set_ChannelInput()), sampling time
 *        only if the channel input was applied successfully.
 *
 * \note  Sampling time must satisfy the minimum sampling time of the channel input
 *        (internal channels, see Adc_Check_SamplingTime()) at the active ADC kernel clock.
 *
 * \pre   ADC peripheral must be disabled (ADEN = 0, no disable or calibration ongoing),
 *        because DIFSEL is writable only while ADEN = 0. Otherwise \ref ADC_REQUEST_ERROR
 *        is returned and no register is modified. To reconfigure a running peripheral call
 *        Adc_Set_PeriphInactive() first and Adc_Set_PeriphActive() afterwards.
 *
 * \param periphId      [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelConfig [in]: Pointer to channel configuration structure \ref adc_ChannelConfig_t.
 *                            Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_ChannelInit( adc_PeriphId_t periphId, adc_ChannelConfig_t * const channelConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId      ) &&
        ( ADC_NULL_PTR  != channelConfig )    )
    {
        const adc_RequestState_t periphState   = Adc_Check_PeriphDisabled( periphId );
        const adc_RequestState_t samplingState = Adc_Check_SamplingTime( channelConfig->ChannelInput, channelConfig->ChannelSampling );

        if( ADC_REQUEST_OK != periphState )
        {
            /* ADC peripheral is running, channel configuration is not allowed */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_REQUEST_OK != samplingState )
        {
            /* Sampling time is shorter than required by the channel input */
            retState = ADC_REQUEST_ERROR;
        }
        else
        {
            /* Register write and read-back verification is done by Adc_Set_ChannelInput() */
            retState = Adc_Set_ChannelInput( periphId, channelConfig->ChannelId, channelConfig->ChannelInput );
        }

        if( ADC_REQUEST_OK == retState )
        {
            /* Register write and read-back verification is done by Adc_Set_SamplingTime() */
            retState = Adc_Set_SamplingTime( periphId, channelConfig->ChannelId, channelConfig->ChannelSampling );
        }
        else
        {
            /* Channel input was not applied, sampling time is not configured */
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
 * \brief Deinitializes ADC peripheral
 *
 * Stops ongoing regular and injected conversions (Adc_Set_RegStop(), Adc_Set_InjStop()),
 * releases data transfer handler resources (DMA channel, interrupts), disables the peripheral
 * (Adc_Set_PeriphInactive()) and its internal voltage regulator.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Deinit( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

        /* --- Stop ongoing conversions (register access verified by called functions) --- */
        retState = Adc_Set_RegStop( periphId );

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_InjStop( periphId );
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        /* --- Release data transfer handler resources (DMA channel, interrupts) --- */
        if( ( ADC_REQUEST_OK == retState ) &&
            ( ADC_FUNCTION_ACTIVE == adc_XferContext[ periphId ].InitState )    )
        {
            const adc_TransferMode_t xferMode = adc_XferContext[ periphId ].Config.TransferMode;

            retState = adc_XferModeLut[ xferMode ].Deinit( periphId );

            if( ADC_REQUEST_OK == retState )
            {
                adc_XferContext[ periphId ].InitState = ADC_FUNCTION_INACTIVE;
            }
            else
            {
                /* Data transfer handler resources could not be released */
            }
        }
        else
        {
            /* Conversion could not be stopped or data transfer handler is not initialized */
        }

        /* --- Disable ADC --- */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_PeriphInactive( periphId );
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        /* --- Disable internal voltage regulator --- */
        if( ADC_REQUEST_OK == retState )
        {
            LL_ADC_DisableInternalRegulator( periphReg );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t regulatorOn = LL_ADC_IsInternalRegulatorEnabled( periphReg );

                if( 0u == regulatorOn )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Internal regulator disable has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        if( ADC_REQUEST_OK == retState )
        {
            for( adc_ChannelId_t channelIdx = ADC_CHANNEL_0; ADC_CHANNEL_CNT > channelIdx; channelIdx ++ )
            {
                adc_ShadowChannelInput[ periphId ][ channelIdx ] = ADC_CHANNEL_INPUT_PIN_SINGLE;
            }
        }
        else
        {
            /* Previous step failed, error state is kept */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Main task of module Adc
 *
 * This function shall be called in the main loop of the application or the task
 * scheduler. It shall be called periodically, depending on the module's
 * requirements.
 *
 * Services every peripheral initialized in ADC_TRANSFER_MODE_POLL (see Adc_Poll_Task()):
 * collects regular results into DataBuffer and reports overrun / injected end of sequence
 * through the configured callbacks. DMA and ISR modes need no periodic service.
 *
 * \note  In polling mode the task has to be called at least once per regular conversion,
 *        otherwise an overrun is reported (ADC_ERROR_OVERRUN).
 */
void Adc_Task( void )
{
    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx ++ )
    {
        const adc_XferContext_t * const xferCtx = &adc_XferContext[ periphIdx ];

        if( ( ADC_FUNCTION_ACTIVE    == xferCtx->InitState           ) &&
            ( ADC_TRANSFER_MODE_POLL == xferCtx->Config.TransferMode )    )
        {
            /* Result is reported through the configured callbacks */
            (void)Adc_Poll_Task( periphIdx );
        }
        else
        {
            /* Peripheral is not serviced by polling */
        }
    }
}


/* -------------------------------------------------------------------------- */
/* -------------------------- Clock configuration --------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Selects the shared ADC/DAC kernel clock source and enables the ADC block
 *
 * \pre   All ADC peripherals must be disabled (ADEN = 0), because the setting is shared
 *        through the ADC common register block(s). Otherwise \ref ADC_REQUEST_ERROR is
 *        returned and no register is modified.
 *        (ADC kernel clock must not be switched while any ADC is running.)
 *
 * \param clkSource [in]: Required ADC/DAC kernel clock source, value from \ref adc_ClkSrc_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_ClockSource( adc_ClkSrc_t clkSource )
{
    adc_RequestState_t retState        = ADC_REQUEST_ERROR;
    rcc_RequestState_t rccRequestState = RCC_REQUEST_ERROR;

    if( ADC_CLK_SRC_CNT > clkSource )
    {
        const adc_RequestState_t periphsState = Adc_Check_AllPeriphsDisabled( );

        if( ADC_REQUEST_OK == periphsState )
        {
            /* Rcc_Set_PeriphActive() selects the kernel clock multiplexer input and enables the
             * ADC block clock. It is called unconditionally, because all 6 kernel clock sources
             * share one enable bit - Rcc_Get_PeriphState() would report "active" for any of them
             * and the multiplexer would never be switched to a newly requested source. */
            rccRequestState = Rcc_Set_PeriphActive( adc_ClkSrcRccLut[ clkSource ] );

            if( RCC_REQUEST_OK == rccRequestState )
            {
                adc_ShadowClkSrc = clkSource;
                retState         = ADC_REQUEST_OK;
            }
            else
            {
                retState = ADC_REQUEST_ERROR;
            }
        }
        else
        {
            /* At least one ADC peripheral is enabled, configuration change is not allowed */
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
 * \brief Returns the last clock source selected via Adc_Set_ClockSource()
 *
 * \note See the comment on adc_ShadowClkSrc: this is a shadow value, not a HW
 *       read-back, since the Rcc abstraction used here cannot distinguish which
 *       of the 6 kernel clock sources is active (they share one enable bit).
 *
 * \param clkSource [out]: Pointer to store the last selected clock source (\ref adc_ClkSrc_t).
 *                         Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_ClockSource( adc_ClkSrc_t * const clkSource )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_NULL_PTR != clkSource )
    {
        *clkSource = adc_ShadowClkSrc;
        retState   = ADC_REQUEST_OK;
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/**
 * \brief Configures the ADC common clock divider
 *
 * \note  STM32U5 ADC1 / ADC2 have no synchronous (HCLK) clock mode - the kernel clock
 *        (HCLK included) is selected by the RCC multiplexer and divided by PRESC of
 *        ADC12_COMMON, all dividers /1../256 are available.
 *
 * \note  Resulting ADC kernel clock frequency (clock source selected by Adc_Set_ClockSource()
 *        / divider) must be within ADC_CLK_FREQ_MIN_HZ - ADC_CLK_FREQ_MAX_HZ. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and no register is modified.
 *
 * \pre   All ADC peripherals must be disabled (ADEN = 0), because the setting is shared
 *        through the ADC common register block(s). Otherwise \ref ADC_REQUEST_ERROR is
 *        returned and no register is modified.
 *
 * \param clkDiv [in]: Required ADC common clock divider, value from \ref adc_ClkDiv_t.
 *                     While clock source is HCLK only ADC_CLK_DIV_1/2/4 are allowed.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_ClockDivider( adc_ClkDiv_t clkDiv )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;
    uint32_t           llClkDiv = 0u;

    if( ADC_CLK_DIV_CNT > clkDiv )
    {
        const adc_RequestState_t periphsState = Adc_Check_AllPeriphsDisabled( );
        const adc_RequestState_t clkState     = Adc_Check_ClockFreq( adc_ShadowClkSrc, clkDiv );

        if( ADC_REQUEST_OK != clkState )
        {
            /* Resulting ADC kernel clock frequency is out of the allowed range */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_REQUEST_OK == periphsState )
        {
            /* STM32U5: kernel clock (also HCLK) is selected by RCC multiplexer, ADC prescaler divides it */
            llClkDiv = adc_ClkDivAsyncLut[ clkDiv ];

            LL_ADC_SetCommonClock( ADC12_COMMON, llClkDiv );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t clkDivReg = LL_ADC_GetCommonClock( ADC12_COMMON );

                if( llClkDiv == clkDivReg )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Clock divider has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* At least one ADC peripheral is enabled, configuration change is not allowed */
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
 * \brief Reads back the ADC common clock divider
 *
 * \note  Reads ADC12_COMMON (common for ADC1 and ADC2).
 *
 * \param clkDiv [out]: Pointer to store the current clock divider (\ref adc_ClkDiv_t).
 *                      Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_ClockDivider( adc_ClkDiv_t * const clkDiv )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_NULL_PTR != clkDiv )
    {
        const uint32_t llClkDiv = LL_ADC_GetCommonClock( ADC12_COMMON );

        for( adc_ClkDiv_t idx = ADC_CLK_DIV_1; ADC_CLK_DIV_CNT > idx; idx ++ )
        {
            if( adc_ClkDivAsyncLut[ idx ] == llClkDiv )
            {
                *clkDiv  = idx;
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* Divider does not match, next one is checked */
            }
        }
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------------ Peripheral configuration ------------------------ */
/* -------------------------------------------------------------------------- */

/**
 * \brief Initializes an ADC peripheral: deep power down exit, internal regulator, channel inputs
 *        and sampling times, calibration + enable (Adc_Set_PeriphActive()), resolution, regular
 *        sequencer + trigger and injected sequencer + trigger
 *
 * Configuration rules (the whole configuration is validated before any register is modified):
 * - RegChannelsCnt: 0 - 16, InjChannelsCnt: 0 - 4, at least one of them must be non-zero.
 * - Only the first RegChannelsCnt / InjChannelsCnt slots of RegChannels[] / InjChannels[] are
 *   used, the remaining slots are ignored. Slot index selects the rank (RegChannels[ 0 ] == rank 1).
 * - Every used slot must have a valid ChannelId, ChannelInput and ChannelSampling. ChannelInput
 *   must be available for the channel on the peripheral (GPIO pin(s) wired for PIN_SINGLE /
 *   PIN_DIFF, internal signal connected to ChannelId for TEMP / VREF / VBAT).
 * - ChannelSampling must satisfy the minimum sampling time of the channel input at the active
 *   ADC kernel clock (see Adc_Check_SamplingTime()).
 * - The same channel may be used in several ranks (also in both groups), but always with the
 *   same ChannelInput and ChannelSampling - both are per-channel HW settings, not per-rank ones.
 * - RegTriggerMode, RegTriggerId and RegTriggerEdge are applied only if RegChannelsCnt > 0,
 *   InjTriggerId and InjTriggerEdge only if InjChannelsCnt > 0.
 * - RegTriggerEdge / InjTriggerEdge must be valid, but are ignored for a software trigger
 *   (InjTriggerEdge also for ADC_INJ_TRIGGER_AUTO).
 * - InjTriggerId == ADC_INJ_TRIGGER_AUTO (auto-injected mode, JAUTO) requires RegChannelsCnt > 0
 *   and InjTriggerMode == ADC_INJ_TRIGGER_MODE_CONTINUOUS (JAUTO can not be combined with JDISCEN).
 * - DataConfig: see Adc_Check_DataConfig(). Data transfer handler is initialized last, regular
 *   data transfer starts with Adc_Set_RegStart().
 *
 * \note  If any initialization step fails after the peripheral was taken over (ADC disabled at
 *        entry), the peripheral is deinitialized (see \ref Adc_Deinit()) and \ref ADC_REQUEST_ERROR
 *        is returned.
 *
 * \pre   ADC peripheral must be disabled (ADEN = 0, no disable or calibration ongoing).
 *        Otherwise \ref ADC_REQUEST_ERROR is returned and no register is modified.
 *        Channel inputs (DIFSEL) and calibration can be configured only while ADEN = 0 - to
 *        reconfigure an already initialized peripheral call Adc_Deinit() first.
 *
 * \param adcConfig [in]: Pointer to peripheral configuration structure \ref adc_PeriphConfig_t.
 *                        Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_PeriphInit( adc_PeriphConfig_t * const adcConfig )
{
    adc_RequestState_t retState    = ADC_REQUEST_ERROR;
    adc_FunctionState_t initStarted = ADC_FUNCTION_INACTIVE;

    /* Complete configuration is validated before any register is modified (NULL included) */
    retState = Adc_Check_PeriphConfig( adcConfig );

    if( ADC_REQUEST_OK == retState )
    {
        ADC_TypeDef * const periphReg = adc_PeriphConf[ adcConfig->PeriphId ].PeriphReg;
        adc_TimeoutCnt_t    timeoutCnt;

        /* Channel inputs and calibration require ADEN = 0 - ADC has to be disabled before (re)initialization */
        retState = Adc_Check_PeriphDisabled( adcConfig->PeriphId );

        if( ADC_REQUEST_OK == retState )
        {
            /* Peripheral is taken over by this initialization, it is deinitialized on failure */
            initStarted = ADC_FUNCTION_ACTIVE;
        }
        else
        {
            /* Peripheral is running - it is not touched */
            initStarted = ADC_FUNCTION_INACTIVE;
        }

        /* --- Exit from deep power down (DEEPPWD = 1 after reset, regulator can not be enabled) --- */
        if( ADC_REQUEST_OK == retState )
        {
            LL_ADC_DisableDeepPowerDown( periphReg );

            for( timeoutCnt = 0u; ADC_TIMEOUT_RAW > timeoutCnt; timeoutCnt ++ )
            {
                const uint32_t deepPwdOn = LL_ADC_IsDeepPowerDownEnabled( periphReg );

                if( 0u == deepPwdOn )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Deep power down exit has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        /* --- Internal voltage regulator start-up --- */
        if( ADC_REQUEST_OK == retState )
        {
            LL_ADC_EnableInternalRegulator( periphReg );

            for( timeoutCnt = 0u; ADC_TIMEOUT_RAW > timeoutCnt; timeoutCnt ++ )
            {
                const uint32_t regulatorOn = LL_ADC_IsInternalRegulatorEnabled( periphReg );

                if( 0u != regulatorOn )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Internal regulator enable has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        /* STM32U5 ADC1 / ADC2: regulator start-up is signaled by LDORDY flag */
        if( ADC_REQUEST_OK == retState )
        {
            retState = ADC_REQUEST_ERROR;

            for( timeoutCnt = 0u; ADC_TIMEOUT_RAW > timeoutCnt; timeoutCnt ++ )
            {
                const uint32_t regulatorReady = LL_ADC_IsActiveFlag_LDORDY( periphReg );

                if( 0u != regulatorReady )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Internal regulator is not ready yet, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Internal regulator was not enabled */
        }

        /* --- Channel input and sampling time (regular and injected slots), ADEN = 0 --- */
        for( adc_RegSequenceId_t slotIdx = ADC_REG_SEQUENCE_1; ( adcConfig->RegChannelsCnt > slotIdx ) && ( ADC_REQUEST_OK == retState ); slotIdx ++ )
        {
            retState = Adc_ChannelInit( adcConfig->PeriphId, &adcConfig->RegChannels[ slotIdx ] );
        }

        for( adc_InjSequenceId_t slotIdx = ADC_INJ_SEQUENCE_1; ( adcConfig->InjChannelsCnt > slotIdx ) && ( ADC_REQUEST_OK == retState ); slotIdx ++ )
        {
            retState = Adc_ChannelInit( adcConfig->PeriphId, &adcConfig->InjChannels[ slotIdx ] );
        }

        /* --- Calibration and enable (register access verified by called function) --- */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_PeriphActive( adcConfig->PeriphId );
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        /* --- Instance-wide resolution (adc_PeriphConfig_t.Resolution) --- */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_Resolution( adcConfig->PeriphId, adcConfig->Resolution );
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        /* --- Regular group: sequencer, trigger source, trigger edge and conversion mode --- */
        if( ( ADC_REQUEST_OK == retState ) &&
            ( 0u < adcConfig->RegChannelsCnt )    )
        {
            retState = Adc_Set_RegSequencer( adcConfig );

            if( ADC_REQUEST_OK == retState )
            {
                /* Trigger source change keeps the previous edge, configured edge is written afterwards */
                retState = Adc_Set_TriggerSrc( adcConfig->PeriphId, adcConfig->RegTriggerId );
            }
            else
            {
                /* Previous step failed, error state is kept */
            }

            if( ( ADC_REQUEST_OK == retState ) &&
                ( ADC_REG_TRIGGER_SOFTWARE != adcConfig->RegTriggerId )    )
            {
                retState = Adc_Set_TriggerEdge( adcConfig->PeriphId, adcConfig->RegTriggerEdge );
            }
            else
            {
                /* Previous step failed, error state is kept */
            }

            if( ADC_REQUEST_OK == retState )
            {
                retState = Adc_Set_TriggerMode( adcConfig->PeriphId, adcConfig->RegTriggerMode );
            }
            else
            {
                /* Previous step failed, error state is kept */
            }
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        /* --- Injected group: sequencer, trigger source and trigger edge --- */
        if( ( ADC_REQUEST_OK == retState ) &&
            ( 0u < adcConfig->InjChannelsCnt )    )
        {
            retState = Adc_Set_InjSequencer( adcConfig );
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        /* --- Data transfer handler (DMA / ISR / POLL) incl. overrun behavior --- */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_XferInit( adcConfig->PeriphId, &adcConfig->DataConfig );
        }
        else
        {
            /* Previous step failed, error state is kept */
        }

        /* --- Initialization result --- */
        if( ( ADC_REQUEST_OK != retState ) &&
            ( ADC_FUNCTION_ACTIVE == initStarted )    )
        {
            /* Result is intentionally not evaluated, initialization is already reported as failed */
            (void)Adc_Deinit( adcConfig->PeriphId );
        }
        else
        {
            /* Initialization succeeded or the peripheral was not touched */
        }
    }
    else
    {
        /* Previous step failed, error state is kept */
    }

    return ( retState );
}


/**
 * \brief Selects the regular group conversion trigger source
 *
 * \note  If the required trigger source is already selected, no register is modified
 *        (trigger edge is kept).
 *
 * \note  On change to an external trigger source the currently configured trigger edge is
 *        kept. If the current trigger source is software (no edge configured), rising edge
 *        is used - call Adc_Set_TriggerEdge() afterwards if another edge is needed.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (ADSTART = 0 and JADSTART = 0). Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param triggerSrc [in]: Required regular group trigger source, value from \ref adc_RegTriggerId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_TriggerSrc( adc_PeriphId_t periphId, adc_RegTriggerId_t triggerSrc )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT      > periphId   ) &&
        ( ADC_REG_TRIGGER_CNT > triggerSrc )    )
    {
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

        if( ADC_REQUEST_OK == convState )
        {
            ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;
            const uint32_t      llSource  = adc_RegTriggerSrcLut[ triggerSrc ];
            const uint32_t      curSource = LL_ADC_REG_GetTriggerSource( periphReg );
            const uint32_t      curEdge   = LL_ADC_REG_GetTriggerEdge( periphReg );
            uint32_t            llEdge    = LL_ADC_REG_TRIG_SOFTWARE;

            if( llSource == curSource )
            {
                /* Trigger source is already selected, no change - trigger edge is kept */
                retState = ADC_REQUEST_OK;
            }
            else
            {
                if( ADC_REG_TRIGGER_SOFTWARE == triggerSrc )
                {
                    /* Software trigger - no edge (EXTEN = 0) */
                    llEdge = LL_ADC_REG_TRIG_SOFTWARE;
                }
                else if( LL_ADC_REG_TRIG_SOFTWARE == curEdge )
                {
                    /* Change from software trigger - no edge configured yet, rising edge is used */
                    llEdge = LL_ADC_REG_TRIG_EXT_RISING;
                }
                else
                {
                    /* Change between external triggers - current edge is kept */
                    llEdge = curEdge;
                }

                /* Trigger source and edge are written at once (EXTSEL + EXTEN), LL literal carries rising edge */
                LL_ADC_REG_SetTriggerSource( periphReg, ( llSource & ( ~LL_ADC_REG_TRIG_EXT_RISINGFALLING ) ) | llEdge );

                for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    const uint32_t sourceReg = LL_ADC_REG_GetTriggerSource( periphReg );
                    const uint32_t edgeReg   = LL_ADC_REG_GetTriggerEdge( periphReg );

                    if( ( llSource == sourceReg ) &&
                        ( llEdge   == edgeReg   )    )
                    {
                        retState = ADC_REQUEST_OK;
                        break;
                    }
                    else
                    {
                        /* Trigger source has not yet been applied, keep return state as error */
                        retState = ADC_REQUEST_ERROR;
                    }
                }
            }
        }
        else
        {
            /* Conversion is ongoing, configuration change is not allowed */
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
 * \brief Reads back the regular group conversion trigger source
 *
 * \note  Only the trigger source is compared, the configured edge does not influence the result.
 *
 * \param periphId    [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param triggerSrc [out]: Pointer to store the current trigger source (\ref adc_RegTriggerId_t).
 *                          Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_TriggerSrc( adc_PeriphId_t periphId, adc_RegTriggerId_t * const triggerSrc )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId   ) &&
        ( ADC_NULL_PTR  != triggerSrc )    )
    {
        uint32_t llTriggerSrc = LL_ADC_REG_GetTriggerSource( adc_PeriphConf[ periphId ].PeriphReg );

        for( adc_RegTriggerId_t triggerId = (adc_RegTriggerId_t)0u; ADC_REG_TRIGGER_CNT > triggerId; triggerId ++ )
        {
            if( adc_RegTriggerSrcLut[ triggerId ] == llTriggerSrc )
            {
                *triggerSrc = (adc_RegTriggerId_t)triggerId;
                retState    = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* No action required */
            }
        }
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/**
 * \brief Selects single vs. continuous regular group conversion mode
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (ADSTART = 0 and JADSTART = 0). Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId    [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param triggerMode [in]: Required conversion mode, value from \ref adc_RegTriggerMode_t
 *                          (ADC_REG_TRIGGER_MODE_SINGLE / ADC_REG_TRIGGER_MODE_CONTINUOUS)
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_TriggerMode( adc_PeriphId_t periphId, adc_RegTriggerMode_t triggerMode )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT       > periphId    ) &&
        ( ADC_REG_TRIGGER_MODE_CNT > triggerMode )    )
    {
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

        if( ADC_REQUEST_OK == convState )
        {
            ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;
            uint32_t            llConvMode = LL_ADC_REG_CONV_SINGLE;

            if( ADC_REG_TRIGGER_MODE_SINGLE == triggerMode )
            {
                llConvMode = LL_ADC_REG_CONV_SINGLE;
            }
            else
            {
                llConvMode = LL_ADC_REG_CONV_CONTINUOUS;
            }

            LL_ADC_REG_SetContinuousMode( periphReg, llConvMode );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t regValue = LL_ADC_REG_GetContinuousMode( periphReg );

                if( llConvMode == regValue )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Conversion mode has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Conversion is ongoing, configuration change is not allowed */
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
 * \brief Reads back the regular group conversion mode (single/continuous)
 *
 * \param periphId     [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param triggerMode [out]: Pointer to store the current conversion mode (\ref adc_RegTriggerMode_t).
 *                           Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_TriggerMode( adc_PeriphId_t periphId, adc_RegTriggerMode_t * const triggerMode )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_NULL_PTR != triggerMode )    )
    {
        const uint32_t llConvMode = LL_ADC_REG_GetContinuousMode( adc_PeriphConf[ periphId ].PeriphReg );

        if( LL_ADC_REG_CONV_CONTINUOUS == llConvMode )
        {
            *triggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
        }
        else
        {
            *triggerMode = ADC_REG_TRIGGER_MODE_SINGLE;
        }

        retState = ADC_REQUEST_OK;
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/**
 * \brief Selects the regular group external trigger active edge
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (ADSTART = 0 and JADSTART = 0). Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \pre   Regular group trigger source must be external (see Adc_Set_TriggerSrc()).
 *        For software trigger (EXTEN = 0) the request is rejected: \ref ADC_REQUEST_ERROR
 *        is returned and no register is modified (writing an edge would enable the
 *        external trigger).
 *
 * \param periphId    [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param triggerEdge [in]: Required external trigger edge, value from \ref adc_TriggerEdge_t
 *                          (ADC_TRIGGER_EDGE_RISING / ADC_TRIGGER_EDGE_FALLING / ADC_TRIGGER_EDGE_BOTH)
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_TriggerEdge( adc_PeriphId_t periphId, adc_TriggerEdge_t triggerEdge )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_TRIGGER_EDGE_CNT > triggerEdge )    )
    {
        ADC_TypeDef * const      periphReg = adc_PeriphConf[ periphId ].PeriphReg;
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );
        const uint32_t           swTrigger = LL_ADC_REG_IsTriggerSourceSWStart( periphReg );

        if( ADC_REQUEST_OK != convState )
        {
            /* Conversion is ongoing, configuration change is not allowed */
            retState = ADC_REQUEST_ERROR;
        }
        else if( 0u != swTrigger )
        {
            /* Software trigger is selected, trigger edge is not applicable */
            retState = ADC_REQUEST_ERROR;
        }
        else
        {
            LL_ADC_REG_SetTriggerEdge( periphReg, adc_RegTriggerEdgeLut[ triggerEdge ] );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t regValue = LL_ADC_REG_GetTriggerEdge( periphReg );

                if( adc_RegTriggerEdgeLut[ triggerEdge ] == regValue )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Trigger edge has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
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
 * \brief Reads back the regular group external trigger active edge
 *
 * \note  For software trigger (EXTEN = 0) no edge is configured and \ref ADC_REQUEST_ERROR
 *        is returned.
 *
 * \param periphId     [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param triggerEdge [out]: Pointer to store the current external trigger edge (\ref adc_TriggerEdge_t).
 *                           Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_TriggerEdge( adc_PeriphId_t periphId, adc_TriggerEdge_t * const triggerEdge )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_NULL_PTR != triggerEdge )    )
    {
        uint32_t llEdge = LL_ADC_REG_GetTriggerEdge( adc_PeriphConf[ periphId ].PeriphReg );

        if( LL_ADC_REG_TRIG_EXT_RISING == llEdge )
        {
            *triggerEdge = ADC_TRIGGER_EDGE_RISING;
            retState     = ADC_REQUEST_OK;
        }
        else if( LL_ADC_REG_TRIG_EXT_FALLING == llEdge )
        {
            *triggerEdge = ADC_TRIGGER_EDGE_FALLING;
            retState     = ADC_REQUEST_OK;
        }
        else if( LL_ADC_REG_TRIG_EXT_RISINGFALLING == llEdge )
        {
            *triggerEdge = ADC_TRIGGER_EDGE_BOTH;
            retState     = ADC_REQUEST_OK;
        }
        else
        {
            retState = ADC_REQUEST_ERROR;
        }
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* -------------------------- Peripheral control ---------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Calibrates and enables an ADC peripheral (ADEN = 1, waits for ADRDY)
 *
 * Single-ended calibration is always run, differential calibration additionally if any
 * channel of the peripheral is configured as differential (DIFSEL). Calibration factors are
 * kept while the internal regulator is enabled, so calibrating on every enable keeps them
 * consistent with the current channel configuration.
 *
 * \note  ADEN set less than 4 ADC clock cycles after end of calibration is reset by the
 *        calibration logic - ADEN is set again until ADRDY is raised (no fixed delay).
 *
 * \pre   ADC peripheral must be disabled (ADEN = 0, no disable or calibration ongoing), its
 *        internal voltage regulator enabled (see Adc_PeriphInit()) and the active ADC kernel
 *        clock frequency within ADC_CLK_FREQ_MIN_HZ - ADC_CLK_FREQ_MAX_HZ. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_PeriphActive( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const      periphReg   = adc_PeriphConf[ periphId ].PeriphReg;
        adc_ClkDiv_t             clkDiv      = ADC_CLK_DIV_CNT;
        const adc_RequestState_t periphState = Adc_Check_PeriphDisabled( periphId );
        const uint32_t           regulatorOn = LL_ADC_IsInternalRegulatorEnabled( periphReg );
        const adc_RequestState_t divState    = Adc_Get_ClockDivider( &clkDiv );
        const adc_RequestState_t clkState    = Adc_Check_ClockFreq( adc_ShadowClkSrc, clkDiv );

        if( ADC_REQUEST_OK != periphState )
        {
            /* ADC is already enabled or disable/calibration procedure is ongoing */
            retState = ADC_REQUEST_ERROR;
        }
        else if( 0u == regulatorOn )
        {
            /* Internal regulator is not enabled, peripheral was not initialized */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ( ADC_REQUEST_OK != divState ) ||
                 ( ADC_REQUEST_OK != clkState )    )
        {
            /* ADC kernel clock frequency is out of the allowed range */
            retState = ADC_REQUEST_ERROR;
        }
        else
        {
            /* --- Self-calibration (offset and linearity, common for single-ended and differential inputs) --- */
            retState = Adc_Set_Calibration( periphId );

            /* --- Enable ADC and wait for ready flag --- */
            if( ADC_REQUEST_OK == retState )
            {
                retState = Adc_Set_EnableWait( periphId );
            }
            else
            {
                /* Calibration failed, ADC is not enabled */
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
 * \brief Disables an ADC peripheral (ADDIS, waits until ADEN = 0)
 *
 * \note  Already disabled peripheral is accepted (waits only for a pending disable procedure).
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (ADSTART = 0 and JADSTART = 0, see Adc_Set_RegStop() / Adc_Set_InjStop()).
 *        Otherwise \ref ADC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_PeriphInactive( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

        if( ADC_REQUEST_OK == convState )
        {
            ADC_TypeDef * const periphReg     = adc_PeriphConf[ periphId ].PeriphReg;
            const uint32_t      periphEnabled = LL_ADC_IsEnabled( periphReg );

            if( 0u != periphEnabled )
            {
                LL_ADC_Disable( periphReg );
            }
            else
            {
                /* ADC is already disabled, only a pending disable procedure is waited for */
            }

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t enabledReg     = LL_ADC_IsEnabled( periphReg );
                const uint32_t disableOngoing = LL_ADC_IsDisableOngoing( periphReg );

                if( ( 0u == enabledReg ) &&
                    ( 0u == disableOngoing )    )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* ADC disable has not yet been finished, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Conversion is ongoing, ADC can not be disabled */
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
 * \brief Starts the regular group conversion (ADSTART = 1)
 *
 * For software trigger the conversion starts immediately, for an external trigger the ADC
 * starts to accept trigger events.
 *
 * \note  Regular data transfer (DataConfig) is armed first: if it is not running, the buffer is
 *        filled from DataBuffer[ 0 ]. If it is already running (buffer not yet full, e.g. single
 *        mode with several software starts), it continues at the current position.
 *
 * \note  ADSTART is cleared by HW at the end of the conversion sequence (single mode), so the
 *        write can not be verified by read-back. Conversion progress is signalled by
 *        \ref ADC_FLAG_REG_EOC / \ref ADC_FLAG_REG_EOS (see Adc_Get_Flag()).
 *
 * \pre   ADC peripheral must be enabled and ready (ADEN = 1, ADRDY = 1, no disable ongoing) and
 *        no regular conversion may be ongoing (ADSTART = 0). Otherwise \ref ADC_REQUEST_ERROR
 *        is returned and no register is modified.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_RegStart( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg      = adc_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      periphEnabled  = LL_ADC_IsEnabled( periphReg );
        const uint32_t      periphReady    = LL_ADC_IsActiveFlag_ADRDY( periphReg );
        const uint32_t      disableOngoing = LL_ADC_IsDisableOngoing( periphReg );
        const uint32_t      convOngoing    = LL_ADC_REG_IsConversionOngoing( periphReg );

        if( ( 0u != periphEnabled  ) &&
            ( 0u != periphReady    ) &&
            ( 0u == disableOngoing ) &&
            ( 0u == convOngoing    )    )
        {
            /* Regular data transfer (DMA / ISR / POLL) is armed before the conversion starts */
            retState = Adc_Set_XferStart( periphId );

            if( ADC_REQUEST_OK == retState )
            {
                LL_ADC_REG_StartConversion( periphReg );
            }
            else
            {
                /* Data transfer could not be armed, conversion is not started */
            }
        }
        else
        {
            /* ADC must be enabled and idle before a new conversion can be started */
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
 * \brief Stops the regular group conversion (ADSTP, waits until ADSTART = 0) and the regular
 *        data transfer (DMA / ISR / POLL)
 *
 * \note  If no regular conversion is ongoing, the conversion is not touched and only the data
 *        transfer is stopped.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_RegStop( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg   = adc_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      convOngoing = LL_ADC_REG_IsConversionOngoing( periphReg );

        if( 0u != convOngoing )
        {
            LL_ADC_REG_StopConversion( periphReg );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t stopOngoing = LL_ADC_REG_IsStopConversionOngoing( periphReg );
                const uint32_t startReg    = LL_ADC_REG_IsConversionOngoing( periphReg );

                if( ( 0u == stopOngoing ) &&
                    ( 0u == startReg )    )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Conversion stop has not yet been finished, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* No regular conversion is ongoing, nothing to stop */
            retState = ADC_REQUEST_OK;
        }

        /* Regular data transfer (DMA / ISR / POLL) is stopped after the conversion */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_XferStop( periphId );
        }
        else
        {
            /* Conversion could not be stopped, data transfer is kept running */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts the injected group conversion (JADSTART = 1)
 *
 * For software trigger the conversion starts immediately, for an external trigger the ADC
 * starts to accept trigger events.
 *
 * \note  JADSTART is cleared by HW at the end of the conversion sequence, so the write can not
 *        be verified by read-back. Conversion progress is signalled by \ref ADC_FLAG_INJ_EOC /
 *        \ref ADC_FLAG_INJ_EOS (see Adc_Get_Flag()).
 *
 * \pre   ADC peripheral must be enabled and ready (ADEN = 1, ADRDY = 1, no disable ongoing),
 *        no injected conversion may be ongoing (JADSTART = 0) and auto-injected mode must be
 *        disabled (JAUTO = 0). Otherwise \ref ADC_REQUEST_ERROR is returned and no register
 *        is modified. In auto-injected mode (ADC_INJ_TRIGGER_AUTO) the injected group is started
 *        together with the regular group by Adc_Set_RegStart().
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_InjStart( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg      = adc_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      periphEnabled  = LL_ADC_IsEnabled( periphReg );
        const uint32_t      periphReady    = LL_ADC_IsActiveFlag_ADRDY( periphReg );
        const uint32_t      disableOngoing = LL_ADC_IsDisableOngoing( periphReg );
        const uint32_t      convOngoing    = LL_ADC_INJ_IsConversionOngoing( periphReg );
        const uint32_t      autoInjected   = LL_ADC_INJ_GetTrigAuto( periphReg );

        if( ( 0u                          != periphEnabled  ) &&
            ( 0u                          != periphReady    ) &&
            ( 0u                          == disableOngoing ) &&
            ( 0u                          == convOngoing    ) &&
            ( LL_ADC_INJ_TRIG_INDEPENDENT == autoInjected   )    )
        {
            LL_ADC_INJ_StartConversion( periphReg );
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* ADC must be enabled and idle, injected group must not be in auto-injected mode */
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
 * \brief Stops the injected group conversion (JADSTP, waits until JADSTART = 0)
 *
 * \note  If no injected conversion is ongoing, no register is modified and
 *        \ref ADC_REQUEST_OK is returned.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_InjStop( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg   = adc_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      convOngoing = LL_ADC_INJ_IsConversionOngoing( periphReg );

        if( 0u != convOngoing )
        {
            LL_ADC_INJ_StopConversion( periphReg );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t stopOngoing = LL_ADC_INJ_IsStopConversionOngoing( periphReg );
                const uint32_t startReg    = LL_ADC_INJ_IsConversionOngoing( periphReg );

                if( ( 0u == stopOngoing ) &&
                    ( 0u == startReg )    )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Conversion stop has not yet been finished, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* No injected conversion is ongoing, nothing to stop */
            retState = ADC_REQUEST_OK;
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ---------------------------- Conversion data ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Reads the last regular group conversion result
 *
 * \note  Reading of the data register clears \ref ADC_FLAG_REG_EOC. The function does not wait
 *        for the conversion end - use Adc_Get_Flag() to check \ref ADC_FLAG_REG_EOC before.
 *
 * \note  While the regular data transfer is running (DMA / ISR / POLL with DataBuffer), the
 *        result belongs to the transfer - 0 and \ref ADC_REQUEST_ERROR are returned and the
 *        data register is not read.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param data    [out]: Pointer to store the conversion result (RAW value). Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_RegData( adc_PeriphId_t periphId, adc_Data_t * const data )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_NULL_PTR != data )    )
    {
        if( ADC_FUNCTION_ACTIVE == adc_XferContext[ periphId ].XferState )
        {
            /* Result is owned by the running data transfer */
            *data    = 0u;
            retState = ADC_REQUEST_ERROR;
        }
        else
        {
            *data    = (adc_Data_t)LL_ADC_REG_ReadConversionData32( adc_PeriphConf[ periphId ].PeriphReg );
            retState = ADC_REQUEST_OK;
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the last injected group conversion result of a rank
 *
 * \note  The function does not wait for the conversion end - use Adc_Get_Flag() to check
 *        \ref ADC_FLAG_INJ_EOS (or \ref ADC_FLAG_INJ_EOC) before.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param rankId   [in]: Injected rank, value from \ref adc_InjSequenceId_t
 * \param data    [out]: Pointer to store the conversion result (RAW value). Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_InjData( adc_PeriphId_t periphId, adc_InjSequenceId_t rankId, adc_Data_t * const data )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT       > periphId ) &&
        ( ADC_INJ_SEQUENCE_CNT > rankId   ) &&
        ( ADC_NULL_PTR        != data     )    )
    {
        *data    = (adc_Data_t)LL_ADC_INJ_ReadConversionData32( adc_PeriphConf[ periphId ].PeriphReg, adc_InjSeqRankLut[ rankId ] );
        retState = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Changes the regular group data transfer configuration (mode, buffer, callbacks)
 *
 * The previous data transfer handler is released (DMA channel, interrupts), overrun behavior
 * of the new mode is set and the new handler is initialized. The transfer starts with the
 * next Adc_Set_RegStart().
 *
 * \note  The configuration is validated as for a used regular group - DMA / ISR mode require
 *        DataBuffer, POLL mode allows DataBuffer == NULL (manual polling by Adc_Get_RegData()).
 *        The configuration is copied, dataConfig may be a temporary variable.
 *
 * \pre   No regular or injected conversion may be ongoing (ADSTART = 0 and JADSTART = 0) and
 *        the regular data transfer must not be running (see Adc_Set_RegStop()). Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and nothing is changed.
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param dataConfig [in]: Pointer to data transfer configuration \ref adc_DataConfig_t.
 *                         Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_DataConfig( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId   ) &&
        ( ADC_NULL_PTR  != dataConfig )    )
    {
        adc_XferContext_t * const xferCtx     = &adc_XferContext[ periphId ];
        const adc_RequestState_t  convState   = Adc_Check_ConversionStopped( periphId );
        const adc_RequestState_t  configState = Adc_Check_DataConfig( periphId, ADC_FUNCTION_ACTIVE, dataConfig );

        if( ADC_REQUEST_OK != convState )
        {
            /* Conversion is ongoing, configuration change is not allowed */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_FUNCTION_ACTIVE == xferCtx->XferState )
        {
            /* Data transfer is running (buffer not yet full), Adc_Set_RegStop() is required first */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_REQUEST_OK != configState )
        {
            /* New configuration is invalid */
            retState = ADC_REQUEST_ERROR;
        }
        else
        {
            /* --- Release resources of the previous transfer mode --- */
            if( ADC_FUNCTION_ACTIVE == xferCtx->InitState )
            {
                retState = adc_XferModeLut[ xferCtx->Config.TransferMode ].Deinit( periphId );

                if( ADC_REQUEST_OK == retState )
                {
                    xferCtx->InitState = ADC_FUNCTION_INACTIVE;
                }
                else
                {
                    /* Previous transfer mode resources could not be released */
                }
            }
            else
            {
                /* No transfer mode is initialized */
                retState = ADC_REQUEST_OK;
            }

            /* --- Initialize the new transfer mode --- */
            if( ADC_REQUEST_OK == retState )
            {
                retState = Adc_Set_XferInit( periphId, dataConfig );
            }
            else
            {
                /* New transfer mode is not initialized */
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
 * \brief Returns the active regular group data transfer configuration
 *
 * \param periphId    [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param dataConfig [out]: Pointer to store the data transfer configuration \ref adc_DataConfig_t.
 *                          Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request was processed
 *         without problems. Otherwise (also if no data transfer is initialized) returns
 *         \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_DataConfig( adc_PeriphId_t periphId, adc_DataConfig_t * const dataConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId   ) &&
        ( ADC_NULL_PTR  != dataConfig )    )
    {
        if( ADC_FUNCTION_ACTIVE == adc_XferContext[ periphId ].InitState )
        {
            *dataConfig = adc_XferContext[ periphId ].Config;
            retState    = ADC_REQUEST_OK;
        }
        else
        {
            /* Data transfer is not initialized */
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
 * \brief Reads the state of an ADC event flag
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param flagId     [in]: Event flag identification, value from \ref adc_FlagId_t
 * \param flagState [out]: Pointer to store the flag state (\ref adc_FlagState_t). Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_Flag( adc_PeriphId_t periphId, adc_FlagId_t flagId, adc_FlagState_t * const flagState )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId  ) &&
        ( ADC_FLAG_CNT   > flagId    ) &&
        ( ADC_NULL_PTR  != flagState )    )
    {
        const uint32_t flagReg = READ_BIT( adc_PeriphConf[ periphId ].PeriphReg->ISR, adc_FlagLut[ flagId ] );

        if( 0u != flagReg )
        {
            *flagState = ADC_FLAG_ACTIVE;
        }
        else
        {
            *flagState = ADC_FLAG_INACTIVE;
        }

        retState = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Clears an ADC event flag
 *
 * \note  Event flags are set by HW at any time (e.g. every conversion end in continuous mode),
 *        so the clear can not be verified by read-back.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param flagId   [in]: Event flag identification, value from \ref adc_FlagId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Clear_Flag( adc_PeriphId_t periphId, adc_FlagId_t flagId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_FLAG_CNT > flagId )    )
    {
        /* ISR bits are cleared by writing 1, other bits are not affected */
        WRITE_REG( adc_PeriphConf[ periphId ].PeriphReg->ISR, adc_FlagLut[ flagId ] );
        retState = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* -------------------------- Channel configuration -------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Selects the ADC resolution (common for all channels of the peripheral)
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (ADSTART = 0 and JADSTART = 0). Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelRes [in]: Required resolution, value from \ref adc_Resolution_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_Resolution( adc_PeriphId_t periphId, adc_Resolution_t channelRes )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT     > periphId   ) &&
        ( ADC_RESOLUTION_CNT > channelRes )    )
    {
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

        if( ADC_REQUEST_OK == convState )
        {
            ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

            LL_ADC_SetResolution( periphReg, adc_ResolutionLut[ channelRes ] );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t regValue = LL_ADC_GetResolution( periphReg );

                if( adc_ResolutionLut[ channelRes ] == regValue )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Resolution has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Conversion is ongoing, configuration change is not allowed */
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
 * \brief Reads back the ADC resolution (common for all channels of the peripheral)
 *
 * \param periphId    [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelRes [out]: Pointer to store the current resolution (\ref adc_Resolution_t).
 *                          Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_Resolution( adc_PeriphId_t periphId, adc_Resolution_t * const channelRes )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_NULL_PTR != channelRes )    )
    {
        uint32_t llResolution = LL_ADC_GetResolution( adc_PeriphConf[ periphId ].PeriphReg );
        adc_Resolution_t idx;

        for( idx = ADC_RESOLUTION_12BIT; ADC_RESOLUTION_CNT > idx; idx ++ )
        {
            if( adc_ResolutionLut[ idx ] == llResolution )
            {
                *channelRes = idx;
                retState    = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* No action required */
            }
        }
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/**
 * \brief Selects a channel's sampling time
 *
 * \note  Sampling time must satisfy the minimum sampling time of the input currently selected
 *        for the channel (see Adc_Set_ChannelInput(), Adc_Check_SamplingTime()) at the active
 *        ADC kernel clock. Otherwise \ref ADC_REQUEST_ERROR is returned and no register is modified.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (ADSTART = 0 and JADSTART = 0). Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId     [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelId    [in]: ADC channel identification, value from \ref adc_ChannelId_t
 * \param samplingTime [in]: Required sampling time, value from \ref adc_ChannelSampling_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_SamplingTime( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelSampling_t samplingTime )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT           > periphId     ) &&
        ( ADC_CHANNEL_CNT          > channelId    ) &&
        ( ADC_CHANNEL_SAMPLING_CNT > samplingTime )    )
    {
        const adc_RequestState_t convState     = Adc_Check_ConversionStopped( periphId );
        const adc_RequestState_t samplingState = Adc_Check_SamplingTime( adc_ShadowChannelInput[ periphId ][ channelId ], samplingTime );

        if( ADC_REQUEST_OK != samplingState )
        {
            /* Sampling time is shorter than required by the channel input */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_REQUEST_OK == convState )
        {
            ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;
            const uint32_t      llChannel = __LL_ADC_DECIMAL_NB_TO_CHANNEL( channelId );

            LL_ADC_SetChannelSamplingTime( periphReg, llChannel, adc_SamplingTimeLut[ samplingTime ] );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t regValue = LL_ADC_GetChannelSamplingTime( periphReg, llChannel );

                if( adc_SamplingTimeLut[ samplingTime ] == regValue )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Sampling time has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Conversion is ongoing, configuration change is not allowed */
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
 * \brief Reads back a channel's sampling time
 *
 * \param periphId      [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelId     [in]: ADC channel identification, value from \ref adc_ChannelId_t
 * \param samplingTime [out]: Pointer to store the current sampling time (\ref adc_ChannelSampling_t).
 *                            Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_SamplingTime( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelSampling_t * const samplingTime )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;
    uint32_t            llChannel;

    if( ( ADC_PERIPH_CNT  > periphId     ) &&
        ( ADC_CHANNEL_CNT > channelId    ) &&
        ( ADC_NULL_PTR   != samplingTime )    )
    {
        llChannel = __LL_ADC_DECIMAL_NB_TO_CHANNEL( channelId );

        uint32_t llSamplingTime = LL_ADC_GetChannelSamplingTime( adc_PeriphConf[ periphId ].PeriphReg, llChannel );
        adc_ChannelSampling_t idx;

        retState = ADC_REQUEST_ERROR;

        for( idx = ADC_CHANNEL_SAMPLING_5_CYCLES; ADC_CHANNEL_SAMPLING_CNT > idx; idx ++ )
        {
            if( adc_SamplingTimeLut[ idx ] == llSamplingTime )
            {
                *samplingTime = idx;
                retState      = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* No action required */
            }
        }
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/**
 * \brief Selects which physical/internal signal a channel samples
 *
 * For ADC_CHANNEL_INPUT_PIN_SINGLE / ADC_CHANNEL_INPUT_PIN_DIFF, the GPIO pin(s)
 * wired to this channel/peripheral (adc_GpioPeriphConfig[]) are configured to
 * analog mode with no pull. For the internal signals (TEMP/VREF/VBAT), see
 * Adc_Set_InternalInput().
 *
 * \note  Input must be available for the channel on the peripheral (see Adc_Check_ChannelInput()):
 *        GPIO pin(s) wired for PIN_SINGLE / PIN_DIFF, internal signal connected to channelId
 *        for the internal inputs. Otherwise \ref ADC_REQUEST_ERROR is returned.
 *
 * \pre   ADC peripheral must be disabled (ADEN = 0, no disable or calibration ongoing).
 *        Otherwise \ref ADC_REQUEST_ERROR is returned and no register is modified.
 *        DIFSEL is writable only while ADEN = 0; GPIO pins are not touched either.
 *
 * \param periphId     [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelId    [in]: ADC channel identification, value from \ref adc_ChannelId_t
 * \param channelInput [in]: Required channel input, value from \ref adc_ChannelInput_t
 *                           (external pin single/differential or internal TEMP/VREF/VBAT)
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_ChannelInput( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    uint32_t            llChannel;

    /* Range check of all parameters and availability of the input for the channel */
    const adc_RequestState_t inputState = Adc_Check_ChannelInput( periphId, channelId, channelInput );

    if( ADC_REQUEST_OK == inputState )
    {
        const adc_RequestState_t periphState = Adc_Check_PeriphDisabled( periphId );

        if( ADC_REQUEST_OK == periphState )
        {
            if( ( ADC_CHANNEL_INPUT_PIN_SINGLE == channelInput ) ||
                ( ADC_CHANNEL_INPUT_PIN_DIFF == channelInput )    )
            {
                const adc_GpioConfig_t * const pinP = &adc_GpioPeriphConfig[ periphId ].Channel[ channelId ].ChannelInP;
                const adc_GpioConfig_t * const pinN = &adc_GpioPeriphConfig[ periphId ].Channel[ channelId ].ChannelInN;

                llChannel = __LL_ADC_DECIMAL_NB_TO_CHANNEL( channelId );

                /* Positive input pin (availability checked by Adc_Check_ChannelInput()) */
                retState = Adc_Set_PinAnalog( pinP );

                /* Negative input pin of a differential pair */
                if( ( ADC_REQUEST_OK == retState ) &&
                    ( ADC_CHANNEL_INPUT_PIN_DIFF == channelInput )    )
                {
                    retState = Adc_Set_PinAnalog( pinN );
                }
                else
                {
                    /* Previous step failed, error state is kept */
                }

                if( ADC_REQUEST_OK == retState )
                {
                    ADC_TypeDef * const periphReg    = adc_PeriphConf[ periphId ].PeriphReg;
                    uint32_t            llSingleDiff = LL_ADC_SINGLE_ENDED;

                    if( ADC_CHANNEL_INPUT_PIN_DIFF == channelInput )
                    {
                        llSingleDiff = LL_ADC_DIFFERENTIAL_ENDED;
                    }
                    else
                    {
                        llSingleDiff = LL_ADC_SINGLE_ENDED;
                    }

                    LL_ADC_SetChannelSingleDiff( periphReg, llChannel, llSingleDiff );

                    for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                    {
                        const uint32_t difselReg = LL_ADC_GetChannelSingleDiff( periphReg, llChannel );

                        if( ( ( ADC_CHANNEL_INPUT_PIN_DIFF == channelInput ) &&
                              ( 0u                         != difselReg    ) ) ||
                            ( ( ADC_CHANNEL_INPUT_PIN_DIFF != channelInput ) &&
                              ( 0u                         == difselReg    ) )    )
                        {
                            retState = ADC_REQUEST_OK;
                            break;
                        }
                        else
                        {
                            /* Single/differential mode has not yet been applied, keep return state as error */
                            retState = ADC_REQUEST_ERROR;
                        }
                    }
                }
                else
                {
                    /* Previous step failed, error state is kept */
                }
            }
            else
            {
                retState = Adc_Set_InternalInput( periphId, channelInput );
            }

            /* STM32U5 ADC1 / ADC2: only preselected channels are converted (PCSEL) */
            if( ADC_REQUEST_OK == retState )
            {
                retState = Adc_Set_ChannelPreselection( periphId, channelId );
            }
            else
            {
                /* Channel input was not configured */
            }

            if( ADC_REQUEST_OK == retState )
            {
                adc_ShadowChannelInput[ periphId ][ channelId ] = channelInput;
            }
            else
            {
                /* Shadow keeps the previous input */
            }
        }
        else
        {
            /* ADC peripheral is enabled, configuration change is not allowed */
            retState = ADC_REQUEST_ERROR;
        }
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/**
 * \brief Returns the last input type selected via Adc_Set_ChannelInput()
 *
 * \note See the comment on adc_ShadowChannelInput: this is a shadow value.
 *
 * \param periphId      [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelId     [in]: ADC channel identification, value from \ref adc_ChannelId_t
 * \param channelInput [out]: Pointer to store the last selected channel input (\ref adc_ChannelInput_t).
 *                            Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_ChannelInput( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelInput_t * const channelInput )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_CHANNEL_CNT > channelId ) &&
        ( ADC_NULL_PTR != channelInput )    )
    {
        *channelInput = adc_ShadowChannelInput[ periphId ][ channelId ];
        retState      = ADC_REQUEST_OK;
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* --------------------- Analog Watch-Dog configuration --------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Initializes an Analog Watch-dog: monitored channel group, thresholds
 *        and (optionally) event filtering
 *
 * Supported modes:
 * - ADC_AWD_1: ADC_AWD_MODE_ALL, ADC_AWD_MODE_ALL_REGULAR, ADC_AWD_MODE_ALL_INJECTED
 * - ADC_AWD_2 / ADC_AWD_3: ADC_AWD_MODE_ALL only (HW does not distinguish between regular
 *   and injected group for these watch-dogs)
 *
 * \note  adc_AwdConfig_t carries no channel selector, so ADC_AWD_MODE_SINGLE,
 *        ADC_AWD_MODE_SINGLE_REGULAR and ADC_AWD_MODE_SINGLE_INJECTED cannot be configured
 *        without fabricating a channel choice - reported as an error rather than guessed.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (ADSTART = 0 and JADSTART = 0). Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId  [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param awdConfig [in]: Pointer to Analog Watch-dog configuration structure \ref adc_AwdConfig_t.
 *                        Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_AwdInit( adc_PeriphId_t periphId, adc_AwdConfig_t * const awdConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;
    uint32_t           llGroup  = 0u;

    if( ( ADC_PERIPH_CNT > periphId  ) &&
        ( ADC_NULL_PTR  != awdConfig )    )
    {
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

        if( ADC_REQUEST_OK == convState )
        {
            if( ADC_AWD_CNT > awdConfig->AwdId )
            {
                ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

                if( ADC_AWD_MODE_ALL == awdConfig->AwdMode )
                {
                    llGroup = LL_ADC_AWD_ALL_CHANNELS_REG_INJ;
                }
                else if( ( ADC_AWD_1                == awdConfig->AwdId   ) &&
                         ( ADC_AWD_MODE_ALL_REGULAR == awdConfig->AwdMode )    )
                {
                    llGroup = LL_ADC_AWD_ALL_CHANNELS_REG;
                }
                else if( ( ADC_AWD_1                 == awdConfig->AwdId   ) &&
                         ( ADC_AWD_MODE_ALL_INJECTED == awdConfig->AwdMode )    )
                {
                    llGroup = LL_ADC_AWD_ALL_CHANNELS_INJ;
                }
                else
                {
                    /* Single-channel modes need a channel selector which adc_AwdConfig_t does not carry,
                     * AWD2/AWD3 cannot distinguish between regular and injected group */
                    llGroup = 0u;
                }

                if( 0u != llGroup )
                {
                    LL_ADC_SetAnalogWDMonitChannels( periphReg, adc_AwdIdLut[ awdConfig->AwdId ], llGroup );

                    for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                    {
                        const uint32_t regValue = LL_ADC_GetAnalogWDMonitChannels( periphReg, adc_AwdIdLut[ awdConfig->AwdId ] );

                        if( llGroup == regValue )
                        {
                            retState = ADC_REQUEST_OK;
                            break;
                        }
                        else
                        {
                            /* Monitored channel group has not yet been applied, keep return state as error */
                            retState = ADC_REQUEST_ERROR;
                        }
                    }
                }
                else
                {
                    retState = ADC_REQUEST_ERROR;
                }

                /* Register write and read-back verification is done by called functions */
                if( ADC_REQUEST_OK == retState )
                {
                    retState = Adc_Set_AwdThresholds( periphId, awdConfig->AwdId, awdConfig->AwdLowThreshold, awdConfig->AwdHighThreshold );
                }
                else
                {
                    /* Previous step failed, error state is kept */
                }

                if( ADC_REQUEST_OK == retState )
                {
                    retState = Adc_Set_AwdFilter( periphId, awdConfig->AwdId, awdConfig->AwdFilter );
                }
                else
                {
                    /* Previous step failed, error state is kept */
                }
            }
            else
            {
                /* No action required */
            }
        }
        else
        {
            /* Conversion is ongoing, configuration change is not allowed */
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
 * \brief Sets an Analog Watch-dog's low/high comparison thresholds
 *
 * Thresholds are raw ADC values of the configured resolution and are converted to
 * the register format of the watch-dog:
 * - AWD1 compares 12 bit values - thresholds are left aligned to 12 bits.
 * - AWD2 / AWD3 compare the 8 most significant bits of the result - thresholds
 *   are converted to 8 bits, lower bits of 12 / 10 bit thresholds are ignored by
 *   HW (\ref Adc_Get_AwdThresholds returns the effective thresholds).
 *
 * \pre   Resolution of the peripheral shall be configured, conversion is done with
 *        the resolution at the time of the call.
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (ADSTART = 0 and JADSTART = 0). Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId      [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param awdId         [in]: Analog Watch-dog identification, value from \ref adc_AwdId_t
 * \param lowThreshold  [in]: Lower comparison threshold as raw ADC value (\ref adc_AwdThreshold_t)
 * \param highThreshold [in]: Upper comparison threshold as raw ADC value (\ref adc_AwdThreshold_t)
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise (also threshold above the
 *         maximal value of the resolution) returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_AwdThresholds( adc_PeriphId_t periphId,
                                          adc_AwdId_t awdId,
                                          adc_AwdThreshold_t lowThreshold,
                                          adc_AwdThreshold_t highThreshold )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_AWD_CNT    > awdId    )    )
    {
        ADC_TypeDef * const      periphReg  = adc_PeriphConf[ periphId ].PeriphReg;
        const adc_RequestState_t convState  = Adc_Check_ConversionStopped( periphId );
        const uint32_t           resShift   = ADC_AWD_RES_SHIFT( LL_ADC_GetResolution( periphReg ) );
        const uint32_t           rawMax     = ADC_AWD1_THRESHOLD_MAX >> resShift;
        const uint32_t           highRegVal = Adc_Get_AwdThresholdReg( awdId, resShift, (uint32_t)highThreshold );
        const uint32_t           lowRegVal  = Adc_Get_AwdThresholdReg( awdId, resShift, (uint32_t)lowThreshold  );

        if( ( ADC_REQUEST_OK == convState              ) &&
            ( rawMax         >= (uint32_t)highThreshold ) &&
            ( rawMax         >= (uint32_t)lowThreshold  )    )
        {
            LL_ADC_ConfigAnalogWDThresholds( periphReg, adc_AwdIdLut[ awdId ], highRegVal, lowRegVal );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t highThresholdReg = LL_ADC_GetAnalogWDThresholds( periphReg, adc_AwdIdLut[ awdId ], LL_ADC_AWD_THRESHOLD_HIGH );
                const uint32_t lowThresholdReg  = LL_ADC_GetAnalogWDThresholds( periphReg, adc_AwdIdLut[ awdId ], LL_ADC_AWD_THRESHOLD_LOW  );

                if( ( highRegVal == highThresholdReg ) &&
                    ( lowRegVal  == lowThresholdReg  )    )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Thresholds have not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Conversion is ongoing or threshold out of range of the resolution */
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
 * \brief Reads back an Analog Watch-dog's low/high comparison thresholds
 *
 * Thresholds are returned as raw ADC values of the current resolution (effective
 * thresholds - AWD2 / AWD3 compare only 8 most significant bits, see
 * \ref Adc_Set_AwdThresholds).
 *
 * \param periphId       [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param awdId          [in]: Analog Watch-dog identification, value from \ref adc_AwdId_t
 * \param lowThreshold  [out]: Pointer to store the lower comparison threshold. Must not be NULL.
 * \param highThreshold [out]: Pointer to store the upper comparison threshold. Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_AwdThresholds( adc_PeriphId_t periphId,
                                          adc_AwdId_t awdId,
                                          adc_AwdThreshold_t * const lowThreshold,
                                          adc_AwdThreshold_t * const highThreshold )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId      ) &&
        ( ADC_AWD_CNT    > awdId         ) &&
        ( ADC_NULL_PTR  != lowThreshold  ) &&
        ( ADC_NULL_PTR  != highThreshold )    )
    {
        ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      resShift  = ADC_AWD_RES_SHIFT( LL_ADC_GetResolution( periphReg ) );

        *highThreshold = (adc_AwdThreshold_t)Adc_Get_AwdThresholdRaw( awdId, resShift, LL_ADC_GetAnalogWDThresholds( periphReg, adc_AwdIdLut[ awdId ], LL_ADC_AWD_THRESHOLD_HIGH ) );
        *lowThreshold  = (adc_AwdThreshold_t)Adc_Get_AwdThresholdRaw( awdId, resShift, LL_ADC_GetAnalogWDThresholds( periphReg, adc_AwdIdLut[ awdId ], LL_ADC_AWD_THRESHOLD_LOW  ) );
        retState       = ADC_REQUEST_OK;
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/**
 * \brief Configures an Analog Watch-dog's event filtering (consecutive out-of-range
 *        samples required before the event is raised)
 *
 * \note  HW limitation: filtering is only available on AWD1.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (ADSTART = 0 and JADSTART = 0). Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId  [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param awdId     [in]: Analog Watch-dog identification, value from \ref adc_AwdId_t
 *                        (filtering other than ADC_AWD_FILTER_NONE is supported only on ADC_AWD_1;
 *                        for ADC_AWD_2/3 ADC_AWD_FILTER_NONE is accepted without register access)
 * \param awdFilter [in]: Required event filtering, value from \ref adc_AwdFilter_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_AwdFilter( adc_PeriphId_t periphId,
                                      adc_AwdId_t awdId,
                                      adc_AwdFilter_t awdFilter )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT     > periphId  ) &&
        ( ADC_AWD_CNT        > awdId     ) &&
        ( ADC_AWD_FILTER_CNT > awdFilter )    )
    {
        if( ADC_AWD_1 == awdId )
        {
            const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

            if( ADC_REQUEST_OK == convState )
            {
                ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

                /* ADC_AWD_FILTER_NONE is written too, so a previously configured filter is cleared */
                LL_ADC_SetAWDFilteringConfiguration( periphReg, LL_ADC_AWD1, adc_AwdFilterLut[ awdFilter ] );

                for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    const uint32_t regValue = LL_ADC_GetAWDFilteringConfiguration( periphReg, LL_ADC_AWD1 );

                    if( adc_AwdFilterLut[ awdFilter ] == regValue )
                    {
                        retState = ADC_REQUEST_OK;
                        break;
                    }
                    else
                    {
                        /* Filtering configuration has not yet been applied, keep return state as error */
                        retState = ADC_REQUEST_ERROR;
                    }
                }
            }
            else
            {
                /* Conversion is ongoing, configuration change is not allowed */
                retState = ADC_REQUEST_ERROR;
            }
        }
        else if( ADC_AWD_FILTER_NONE == awdFilter )
        {
            /* AWD2/AWD3 have no filtering HW - "no filtering" is their only (and current) state */
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* Filtering is supported only on AWD1 */
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
 * \brief Reads back an Analog Watch-dog's event filtering configuration
 *
 * \note  Same AWD1-only HW limitation as Adc_Set_AwdFilter().
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param awdId      [in]: Analog Watch-dog identification, only ADC_AWD_1 is supported
 * \param awdFilter [out]: Pointer to store the current event filtering (\ref adc_AwdFilter_t).
 *                         Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_AwdFilter( adc_PeriphId_t periphId, adc_AwdId_t awdId, adc_AwdFilter_t * const awdFilter )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_AWD_1 == awdId ) &&
        ( ADC_NULL_PTR != awdFilter )    )
    {
        uint32_t llFilter = LL_ADC_GetAWDFilteringConfiguration( adc_PeriphConf[ periphId ].PeriphReg, LL_ADC_AWD1 );

        for( adc_AwdFilter_t awdFilterId = 0u; ADC_AWD_FILTER_CNT > (adc_AwdFilter_t)awdFilterId; awdFilterId ++ )
        {
            if( adc_AwdFilterLut[ awdFilterId ] == llFilter )
            {
                *awdFilter = (adc_AwdFilter_t)awdFilterId;
                retState   = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* No action required */
            }
        }
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Converts raw ADC value of the resolution to threshold register value of the
 *        analog watch-dog (STM32U5 ADC1 / ADC2: thresholds of all watch-dogs compare 14 bit
 *        values - raw value left aligned to 14 bits).
 *
 * \param awdId        [in]: Analog Watch-dog identification, value from \ref adc_AwdId_t
 * \param resShift     [in]: Shift of 14 bit value to the resolution (\ref ADC_AWD_RES_SHIFT)
 * \param rawThreshold [in]: Raw threshold of the resolution
 *
 * \return Threshold register value.
 */
static uint32_t Adc_Get_AwdThresholdReg( adc_AwdId_t awdId, uint32_t resShift, uint32_t rawThreshold )
{
    (void)awdId;

    return ( rawThreshold << resShift );
}


/**
 * \brief Converts threshold register value of the analog watch-dog to raw ADC
 *        value of the resolution (reverse of \ref Adc_Get_AwdThresholdReg).
 *
 * \param awdId        [in]: Analog Watch-dog identification, value from \ref adc_AwdId_t
 * \param resShift     [in]: Shift of 14 bit value to the resolution (\ref ADC_AWD_RES_SHIFT)
 * \param regThreshold [in]: Threshold register value
 *
 * \return Raw threshold of the resolution.
 */
static uint32_t Adc_Get_AwdThresholdRaw( adc_AwdId_t awdId, uint32_t resShift, uint32_t regThreshold )
{
    (void)awdId;

    return ( regThreshold >> resShift );
}


/**
 * \brief Checks that neither a regular nor an injected conversion is ongoing
 *        (ADSTART = 0 and JADSTART = 0)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Returns \ref ADC_REQUEST_OK if no conversion is ongoing. Otherwise (or for an
 *         invalid periphId) returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_ConversionStopped( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

        const uint32_t      regConvOngoing = LL_ADC_REG_IsConversionOngoing( periphReg );
        const uint32_t      injConvOngoing = LL_ADC_INJ_IsConversionOngoing( periphReg );

        if( ( 0u == regConvOngoing ) &&
            ( 0u == injConvOngoing )    )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* Regular or injected conversion is ongoing */
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
 * \brief Checks that the ADC peripheral is disabled (ADEN = 0) and that neither disable
 *        nor calibration procedure is ongoing (ADDIS = 0, ADCAL = 0)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Returns \ref ADC_REQUEST_OK if the peripheral is disabled. Otherwise (or for an
 *         invalid periphId) returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_PeriphDisabled( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

        const uint32_t      periphEnabled  = LL_ADC_IsEnabled( periphReg );
        const uint32_t      disableOngoing = LL_ADC_IsDisableOngoing( periphReg );
        const uint32_t      calOngoing     = LL_ADC_IsCalibrationOnGoing( periphReg );

        if( ( 0u == periphEnabled  ) &&
            ( 0u == disableOngoing ) &&
            ( 0u == calOngoing     )    )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* ADC is enabled or disable/calibration procedure is still ongoing */
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
 * \brief Checks that all ADC peripherals are disabled - required for settings shared
 *        through the ADC common register block(s) (clock mode / prescaler, kernel clock)
 *
 * \return Returns \ref ADC_REQUEST_OK if all ADC peripherals are disabled. Otherwise
 *         returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_AllPeriphsDisabled( void )
{
    adc_RequestState_t retState = ADC_REQUEST_OK;

    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ( ADC_PERIPH_CNT > periphIdx ) && ( ADC_REQUEST_OK == retState ); periphIdx ++ )
    {
        retState = Adc_Check_PeriphDisabled( periphIdx );
    }

    return ( retState );
}


/**
 * \brief Checks that the channel input is available for the channel on the peripheral
 *
 * - ADC_CHANNEL_INPUT_PIN_SINGLE: positive input GPIO pin is wired to the channel
 * - ADC_CHANNEL_INPUT_PIN_DIFF:   positive and negative input GPIO pins are wired to the channel
 * - internal inputs:              internal signal exists on the peripheral and is connected to channelId
 *
 * \param periphId     [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelId    [in]: ADC channel identification, value from \ref adc_ChannelId_t
 * \param channelInput [in]: Channel input, value from \ref adc_ChannelInput_t
 *
 * \return Returns \ref ADC_REQUEST_OK if the input is available for the channel. Otherwise (or for
 *         a parameter out of range) returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_ChannelInput( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT        > periphId     ) &&
        ( ADC_CHANNEL_CNT       > channelId    ) &&
        ( ADC_CHANNEL_INPUT_CNT > channelInput )    )
    {
        const adc_GpioChannelConfig_t * const gpioChannel     = &adc_GpioPeriphConfig[ periphId ].Channel[ channelId ];
        const adc_ChannelId_t                 internalChannel = adc_InputConfig[ periphId ].ChannelId[ channelInput ];

        if( ADC_CHANNEL_INPUT_PIN_SINGLE == channelInput )
        {
            if( GPIO_PORT_CNT != gpioChannel->ChannelInP.PortId )
            {
                retState = ADC_REQUEST_OK;
            }
            else
            {
                /* Channel has no external pin wired on this peripheral */
                retState = ADC_REQUEST_ERROR;
            }
        }
        else if( ADC_CHANNEL_INPUT_PIN_DIFF == channelInput )
        {
            if( ( GPIO_PORT_CNT != gpioChannel->ChannelInP.PortId ) &&
                ( GPIO_PORT_CNT != gpioChannel->ChannelInN.PortId )    )
            {
                retState = ADC_REQUEST_OK;
            }
            else
            {
                /* Channel has no differential pin pair wired on this peripheral */
                retState = ADC_REQUEST_ERROR;
            }
        }
        else if( channelId == internalChannel )
        {
            /* Internal signal exists on this peripheral and is connected to the requested channel */
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* Internal signal is not available on this peripheral (ADC_CHANNEL_CNT) or
             * is connected to another channel */
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
 * \brief Validates one used sequencer slot of adc_PeriphConfig_t (RegChannels[] / InjChannels[])
 *
 * \param periphId     [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelSlot  [in]: Pointer to validated slot
 * \param channelTable [in/out]: Per-channel pointer to the first slot using the channel, collected
 *                               so far over both groups (NULL == channel not used yet)
 *
 * \return Returns \ref ADC_REQUEST_OK if the slot channel, input and sampling time are valid (sampling
 *         time satisfies the minimum of the input at the active ADC kernel clock) and the
 *         channel has no conflicting input or sampling time in previously checked slots.
 *         Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_ChannelSlot( adc_PeriphId_t periphId,
                                                 const adc_ChannelConfig_t * const channelSlot,
                                                 const adc_ChannelConfig_t ** const channelTable )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_CHANNEL_CNT          > channelSlot->ChannelId       ) &&
        ( ADC_CHANNEL_SAMPLING_CNT > channelSlot->ChannelSampling )    )
    {
        const adc_RequestState_t          inputState    = Adc_Check_ChannelInput( periphId, channelSlot->ChannelId, channelSlot->ChannelInput );
        const adc_RequestState_t          samplingState = Adc_Check_SamplingTime( channelSlot->ChannelInput, channelSlot->ChannelSampling );
        const adc_ChannelConfig_t * const usedSlot      = channelTable[ channelSlot->ChannelId ];

        if( ADC_REQUEST_OK != inputState )
        {
            /* Channel input is not available for the channel */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_REQUEST_OK != samplingState )
        {
            /* Sampling time is shorter than required by the channel input */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_NULL_PTR == usedSlot )
        {
            /* First use of the channel */
            channelTable[ channelSlot->ChannelId ] = channelSlot;
            retState                               = ADC_REQUEST_OK;
        }
        else if( ( channelSlot->ChannelInput    == usedSlot->ChannelInput    ) &&
                 ( channelSlot->ChannelSampling == usedSlot->ChannelSampling )    )
        {
            /* Channel already used with the same input and sampling time */
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* Channel already used with a different input or sampling time - both are per channel */
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
 * \brief Validates complete peripheral configuration before any register is modified
 *
 * See \ref Adc_PeriphInit() for the configuration rules.
 *
 * \param adcConfig [in]: Pointer to peripheral configuration structure \ref adc_PeriphConfig_t
 *
 * \return Returns \ref ADC_REQUEST_OK if the configuration is valid. Otherwise (or for NULL pointer)
 *         returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_PeriphConfig( const adc_PeriphConfig_t * const adcConfig )
{
    adc_RequestState_t          retState = ADC_REQUEST_ERROR;
    const adc_ChannelConfig_t * channelTable[ ADC_CHANNEL_CNT ];

    if( ADC_NULL_PTR != adcConfig )
    {
        const adc_RegSequenceLen_t regLen = adcConfig->RegChannelsCnt;
        const adc_InjSequenceLen_t injLen = adcConfig->InjChannelsCnt;

        if( ( ADC_PERIPH_CNT                 > adcConfig->PeriphId       ) &&
            ( ADC_RESOLUTION_CNT             > adcConfig->Resolution     ) &&
            ( ADC_REG_TRIGGER_MODE_CNT       > adcConfig->RegTriggerMode ) &&
            ( ADC_TRIGGER_EDGE_CNT           > adcConfig->RegTriggerEdge ) &&
            ( ADC_REG_TRIGGER_CNT            > adcConfig->RegTriggerId   ) &&
            ( ADC_INJ_TRIGGER_MODE_CNT       > adcConfig->InjTriggerMode ) &&
            ( ADC_TRIGGER_EDGE_CNT           > adcConfig->InjTriggerEdge ) &&
            ( ADC_INJ_TRIGGER_CNT            > adcConfig->InjTriggerId   ) &&
            ( ADC_REG_SEQUENCE_CNT           >= regLen                   ) &&
            ( ADC_INJ_SEQUENCE_CNT           >= injLen                   ) &&
            ( 0u                             < ( regLen + injLen )       )    )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            retState = ADC_REQUEST_ERROR;
        }

        /* Auto-injected mode (injected group converted after every regular sequence) */
        if( ( ADC_REQUEST_OK == retState ) &&
            ( 0u < injLen ) &&
            ( ADC_INJ_TRIGGER_AUTO == adcConfig->InjTriggerId )    )
        {
            if( 0u == regLen )
            {
                /* Injected group is started by the regular group - regular group must be used */
                retState = ADC_REQUEST_ERROR;
            }
            else if( ADC_INJ_TRIGGER_MODE_SINGLE == adcConfig->InjTriggerMode )
            {
                /* Auto-injected mode (JAUTO) and discontinuous mode (JDISCEN) can not be combined */
                retState = ADC_REQUEST_ERROR;
            }
            else
            {
                /* Valid auto-injected configuration */
                retState = ADC_REQUEST_OK;
            }
        }
        else
        {
            /* Injected group is not used or is not in auto-injected mode */
        }

        for( adc_ChannelId_t channelIdx = ADC_CHANNEL_0; ADC_CHANNEL_CNT > channelIdx; channelIdx ++ )
        {
            channelTable[ channelIdx ] = ADC_NULL_PTR;
        }

        /* Regular slots - slot index is the rank */
        for( adc_RegSequenceId_t slotIdx = ADC_REG_SEQUENCE_1; ( regLen > slotIdx ) && ( ADC_REQUEST_OK == retState ); slotIdx ++ )
        {
            retState = Adc_Check_ChannelSlot( adcConfig->PeriphId, &adcConfig->RegChannels[ slotIdx ], channelTable );
        }

        /* Injected slots - slot index is the rank */
        for( adc_InjSequenceId_t slotIdx = ADC_INJ_SEQUENCE_1; ( injLen > slotIdx ) && ( ADC_REQUEST_OK == retState ); slotIdx ++ )
        {
            retState = Adc_Check_ChannelSlot( adcConfig->PeriphId, &adcConfig->InjChannels[ slotIdx ], channelTable );
        }

        /* Data transfer configuration */
        if( ADC_REQUEST_OK == retState )
        {
            adc_FunctionState_t regUsed = ADC_FUNCTION_INACTIVE;

            if( 0u < regLen )
            {
                regUsed = ADC_FUNCTION_ACTIVE;
            }
            else
            {
                /* Injected group only */
                regUsed = ADC_FUNCTION_INACTIVE;
            }

            retState = Adc_Check_DataConfig( adcConfig->PeriphId, regUsed, &adcConfig->DataConfig );
        }
        else
        {
            /* Configuration is already invalid */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures regular group sequencer (ranks and length) from adc_PeriphConfig_t
 *
 * \pre   Configuration was validated by Adc_Check_PeriphConfig() and RegChannelsCnt > 0.
 *        No regular conversion is ongoing (called from Adc_PeriphInit() only).
 *
 * \param adcConfig [in]: Pointer to peripheral configuration structure \ref adc_PeriphConfig_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_RegSequencer( const adc_PeriphConfig_t * const adcConfig )
{
    adc_RequestState_t  retState  = ADC_REQUEST_OK;
    ADC_TypeDef * const periphReg = adc_PeriphConf[ adcConfig->PeriphId ].PeriphReg;
    const adc_RegSequenceLen_t regLen = adcConfig->RegChannelsCnt;

    /* --- Ranks --- */
    for( adc_RegSequenceId_t slotIdx = ADC_REG_SEQUENCE_1; ( regLen > slotIdx ) && ( ADC_REQUEST_OK == retState ); slotIdx ++ )
    {
        const adc_ChannelConfig_t * const channelSlot = &adcConfig->RegChannels[ slotIdx ];
        const uint32_t                    llRank      = adc_RegSeqRankLut[ slotIdx ];
        const uint32_t                    llChannel   = __LL_ADC_DECIMAL_NB_TO_CHANNEL( channelSlot->ChannelId );

        LL_ADC_REG_SetSequencerRanks( periphReg, llRank, llChannel );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t rankReg     = LL_ADC_REG_GetSequencerRanks( periphReg, llRank );
            const uint32_t rankChannel = __LL_ADC_CHANNEL_TO_DECIMAL_NB( rankReg );

            if( (uint32_t)channelSlot->ChannelId == rankChannel )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* Sequencer rank has not yet been applied, keep return state as error */
                retState = ADC_REQUEST_ERROR;
            }
        }
    }

    /* --- Length --- */
    if( ADC_REQUEST_OK == retState )
    {
        const uint32_t llSeqLen = adc_RegSeqLenLut[ regLen - ADC_SEQ_LEN_IDX_OFFSET ];

        LL_ADC_REG_SetSequencerLength( periphReg, llSeqLen );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_ADC_REG_GetSequencerLength( periphReg );

            if( llSeqLen == regValue )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* Sequencer length has not yet been applied, keep return state as error */
                retState = ADC_REQUEST_ERROR;
            }
        }
    }
    else
    {
        /* Previous step failed, error state is kept */
    }

    return ( retState );
}


/**
 * \brief Configures injected group sequencer (ranks, length), trigger source, trigger edge,
 *        auto-injected mode and discontinuous mode from adc_PeriphConfig_t
 *
 * All JSQR settings are written at once through LL_ADC_INJ_ConfigQueueContext() and verified
 * together. For software trigger and auto-injected mode the trigger edge is discarded (JEXTEN = 0).
 * - ADC_INJ_TRIGGER_AUTO:          JAUTO = 1, injected sequence is converted after each regular sequence
 * - ADC_INJ_TRIGGER_MODE_SINGLE:   JDISCEN = 1, every trigger converts one rank
 * - ADC_INJ_TRIGGER_MODE_CONTINUOUS: JDISCEN = 0, every trigger converts the whole sequence
 *
 * \pre   Configuration was validated by Adc_Check_PeriphConfig() and InjChannelsCnt > 0.
 *        No injected conversion is ongoing (called from Adc_PeriphInit() only).
 *
 * \param adcConfig [in]: Pointer to peripheral configuration structure \ref adc_PeriphConfig_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_InjSequencer( const adc_PeriphConfig_t * const adcConfig )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef * const periphReg = adc_PeriphConf[ adcConfig->PeriphId ].PeriphReg;
    const adc_InjSequenceLen_t injLen = adcConfig->InjChannelsCnt;
    const uint32_t      llTrigger = adc_InjTriggerSrcLut[ adcConfig->InjTriggerId ];
    const uint32_t      llSeqLen  = adc_InjSeqLenLut[ injLen - ADC_SEQ_LEN_IDX_OFFSET ];
    uint32_t            llEdge    = adc_InjTriggerEdgeLut[ adcConfig->InjTriggerEdge ];
    uint32_t            llRankChannel[ ADC_INJ_SEQUENCE_CNT ];

    if( ( ADC_INJ_TRIGGER_SOFTWARE == adcConfig->InjTriggerId ) ||
        ( ADC_INJ_TRIGGER_AUTO     == adcConfig->InjTriggerId )    )
    {
        /* Software trigger / auto-injected mode - edge is discarded by HW write, JEXTEN reads back as 0 */
        llEdge = LL_ADC_INJ_TRIG_SOFTWARE;
    }
    else
    {
        /* External trigger - edge from configuration */
    }

    /* Unused ranks are written with channel 0, HW ignores ranks above the sequence length */
    for( adc_InjSequenceId_t rankIdx = ADC_INJ_SEQUENCE_1; ADC_INJ_SEQUENCE_CNT > rankIdx; rankIdx ++ )
    {
        llRankChannel[ rankIdx ] = LL_ADC_CHANNEL_0;
    }

    /* Slot index is the rank */
    for( adc_InjSequenceId_t slotIdx = ADC_INJ_SEQUENCE_1; injLen > slotIdx; slotIdx ++ )
    {
        llRankChannel[ slotIdx ] = __LL_ADC_DECIMAL_NB_TO_CHANNEL( adcConfig->InjChannels[ slotIdx ].ChannelId );
    }

    LL_ADC_INJ_ConfigQueueContext( periphReg,
                                   llTrigger,
                                   llEdge,
                                   llSeqLen,
                                   llRankChannel[ ADC_INJ_SEQUENCE_1 ],
                                   llRankChannel[ ADC_INJ_SEQUENCE_2 ],
                                   llRankChannel[ ADC_INJ_SEQUENCE_3 ],
                                   llRankChannel[ ADC_INJ_SEQUENCE_4 ] );

    for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const uint32_t triggerReg = LL_ADC_INJ_GetTriggerSource( periphReg );
        const uint32_t edgeReg    = LL_ADC_INJ_GetTriggerEdge( periphReg );
        const uint32_t seqLenReg  = LL_ADC_INJ_GetSequencerLength( periphReg );
        adc_FlagState_t ranksOk   = ADC_FLAG_ACTIVE;

        for( adc_InjSequenceId_t slotIdx = ADC_INJ_SEQUENCE_1; injLen > slotIdx; slotIdx ++ )
        {
            const uint32_t rankReg     = LL_ADC_INJ_GetSequencerRanks( periphReg, adc_InjSeqRankLut[ slotIdx ] );
            const uint32_t rankChannel = __LL_ADC_CHANNEL_TO_DECIMAL_NB( rankReg );

            if( (uint32_t)adcConfig->InjChannels[ slotIdx ].ChannelId != rankChannel )
            {
                ranksOk = ADC_FLAG_INACTIVE;
            }
            else
            {
                /* Rank matches the requested channel */
            }
        }

        if( ( llTrigger == triggerReg ) &&
            ( llEdge    == edgeReg    ) &&
            ( llSeqLen  == seqLenReg  ) &&
            ( ADC_FLAG_ACTIVE == ranksOk )    )
        {
            retState = ADC_REQUEST_OK;
            break;
        }
        else
        {
            /* Injected context has not yet been applied, keep return state as error */
            retState = ADC_REQUEST_ERROR;
        }
    }

    /* --- Auto-injected (JAUTO) and discontinuous (JDISCEN) mode ---
     * JAUTO and JDISCEN must never be set at the same time: JAUTO is cleared first, then JDISCEN
     * is written and finally JAUTO is set if required (configuration excludes JAUTO + JDISCEN). */
    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Set_InjAutoMode( adcConfig->PeriphId, LL_ADC_INJ_TRIG_INDEPENDENT );
    }
    else
    {
        /* Previous step failed, error state is kept */
    }

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Set_InjDiscontMode( adcConfig->PeriphId, adc_InjTriggerModeLut[ adcConfig->InjTriggerMode ] );
    }
    else
    {
        /* Previous step failed, error state is kept */
    }

    if( ( ADC_REQUEST_OK == retState ) &&
        ( ADC_INJ_TRIGGER_AUTO == adcConfig->InjTriggerId )    )
    {
        retState = Adc_Set_InjAutoMode( adcConfig->PeriphId, LL_ADC_INJ_TRIG_FROM_GRP_REGULAR );
    }
    else
    {
        /* Previous step failed, error state is kept */
    }

    return ( retState );
}


/**
 * \brief Selects injected group auto-injected mode (JAUTO)
 *
 * \pre   Called from Adc_Set_InjSequencer() only (validated configuration, no conversion ongoing).
 *        JDISCEN must be cleared before JAUTO is set.
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param llTrigAuto [in]: LL_ADC_INJ_TRIG_INDEPENDENT / LL_ADC_INJ_TRIG_FROM_GRP_REGULAR
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_InjAutoMode( adc_PeriphId_t periphId, uint32_t llTrigAuto )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

        LL_ADC_INJ_SetTrigAuto( periphReg, llTrigAuto );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_ADC_INJ_GetTrigAuto( periphReg );

            if( llTrigAuto == regValue )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* Auto-injected mode has not yet been applied, keep return state as error */
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
 * \brief Selects injected group discontinuous mode (JDISCEN)
 *
 * \pre   Called from Adc_Set_InjSequencer() only (validated configuration, no conversion ongoing).
 *        JAUTO must be cleared before JDISCEN is set.
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param llDiscont  [in]: LL_ADC_INJ_SEQ_DISCONT_DISABLE / LL_ADC_INJ_SEQ_DISCONT_1RANK
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_InjDiscontMode( adc_PeriphId_t periphId, uint32_t llDiscont )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

        LL_ADC_INJ_SetSequencerDiscont( periphReg, llDiscont );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_ADC_INJ_GetSequencerDiscont( periphReg );

            if( llDiscont == regValue )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* Discontinuous mode has not yet been applied, keep return state as error */
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
 * \brief Runs ADC self-calibration (offset and linearity) of ADC1 / ADC2
 *
 * STM32U5 calibration is common for single-ended and differential inputs. Devices with
 * extended calibration (\ref Adc_Get_ExtendedCalibration) get the calibration factor index 9
 * trimmed with the ADC enabled before the calibration (sequence of RM0456 / ST HAL).
 *
 * \pre   ADC peripheral is disabled with internal regulator started (called from
 *        Adc_Set_PeriphActive() only).
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if calibration was finished
 *         within the timeout. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_Calibration( adc_PeriphId_t periphId )
{
    adc_RequestState_t        retState      = ADC_REQUEST_OK;
    ADC_TypeDef * const       periphReg     = adc_PeriphConf[ periphId ].PeriphReg;
    const adc_FunctionState_t extendedCalib = Adc_Get_ExtendedCalibration();

    if( ADC_FUNCTION_ACTIVE == extendedCalib )
    {
        retState = Adc_Set_EnableWait( periphId );

        if( ADC_REQUEST_OK == retState )
        {
            MODIFY_REG( periphReg->CR, ADC_CR_CALINDEX, ADC_CALIB_EXT_INDEX << ADC_CR_CALINDEX_Pos );
            __DMB();
            MODIFY_REG( periphReg->CALFACT2, ADC_CALIB_EXT_CALFACT2_MASK, ADC_CALIB_EXT_CALFACT2_VALUE );
            __DMB();
            SET_BIT( periphReg->CALFACT, ADC_CALFACT_LATCH_COEF );

            retState = Adc_Set_PeriphInactive( periphId );
        }
        else
        {
            /* ADC could not be enabled for the extended calibration */
            retState = ADC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Calibration without extended calibration step */
        retState = ADC_REQUEST_OK;
    }

    if( ADC_REQUEST_OK == retState )
    {
        LL_ADC_StartCalibration( periphReg, LL_ADC_CALIB_OFFSET_LINEARITY );

        retState = ADC_REQUEST_ERROR;

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t calOngoing = LL_ADC_IsCalibrationOnGoing( periphReg );

            if( 0u == calOngoing )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* Calibration has not yet been finished, keep return state as error */
                retState = ADC_REQUEST_ERROR;
            }
        }
    }
    else
    {
        /* Extended calibration step failed, calibration is not started */
    }

    return ( retState );
}


/**
 * \brief Returns availability of the extended calibration of ADC1 / ADC2 on the device
 *
 * Extended calibration is available on STM32U535 / U545 and STM32U5Fx / U5Gx, and on
 * STM32U575 / U585 / U59x / U5Ax from revision 0x3000 (DBGMCU IDCODE, ST HAL).
 *
 * \return \ref ADC_FUNCTION_ACTIVE if the extended calibration is available.
 */
static adc_FunctionState_t Adc_Get_ExtendedCalibration( void )
{
    adc_FunctionState_t retState = ADC_FUNCTION_INACTIVE;
    const uint32_t      devId    = READ_BIT( DBGMCU->IDCODE, DBGMCU_IDCODE_DEV_ID );
    const uint32_t      revId    = READ_BIT( DBGMCU->IDCODE, DBGMCU_IDCODE_REV_ID ) >> DBGMCU_IDCODE_REV_ID_Pos;

    if( ( ADC_DEV_ID_U535_U545 == devId ) ||
        ( ADC_DEV_ID_U5F_U5G   == devId )    )
    {
        retState = ADC_FUNCTION_ACTIVE;
    }
    else if( ( ( ADC_DEV_ID_U575_U585 == devId ) ||
               ( ADC_DEV_ID_U59X_U5AX == devId ) ) &&
             ( ADC_REV_ID_EXT_CALIB <= revId )    )
    {
        retState = ADC_FUNCTION_ACTIVE;
    }
    else
    {
        /* Extended calibration is not available */
        retState = ADC_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Enables the ADC and waits for its ready flag
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if ADRDY was raised within
 *         the timeout. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_EnableWait( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

    LL_ADC_ClearFlag_ADRDY( periphReg );
    LL_ADC_Enable( periphReg );

    for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const uint32_t readyFlag     = LL_ADC_IsActiveFlag_ADRDY( periphReg );
        const uint32_t periphEnabled = LL_ADC_IsEnabled( periphReg );

        if( 0u != readyFlag )
        {
            retState = ADC_REQUEST_OK;
            break;
        }
        else if( 0u == periphEnabled )
        {
            /* ADEN set less than 4 ADC clock cycles after end of calibration is reset
             * by the calibration logic - set it again until ADRDY is raised */
            LL_ADC_Enable( periphReg );
            retState = ADC_REQUEST_ERROR;
        }
        else
        {
            /* ADC is not yet ready, keep return state as error */
            retState = ADC_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Preselects the channel for conversion (PCSEL register of STM32U5 ADC1 / ADC2)
 *
 * \param periphId  [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelId [in]: ADC channel identification, value from \ref adc_ChannelId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if the preselection was
 *         verified by read-back. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_ChannelPreselection( adc_PeriphId_t periphId, adc_ChannelId_t channelId )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;
    const uint32_t      pcselBit  = 1uL << (uint32_t)channelId;

    LL_ADC_SetChannelPreselection( periphReg, __LL_ADC_DECIMAL_NB_TO_CHANNEL( channelId ) );

    for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const uint32_t pcselReg = READ_BIT( periphReg->PCSEL, pcselBit );

        if( 0u != pcselReg )
        {
            retState = ADC_REQUEST_OK;
            break;
        }
        else
        {
            /* Preselection has not yet been applied, keep return state as error */
            retState = ADC_REQUEST_ERROR;
        }
    }

    return ( retState );
}



/**
 * \brief Configures a GPIO pin as analog input without pull (GPIO port clock is enabled by Gpio_Init())
 *
 * \param pinConfig [in]: Pointer to pin identification from adc_GpioPeriphConfig[]. Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise (also for unwired pin) returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_PinAnalog( const adc_GpioConfig_t * const pinConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_NULL_PTR  != pinConfig         ) &&
        ( GPIO_PORT_CNT  > pinConfig->PortId ) &&
        ( GPIO_PIN_ID_CNT > pinConfig->PinId ) )
    {
        gpio_Config_t gpioConfig;

        gpioConfig.PortId         = pinConfig->PortId;
        gpioConfig.PinId          = pinConfig->PinId;
        gpioConfig.PinMode        = GPIO_PIN_MODE_ANALOG;
        gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
        gpioConfig.PinSpeed       = GPIO_PIN_SPEED_LOW;
        gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
        gpioConfig.PinAltFunction = GPIO_ALT_FUNC_0;
        gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

        /* Port clock activation, pin configuration and read-back verification is done by Gpio_Init() */
        const gpio_RequestState_t gpioState = Gpio_Init( &gpioConfig );

        if( GPIO_REQUEST_OK == gpioState )
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
        /* Pin is not wired to the channel on this peripheral */
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures one of the internal (non-pin) channel inputs: TEMP, VREF or VBAT
 *
 * - VREFINT / TEMPSENSOR / VBAT require setting a CCR path-enable bit of the common block.
 * - After enabling VREFINT / TEMPSENSOR path, the stabilization time is waited
 *   (LL_ADC_DELAY_VREFINT_STAB_US / LL_ADC_DELAY_TEMPSENSOR_STAB_US).
 * - DAC channels need only channel selection.
 *
 * \param periphId     [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelInput [in]: Required internal input, value from \ref adc_ChannelInput_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_InternalInput( adc_PeriphId_t periphId, adc_ChannelInput_t channelInput )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT               > periphId                                          ) &&
        ( ADC_CHANNEL_INPUT_CNT        > channelInput                                      ) &&
        ( ADC_CHANNEL_INPUT_PIN_SINGLE != channelInput                                     ) &&
        ( ADC_CHANNEL_INPUT_PIN_DIFF   != channelInput                                     ) &&
        ( ADC_CHANNEL_CNT              > adc_InputConfig[ periphId ].ChannelId[ channelInput ] ) )
    {
        /* Internal input exists on this peripheral (ADC_CHANNEL_CNT in adc_InputConfig means "not available") */
        ADC_TypeDef        * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;
        ADC_Common_TypeDef * const commonReg = __LL_ADC_COMMON_INSTANCE( periphReg );
        uint32_t                   llPath    = LL_ADC_PATH_INTERNAL_NONE;

        if( ADC_CHANNEL_INPUT_VREF == channelInput )
        {
            llPath = LL_ADC_PATH_INTERNAL_VREFINT;
        }
        else if( ADC_CHANNEL_INPUT_TEMP == channelInput )
        {
            llPath = LL_ADC_PATH_INTERNAL_TEMPSENSOR;
        }
        else if( ADC_CHANNEL_INPUT_VBAT == channelInput )
        {
            llPath = LL_ADC_PATH_INTERNAL_VBAT;
        }
        else
        {
            /* Other internal channels does not need extra activation step */
        }

        if( LL_ADC_PATH_INTERNAL_NONE != llPath )
        {
            /* Activate internal component path */
            LL_ADC_SetCommonPathInternalChAdd( commonReg, llPath );
        }
        else
        {
            /* No action required */
        }

        /* Get channel register identification */
        const uint32_t adcChannel = ADC_CHANNEL_ID_INTERNAL_CH | __LL_ADC_DECIMAL_NB_TO_CHANNEL( adc_InputConfig[ periphId ].ChannelId[ channelInput ] );

        /* Configure channel input */
        LL_ADC_SetChannelSingleDiff( periphReg, adcChannel, LL_ADC_SINGLE_ENDED );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t pathReg   = LL_ADC_GetCommonPathInternalCh( commonReg );
            const uint32_t difselReg = LL_ADC_GetChannelSingleDiff( periphReg, adcChannel );

            if( ( llPath == ( pathReg & llPath ) ) &&
                ( 0u     == difselReg            )    )
            {
                retState = ADC_REQUEST_OK;
                break;
            }
            else
            {
                /* Internal input has not yet been applied, keep return state as error */
                retState = ADC_REQUEST_ERROR;
            }
        }

        /* --- Stabilization time of the enabled measurement path --- */
        if( ( ADC_REQUEST_OK == retState ) &&
            ( ADC_CHANNEL_INPUT_VREF == channelInput )    )
        {
            retState = Adc_Set_Delay( LL_ADC_DELAY_VREFINT_STAB_US );
        }
        else if( ( ADC_REQUEST_OK == retState ) &&
                 ( ADC_CHANNEL_INPUT_TEMP == channelInput )    )
        {
            retState = Adc_Set_Delay( LL_ADC_DELAY_TEMPSENSOR_STAB_US );
        }
        else
        {
            /* Other internal inputs have no stabilization time requirement */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}



/**
 * \brief Busy-wait delay in microseconds derived from the actual CPU (HCLK) frequency
 *
 * \note  One loop iteration takes at least one CPU clock cycle, so the delay is never shorter
 *        than required (it is longer, depending on the compiler optimization).
 *
 * \param delayUs [in]: Required delay in microseconds
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise (HCLK frequency not available) returns
 *         \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_Delay( adc_TimeUs_t delayUs )
{
    adc_RequestState_t       retState = ADC_REQUEST_ERROR;
    rcc_FreqHz_t             hclkFreq = 0u;
    /* CPU clock == HCLK, provided by Rcc as frequency of the ADC HCLK clock source */
    const rcc_RequestState_t rccState = Rcc_Get_PeriphClk( adc_ClkSrcRccLut[ ADC_CLK_SRC_HCLK ], &hclkFreq );

    if( ( RCC_REQUEST_OK == rccState ) &&
        ( 0u != hclkFreq )    )
    {
        /* CPU clock cycles per microsecond, rounded up */
        const adc_TimeoutCnt_t cyclesPerUs = (adc_TimeoutCnt_t)ADC_DIV_ROUND_UP( hclkFreq, ADC_US_PER_S );
        const adc_TimeoutCnt_t loopCnt     = (adc_TimeoutCnt_t)delayUs * cyclesPerUs;

        for( volatile adc_TimeoutCnt_t delayCnt = 0u; loopCnt > delayCnt; delayCnt ++ )
        {
            /* Busy-wait: intentionally empty */
        }

        retState = ADC_REQUEST_OK;
    }
    else
    {
        /* CPU clock frequency is not available, delay can not be derived */
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Calculates ADC kernel clock frequency for a clock source and divider
 *
 * \param clkSource  [in]: ADC clock source, value from \ref adc_ClkSrc_t
 * \param clkDiv     [in]: ADC clock divider, value from \ref adc_ClkDiv_t
 * \param clkFreqHz [out]: Pointer to store the ADC kernel clock frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Get_ClockFreq( adc_ClkSrc_t clkSource, adc_ClkDiv_t clkDiv, adc_FreqHz_t * const clkFreqHz )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_CLK_SRC_CNT > clkSource ) &&
        ( ADC_CLK_DIV_CNT > clkDiv    ) &&
        ( ADC_NULL_PTR   != clkFreqHz )    )
    {
        rcc_FreqHz_t             srcFreq  = 0u;
        const rcc_RequestState_t rccState = Rcc_Get_PeriphClk( adc_ClkSrcRccLut[ clkSource ], &srcFreq );

        if( RCC_REQUEST_OK == rccState )
        {
            *clkFreqHz = (adc_FreqHz_t)srcFreq / adc_ClkDivValueLut[ clkDiv ];
            retState   = ADC_REQUEST_OK;
        }
        else
        {
            /* Clock source frequency is not available */
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
 * \brief Returns the active ADC kernel clock frequency (last selected clock source and the
 *        divider configured in HW)
 *
 * \param clkFreqHz [out]: Pointer to store the ADC kernel clock frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Get_ActiveClockFreq( adc_FreqHz_t * const clkFreqHz )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;
    adc_ClkDiv_t       clkDiv   = ADC_CLK_DIV_CNT;

    retState = Adc_Get_ClockDivider( &clkDiv );

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Get_ClockFreq( adc_ShadowClkSrc, clkDiv, clkFreqHz );
    }
    else
    {
        /* Clock divider can not be decoded */
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that a clock source / divider combination is valid and results in an ADC kernel
 *        clock frequency within ADC_CLK_FREQ_MIN_HZ - ADC_CLK_FREQ_MAX_HZ
 *
 * \param clkSource [in]: ADC clock source, value from \ref adc_ClkSrc_t
 * \param clkDiv    [in]: ADC clock divider, value from \ref adc_ClkDiv_t (HCLK source supports
 *                        only ADC_CLK_DIV_1 / 2 / 4)
 *
 * \return Returns \ref ADC_REQUEST_OK if the resulting frequency is within the allowed range.
 *         Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_ClockFreq( adc_ClkSrc_t clkSource, adc_ClkDiv_t clkDiv )
{
    adc_RequestState_t retState  = ADC_REQUEST_ERROR;
    adc_FreqHz_t       clkFreqHz = 0u;

    retState = Adc_Get_ClockFreq( clkSource, clkDiv, &clkFreqHz );

    if( ADC_REQUEST_OK == retState )
    {
        if( ( ADC_CLK_FREQ_MIN_HZ <= clkFreqHz ) &&
            ( ADC_CLK_FREQ_MAX_HZ >= clkFreqHz )    )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* ADC kernel clock frequency is out of the allowed range */
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
 * \brief Checks that a sampling time satisfies the minimum sampling time of a channel input
 *        at the active ADC kernel clock (internal channels, see adc_SamplingMinNsLut)
 *
 * \param channelInput [in]: Channel input, value from \ref adc_ChannelInput_t
 * \param samplingTime [in]: Sampling time, value from \ref adc_ChannelSampling_t
 *
 * \return Returns \ref ADC_REQUEST_OK if the input has no minimum sampling time or the sampling
 *         time is long enough. Otherwise (also when the ADC clock frequency is not available)
 *         returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_SamplingTime( adc_ChannelInput_t channelInput, adc_ChannelSampling_t samplingTime )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_CHANNEL_INPUT_CNT    > channelInput ) &&
        ( ADC_CHANNEL_SAMPLING_CNT > samplingTime )    )
    {
        const adc_TimeNs_t minSamplingNs = adc_SamplingMinNsLut[ channelInput ];
        adc_FreqHz_t       clkFreqHz     = 0u;

        if( ADC_SAMPLING_MIN_NONE_NS == minSamplingNs )
        {
            /* Input has no minimum sampling time requirement */
            retState = ADC_REQUEST_OK;
        }
        else
        {
            retState = Adc_Get_ActiveClockFreq( &clkFreqHz );

            if( ( ADC_REQUEST_OK == retState ) &&
                ( 0u != clkFreqHz )    )
            {
                /* Sampling time in ns = half cycles * 1e9 / ( 2 * fADC ) */
                const uint64_t samplingNs = ( (uint64_t)adc_SamplingHalfCyclesLut[ samplingTime ] * ADC_NS_PER_S ) /
                                            ( (uint64_t)clkFreqHz * ADC_HALF_CYCLES_PER_CYCLE );

                if( (uint64_t)minSamplingNs <= samplingNs )
                {
                    retState = ADC_REQUEST_OK;
                }
                else
                {
                    /* Sampling time is shorter than required by the input */
                    retState = ADC_REQUEST_ERROR;
                }
            }
            else
            {
                /* ADC kernel clock frequency is not available */
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


/* -------------------------------------------------------------------------- */
/* --------------------------- Data transfer core --------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Validates data transfer configuration
 *
 * - TransferMode and BufferMode must be valid.
 * - DataBuffer == NULL is allowed only in POLL mode (manual polling by Adc_Get_RegData()) or
 *   if the regular group is not used (injected group only). Otherwise BufferSize must be > 0.
 * - Mode specific rules are checked by the mode handler (DMA: DMA request, channel, buffer
 *   size limit; ISR / DMA: ADC interrupt available).
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param regUsed    [in]: Regular group is used (\ref ADC_FUNCTION_INACTIVE = injected group only)
 * \param dataConfig [in]: Pointer to data transfer configuration \ref adc_DataConfig_t
 *
 * \return Returns \ref ADC_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_DataConfig( adc_PeriphId_t periphId, adc_FunctionState_t regUsed, const adc_DataConfig_t * const dataConfig )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT        > periphId                 ) &&
        ( ADC_NULL_PTR         != dataConfig               ) &&
        ( ADC_TRANSFER_MODE_CNT > dataConfig->TransferMode ) &&
        ( ADC_BUFFER_MODE_CNT   > dataConfig->BufferMode   )    )
    {
        if( ADC_NULL_PTR != dataConfig->DataBuffer )
        {
            if( 0u < dataConfig->BufferSize )
            {
                retState = ADC_REQUEST_OK;
            }
            else
            {
                /* Buffer without size */
                retState = ADC_REQUEST_ERROR;
            }
        }
        else if( ( ADC_TRANSFER_MODE_POLL == dataConfig->TransferMode ) ||
                 ( ADC_FUNCTION_INACTIVE  == regUsed                  )    )
        {
            /* Regular results are not collected (manual polling or regular group not used) */
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* DMA / ISR mode needs a buffer for regular results */
            retState = ADC_REQUEST_ERROR;
        }

        if( ADC_REQUEST_OK == retState )
        {
            retState = adc_XferModeLut[ dataConfig->TransferMode ].CheckConfig( periphId, dataConfig );
        }
        else
        {
            /* Common part of the configuration is invalid */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stores the data transfer configuration, sets overrun behavior of the transfer mode
 *        (adc_OvrModeLut) and initializes the mode handler
 *
 * \note  Initialization state is set before the mode handler is initialized, so a partially
 *        initialized handler is released by Adc_Deinit().
 *
 * \pre   Configuration was validated by Adc_Check_DataConfig(), no conversion is ongoing and the
 *        previous mode handler is deinitialized (called from Adc_PeriphInit() / Adc_Set_DataConfig()).
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param dataConfig [in]: Pointer to data transfer configuration \ref adc_DataConfig_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_XferInit( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig )
{
    adc_RequestState_t        retState  = ADC_REQUEST_ERROR;
    adc_XferContext_t * const xferCtx   = &adc_XferContext[ periphId ];
    ADC_TypeDef * const       periphReg = adc_PeriphConf[ periphId ].PeriphReg;
    const uint32_t            llOvrMode = adc_OvrModeLut[ dataConfig->TransferMode ];

    /* Overrun behavior by transfer mode: DMA keeps the unread result (lost sample reported as
     * overrun), ISR / POLL keep the latest result in DR */
    LL_ADC_REG_SetOverrun( periphReg, llOvrMode );

    for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const uint32_t ovrMode = LL_ADC_REG_GetOverrun( periphReg );

        if( llOvrMode == ovrMode )
        {
            retState = ADC_REQUEST_OK;
            break;
        }
        else
        {
            /* Overrun mode has not yet been applied, keep return state as error */
            retState = ADC_REQUEST_ERROR;
        }
    }

    if( ADC_REQUEST_OK == retState )
    {
        /* Configuration is copied - user structure may be a temporary (stack) variable */
        xferCtx->Config    = *dataConfig;
        xferCtx->BufferIdx = 0u;
        xferCtx->XferState = ADC_FUNCTION_INACTIVE;
        xferCtx->InitState = ADC_FUNCTION_ACTIVE;

        retState = adc_XferModeLut[ xferCtx->Config.TransferMode ].Init( periphId );
    }
    else
    {
        /* Overrun behavior could not be configured, mode handler is not initialized */
    }

    return ( retState );
}


/**
 * \brief Arms the regular data transfer (called by Adc_Set_RegStart() before ADSTART)
 *
 * \note  If the transfer is already running (buffer not yet full), it continues at the current
 *        buffer position. Without DataBuffer (manual polling) nothing is armed.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_XferStart( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        adc_XferContext_t * const xferCtx = &adc_XferContext[ periphId ];

        if( ( ADC_FUNCTION_ACTIVE   == xferCtx->InitState         ) &&
            ( ADC_FUNCTION_INACTIVE == xferCtx->XferState         ) &&
            ( ADC_NULL_PTR         != xferCtx->Config.DataBuffer )    )
        {
            xferCtx->BufferIdx = 0u;
            xferCtx->XferState = ADC_FUNCTION_ACTIVE;

            retState = adc_XferModeLut[ xferCtx->Config.TransferMode ].Start( periphId );

            if( ADC_REQUEST_OK != retState )
            {
                xferCtx->XferState = ADC_FUNCTION_INACTIVE;
            }
            else
            {
                /* Data transfer is armed */
            }
        }
        else
        {
            /* Transfer is not initialized, already running or no buffer is used */
            retState = ADC_REQUEST_OK;
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops the regular data transfer (called by Adc_Set_RegStop())
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_XferStop( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        adc_XferContext_t * const xferCtx = &adc_XferContext[ periphId ];

        if( ( ADC_FUNCTION_ACTIVE == xferCtx->InitState ) &&
            ( ADC_FUNCTION_ACTIVE == xferCtx->XferState )    )
        {
            xferCtx->XferState = ADC_FUNCTION_INACTIVE;

            retState = adc_XferModeLut[ xferCtx->Config.TransferMode ].Stop( periphId );
        }
        else
        {
            /* Data transfer is not running */
            retState = ADC_REQUEST_OK;
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}

/* -------------------------------------------------------------------------- */
/* ------------------ Private interface (see Adc_Priv.h) -------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Returns CMSIS register pointer of an ADC peripheral (for data transfer handlers)
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param periphReg [out]: Pointer to store the register pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_PeriphReg( adc_PeriphId_t periphId, ADC_TypeDef ** const periphReg )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId  ) &&
        ( ADC_NULL_PTR  != periphReg )    )
    {
        *periphReg = adc_PeriphConf[ periphId ].PeriphReg;
        retState   = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns data transfer context of an ADC peripheral (for data transfer handlers)
 *
 * \param periphId     [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param xferContext [out]: Pointer to store the context pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_XferContext( adc_PeriphId_t periphId, adc_XferContext_t ** const xferContext )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId    ) &&
        ( ADC_NULL_PTR  != xferContext )    )
    {
        *xferContext = &adc_XferContext[ periphId ];
        retState     = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stores one regular result into DataBuffer (ISR / POLL mode) and reports half / full
 *        buffer events
 *
 * \note  Called from ADC interrupt (ISR mode) or Adc_Task() (POLL mode). Results received
 *        while the transfer is not running are dropped.
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param data     [in]: Regular conversion result (RAW value)
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_XferData( adc_PeriphId_t periphId, adc_Data_t data )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        adc_XferContext_t * const xferCtx = &adc_XferContext[ periphId ];

        retState = ADC_REQUEST_OK;

        if( ( ADC_FUNCTION_ACTIVE == xferCtx->XferState                ) &&
            ( ADC_NULL_PTR       != xferCtx->Config.DataBuffer         ) &&
            ( xferCtx->Config.BufferSize > xferCtx->BufferIdx          )    )
        {
            const adc_BufferSize_t halfSize = xferCtx->Config.BufferSize / ADC_BUFFER_HALF_DIVIDER;

            xferCtx->Config.DataBuffer[ xferCtx->BufferIdx ] = data;
            xferCtx->BufferIdx ++;

            if( ( 0u != halfSize ) &&
                ( halfSize == xferCtx->BufferIdx )    )
            {
                retState = Adc_Set_XferHalf( periphId );
            }
            else
            {
                /* Half of the buffer not reached in this step */
            }

            if( xferCtx->Config.BufferSize <= xferCtx->BufferIdx )
            {
                retState = Adc_Set_XferDone( periphId );
            }
            else
            {
                /* Buffer is not full yet */
            }
        }
        else
        {
            /* Transfer is not running - result is dropped */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reports half filled buffer (HalfTransferCallback)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_XferHalf( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        adc_Callback_t * const callback = adc_XferContext[ periphId ].Config.HalfTransferCallback;

        if( ADC_NULL_PTR != callback )
        {
            callback();
        }
        else
        {
            /* Event is not reported */
        }

        retState = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Handles full buffer: circular buffer continues from DataBuffer[ 0 ], one shot buffer
 *        stops the regular conversion and transfer (Adc_Set_RegStop()); then
 *        TransferCompleteCallback is called (the transfer can be restarted from the callback)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_XferDone( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        adc_XferContext_t * const xferCtx  = &adc_XferContext[ periphId ];
        adc_Callback_t * const    callback = xferCtx->Config.TransferCompleteCallback;

        if( ADC_BUFFER_MODE_CIRCULAR == xferCtx->Config.BufferMode )
        {
            /* Next result is stored to the buffer start (DMA is re-armed by Adc_Dma.c) */
            xferCtx->BufferIdx = 0u;
            retState           = ADC_REQUEST_OK;
        }
        else
        {
            /* One shot - conversion and data transfer are stopped */
            retState = Adc_Set_RegStop( periphId );
        }

        if( ADC_NULL_PTR != callback )
        {
            callback();
        }
        else
        {
            /* Event is not reported */
        }
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reports a data transfer error (ErrorCallback)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param errorId  [in]: Error identification, value from \ref adc_ErrorId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_XferError( adc_PeriphId_t periphId, adc_ErrorId_t errorId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT > periphId ) &&
        ( ADC_ERROR_CNT > errorId )    )
    {
        adc_ErrCallback_t * const callback = adc_XferContext[ periphId ].Config.ErrorCallback;

        if( ADC_NULL_PTR != callback )
        {
            callback( errorId );
        }
        else
        {
            /* Error is not reported */
        }

        retState = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reports injected end of sequence (InjCompleteCallback)
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_XferInjDone( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        adc_Callback_t * const callback = adc_XferContext[ periphId ].Config.InjCompleteCallback;

        if( ADC_NULL_PTR != callback )
        {
            callback();
        }
        else
        {
            /* Event is not reported */
        }

        retState = ADC_REQUEST_OK;
    }
    else
    {
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
