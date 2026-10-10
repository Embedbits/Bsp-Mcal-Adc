/**
 * \author Mr.Nobody
 * \file Adc.c
 * \ingroup Adc
 * \brief Adc module common functionality
 *
 * STM32F7 family implementation. Differences to the families with the newer ADC IP (G4, H5):
 * - ADC clock is always PCLK2 divided by the common prescaler (2 / 4 / 6 / 8), no calibration,
 *   no internal regulator / deep power down, no differential inputs.
 * - There is no ADSTART / ADSTP: regular conversion is started by SWSTART or by enabling the
 *   external trigger (EXTEN), injected by JSWSTART or JEXTEN. The module keeps the external
 *   trigger edge and the continuous mode in a shadow and writes them to EXTEN / CONT at the start
 *   of the conversion (Adc_Set_RegStart()), Adc_Set_RegStop() clears them. A regular / injected
 *   conversion is "ongoing" while EXTEN / JEXTEN or CONT is set - a single software conversion
 *   ends by itself within a few microseconds and is not treated as ongoing.
 * - One regular end of conversion flag (EOC), the module sets it per conversion (EOCS = 1).
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
/* ============================== TYPEDEFS ================================== */

typedef struct
{
    gpio_PortId_t PortId;
    gpio_PinId_t  PinId;
}   adc_GpioConfig_t;

typedef struct
{
    adc_PeriphId_t   PeriphId;
    adc_GpioConfig_t Channel[ ADC_CHANNEL_CNT ];
}   adc_GpioPeriphConfig_t;

typedef struct
{
    ADC_TypeDef        *PeriphReg;   /**< CMSIS ADC instance register pointer  */
    rcc_PeriphId_t      RccPeriphId; /**< RCC clock enable of the peripheral   */
}   adc_PeriphConfigStruct_t;


typedef struct
{
    adc_PeriphId_t  PeriphId;
    adc_ChannelId_t ChannelId[ ADC_CHANNEL_INPUT_CNT ];
}   adc_InputConfigStruct_t;


/** Configuration kept by the module, not readable from the HW while the conversion is stopped
 *  (external trigger edge and continuous mode are written to EXTEN / JEXTEN / CONT at start) */
typedef struct
{
    adc_RegTriggerId_t   RegTriggerId;                    /**< Regular group trigger source        */
    adc_TriggerEdge_t    RegTriggerEdge;                  /**< Regular group external trigger edge */
    adc_RegTriggerMode_t RegTriggerMode;                  /**< Regular group single / continuous   */
    adc_InjTriggerId_t   InjTriggerId;                    /**< Injected group trigger source       */
    adc_TriggerEdge_t    InjTriggerEdge;                  /**< Injected group external trigger edge*/
    adc_ChannelInput_t   ChannelInput[ ADC_CHANNEL_CNT ]; /**< Input selected per channel          */
}   adc_PeriphShadow_t;


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

static adc_RequestState_t Adc_Set_InternalInput       ( adc_PeriphId_t periphId, adc_ChannelInput_t channelInput );
static adc_RequestState_t Adc_Check_ConversionStopped ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Check_PeriphDisabled    ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Check_AllPeriphsDisabled( void );
static adc_RequestState_t Adc_Check_ChannelInput      ( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput );
static adc_RequestState_t Adc_Check_ChannelSlot       ( adc_PeriphId_t periphId,
                                                        const adc_ChannelConfig_t * const channelSlot,
                                                        const adc_ChannelConfig_t ** const channelTable );
static adc_RequestState_t Adc_Check_PeriphConfig      ( const adc_PeriphConfig_t * const adcConfig );
static adc_RequestState_t Adc_Set_PeriphClock         ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Set_ConversionConfig    ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Set_PinAnalog           ( const adc_GpioConfig_t * const pinConfig );
static adc_RequestState_t Adc_Set_RegSequencer        ( const adc_PeriphConfig_t * const adcConfig );
static adc_RequestState_t Adc_Set_InjSequencer        ( const adc_PeriphConfig_t * const adcConfig );
static adc_RequestState_t Adc_Set_InjAutoMode         ( adc_PeriphId_t periphId, uint32_t llTrigAuto );
static adc_RequestState_t Adc_Set_InjDiscontMode      ( adc_PeriphId_t periphId, uint32_t llDiscont );
static adc_RequestState_t Adc_Set_Delay               ( adc_TimeUs_t delayUs );
static adc_RequestState_t Adc_Get_ClockFreq           ( adc_ClkDiv_t clkDiv, adc_FreqHz_t * const clkFreqHz );
static adc_RequestState_t Adc_Get_ActiveClockFreq     ( adc_FreqHz_t * const clkFreqHz );
static adc_RequestState_t Adc_Check_ClockFreq         ( adc_ClkDiv_t clkDiv );
static adc_RequestState_t Adc_Check_SamplingTime      ( adc_ChannelInput_t channelInput, adc_ChannelSampling_t samplingTime );
static adc_RequestState_t Adc_Check_DataConfig        ( adc_PeriphId_t periphId, adc_FunctionState_t regUsed, const adc_DataConfig_t * const dataConfig );
static adc_RequestState_t Adc_Set_XferInit            ( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig );
static adc_RequestState_t Adc_Set_XferStart           ( adc_PeriphId_t periphId );
static adc_RequestState_t Adc_Set_XferStop            ( adc_PeriphId_t periphId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define ADC_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define ADC_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define ADC_PATCH_VERSION           ( 0u )


/** Minimum ADC clock frequency after prescaler (fADC min, refer to device datasheet) */
#define ADC_CLK_FREQ_MIN_HZ          ( (adc_FreqHz_t)600000u )

/** Maximum ADC clock frequency after prescaler (fADC max for VDDA 2.4 - 3.6 V, refer to device datasheet) */
#define ADC_CLK_FREQ_MAX_HZ          ( (adc_FreqHz_t)36000000u )


/** Minimum sampling time of temperature sensor channel in ns (TS_temp, refer to device datasheet) */
#define ADC_SAMPLING_MIN_TEMP_NS     ( 10000u )

/** Minimum sampling time of internal reference voltage channel in ns (TS_vrefint, refer to device datasheet) */
#define ADC_SAMPLING_MIN_VREF_NS     ( 10000u )

/** Minimum sampling time of VBAT channel in ns (TS_vbat, refer to device datasheet) */
#define ADC_SAMPLING_MIN_VBAT_NS     ( 5000u )

/** Channel input without minimum sampling time requirement */
#define ADC_SAMPLING_MIN_NONE_NS     ( 0u )


/** ADC power-up time after ADON is set in us (tSTAB, refer to device datasheet) */
#define ADC_DELAY_STAB_US            ( 3u )


/** Count of microseconds in one second */
#define ADC_US_PER_S                 ( 1000000u )

/** Count of nanoseconds in one second */
#define ADC_NS_PER_S                 ( 1000000000u )

/** Offset between sequence length and index of the length in adc_RegSeqLenLut / adc_InjSeqLenLut
 *  (sequence length 1 is stored at index 0) */
#define ADC_SEQ_LEN_IDX_OFFSET       ( 1u )

/** Divider of the buffer size giving the half transfer position */
#define ADC_BUFFER_HALF_DIVIDER      ( 2u )


/** Regular group is running while external trigger is enabled or continuous mode is set */
#define ADC_REG_RUN_MASK             ( ADC_CR2_EXTEN | ADC_CR2_CONT )

/** Injected group is running while external trigger is enabled */
#define ADC_INJ_RUN_MASK             ( ADC_CR2_JEXTEN )


/** ADC1 channel connected to the temperature sensor (16 or 18, depends on the device) */
#define ADC_TEMP_CHANNEL             ( (adc_ChannelId_t)__LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_CHANNEL_TEMPSENSOR ) )

/** ADC1 channel connected to VREFINT */
#define ADC_VREF_CHANNEL             ( (adc_ChannelId_t)__LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_CHANNEL_VREFINT ) )

/** ADC1 channel connected to VBAT */
#define ADC_VBAT_CHANNEL             ( (adc_ChannelId_t)__LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_CHANNEL_VBAT ) )

/* =============================== MACROS =================================== */

/** Integer division rounded up (e.g. CPU cycles per microsecond, the delay is never shorter) */
#define ADC_DIV_ROUND_UP( dividend, divisor )    ( ( (dividend) + (divisor) - 1u ) / (divisor) )

/** GPIO pin wired to an ADC channel */
#define ADC_PIN( port, pin )                     { .PortId = (port), .PinId = (pin) }

/** ADC channel without GPIO pin */
#define ADC_PIN_NONE                             { .PortId = GPIO_PORT_CNT, .PinId = GPIO_PIN_ID_CNT }

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** GPIO pins of the ADC channels (ADC123_INx / ADC12_INx / ADC3_INx, refer to device datasheet) */
static const adc_GpioPeriphConfig_t adc_GpioPeriphConfig[ ADC_PERIPH_CNT ] =
{
 { .PeriphId = ADC_PERIPH_1,
   .Channel  = { ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_0 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_1 ),
                 ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_2 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_3 ),
                 ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_4 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_5 ),
                 ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_6 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_7 ),
                 ADC_PIN( GPIO_PORT_B, GPIO_PIN_ID_0 ), ADC_PIN( GPIO_PORT_B, GPIO_PIN_ID_1 ),
                 ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_0 ), ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_1 ),
                 ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_2 ), ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_3 ),
                 ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_4 ), ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_5 ),
                 ADC_PIN_NONE,                          ADC_PIN_NONE,
                 ADC_PIN_NONE } },
#if defined (ADC2)
 { .PeriphId = ADC_PERIPH_2,
   .Channel  = { ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_0 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_1 ),
                 ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_2 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_3 ),
                 ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_4 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_5 ),
                 ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_6 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_7 ),
                 ADC_PIN( GPIO_PORT_B, GPIO_PIN_ID_0 ), ADC_PIN( GPIO_PORT_B, GPIO_PIN_ID_1 ),
                 ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_0 ), ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_1 ),
                 ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_2 ), ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_3 ),
                 ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_4 ), ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_5 ),
                 ADC_PIN_NONE,                          ADC_PIN_NONE,
                 ADC_PIN_NONE } },
#endif /* ADC2 */
#if defined (ADC3)
 { .PeriphId = ADC_PERIPH_3,
   .Channel  = { ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_0 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_1  ),
                 ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_2 ), ADC_PIN( GPIO_PORT_A, GPIO_PIN_ID_3  ),
                 ADC_PIN( GPIO_PORT_F, GPIO_PIN_ID_6 ), ADC_PIN( GPIO_PORT_F, GPIO_PIN_ID_7  ),
                 ADC_PIN( GPIO_PORT_F, GPIO_PIN_ID_8 ), ADC_PIN( GPIO_PORT_F, GPIO_PIN_ID_9  ),
                 ADC_PIN( GPIO_PORT_F, GPIO_PIN_ID_10), ADC_PIN( GPIO_PORT_F, GPIO_PIN_ID_3  ),
                 ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_0 ), ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_1  ),
                 ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_2 ), ADC_PIN( GPIO_PORT_C, GPIO_PIN_ID_3  ),
                 ADC_PIN( GPIO_PORT_F, GPIO_PIN_ID_4 ), ADC_PIN( GPIO_PORT_F, GPIO_PIN_ID_5  ),
                 ADC_PIN_NONE,                          ADC_PIN_NONE,
                 ADC_PIN_NONE } },
#endif /* ADC3 */
};

/** CMSIS instance and RCC clock lookup, indexed by adc_PeriphId_t. All ADC peripherals
 *  share one common register block (ADC1_COMMON / ADC12_COMMON / ADC123_COMMON). */
static const adc_PeriphConfigStruct_t adc_PeriphConf[ ] =
{
    { .PeriphReg = ADC1, .RccPeriphId = RCC_PERIPH_ADC1 },
#if defined (ADC2)
    { .PeriphReg = ADC2, .RccPeriphId = RCC_PERIPH_ADC2 },
#endif /* ADC2 */
#if defined (ADC3)
    { .PeriphReg = ADC3, .RccPeriphId = RCC_PERIPH_ADC3 },
#endif /* ADC3 */
};

_Static_assert( ADC_PERIPH_CNT == ( sizeof(adc_PeriphConf) / sizeof(adc_PeriphConfigStruct_t) ), "Adc: adc_PeriphConf has incorrect size." );


/** Internal channel mapping per peripheral (ADC_CHANNEL_CNT == input not available), taken over
 *  from LL_ADC_CHANNEL_TEMPSENSOR / VREFINT / VBAT definitions (internal channels on ADC1 only). */
static const adc_InputConfigStruct_t    adc_InputConfig[ ] =
{
 { .PeriphId = ADC_PERIPH_1,
   .ChannelId[ ADC_CHANNEL_INPUT_PIN_SINGLE ] = ADC_CHANNEL_CNT ,
   .ChannelId[ ADC_CHANNEL_INPUT_TEMP       ] = ADC_TEMP_CHANNEL,
   .ChannelId[ ADC_CHANNEL_INPUT_VREF       ] = ADC_VREF_CHANNEL,
   .ChannelId[ ADC_CHANNEL_INPUT_VBAT       ] = ADC_VBAT_CHANNEL,
 },
#if defined (ADC2)
 { .PeriphId = ADC_PERIPH_2,
   .ChannelId[ ADC_CHANNEL_INPUT_PIN_SINGLE ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_TEMP       ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_VREF       ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_VBAT       ] = ADC_CHANNEL_CNT,
 },
#endif /* ADC2 */
#if defined (ADC3)
 { .PeriphId = ADC_PERIPH_3,
   .ChannelId[ ADC_CHANNEL_INPUT_PIN_SINGLE ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_TEMP       ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_VREF       ] = ADC_CHANNEL_CNT,
   .ChannelId[ ADC_CHANNEL_INPUT_VBAT       ] = ADC_CHANNEL_CNT,
 },
#endif /* ADC3 */
};

_Static_assert( ADC_PERIPH_CNT == ( sizeof(adc_InputConfig) / sizeof(adc_InputConfigStruct_t) ), "Adc: adc_InputConfig has incorrect size." );


/** adc_ClkDiv_t -> LL_ADC_CLOCK_SYNC_PCLK_DIVx (ADC_CCR ADCPRE) */
static const uint32_t adc_ClkDivLut[ ADC_CLK_DIV_CNT ] =
{
    [ADC_CLK_DIV_2] = LL_ADC_CLOCK_SYNC_PCLK_DIV2,
    [ADC_CLK_DIV_4] = LL_ADC_CLOCK_SYNC_PCLK_DIV4,
    [ADC_CLK_DIV_6] = LL_ADC_CLOCK_SYNC_PCLK_DIV6,
    [ADC_CLK_DIV_8] = LL_ADC_CLOCK_SYNC_PCLK_DIV8,
};


/** adc_ClkDiv_t -> numeric value of the ADC clock divider */
static const uint32_t adc_ClkDivValueLut[ ADC_CLK_DIV_CNT ] =
{
    [ADC_CLK_DIV_2] = 2u,
    [ADC_CLK_DIV_4] = 4u,
    [ADC_CLK_DIV_6] = 6u,
    [ADC_CLK_DIV_8] = 8u,
};


/** adc_RegTriggerId_t -> ADC_CR2 EXTSEL value. Entries follow the exact order and \#if guarding
 *  of adc_RegTriggerId_t in Adc_Types.h. Software trigger has no EXTSEL value (EXTEN = 0), the
 *  field is written with 0. */
static const uint32_t adc_RegTriggerSrcLut[ ] =
{
    [ADC_REG_TRIGGER_SOFTWARE]        = 0u,
    [ADC_REG_TRIGGER_EXT_TIM1_CH1]    = ( LL_ADC_REG_TRIG_EXT_TIM1_CH1    & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM1_CH2]    = ( LL_ADC_REG_TRIG_EXT_TIM1_CH2    & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM1_CH3]    = ( LL_ADC_REG_TRIG_EXT_TIM1_CH3    & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM2_CH2]    = ( LL_ADC_REG_TRIG_EXT_TIM2_CH2    & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM5_TRGO]   = ( LL_ADC_REG_TRIG_EXT_TIM5_TRGO   & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM4_CH4]    = ( LL_ADC_REG_TRIG_EXT_TIM4_CH4    & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM3_CH4]    = ( LL_ADC_REG_TRIG_EXT_TIM3_CH4    & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM8_TRGO]   = ( LL_ADC_REG_TRIG_EXT_TIM8_TRGO   & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM8_TRGO2]  = ( LL_ADC_REG_TRIG_EXT_TIM8_TRGO2  & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM1_TRGO]   = ( LL_ADC_REG_TRIG_EXT_TIM1_TRGO   & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM1_TRGO2]  = ( LL_ADC_REG_TRIG_EXT_TIM1_TRGO2  & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM2_TRGO]   = ( LL_ADC_REG_TRIG_EXT_TIM2_TRGO   & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM4_TRGO]   = ( LL_ADC_REG_TRIG_EXT_TIM4_TRGO   & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_TIM6_TRGO]   = ( LL_ADC_REG_TRIG_EXT_TIM6_TRGO   & ADC_CR2_EXTSEL ),
    [ADC_REG_TRIGGER_EXT_EXTI_LINE11] = ( LL_ADC_REG_TRIG_EXT_EXTI_LINE11 & ADC_CR2_EXTSEL ),
};

_Static_assert( ADC_REG_TRIGGER_CNT == ( sizeof(adc_RegTriggerSrcLut) / sizeof(uint32_t) ), "Adc: adc_RegTriggerSrcLut has incorrect size." );


/** adc_InjTriggerId_t -> ADC_CR2 JEXTSEL value. Entries follow the exact order and \#if guarding
 *  of adc_InjTriggerId_t in Adc_Types.h. Software trigger and ADC_INJ_TRIGGER_AUTO have no own
 *  trigger source - auto-injected mode (JAUTO) requires the injected external trigger disabled. */
static const uint32_t adc_InjTriggerSrcLut[ ] =
{
    [ADC_INJ_TRIGGER_SOFTWARE]        = 0u,
    [ADC_INJ_TRIGGER_AUTO]            = 0u,
    [ADC_INJ_TRIGGER_EXT_TIM1_TRGO]   = ( LL_ADC_INJ_TRIG_EXT_TIM1_TRGO   & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM1_CH4]    = ( LL_ADC_INJ_TRIG_EXT_TIM1_CH4    & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM2_TRGO]   = ( LL_ADC_INJ_TRIG_EXT_TIM2_TRGO   & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM2_CH1]    = ( LL_ADC_INJ_TRIG_EXT_TIM2_CH1    & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM3_CH4]    = ( LL_ADC_INJ_TRIG_EXT_TIM3_CH4    & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM4_TRGO]   = ( LL_ADC_INJ_TRIG_EXT_TIM4_TRGO   & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM8_CH4]    = ( LL_ADC_INJ_TRIG_EXT_TIM8_CH4    & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM1_TRGO2]  = ( LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2  & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM8_TRGO]   = ( LL_ADC_INJ_TRIG_EXT_TIM8_TRGO   & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM8_TRGO2]  = ( LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2  & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM3_CH3]    = ( LL_ADC_INJ_TRIG_EXT_TIM3_CH3    & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM5_TRGO]   = ( LL_ADC_INJ_TRIG_EXT_TIM5_TRGO   & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM3_CH1]    = ( LL_ADC_INJ_TRIG_EXT_TIM3_CH1    & ADC_CR2_JEXTSEL ),
    [ADC_INJ_TRIGGER_EXT_TIM6_TRGO]   = ( LL_ADC_INJ_TRIG_EXT_TIM6_TRGO   & ADC_CR2_JEXTSEL ),
};

_Static_assert( ADC_INJ_TRIGGER_CNT == ( sizeof(adc_InjTriggerSrcLut) / sizeof(uint32_t) ), "Adc: adc_InjTriggerSrcLut has incorrect size." );


/** adc_TriggerEdge_t -> ADC_CR2 EXTEN value (regular group) */
static const uint32_t adc_RegTriggerEdgeLut[ ADC_TRIGGER_EDGE_CNT ] =
{
    [ADC_TRIGGER_EDGE_RISING]  = LL_ADC_REG_TRIG_EXT_RISING,
    [ADC_TRIGGER_EDGE_FALLING] = LL_ADC_REG_TRIG_EXT_FALLING,
    [ADC_TRIGGER_EDGE_BOTH]    = LL_ADC_REG_TRIG_EXT_RISINGFALLING,
};


/** adc_TriggerEdge_t -> ADC_CR2 JEXTEN value (injected group) */
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
    [ADC_RESOLUTION_6BIT]  = LL_ADC_RESOLUTION_6B,
};


/** adc_ChannelSampling_t -> LL_ADC_SAMPLINGTIME_x */
static const uint32_t adc_SamplingTimeLut[ ADC_CHANNEL_SAMPLING_CNT ] =
{
    [ADC_CHANNEL_SAMPLING_3_CYCLES]   = LL_ADC_SAMPLINGTIME_3CYCLES,
    [ADC_CHANNEL_SAMPLING_15_CYCLES]  = LL_ADC_SAMPLINGTIME_15CYCLES,
    [ADC_CHANNEL_SAMPLING_28_CYCLES]  = LL_ADC_SAMPLINGTIME_28CYCLES,
    [ADC_CHANNEL_SAMPLING_56_CYCLES]  = LL_ADC_SAMPLINGTIME_56CYCLES,
    [ADC_CHANNEL_SAMPLING_84_CYCLES]  = LL_ADC_SAMPLINGTIME_84CYCLES,
    [ADC_CHANNEL_SAMPLING_112_CYCLES] = LL_ADC_SAMPLINGTIME_112CYCLES,
    [ADC_CHANNEL_SAMPLING_144_CYCLES] = LL_ADC_SAMPLINGTIME_144CYCLES,
    [ADC_CHANNEL_SAMPLING_480_CYCLES] = LL_ADC_SAMPLINGTIME_480CYCLES,
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


/** adc_ChannelSampling_t -> sampling time in ADC clock cycles */
static const uint32_t adc_SamplingCyclesLut[ ADC_CHANNEL_SAMPLING_CNT ] =
{
    [ADC_CHANNEL_SAMPLING_3_CYCLES]   = 3u,
    [ADC_CHANNEL_SAMPLING_15_CYCLES]  = 15u,
    [ADC_CHANNEL_SAMPLING_28_CYCLES]  = 28u,
    [ADC_CHANNEL_SAMPLING_56_CYCLES]  = 56u,
    [ADC_CHANNEL_SAMPLING_84_CYCLES]  = 84u,
    [ADC_CHANNEL_SAMPLING_112_CYCLES] = 112u,
    [ADC_CHANNEL_SAMPLING_144_CYCLES] = 144u,
    [ADC_CHANNEL_SAMPLING_480_CYCLES] = 480u,
};


/** adc_ChannelInput_t -> minimum sampling time in ns required by the connected signal */
static const adc_TimeNs_t adc_SamplingMinNsLut[ ADC_CHANNEL_INPUT_CNT ] =
{
    [ADC_CHANNEL_INPUT_PIN_SINGLE] = ADC_SAMPLING_MIN_NONE_NS,
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


/** adc_FlagId_t -> ADC_SR bit */
static const uint32_t adc_FlagLut[ ADC_FLAG_CNT ] =
{
    [ADC_FLAG_REG_EOC]   = ADC_SR_EOC,
    [ADC_FLAG_REG_OVR]   = ADC_SR_OVR,
    [ADC_FLAG_REG_START] = ADC_SR_STRT,
    [ADC_FLAG_INJ_EOS]   = ADC_SR_JEOC,
    [ADC_FLAG_INJ_START] = ADC_SR_JSTRT,
    [ADC_FLAG_AWD1]      = ADC_SR_AWD,
};


/** Configuration kept by the module per peripheral (zero initialization: software triggers,
 *  rising edges, single mode, single-ended pin inputs) */
static adc_PeriphShadow_t adc_Shadow[ ADC_PERIPH_CNT ];


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
 * Configures the common ADC clock source and clock divider, then initializes every
 * peripheral from adcConfig->PeriphConfig[] that is in use (see \ref Adc_PeriphInit()).
 *
 * \note  adcConfig->PeriphConfig[] is indexed by \ref adc_PeriphId_t. A slot with
 *        RegChannelsCnt == 0 and InjChannelsCnt == 0 is treated as "peripheral not used" and
 *        is skipped. For a used slot, PeriphConfig[ i ].PeriphId must be equal to i, otherwise
 *        \ref ADC_REQUEST_ERROR is returned.
 *
 * \note  ADC clock frequency (PCLK2 / divider) must be within ADC_CLK_FREQ_MIN_HZ -
 *        ADC_CLK_FREQ_MAX_HZ, it is checked before any register is modified.
 *
 * \note  If initialization of any peripheral fails, all peripherals initialized by this call
 *        are deinitialized (see \ref Adc_Deinit()) and \ref ADC_REQUEST_ERROR is returned.
 *
 * \pre   All ADC peripherals must be disabled (ADON = 0), because the setting is shared
 *        through the ADC common register block. Otherwise \ref ADC_REQUEST_ERROR is
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

        /* Resulting ADC clock frequency is validated before any register is modified */
        if( ADC_CLK_SRC_CNT > adcConfig->ClockSource )
        {
            retState = Adc_Check_ClockFreq( adcConfig->ClockDivider );
        }
        else
        {
            retState = ADC_REQUEST_ERROR;
        }

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_ClockSource( adcConfig->ClockSource );
        }
        else
        {
            /* Previous step failed */
        }

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_ClockDivider( adcConfig->ClockDivider );
        }
        else
        {
            /* Previous step failed */
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

        for( adc_PeriphId_t periphIdx = ADC_PERIPH_1;
             ( ADC_PERIPH_CNT > periphIdx ) &&
             ( ADC_REQUEST_OK == retState );
             periphIdx ++ )
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

        if( ( ADC_REQUEST_OK      != retState  ) &&
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
 *        (internal channels, see Adc_Check_SamplingTime()) at the active ADC clock.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral (STM32F7 channel
 *        settings do not require the ADC to be disabled). Otherwise \ref ADC_REQUEST_ERROR
 *        is returned and no register is modified.
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
        const adc_RequestState_t convState     = Adc_Check_ConversionStopped( periphId );
        const adc_RequestState_t samplingState = Adc_Check_SamplingTime( channelConfig->ChannelInput, channelConfig->ChannelSampling );

        if( ADC_REQUEST_OK != convState )
        {
            /* Conversion is ongoing, channel configuration is not allowed */
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
 * releases data transfer handler resources (DMA stream, interrupts) and disables the peripheral
 * (Adc_Set_PeriphInactive()). Configuration kept by the module (trigger sources, edges,
 * continuous mode, channel inputs) is reset.
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
        /* --- Stop ongoing conversions (register access verified by called functions) --- */
        retState = Adc_Set_RegStop( periphId );

        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_InjStop( periphId );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Release data transfer handler resources (DMA stream, interrupts) --- */
        if( ( ADC_REQUEST_OK      == retState                              ) &&
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
            /* Previous step failed */
        }

        if( ADC_REQUEST_OK == retState )
        {
            adc_PeriphShadow_t * const shadow = &adc_Shadow[ periphId ];

            shadow->RegTriggerId   = ADC_REG_TRIGGER_SOFTWARE;
            shadow->RegTriggerEdge = ADC_TRIGGER_EDGE_RISING;
            shadow->RegTriggerMode = ADC_REG_TRIGGER_MODE_SINGLE;
            shadow->InjTriggerId   = ADC_INJ_TRIGGER_SOFTWARE;
            shadow->InjTriggerEdge = ADC_TRIGGER_EDGE_RISING;

            for( adc_ChannelId_t channelIdx = ADC_CHANNEL_0; ADC_CHANNEL_CNT > channelIdx; channelIdx ++ )
            {
                shadow->ChannelInput[ channelIdx ] = ADC_CHANNEL_INPUT_PIN_SINGLE;
            }
        }
        else
        {
            /* Previous step failed */
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
 * \brief Selects the ADC clock source and enables the clock of all ADC peripherals
 *
 * \note  STM32F7 ADC clock is always PCLK2 (\ref ADC_CLK_SRC_PCLK2). The function enables the
 *        RCC clock of every ADC peripheral, so the common register block (clock divider,
 *        internal channel paths) is accessible.
 *
 * \pre   All ADC peripherals must be disabled (ADON = 0), because the setting is shared
 *        through the ADC common register block. Otherwise \ref ADC_REQUEST_ERROR is
 *        returned and no register is modified.
 *
 * \param clkSource [in]: Required ADC clock source, value from \ref adc_ClkSrc_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_ClockSource( adc_ClkSrc_t clkSource )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_CLK_SRC_CNT > clkSource )
    {
        const adc_RequestState_t periphsState = Adc_Check_AllPeriphsDisabled( );

        if( ADC_REQUEST_OK == periphsState )
        {
            retState = ADC_REQUEST_OK;

            for( adc_PeriphId_t periphIdx = ADC_PERIPH_1;
                 ( ADC_PERIPH_CNT > periphIdx ) &&
                 ( ADC_REQUEST_OK == retState );
                 periphIdx ++ )
            {
                retState = Adc_Set_PeriphClock( periphIdx );
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
 * \brief Returns the ADC clock source
 *
 * \note  STM32F7 ADC clock source is fixed - \ref ADC_CLK_SRC_PCLK2 is always returned.
 *
 * \param clkSource [out]: Pointer to store the clock source (\ref adc_ClkSrc_t).
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
        *clkSource = ADC_CLK_SRC_PCLK2;
        retState   = ADC_REQUEST_OK;
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/**
 * \brief Configures the ADC common clock divider (ADC_CCR ADCPRE)
 *
 * \note  Resulting ADC clock frequency (PCLK2 / divider) must be within ADC_CLK_FREQ_MIN_HZ -
 *        ADC_CLK_FREQ_MAX_HZ. Otherwise \ref ADC_REQUEST_ERROR is returned and no register is
 *        modified.
 *
 * \pre   All ADC peripherals must be disabled (ADON = 0), because the setting is shared
 *        through the ADC common register block, and the ADC clock must be enabled (see
 *        Adc_Set_ClockSource()). Otherwise \ref ADC_REQUEST_ERROR is returned.
 *
 * \param clkDiv [in]: Required ADC common clock divider, value from \ref adc_ClkDiv_t.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_ClockDivider( adc_ClkDiv_t clkDiv )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_CLK_DIV_CNT > clkDiv )
    {
        const adc_RequestState_t periphsState = Adc_Check_AllPeriphsDisabled( );
        const adc_RequestState_t clkState     = Adc_Check_ClockFreq( clkDiv );

        if( ADC_REQUEST_OK != clkState )
        {
            /* Resulting ADC clock frequency is out of the allowed range */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_REQUEST_OK == periphsState )
        {
            ADC_Common_TypeDef * const commonReg = __LL_ADC_COMMON_INSTANCE( ADC1 );

            LL_ADC_SetCommonClock( commonReg, adc_ClkDivLut[ clkDiv ] );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t clkDivReg = LL_ADC_GetCommonClock( commonReg );

                if( adc_ClkDivLut[ clkDiv ] == clkDivReg )
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
        const uint32_t llClkDiv = LL_ADC_GetCommonClock( __LL_ADC_COMMON_INSTANCE( ADC1 ) );

        for( adc_ClkDiv_t idx = ADC_CLK_DIV_2; ADC_CLK_DIV_CNT > idx; idx ++ )
        {
            if( adc_ClkDivLut[ idx ] == llClkDiv )
            {
                *clkDiv  = idx;
                retState = ADC_REQUEST_OK;
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


/* -------------------------------------------------------------------------- */
/* ------------------------ Peripheral configuration ------------------------ */
/* -------------------------------------------------------------------------- */

/**
 * \brief Initializes an ADC peripheral: clock, channel inputs and sampling times, enable
 *        (Adc_Set_PeriphActive()), resolution, conversion mode (scan, EOC per conversion,
 *        right alignment), regular sequencer + trigger and injected sequencer + trigger
 *
 * Configuration rules (the whole configuration is validated before any register is modified):
 * - RegChannelsCnt: 0 - 16, InjChannelsCnt: 0 - 4, at least one of them must be non-zero.
 * - Only the first RegChannelsCnt / InjChannelsCnt slots of RegChannels[] / InjChannels[] are
 *   used, the remaining slots are ignored. Slot index selects the rank (RegChannels[ 0 ] == rank 1).
 * - Every used slot must have a valid ChannelId, ChannelInput and ChannelSampling. ChannelInput
 *   must be available for the channel on the peripheral (GPIO pin wired for PIN_SINGLE,
 *   internal signal connected to ChannelId for TEMP / VREF / VBAT).
 * - ChannelSampling must satisfy the minimum sampling time of the channel input at the active
 *   ADC clock (see Adc_Check_SamplingTime()).
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
 * \pre   ADC peripheral must be disabled (ADON = 0). Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified. To reconfigure an already initialized peripheral call
 *        Adc_Deinit() first.
 *
 * \param adcConfig [in]: Pointer to peripheral configuration structure \ref adc_PeriphConfig_t.
 *                        Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_PeriphInit( adc_PeriphConfig_t * const adcConfig )
{
    adc_RequestState_t  retState    = ADC_REQUEST_ERROR;
    adc_FunctionState_t initStarted = ADC_FUNCTION_INACTIVE;

    /* Complete configuration is validated before any register is modified (NULL included) */
    retState = Adc_Check_PeriphConfig( adcConfig );

    if( ADC_REQUEST_OK == retState )
    {
        /* ADC has to be disabled before (re)initialization */
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

        /* --- Peripheral clock (normally enabled already by Adc_Set_ClockSource()) --- */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_PeriphClock( adcConfig->PeriphId );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Channel input and sampling time (regular and injected slots) --- */
        for( adc_RegSequenceId_t slotIdx = ADC_REG_SEQUENCE_1;
             ( adcConfig->RegChannelsCnt > slotIdx ) &&
             ( ADC_REQUEST_OK == retState );
             slotIdx ++ )
        {
            retState = Adc_ChannelInit( adcConfig->PeriphId, &adcConfig->RegChannels[ slotIdx ] );
        }

        for( adc_InjSequenceId_t slotIdx = ADC_INJ_SEQUENCE_1;
             ( adcConfig->InjChannelsCnt > slotIdx ) &&
             ( ADC_REQUEST_OK == retState );
             slotIdx ++ )
        {
            retState = Adc_ChannelInit( adcConfig->PeriphId, &adcConfig->InjChannels[ slotIdx ] );
        }

        /* --- Enable (register access verified by called function) --- */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_PeriphActive( adcConfig->PeriphId );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Instance-wide resolution (adc_PeriphConfig_t.Resolution) --- */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_Resolution( adcConfig->PeriphId, adcConfig->Resolution );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Scan mode, EOC flag per conversion, right data alignment --- */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_ConversionConfig( adcConfig->PeriphId );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Regular group: sequencer, trigger source, trigger edge and conversion mode --- */
        if( ( ADC_REQUEST_OK == retState                  ) &&
            ( 0u             <  adcConfig->RegChannelsCnt )    )
        {
            retState = Adc_Set_RegSequencer( adcConfig );

            if( ADC_REQUEST_OK == retState )
            {
                /* Trigger source change keeps the previous edge, configured edge is written afterwards */
                retState = Adc_Set_TriggerSrc( adcConfig->PeriphId, adcConfig->RegTriggerId );
            }
            else
            {
                /* Previous step failed */
            }

            if( ( ADC_REQUEST_OK           == retState                ) &&
                ( ADC_REG_TRIGGER_SOFTWARE != adcConfig->RegTriggerId )    )
            {
                retState = Adc_Set_TriggerEdge( adcConfig->PeriphId, adcConfig->RegTriggerEdge );
            }
            else
            {
                /* No action required */
            }

            if( ADC_REQUEST_OK == retState )
            {
                retState = Adc_Set_TriggerMode( adcConfig->PeriphId, adcConfig->RegTriggerMode );
            }
            else
            {
                /* Previous step failed */
            }
        }
        else
        {
            /* No action required */
        }

        /* --- Injected group: sequencer, trigger source and trigger edge --- */
        if( ( ADC_REQUEST_OK == retState                  ) &&
            ( 0u             <  adcConfig->InjChannelsCnt )    )
        {
            retState = Adc_Set_InjSequencer( adcConfig );
        }
        else
        {
            /* No action required */
        }

        /* --- Data transfer handler (DMA / ISR / POLL) --- */
        if( ADC_REQUEST_OK == retState )
        {
            retState = Adc_Set_XferInit( adcConfig->PeriphId, &adcConfig->DataConfig );
        }
        else
        {
            /* Previous step failed */
        }

        /* --- Initialization result --- */
        if( ( ADC_REQUEST_OK      != retState    ) &&
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
        /* Previous step failed */
    }

    return ( retState );
}


/**
 * \brief Selects the regular group conversion trigger source
 *
 * \note  If the required trigger source is already selected, nothing is modified (trigger edge
 *        is kept).
 *
 * \note  On change to an external trigger source the currently configured trigger edge is
 *        kept. If the current trigger source is software (no edge configured), rising edge
 *        is used - call Adc_Set_TriggerEdge() afterwards if another edge is needed.
 *
 * \note  External trigger is enabled (EXTEN) only while the regular conversion is started
 *        (Adc_Set_RegStart() - Adc_Set_RegStop()), EXTSEL is written immediately.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and no register is modified.
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
            ADC_TypeDef * const        periphReg = adc_PeriphConf[ periphId ].PeriphReg;
            adc_PeriphShadow_t * const shadow    = &adc_Shadow[ periphId ];
            const uint32_t             llSource  = adc_RegTriggerSrcLut[ triggerSrc ];

            if( triggerSrc == shadow->RegTriggerId )
            {
                /* Trigger source is already selected, no change - trigger edge is kept */
                retState = ADC_REQUEST_OK;
            }
            else
            {
                LL_ADC_REG_SetTriggerSource( periphReg, llSource );

                for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    const uint32_t sourceReg = READ_BIT( periphReg->CR2, ADC_CR2_EXTSEL );

                    if( llSource == sourceReg )
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

                if( ADC_REQUEST_OK == retState )
                {
                    if( ADC_REG_TRIGGER_SOFTWARE == shadow->RegTriggerId )
                    {
                        /* Change from software trigger - no edge configured yet, rising edge is used */
                        shadow->RegTriggerEdge = ADC_TRIGGER_EDGE_RISING;
                    }
                    else
                    {
                        /* Change between external triggers (current edge is kept) or to software trigger */
                    }

                    shadow->RegTriggerId = triggerSrc;
                }
                else
                {
                    /* Trigger source was not applied, configuration is kept */
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
 * \note  Software trigger is kept by the module (EXTEN = 0 while stopped), external trigger
 *        source is read back from EXTSEL.
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
        if( ADC_REG_TRIGGER_SOFTWARE == adc_Shadow[ periphId ].RegTriggerId )
        {
            *triggerSrc = ADC_REG_TRIGGER_SOFTWARE;
            retState    = ADC_REQUEST_OK;
        }
        else
        {
            const uint32_t llTriggerSrc = READ_BIT( adc_PeriphConf[ periphId ].PeriphReg->CR2, ADC_CR2_EXTSEL );

            /* Software trigger shares EXTSEL value 0 with the first external trigger - skipped */
            for( adc_RegTriggerId_t triggerId = (adc_RegTriggerId_t)( ADC_REG_TRIGGER_SOFTWARE + 1u ); ADC_REG_TRIGGER_CNT > triggerId; triggerId ++ )
            {
                if( adc_RegTriggerSrcLut[ triggerId ] == llTriggerSrc )
                {
                    *triggerSrc = triggerId;
                    retState    = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* No action required */
                }
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
 * \note  Continuous mode (CONT) is written to the HW by Adc_Set_RegStart() and cleared by
 *        Adc_Set_RegStop() - CONT is the stop mechanism of a continuous conversion on STM32F7.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and nothing is changed.
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

    if( ( ADC_PERIPH_CNT           > periphId    ) &&
        ( ADC_REG_TRIGGER_MODE_CNT > triggerMode )    )
    {
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

        if( ADC_REQUEST_OK == convState )
        {
            adc_Shadow[ periphId ].RegTriggerMode = triggerMode;
            retState                              = ADC_REQUEST_OK;
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

    if( ( ADC_PERIPH_CNT >  periphId    ) &&
        ( ADC_NULL_PTR   != triggerMode )    )
    {
        *triggerMode = adc_Shadow[ periphId ].RegTriggerMode;
        retState     = ADC_REQUEST_OK;
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
 * \note  The edge is written to the HW (EXTEN) by Adc_Set_RegStart().
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and nothing is changed.
 *
 * \pre   Regular group trigger source must be external (see Adc_Set_TriggerSrc()).
 *        For software trigger the request is rejected: \ref ADC_REQUEST_ERROR is returned.
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

    if( ( ADC_PERIPH_CNT       > periphId    ) &&
        ( ADC_TRIGGER_EDGE_CNT > triggerEdge )    )
    {
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

        if( ADC_REQUEST_OK != convState )
        {
            /* Conversion is ongoing, configuration change is not allowed */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_REG_TRIGGER_SOFTWARE == adc_Shadow[ periphId ].RegTriggerId )
        {
            /* Software trigger is selected, trigger edge is not applicable */
            retState = ADC_REQUEST_ERROR;
        }
        else
        {
            adc_Shadow[ periphId ].RegTriggerEdge = triggerEdge;
            retState                              = ADC_REQUEST_OK;
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
 * \note  For software trigger no edge is configured and \ref ADC_REQUEST_ERROR is returned.
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

    if( ( ADC_PERIPH_CNT >  periphId    ) &&
        ( ADC_NULL_PTR   != triggerEdge )    )
    {
        if( ADC_REG_TRIGGER_SOFTWARE != adc_Shadow[ periphId ].RegTriggerId )
        {
            *triggerEdge = adc_Shadow[ periphId ].RegTriggerEdge;
            retState     = ADC_REQUEST_OK;
        }
        else
        {
            /* Software trigger has no edge */
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
 * \brief Enables an ADC peripheral (ADON = 1) and waits for the ADC power-up time (tSTAB)
 *
 * \note  STM32F7 ADC has no self-calibration and no ready flag.
 *
 * \pre   ADC peripheral must be disabled (ADON = 0) and the active ADC clock frequency within
 *        ADC_CLK_FREQ_MIN_HZ - ADC_CLK_FREQ_MAX_HZ. Otherwise \ref ADC_REQUEST_ERROR is returned
 *        and no register is modified.
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
        adc_FreqHz_t             clkFreqHz   = 0u;
        const adc_RequestState_t periphState = Adc_Check_PeriphDisabled( periphId );
        const adc_RequestState_t clkState    = Adc_Get_ActiveClockFreq( &clkFreqHz );

        if( ADC_REQUEST_OK != periphState )
        {
            /* ADC is already enabled */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ( ADC_REQUEST_OK      != clkState  ) ||
                 ( ADC_CLK_FREQ_MIN_HZ  > clkFreqHz ) ||
                 ( ADC_CLK_FREQ_MAX_HZ  < clkFreqHz )    )
        {
            /* ADC clock frequency is out of the allowed range */
            retState = ADC_REQUEST_ERROR;
        }
        else
        {
            LL_ADC_Enable( periphReg );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t periphEnabled = LL_ADC_IsEnabled( periphReg );

                if( 0u != periphEnabled )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* ADC enable has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }

            /* HW provides no ready flag - ADC power-up time tSTAB is waited */
            if( ADC_REQUEST_OK == retState )
            {
                retState = Adc_Set_Delay( ADC_DELAY_STAB_US );
            }
            else
            {
                /* Previous step failed */
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
 * \brief Disables an ADC peripheral (ADON = 0)
 *
 * \note  Already disabled peripheral is accepted.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral
 *        (see Adc_Set_RegStop() / Adc_Set_InjStop()). Otherwise \ref ADC_REQUEST_ERROR is
 *        returned and no register is modified.
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
            ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

            LL_ADC_Disable( periphReg );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t enabledReg = LL_ADC_IsEnabled( periphReg );

                if( 0u == enabledReg )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* ADC disable has not yet been applied, keep return state as error */
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
 * \brief Starts the regular group conversion
 *
 * - software trigger: continuous mode is applied (CONT) and the conversion is started (SWSTART),
 * - external trigger: continuous mode and the trigger edge (EXTEN) are applied, the ADC starts
 *   to accept trigger events.
 *
 * \note  Regular data transfer (DataConfig) is armed first: if it is not running, the buffer is
 *        filled from DataBuffer[ 0 ]. If it is already running (buffer not yet full, e.g. single
 *        mode with several software starts), it continues at the current position.
 *
 * \note  SWSTART is cleared by HW when the conversion starts, so the write can not be verified
 *        by read-back. Conversion progress is signalled by \ref ADC_FLAG_REG_START /
 *        \ref ADC_FLAG_REG_EOC (see Adc_Get_Flag()).
 *
 * \pre   ADC peripheral must be enabled (ADON = 1) and no regular conversion may be ongoing
 *        (external trigger / continuous mode not running). Otherwise \ref ADC_REQUEST_ERROR is
 *        returned and no register is modified.
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
        ADC_TypeDef * const              periphReg     = adc_PeriphConf[ periphId ].PeriphReg;
        const adc_PeriphShadow_t * const shadow        = &adc_Shadow[ periphId ];
        const uint32_t                   periphEnabled = LL_ADC_IsEnabled( periphReg );
        const uint32_t                   regRunning    = READ_BIT( periphReg->CR2, ADC_REG_RUN_MASK );

        if( ( 0u != periphEnabled ) &&
            ( 0u == regRunning    )    )
        {
            uint32_t llRunBits = 0u;

            if( ADC_REG_TRIGGER_MODE_CONTINUOUS == shadow->RegTriggerMode )
            {
                llRunBits |= ADC_CR2_CONT;
            }
            else
            {
                /* Single mode - one sequence per trigger */
            }

            if( ADC_REG_TRIGGER_SOFTWARE != shadow->RegTriggerId )
            {
                llRunBits |= adc_RegTriggerEdgeLut[ shadow->RegTriggerEdge ];
            }
            else
            {
                /* Software trigger - external trigger stays disabled */
            }

            /* Regular data transfer (DMA / ISR / POLL) is armed before the conversion starts */
            retState = Adc_Set_XferStart( periphId );

            if( ( ADC_REQUEST_OK == retState  ) &&
                ( 0u             != llRunBits )    )
            {
                MODIFY_REG( periphReg->CR2, ADC_REG_RUN_MASK, llRunBits );

                for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    const uint32_t runReg = READ_BIT( periphReg->CR2, ADC_REG_RUN_MASK );

                    if( llRunBits == runReg )
                    {
                        retState = ADC_REQUEST_OK;
                        break;
                    }
                    else
                    {
                        /* Conversion mode / trigger edge has not yet been applied, keep return state as error */
                        retState = ADC_REQUEST_ERROR;
                    }
                }
            }
            else
            {
                /* Data transfer could not be armed or single software conversion (nothing to apply) */
            }

            if( ( ADC_REQUEST_OK           == retState             ) &&
                ( ADC_REG_TRIGGER_SOFTWARE == shadow->RegTriggerId )    )
            {
                LL_ADC_REG_StartConversionSWStart( periphReg );
            }
            else
            {
                /* External trigger starts the conversion or start failed */
            }
        }
        else
        {
            /* ADC must be enabled and the regular group idle before a new conversion can be started */
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
 * \brief Stops the regular group conversion (external trigger and continuous mode disabled)
 *        and the regular data transfer (DMA / ISR / POLL)
 *
 * \note  STM32F7 has no conversion abort - a conversion (sequence) in progress is finished by the
 *        HW, its results are not collected by the stopped data transfer.
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
        ADC_TypeDef * const periphReg  = adc_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      regRunning = READ_BIT( periphReg->CR2, ADC_REG_RUN_MASK );

        if( 0u != regRunning )
        {
            CLEAR_BIT( periphReg->CR2, ADC_REG_RUN_MASK );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t runReg = READ_BIT( periphReg->CR2, ADC_REG_RUN_MASK );

                if( 0u == runReg )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Conversion stop has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* No regular conversion is running, nothing to stop */
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
 * \brief Starts the injected group conversion
 *
 * For software trigger the conversion starts immediately (JSWSTART), for an external trigger
 * the trigger edge is applied (JEXTEN) and the ADC starts to accept trigger events.
 *
 * \note  JSWSTART is cleared by HW when the conversion starts, so the write can not be verified
 *        by read-back. Conversion progress is signalled by \ref ADC_FLAG_INJ_START /
 *        \ref ADC_FLAG_INJ_EOS (see Adc_Get_Flag()).
 *
 * \pre   ADC peripheral must be enabled (ADON = 1), no injected external trigger may be running
 *        (JEXTEN = 0) and auto-injected mode must be disabled (JAUTO = 0). Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and no register is modified. In auto-injected mode
 *        (ADC_INJ_TRIGGER_AUTO) the injected group is started together with the regular group
 *        by Adc_Set_RegStart().
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
        ADC_TypeDef * const              periphReg     = adc_PeriphConf[ periphId ].PeriphReg;
        const adc_PeriphShadow_t * const shadow        = &adc_Shadow[ periphId ];
        const uint32_t                   periphEnabled = LL_ADC_IsEnabled( periphReg );
        const uint32_t                   injRunning    = READ_BIT( periphReg->CR2, ADC_INJ_RUN_MASK );
        const uint32_t                   autoInjected  = LL_ADC_INJ_GetTrigAuto( periphReg );

        if( ( 0u                          != periphEnabled ) &&
            ( 0u                          == injRunning    ) &&
            ( LL_ADC_INJ_TRIG_INDEPENDENT == autoInjected  )    )
        {
            if( ADC_INJ_TRIGGER_SOFTWARE == shadow->InjTriggerId )
            {
                LL_ADC_INJ_StartConversionSWStart( periphReg );
                retState = ADC_REQUEST_OK;
            }
            else
            {
                const uint32_t llEdge = adc_InjTriggerEdgeLut[ shadow->InjTriggerEdge ];

                MODIFY_REG( periphReg->CR2, ADC_CR2_JEXTEN, llEdge );

                for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    const uint32_t edgeReg = READ_BIT( periphReg->CR2, ADC_CR2_JEXTEN );

                    if( llEdge == edgeReg )
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
 * \brief Stops the injected group conversion (injected external trigger disabled)
 *
 * \note  If no injected external trigger is running, no register is modified and
 *        \ref ADC_REQUEST_OK is returned. A conversion in progress is finished by the HW.
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
        ADC_TypeDef * const periphReg  = adc_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      injRunning = READ_BIT( periphReg->CR2, ADC_INJ_RUN_MASK );

        if( 0u != injRunning )
        {
            CLEAR_BIT( periphReg->CR2, ADC_INJ_RUN_MASK );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t runReg = READ_BIT( periphReg->CR2, ADC_INJ_RUN_MASK );

                if( 0u == runReg )
                {
                    retState = ADC_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Conversion stop has not yet been applied, keep return state as error */
                    retState = ADC_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* No injected external trigger is running, nothing to stop */
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

    if( ( ADC_PERIPH_CNT >  periphId ) &&
        ( ADC_NULL_PTR   != data     )    )
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
 *        \ref ADC_FLAG_INJ_EOS before.
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
 * The previous data transfer handler is released (DMA stream, interrupts) and the new handler
 * is initialized. The transfer starts with the next Adc_Set_RegStart().
 *
 * \note  The configuration is validated as for a used regular group - DMA / ISR mode require
 *        DataBuffer, POLL mode allows DataBuffer == NULL (manual polling by Adc_Get_RegData()).
 *        The configuration is copied, dataConfig may be a temporary variable.
 *
 * \pre   No regular or injected conversion may be ongoing and the regular data transfer must not
 *        be running (see Adc_Set_RegStop()). Otherwise \ref ADC_REQUEST_ERROR is returned and
 *        nothing is changed.
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
        const uint32_t flagReg = READ_BIT( adc_PeriphConf[ periphId ].PeriphReg->SR, adc_FlagLut[ flagId ] );

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
        ( ADC_FLAG_CNT   > flagId   )    )
    {
        /* SR bits are cleared by writing 0, writing 1 has no effect (no read-modify-write) */
        WRITE_REG( adc_PeriphConf[ periphId ].PeriphReg->SR, ~adc_FlagLut[ flagId ] );
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
 * \note  Analog watch-dog thresholds are given in RAW of the resolution - configure the
 *        watch-dog (Adc_AwdInit() / Adc_Set_AwdThresholds()) after the resolution change.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and no register is modified.
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

    if( ( ADC_PERIPH_CNT >  periphId   ) &&
        ( ADC_NULL_PTR   != channelRes )    )
    {
        const uint32_t llResolution = LL_ADC_GetResolution( adc_PeriphConf[ periphId ].PeriphReg );

        for( adc_Resolution_t idx = ADC_RESOLUTION_12BIT; ADC_RESOLUTION_CNT > idx; idx ++ )
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
 *        ADC clock. Otherwise \ref ADC_REQUEST_ERROR is returned and no register is modified.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and no register is modified.
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
        const adc_RequestState_t samplingState = Adc_Check_SamplingTime( adc_Shadow[ periphId ].ChannelInput[ channelId ], samplingTime );

        if( ADC_REQUEST_OK != samplingState )
        {
            /* Sampling time is shorter than required by the channel input */
            retState = ADC_REQUEST_ERROR;
        }
        else if( ADC_REQUEST_OK == convState )
        {
            ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;
            const uint32_t      llChannel = __LL_ADC_DECIMAL_NB_TO_CHANNEL( (uint32_t)channelId );

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

    if( ( ADC_PERIPH_CNT  > periphId     ) &&
        ( ADC_CHANNEL_CNT > channelId    ) &&
        ( ADC_NULL_PTR   != samplingTime )    )
    {
        const uint32_t llChannel      = __LL_ADC_DECIMAL_NB_TO_CHANNEL( (uint32_t)channelId );
        const uint32_t llSamplingTime = LL_ADC_GetChannelSamplingTime( adc_PeriphConf[ periphId ].PeriphReg, llChannel );

        for( adc_ChannelSampling_t idx = ADC_CHANNEL_SAMPLING_3_CYCLES; ADC_CHANNEL_SAMPLING_CNT > idx; idx ++ )
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
 * For ADC_CHANNEL_INPUT_PIN_SINGLE the GPIO pin wired to this channel/peripheral
 * (adc_GpioPeriphConfig[]) is configured to analog mode with no pull. For the internal
 * signals (TEMP / VREF / VBAT), see Adc_Set_InternalInput().
 *
 * \note  Input must be available for the channel on the peripheral (see Adc_Check_ChannelInput()):
 *        GPIO pin wired for PIN_SINGLE, internal signal connected to channelId for the internal
 *        inputs. Otherwise \ref ADC_REQUEST_ERROR is returned.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId     [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param channelId    [in]: ADC channel identification, value from \ref adc_ChannelId_t
 * \param channelInput [in]: Required channel input, value from \ref adc_ChannelInput_t
 *                           (external pin or internal TEMP / VREF / VBAT)
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Set_ChannelInput( adc_PeriphId_t periphId, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    /* Range check of all parameters and availability of the input for the channel */
    const adc_RequestState_t inputState = Adc_Check_ChannelInput( periphId, channelId, channelInput );

    if( ADC_REQUEST_OK == inputState )
    {
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

        if( ADC_REQUEST_OK == convState )
        {
            if( ADC_CHANNEL_INPUT_PIN_SINGLE == channelInput )
            {
                /* Pin availability checked by Adc_Check_ChannelInput() */
                retState = Adc_Set_PinAnalog( &adc_GpioPeriphConfig[ periphId ].Channel[ channelId ] );
            }
            else
            {
                retState = Adc_Set_InternalInput( periphId, channelInput );
            }

            if( ADC_REQUEST_OK == retState )
            {
                adc_Shadow[ periphId ].ChannelInput[ channelId ] = channelInput;
            }
            else
            {
                /* Previous step failed */
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
        /* Previous step failed */
    }

    return ( retState );
}


/**
 * \brief Returns the last input type selected via Adc_Set_ChannelInput()
 *
 * \note  The input is kept by the module - HW read-back does not distinguish a pin from an
 *        internal signal of the same channel.
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

    if( ( ADC_PERIPH_CNT  >  periphId     ) &&
        ( ADC_CHANNEL_CNT >  channelId    ) &&
        ( ADC_NULL_PTR    != channelInput )    )
    {
        *channelInput = adc_Shadow[ periphId ].ChannelInput[ channelId ];
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
 *        and event filtering
 *
 * Supported modes (ADC_AWD_1): ADC_AWD_MODE_ALL, ADC_AWD_MODE_ALL_REGULAR,
 * ADC_AWD_MODE_ALL_INJECTED. Filtering: ADC_AWD_FILTER_NONE only.
 *
 * \note  adc_AwdConfig_t carries no channel selector, so ADC_AWD_MODE_SINGLE,
 *        ADC_AWD_MODE_SINGLE_REGULAR and ADC_AWD_MODE_SINGLE_INJECTED cannot be configured
 *        without fabricating a channel choice - reported as an error rather than guessed.
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and no register is modified.
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
    uint32_t           llGroup  = LL_ADC_AWD_DISABLE;

    if( ( ADC_PERIPH_CNT > periphId  ) &&
        ( ADC_NULL_PTR  != awdConfig )    )
    {
        const adc_RequestState_t convState = Adc_Check_ConversionStopped( periphId );

        if( ( ADC_REQUEST_OK     == convState            ) &&
            ( ADC_AWD_CNT         > awdConfig->AwdId     ) &&
            ( ADC_AWD_FILTER_CNT  > awdConfig->AwdFilter )    )
        {
            ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

            if( ADC_AWD_MODE_ALL == awdConfig->AwdMode )
            {
                llGroup = LL_ADC_AWD_ALL_CHANNELS_REG_INJ;
            }
            else if( ADC_AWD_MODE_ALL_REGULAR == awdConfig->AwdMode )
            {
                llGroup = LL_ADC_AWD_ALL_CHANNELS_REG;
            }
            else if( ADC_AWD_MODE_ALL_INJECTED == awdConfig->AwdMode )
            {
                llGroup = LL_ADC_AWD_ALL_CHANNELS_INJ;
            }
            else
            {
                /* Single-channel modes need a channel selector which adc_AwdConfig_t does not carry */
                llGroup = LL_ADC_AWD_DISABLE;
            }

            if( LL_ADC_AWD_DISABLE != llGroup )
            {
                LL_ADC_SetAnalogWDMonitChannels( periphReg, llGroup );

                for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    const uint32_t regValue = LL_ADC_GetAnalogWDMonitChannels( periphReg );

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
                /* Previous step failed */
            }

            if( ADC_REQUEST_OK == retState )
            {
                retState = Adc_Set_AwdFilter( periphId, awdConfig->AwdId, awdConfig->AwdFilter );
            }
            else
            {
                /* Previous step failed */
            }
        }
        else
        {
            /* Conversion is ongoing or configuration is invalid */
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
 * Thresholds are given in RAW value of the configured resolution and converted to the 12 bit
 * HW threshold format (__LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION).
 *
 * \pre   No regular or injected conversion may be ongoing on the peripheral. Otherwise
 *        \ref ADC_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId      [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param awdId         [in]: Analog Watch-dog identification, value from \ref adc_AwdId_t
 * \param lowThreshold  [in]: Lower comparison threshold as raw ADC value (\ref adc_AwdThreshold_t)
 * \param highThreshold [in]: Upper comparison threshold as raw ADC value (\ref adc_AwdThreshold_t)
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request was processed
 *         without problems. Otherwise (also for a threshold above the maximum RAW value of the
 *         resolution) returns \ref ADC_REQUEST_ERROR.
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
        ADC_TypeDef * const      periphReg    = adc_PeriphConf[ periphId ].PeriphReg;
        const adc_RequestState_t convState    = Adc_Check_ConversionStopped( periphId );
        const uint32_t           llResolution = LL_ADC_GetResolution( periphReg );
        const uint32_t           llHigh       = __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION( llResolution, (uint32_t)highThreshold );
        const uint32_t           llLow        = __LL_ADC_ANALOGWD_SET_THRESHOLD_RESOLUTION( llResolution, (uint32_t)lowThreshold );

        if( ( ADC_REQUEST_OK == convState ) &&
            ( ADC_HTR_HT     >= llHigh    ) &&
            ( ADC_LTR_LT     >= llLow     )    )
        {
            LL_ADC_SetAnalogWDThresholds( periphReg, LL_ADC_AWD_THRESHOLD_HIGH, llHigh );
            LL_ADC_SetAnalogWDThresholds( periphReg, LL_ADC_AWD_THRESHOLD_LOW,  llLow  );

            for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t highThresholdReg = LL_ADC_GetAnalogWDThresholds( periphReg, LL_ADC_AWD_THRESHOLD_HIGH );
                const uint32_t lowThresholdReg  = LL_ADC_GetAnalogWDThresholds( periphReg, LL_ADC_AWD_THRESHOLD_LOW  );

                if( ( llHigh == highThresholdReg ) &&
                    ( llLow  == lowThresholdReg  )    )
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
 * \note  Thresholds are returned in RAW value of the configured resolution.
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
        ADC_TypeDef * const periphReg    = adc_PeriphConf[ periphId ].PeriphReg;
        const uint32_t      llResolution = LL_ADC_GetResolution( periphReg );

        *highThreshold = (adc_AwdThreshold_t)__LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION( llResolution, LL_ADC_GetAnalogWDThresholds( periphReg, LL_ADC_AWD_THRESHOLD_HIGH ) );
        *lowThreshold  = (adc_AwdThreshold_t)__LL_ADC_ANALOGWD_GET_THRESHOLD_RESOLUTION( llResolution, LL_ADC_GetAnalogWDThresholds( periphReg, LL_ADC_AWD_THRESHOLD_LOW  ) );
        retState       = ADC_REQUEST_OK;
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/**
 * \brief Configures an Analog Watch-dog's event filtering
 *
 * \note  HW limitation: STM32F7 analog watch-dog has no event filtering - only
 *        ADC_AWD_FILTER_NONE is accepted (without register access).
 *
 * \param periphId  [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param awdId     [in]: Analog Watch-dog identification, value from \ref adc_AwdId_t
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

    if( ( ADC_PERIPH_CNT      > periphId  ) &&
        ( ADC_AWD_CNT         > awdId     ) &&
        ( ADC_AWD_FILTER_NONE == awdFilter )    )
    {
        /* No filtering HW - "no filtering" is the only (and current) state */
        retState = ADC_REQUEST_OK;
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
 * \note  Same HW limitation as Adc_Set_AwdFilter() - ADC_AWD_FILTER_NONE is always returned.
 *
 * \param periphId   [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 * \param awdId      [in]: Analog Watch-dog identification, value from \ref adc_AwdId_t
 * \param awdFilter [out]: Pointer to store the current event filtering (\ref adc_AwdFilter_t).
 *                         Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
adc_RequestState_t Adc_Get_AwdFilter( adc_PeriphId_t periphId, adc_AwdId_t awdId, adc_AwdFilter_t * const awdFilter )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_PERIPH_CNT >  periphId  ) &&
        ( ADC_AWD_CNT    >  awdId     ) &&
        ( ADC_NULL_PTR   != awdFilter )    )
    {
        *awdFilter = ADC_AWD_FILTER_NONE;
        retState   = ADC_REQUEST_OK;
    }
    else
    {
        /* No action required */
    }

    return ( retState );
}


/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks that neither a regular nor an injected conversion is ongoing
 *        (EXTEN = 0, CONT = 0 and JEXTEN = 0)
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
        const uint32_t runReg = READ_BIT( adc_PeriphConf[ periphId ].PeriphReg->CR2, ( ADC_REG_RUN_MASK | ADC_INJ_RUN_MASK ) );

        if( 0u == runReg )
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
 * \brief Checks that the ADC peripheral is disabled (ADON = 0)
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
        const uint32_t periphEnabled = LL_ADC_IsEnabled( adc_PeriphConf[ periphId ].PeriphReg );

        if( 0u == periphEnabled )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* ADC is enabled */
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
 *        through the ADC common register block (clock prescaler)
 *
 * \return Returns \ref ADC_REQUEST_OK if all ADC peripherals are disabled. Otherwise
 *         returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_AllPeriphsDisabled( void )
{
    adc_RequestState_t retState = ADC_REQUEST_OK;

    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1;
         ( ADC_PERIPH_CNT > periphIdx ) &&
         ( ADC_REQUEST_OK == retState );
         periphIdx ++ )
    {
        retState = Adc_Check_PeriphDisabled( periphIdx );
    }

    return ( retState );
}


/**
 * \brief Checks that the channel input is available for the channel on the peripheral
 *
 * - ADC_CHANNEL_INPUT_PIN_SINGLE: GPIO pin is wired to the channel
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
        const adc_GpioConfig_t * const gpioPin         = &adc_GpioPeriphConfig[ periphId ].Channel[ channelId ];
        const adc_ChannelId_t          internalChannel = adc_InputConfig[ periphId ].ChannelId[ channelInput ];

        if( ADC_CHANNEL_INPUT_PIN_SINGLE == channelInput )
        {
            if( GPIO_PORT_CNT != gpioPin->PortId )
            {
                retState = ADC_REQUEST_OK;
            }
            else
            {
                /* Channel has no external pin wired on this peripheral */
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
 *         time satisfies the minimum of the input at the active ADC clock) and the channel has no
 *         conflicting input or sampling time in previously checked slots. Otherwise returns
 *         \ref ADC_REQUEST_ERROR.
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
        if( ( ADC_REQUEST_OK       == retState                ) &&
            ( 0u                   <  injLen                  ) &&
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
        for( adc_RegSequenceId_t slotIdx = ADC_REG_SEQUENCE_1;
             ( regLen > slotIdx ) &&
             ( ADC_REQUEST_OK == retState );
             slotIdx ++ )
        {
            retState = Adc_Check_ChannelSlot( adcConfig->PeriphId, &adcConfig->RegChannels[ slotIdx ], channelTable );
        }

        /* Injected slots - slot index is the rank */
        for( adc_InjSequenceId_t slotIdx = ADC_INJ_SEQUENCE_1;
             ( injLen > slotIdx ) &&
             ( ADC_REQUEST_OK == retState );
             slotIdx ++ )
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
 * \brief Enables the RCC clock of an ADC peripheral if it is not enabled yet
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_PeriphClock( adc_PeriphId_t periphId )
{
    adc_RequestState_t  retState  = ADC_REQUEST_ERROR;
    rcc_FunctionState_t clkState  = RCC_FUNCTION_INACTIVE;
    rcc_RequestState_t  rccState  = RCC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        const rcc_PeriphId_t rccPeriph = adc_PeriphConf[ periphId ].RccPeriphId;

        rccState = Rcc_Get_PeriphState( rccPeriph, &clkState );

        if( ( RCC_REQUEST_OK        == rccState ) &&
            ( RCC_FUNCTION_INACTIVE == clkState )    )
        {
            rccState = Rcc_Set_PeriphActive( rccPeriph );
        }
        else
        {
            /* Clock is already enabled or its state is not available */
        }

        if( RCC_REQUEST_OK == rccState )
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
 * \brief Configures conversion mode of the peripheral: scan mode (sequences of both groups),
 *        EOC flag at the end of each regular conversion (EOCS = 1) and right data alignment
 *
 * \pre   No conversion is ongoing (called from Adc_PeriphInit() only).
 *
 * \param periphId [in]: ADC peripheral identification, value from \ref adc_PeriphId_t
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Set_ConversionConfig( adc_PeriphId_t periphId )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ADC_PERIPH_CNT > periphId )
    {
        ADC_TypeDef * const periphReg = adc_PeriphConf[ periphId ].PeriphReg;

        LL_ADC_SetSequencersScanMode( periphReg, LL_ADC_SEQ_SCAN_ENABLE );
        LL_ADC_REG_SetFlagEndOfConversion( periphReg, LL_ADC_REG_FLAG_EOC_UNITARY_CONV );
        LL_ADC_SetDataAlignment( periphReg, LL_ADC_DATA_ALIGN_RIGHT );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t scanMode  = LL_ADC_GetSequencersScanMode( periphReg );
            const uint32_t eocMode   = LL_ADC_REG_GetFlagEndOfConversion( periphReg );
            const uint32_t alignMode = LL_ADC_GetDataAlignment( periphReg );

            if( ( LL_ADC_SEQ_SCAN_ENABLE           == scanMode  ) &&
                ( LL_ADC_REG_FLAG_EOC_UNITARY_CONV == eocMode   ) &&
                ( LL_ADC_DATA_ALIGN_RIGHT          == alignMode )    )
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
    adc_RequestState_t         retState  = ADC_REQUEST_OK;
    ADC_TypeDef * const        periphReg = adc_PeriphConf[ adcConfig->PeriphId ].PeriphReg;
    const adc_RegSequenceLen_t regLen    = adcConfig->RegChannelsCnt;

    /* --- Ranks --- */
    for( adc_RegSequenceId_t slotIdx = ADC_REG_SEQUENCE_1;
         ( regLen > slotIdx ) &&
         ( ADC_REQUEST_OK == retState );
         slotIdx ++ )
    {
        const adc_ChannelConfig_t * const channelSlot = &adcConfig->RegChannels[ slotIdx ];
        const uint32_t                    llRank      = adc_RegSeqRankLut[ slotIdx ];
        const uint32_t                    llChannel   = __LL_ADC_DECIMAL_NB_TO_CHANNEL( (uint32_t)channelSlot->ChannelId );

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
        /* Previous step failed */
    }

    return ( retState );
}


/**
 * \brief Configures injected group sequencer (length, ranks), trigger source, trigger edge,
 *        auto-injected mode and discontinuous mode from adc_PeriphConfig_t
 *
 * - Sequence length is written before the ranks - STM32F7 aligns the ranks to the end of JSQR
 *   according to the length (handled by LL_ADC_INJ_SetSequencerRanks()).
 * - Trigger source is written to JEXTSEL, the edge is kept by the module and applied by
 *   Adc_Set_InjStart() (JEXTEN).
 * - ADC_INJ_TRIGGER_AUTO:            JAUTO = 1, injected sequence is converted after each regular sequence
 * - ADC_INJ_TRIGGER_MODE_SINGLE:     JDISCEN = 1, every trigger converts one rank
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
    adc_RequestState_t         retState  = ADC_REQUEST_ERROR;
    ADC_TypeDef * const        periphReg = adc_PeriphConf[ adcConfig->PeriphId ].PeriphReg;
    adc_PeriphShadow_t * const shadow    = &adc_Shadow[ adcConfig->PeriphId ];
    const adc_InjSequenceLen_t injLen    = adcConfig->InjChannelsCnt;
    const uint32_t             llTrigger = adc_InjTriggerSrcLut[ adcConfig->InjTriggerId ];
    const uint32_t             llSeqLen  = adc_InjSeqLenLut[ injLen - ADC_SEQ_LEN_IDX_OFFSET ];

    /* Length first - rank positions in JSQR depend on it */
    LL_ADC_INJ_SetSequencerLength( periphReg, llSeqLen );
    LL_ADC_INJ_SetTriggerSource( periphReg, llTrigger );

    /* Slot index is the rank */
    for( adc_InjSequenceId_t slotIdx = ADC_INJ_SEQUENCE_1; injLen > slotIdx; slotIdx ++ )
    {
        LL_ADC_INJ_SetSequencerRanks( periphReg, adc_InjSeqRankLut[ slotIdx ], __LL_ADC_DECIMAL_NB_TO_CHANNEL( (uint32_t)adcConfig->InjChannels[ slotIdx ].ChannelId ) );
    }

    for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        const uint32_t  triggerReg = READ_BIT( periphReg->CR2, ADC_CR2_JEXTSEL );
        const uint32_t  seqLenReg  = LL_ADC_INJ_GetSequencerLength( periphReg );
        adc_FlagState_t ranksOk    = ADC_FLAG_ACTIVE;

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

        if( ( llTrigger       == triggerReg ) &&
            ( llSeqLen        == seqLenReg  ) &&
            ( ADC_FLAG_ACTIVE == ranksOk    )    )
        {
            retState = ADC_REQUEST_OK;
            break;
        }
        else
        {
            /* Injected sequence has not yet been applied, keep return state as error */
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
        /* Previous step failed */
    }

    if( ADC_REQUEST_OK == retState )
    {
        retState = Adc_Set_InjDiscontMode( adcConfig->PeriphId, adc_InjTriggerModeLut[ adcConfig->InjTriggerMode ] );
    }
    else
    {
        /* Previous step failed */
    }

    if( ( ADC_REQUEST_OK       == retState                ) &&
        ( ADC_INJ_TRIGGER_AUTO == adcConfig->InjTriggerId )    )
    {
        retState = Adc_Set_InjAutoMode( adcConfig->PeriphId, LL_ADC_INJ_TRIG_FROM_GRP_REGULAR );
    }
    else
    {
        /* No action required */
    }

    if( ADC_REQUEST_OK == retState )
    {
        shadow->InjTriggerId   = adcConfig->InjTriggerId;
        shadow->InjTriggerEdge = adcConfig->InjTriggerEdge;
    }
    else
    {
        /* Injected group is not configured */
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

    if( ( ADC_NULL_PTR    != pinConfig         ) &&
        ( GPIO_PORT_CNT    > pinConfig->PortId ) &&
        ( GPIO_PIN_ID_CNT  > pinConfig->PinId  )    )
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
 * - TEMP / VREF require TSVREFE, VBAT requires VBATE in the common control register (CCR).
 * - On devices where the temperature sensor shares channel 18 with VBAT, VBAT measurement has
 *   priority - VBATE is cleared when the temperature sensor is selected.
 * - After enabling the TEMP / VREF path, the stabilization time is waited
 *   (LL_ADC_DELAY_VREFINT_STAB_US / LL_ADC_DELAY_TEMPSENSOR_STAB_US).
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

    if( ( ADC_PERIPH_CNT               > periphId                                              ) &&
        ( ADC_CHANNEL_INPUT_CNT        > channelInput                                          ) &&
        ( ADC_CHANNEL_INPUT_PIN_SINGLE != channelInput                                         ) &&
        ( ADC_CHANNEL_CNT              > adc_InputConfig[ periphId ].ChannelId[ channelInput ] )    )
    {
        /* Internal input exists on this peripheral (ADC_CHANNEL_CNT in adc_InputConfig means "not available") */
        ADC_Common_TypeDef * const commonReg = __LL_ADC_COMMON_INSTANCE( adc_PeriphConf[ periphId ].PeriphReg );
        const uint32_t             pathCur   = LL_ADC_GetCommonPathInternalCh( commonReg );
        uint32_t                   pathNew   = pathCur;
        adc_TimeUs_t               stabUs    = 0u;

        if( ADC_CHANNEL_INPUT_VBAT == channelInput )
        {
            pathNew |= LL_ADC_PATH_INTERNAL_VBAT;
        }
        else if( ADC_CHANNEL_INPUT_VREF == channelInput )
        {
            pathNew |= LL_ADC_PATH_INTERNAL_VREFINT;
            stabUs   = LL_ADC_DELAY_VREFINT_STAB_US;
        }
        else
        {
            pathNew |= LL_ADC_PATH_INTERNAL_TEMPSENSOR;
            stabUs   = LL_ADC_DELAY_TEMPSENSOR_STAB_US;

            if( ADC_VBAT_CHANNEL == ADC_TEMP_CHANNEL )
            {
                /* Channel shared with VBAT - VBAT path would take precedence over the sensor */
                pathNew &= ~LL_ADC_PATH_INTERNAL_VBAT;
            }
            else
            {
                /* Temperature sensor has its own channel */
            }
        }

        LL_ADC_SetCommonPathInternalCh( commonReg, pathNew );

        for( adc_TimeoutCnt_t iterationCnt = 0u; ADC_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t pathReg = LL_ADC_GetCommonPathInternalCh( commonReg );

            if( pathNew == pathReg )
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

        /* --- Stabilization time of a newly enabled measurement path --- */
        if( ( ADC_REQUEST_OK == retState ) &&
            ( pathNew        != pathCur  ) &&
            ( 0u             != stabUs   )    )
        {
            retState = Adc_Set_Delay( stabUs );
        }
        else
        {
            /* Path was already enabled or has no stabilization time requirement */
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
    /* CPU clock == HCLK, provided by Rcc as frequency of the SysTick clock (processor clock) */
    const rcc_RequestState_t rccState = Rcc_Get_PeriphClk( RCC_PERIPH_SYSTICK, &hclkFreq );

    if( ( RCC_REQUEST_OK == rccState ) &&
        ( 0u             != hclkFreq )    )
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
 * \brief Calculates ADC clock frequency for a clock divider (PCLK2 / divider)
 *
 * \param clkDiv     [in]: ADC clock divider, value from \ref adc_ClkDiv_t
 * \param clkFreqHz [out]: Pointer to store the ADC clock frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref ADC_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Get_ClockFreq( adc_ClkDiv_t clkDiv, adc_FreqHz_t * const clkFreqHz )
{
    adc_RequestState_t retState = ADC_REQUEST_ERROR;

    if( ( ADC_CLK_DIV_CNT > clkDiv    ) &&
        ( ADC_NULL_PTR   != clkFreqHz )    )
    {
        rcc_FreqHz_t             srcFreq  = 0u;
        /* All ADC peripherals are clocked by PCLK2 - frequency of ADC1 clock is used */
        const rcc_RequestState_t rccState = Rcc_Get_PeriphClk( adc_PeriphConf[ ADC_PERIPH_1 ].RccPeriphId, &srcFreq );

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
 * \brief Returns the active ADC clock frequency (clock divider configured in HW)
 *
 * \param clkFreqHz [out]: Pointer to store the ADC clock frequency in Hz. Must not be NULL.
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
        retState = Adc_Get_ClockFreq( clkDiv, clkFreqHz );
    }
    else
    {
        /* Clock divider can not be decoded */
        retState = ADC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that a clock divider results in an ADC clock frequency within
 *        ADC_CLK_FREQ_MIN_HZ - ADC_CLK_FREQ_MAX_HZ
 *
 * \param clkDiv [in]: ADC clock divider, value from \ref adc_ClkDiv_t
 *
 * \return Returns \ref ADC_REQUEST_OK if the resulting frequency is within the allowed range.
 *         Otherwise returns \ref ADC_REQUEST_ERROR.
 */
static adc_RequestState_t Adc_Check_ClockFreq( adc_ClkDiv_t clkDiv )
{
    adc_RequestState_t retState  = ADC_REQUEST_ERROR;
    adc_FreqHz_t       clkFreqHz = 0u;

    retState = Adc_Get_ClockFreq( clkDiv, &clkFreqHz );

    if( ADC_REQUEST_OK == retState )
    {
        if( ( ADC_CLK_FREQ_MIN_HZ <= clkFreqHz ) &&
            ( ADC_CLK_FREQ_MAX_HZ >= clkFreqHz )    )
        {
            retState = ADC_REQUEST_OK;
        }
        else
        {
            /* ADC clock frequency is out of the allowed range */
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
 *        at the active ADC clock (internal channels, see adc_SamplingMinNsLut)
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

            if( ( ADC_REQUEST_OK == retState  ) &&
                ( 0u             != clkFreqHz )    )
            {
                /* Sampling time in ns = cycles * 1e9 / fADC */
                const uint64_t samplingNs = ( (uint64_t)adc_SamplingCyclesLut[ samplingTime ] * ADC_NS_PER_S ) / (uint64_t)clkFreqHz;

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
                /* ADC clock frequency is not available */
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
 * - Mode specific rules are checked by the mode handler (DMA: DMA stream of the ADC request,
 *   buffer size limit).
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
 * \brief Stores the data transfer configuration and initializes the mode handler
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
    adc_XferContext_t * const xferCtx = &adc_XferContext[ periphId ];

    /* Configuration is copied - user structure may be a temporary (stack) variable */
    xferCtx->Config    = *dataConfig;
    xferCtx->BufferIdx = 0u;
    xferCtx->XferState = ADC_FUNCTION_INACTIVE;
    xferCtx->InitState = ADC_FUNCTION_ACTIVE;

    return ( adc_XferModeLut[ xferCtx->Config.TransferMode ].Init( periphId ) );
}


/**
 * \brief Arms the regular data transfer (called by Adc_Set_RegStart() before the start)
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
/* ---------------------- Private interface (see Adc.h) --------------------- */
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

            if( ( 0u       != halfSize           ) &&
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
            /* Next result is stored to the buffer start (DMA stream runs in circular mode) */
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
        ( ADC_ERROR_CNT  > errorId  )    )
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
