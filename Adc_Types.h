/**
 * \defgroup Adc Adc
 * \brief Adc module
 */

/**
 * \author Mr.Nobody
 * \file Adc_Types.h
 * \ingroup Adc
 * \brief Analog-to-Digital Converter (ADC) module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 * \note STM32L4 / STM32L4+ family: configuration structures and type names are common
 *       for all families, enumerations with hardware specific values (clock, triggers,
 *       peripherals, channels, channel inputs, DMA) follow STM32L4 ADC. The lists of the
 *       triggers, ADC peripherals and clock sources contain the values of the device line
 *       only (timer, ADC2 / ADC3, PLLSAI clock sources missing on the device are not in the
 *       list); the DAC channel inputs missing on the device are refused with
 *       \ref ADC_REQUEST_ERROR at run time.
 *
 */

#ifndef ADC_ADC_TYPES_H
#define ADC_ADC_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Stm32_adc.h"                      /* ADC utilities functionality    */
#include "Dma_Types.h"                      /* DMA types definitions          */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define ADC_NULL_PTR                        ( ( void* ) 0u )

#if defined(RCC_PLLSAI2CFGR_PLLSAI2R) && \
    !defined(DMAMUX1)
/** PLLSAI2 output R can be selected as the ADC kernel clock (STM32L47x / L48x / L49x / L4A6; the RCC
 *  module has the same condition in RCC_TYPES_ADC_PLLSAI2R_SUPPORT) */
#define ADC_TYPES_PLLSAI2R_SUPPORT
#endif /* RCC_PLLSAI2CFGR_PLLSAI2R && !DMAMUX1 */

/** ADC peripheral identification bit offset in encoded DMA channel value */
#define ADC_DMA_BIT_MASK_PERIPH_BIT_OFFSET  ( 15u )

/** DMA peripheral identification bit offset in encoded DMA channel value */
#define ADC_DMA_BIT_MASK_DMA_BIT_OFFSET     ( 10u )

/** DMA channel identification bit offset in encoded DMA channel value */
#define ADC_DMA_BIT_MASK_CHANNEL_BIT_OFFSET ( 5u )

/** Request selection (DMA_CSELR value, 0 with DMAMUX1) bit offset in encoded DMA channel value */
#define ADC_DMA_BIT_MASK_CSELR_BIT_OFFSET   ( 0u )

/** Mask of one field (5 bits) in encoded DMA channel value */
#define ADC_DMA_BIT_MASK_FIELD              ( 0x1Fu )

/* ========================== EXPORTED MACROS =============================== */

/**
 * \brief Encodes DMA channel (ADC peripheral, DMA peripheral, channel, request selection) into single
 *        value of \ref adc_DmaCode_t
 *
 * The macro defines the values of the DMA channel list \ref adc_Dma_t, e.g. the
 * ADC1 conversion request on DMA1 channel 1 (request selection 0) is \ref ADC_DMA_ADC1_DMA1_CHANNEL1:
 * ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_1, 0u )
 */
#define ADC_DMA_ENCODE( PERIPH_ID, DMA_ID, CHANNEL_ID, CSELR )   ( (adc_DmaCode_t)( ( (uint32_t)(PERIPH_ID)  << ADC_DMA_BIT_MASK_PERIPH_BIT_OFFSET  ) | \
                                                                                    ( (uint32_t)(DMA_ID)     << ADC_DMA_BIT_MASK_DMA_BIT_OFFSET     ) | \
                                                                                    ( (uint32_t)(CHANNEL_ID) << ADC_DMA_BIT_MASK_CHANNEL_BIT_OFFSET ) | \
                                                                                    ( (uint32_t)(CSELR)      << ADC_DMA_BIT_MASK_CSELR_BIT_OFFSET   )   ) )

/** DMA channel is not configured by the module (value of the *_DMA_UNUSED items of the DMA channel lists) */
#define ADC_DMA_CODE_UNUSED                 ADC_DMA_ENCODE( ADC_PERIPH_CNT, ADC_DMA_PERIPH_CNT, ADC_DMA_CHANNEL_CNT, 0u )

/** Extract ADC peripheral ID from encoded DMA channel value */
#define ADC_DMA_BIT_MASK_DECODE_PERIPH( CODED_VAL )  ( ( (CODED_VAL) >> ADC_DMA_BIT_MASK_PERIPH_BIT_OFFSET ) & ADC_DMA_BIT_MASK_FIELD )

/** Extract DMA peripheral ID from encoded DMA channel value */
#define ADC_DMA_BIT_MASK_DECODE_DMA( CODED_VAL )     ( ( (CODED_VAL) >> ADC_DMA_BIT_MASK_DMA_BIT_OFFSET ) & ADC_DMA_BIT_MASK_FIELD )

/** Extract DMA channel ID from encoded DMA channel value */
#define ADC_DMA_BIT_MASK_DECODE_CHANNEL( CODED_VAL ) ( ( (CODED_VAL) >> ADC_DMA_BIT_MASK_CHANNEL_BIT_OFFSET ) & ADC_DMA_BIT_MASK_FIELD )

/** Extract request selection (DMA_CSELR value, 0 with DMAMUX1) from encoded DMA channel value */
#define ADC_DMA_BIT_MASK_DECODE_CSELR( CODED_VAL )   ( ( (CODED_VAL) >> ADC_DMA_BIT_MASK_CSELR_BIT_OFFSET ) & ADC_DMA_BIT_MASK_FIELD )

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t adc_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t adc_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t adc_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    adc_MajorVersion_t Major; /**< Major version */
    adc_MinorVersion_t Minor; /**< Minor version */
    adc_PatchVersion_t Patch; /**< Patch version */
}   adc_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    ADC_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    ADC_FUNCTION_ACTIVE         /**< Function status is active   */
}   adc_FunctionState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    ADC_REQUEST_ERROR = 0u, /**< Processing request failed  */
    ADC_REQUEST_OK          /**< Processing request succeed */
}   adc_RequestState_t;


/** Flag states enumeration */
typedef enum
{
    ADC_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    ADC_FLAG_ACTIVE         /**< Active flag state   */
}   adc_FlagState_t;

/* -------------------------------------------------------------------------- */
/* ------------------------ Clock source configuration ---------------------- */
/* -------------------------------------------------------------------------- */

/** \brief List of available ADC clock sources
 *
 * \note The clock source is common for all ADC peripherals (one RCC clock enable and one
 *       ADCSEL kernel clock multiplexer for ADC1 / ADC2 / ADC3). HCLK selects the synchronous
 *       clock mode (CKMODE, no kernel clock selected in RCC).
 * \note The list contains the clock sources of the device line only: PLLSAI1 output R is not in
 *       the list of STM32L41x / L42x (guard RCC_CR_PLLSAI1ON), PLLSAI2 output R is in the list of
 *       STM32L47x / L48x / L49x / L4Ax only (guard ADC_TYPES_PLLSAI2R_SUPPORT). */
typedef enum
{
    ADC_CLK_SRC_HCLK = 0u, /**< ADC clocked synchronously by AHB clock (HCLK / 1, 2, 4)                       */
    ADC_CLK_SRC_SYSCLK,    /**< ADC kernel clock from system clock (SYSCLK, asynchronous mode)                 */
#if defined(RCC_CR_PLLSAI1ON)
    ADC_CLK_SRC_PLLSAI1R,  /**< ADC kernel clock from PLLSAI1 output R (asynchronous mode, not on STM32L41x / L42x) */
#endif /* RCC_CR_PLLSAI1ON */
#if defined(ADC_TYPES_PLLSAI2R_SUPPORT)
    ADC_CLK_SRC_PLLSAI2R,  /**< ADC kernel clock from PLLSAI2 output R (asynchronous mode, STM32L47x / L48x / L49x / L4Ax only) */
#endif /* ADC_TYPES_PLLSAI2R_SUPPORT */
    ADC_CLK_SRC_CNT        /**< Count of ADC clock sources                                                     */
}   adc_ClkSrc_t;


/** \brief List of available ADC kernel clock dividers (common for all ADC peripherals) */
typedef enum
{
    ADC_CLK_DIV_1 = 0u, /**< Clock source is not divided                             */
    ADC_CLK_DIV_2,      /**< Clock source is divided by 2                            */
    ADC_CLK_DIV_4,      /**< Clock source is divided by 4                            */
    ADC_CLK_DIV_8,      /**< Clock source is divided by 8   (Only if ClkSrc != HCLK) */
    ADC_CLK_DIV_16,     /**< Clock source is divided by 16  (Only if ClkSrc != HCLK) */
    ADC_CLK_DIV_32,     /**< Clock source is divided by 32  (Only if ClkSrc != HCLK) */
    ADC_CLK_DIV_64,     /**< Clock source is divided by 64  (Only if ClkSrc != HCLK) */
    ADC_CLK_DIV_128,    /**< Clock source is divided by 128 (Only if ClkSrc != HCLK) */
    ADC_CLK_DIV_256,    /**< Clock source is divided by 256 (Only if ClkSrc != HCLK) */
    ADC_CLK_DIV_CNT     /**< Count of clock dividers                                 */
}   adc_ClkDiv_t;

/* -------------------------------------------------------------------------- */
/* ----------------------- Data transfer configuration ---------------------- */
/* -------------------------------------------------------------------------- */

/** \brief Type representing Analog-to-Digital Converter (ADC) data */
typedef uint16_t adc_Data_t;


/** \brief List of ADC event flags */
typedef enum
{
    ADC_FLAG_REG_EOC = 0u, /**< Regular group end of unitary conversion (EOC)  */
    ADC_FLAG_REG_EOS,      /**< Regular group end of sequence (EOS)            */
    ADC_FLAG_REG_OVR,      /**< Regular group overrun (OVR)                    */
    ADC_FLAG_INJ_EOC,      /**< Injected group end of unitary conversion (JEOC) */
    ADC_FLAG_INJ_EOS,      /**< Injected group end of sequence (JEOS)          */
    ADC_FLAG_AWD1,         /**< Analog Watch-dog 1 out of window event (AWD1)  */
    ADC_FLAG_AWD2,         /**< Analog Watch-dog 2 out of window event (AWD2)  */
    ADC_FLAG_AWD3,         /**< Analog Watch-dog 3 out of window event (AWD3)  */
    ADC_FLAG_CNT           /**< Count of ADC event flags                       */
}   adc_FlagId_t;

/* -------------------------------------------------------------------------- */
/* ------------------------ Peripheral configuration ------------------------ */
/* -------------------------------------------------------------------------- */

/** \brief Analog-to-Digital Converter (ADC) peripheral enumeration */
typedef enum
{
    ADC_PERIPH_1 = 0u, /**< Analog-to-Digital Converter (ADC) peripheral 1 */
#if defined (ADC2)
    ADC_PERIPH_2,      /**< Analog-to-Digital Converter (ADC) peripheral 2 */
#endif
#if defined (ADC3)
    ADC_PERIPH_3,      /**< Analog-to-Digital Converter (ADC) peripheral 3 */
#endif
    ADC_PERIPH_CNT     /**< Count of Analog-to-Digital Converter (ADC) peripherals */
}   adc_PeriphId_t;


/** \brief List of available regular group conversion trigger sources (ADC_CFGR EXTSEL)
 *
 * \note The triggers are common for all ADC peripherals. The list contains the triggers of the
 *       device line only: the triggers of a timer missing on the device (TIM3 not on STM32L41x /
 *       L43x, TIM4 / TIM8 only on STM32L47x / L48x / L49x / L4Ax and STM32L4+) are not in the
 *       list (guards by the CMSIS instance macros). */
typedef enum
{
    ADC_REG_TRIGGER_SOFTWARE = 0u,     /**< Software start (Adc_Set_RegStart())  */
    ADC_REG_TRIGGER_EXT_TIM1_TRGO,     /**< External trigger TIM1 TRGO           */
    ADC_REG_TRIGGER_EXT_TIM1_TRGO2,    /**< External trigger TIM1 TRGO2          */
    ADC_REG_TRIGGER_EXT_TIM1_CH1,      /**< External trigger TIM1 channel 1      */
    ADC_REG_TRIGGER_EXT_TIM1_CH2,      /**< External trigger TIM1 channel 2      */
    ADC_REG_TRIGGER_EXT_TIM1_CH3,      /**< External trigger TIM1 channel 3      */
    ADC_REG_TRIGGER_EXT_TIM2_TRGO,     /**< External trigger TIM2 TRGO           */
    ADC_REG_TRIGGER_EXT_TIM2_CH2,      /**< External trigger TIM2 channel 2      */
#if defined(TIM3)
    ADC_REG_TRIGGER_EXT_TIM3_TRGO,     /**< External trigger TIM3 TRGO           */
    ADC_REG_TRIGGER_EXT_TIM3_CH4,      /**< External trigger TIM3 channel 4      */
#endif /* TIM3 */
#if defined(TIM4)
    ADC_REG_TRIGGER_EXT_TIM4_TRGO,     /**< External trigger TIM4 TRGO           */
    ADC_REG_TRIGGER_EXT_TIM4_CH4,      /**< External trigger TIM4 channel 4      */
#endif /* TIM4 */
#if defined(TIM6)
    ADC_REG_TRIGGER_EXT_TIM6_TRGO,     /**< External trigger TIM6 TRGO           */
#endif /* TIM6 */
#if defined(TIM8)
    ADC_REG_TRIGGER_EXT_TIM8_TRGO,     /**< External trigger TIM8 TRGO           */
    ADC_REG_TRIGGER_EXT_TIM8_TRGO2,    /**< External trigger TIM8 TRGO2          */
#endif /* TIM8 */
    ADC_REG_TRIGGER_EXT_TIM15_TRGO,    /**< External trigger TIM15 TRGO          */
    ADC_REG_TRIGGER_EXT_EXTI_LINE11,   /**< External trigger EXTI line 11        */
    ADC_REG_TRIGGER_CNT                /**< Count of regular group trigger sources */
}   adc_RegTriggerId_t;


/** \brief List of available injected group conversion trigger sources (ADC_JSQR JEXTSEL)
 *
 * \note The triggers are common for all ADC peripherals. The list contains the triggers of the
 *       device line only: the triggers of a timer missing on the device (TIM3 not on STM32L41x /
 *       L43x, TIM4 / TIM8 only on STM32L47x / L48x / L49x / L4Ax and STM32L4+) are not in the
 *       list (guards by the CMSIS instance macros). */
typedef enum
{
    ADC_INJ_TRIGGER_SOFTWARE = 0u,     /**< Software start (Adc_Set_InjStart())     */
    ADC_INJ_TRIGGER_AUTO,              /**< Automatic trigger (after regular group) */
    ADC_INJ_TRIGGER_EXT_TIM1_TRGO,     /**< External trigger TIM1 TRGO              */
    ADC_INJ_TRIGGER_EXT_TIM1_TRGO2,    /**< External trigger TIM1 TRGO2             */
    ADC_INJ_TRIGGER_EXT_TIM1_CH4,      /**< External trigger TIM1 channel 4         */
    ADC_INJ_TRIGGER_EXT_TIM2_TRGO,     /**< External trigger TIM2 TRGO              */
    ADC_INJ_TRIGGER_EXT_TIM2_CH1,      /**< External trigger TIM2 channel 1         */
#if defined(TIM3)
    ADC_INJ_TRIGGER_EXT_TIM3_TRGO,     /**< External trigger TIM3 TRGO              */
    ADC_INJ_TRIGGER_EXT_TIM3_CH1,      /**< External trigger TIM3 channel 1         */
    ADC_INJ_TRIGGER_EXT_TIM3_CH3,      /**< External trigger TIM3 channel 3         */
    ADC_INJ_TRIGGER_EXT_TIM3_CH4,      /**< External trigger TIM3 channel 4         */
#endif /* TIM3 */
#if defined(TIM4)
    ADC_INJ_TRIGGER_EXT_TIM4_TRGO,     /**< External trigger TIM4 TRGO              */
#endif /* TIM4 */
#if defined(TIM6)
    ADC_INJ_TRIGGER_EXT_TIM6_TRGO,     /**< External trigger TIM6 TRGO              */
#endif /* TIM6 */
#if defined(TIM8)
    ADC_INJ_TRIGGER_EXT_TIM8_TRGO,     /**< External trigger TIM8 TRGO              */
    ADC_INJ_TRIGGER_EXT_TIM8_TRGO2,    /**< External trigger TIM8 TRGO2             */
    ADC_INJ_TRIGGER_EXT_TIM8_CH4,      /**< External trigger TIM8 channel 4         */
#endif /* TIM8 */
    ADC_INJ_TRIGGER_EXT_TIM15_TRGO,    /**< External trigger TIM15 TRGO             */
    ADC_INJ_TRIGGER_EXT_EXTI_LINE15,   /**< External trigger EXTI line 15           */
    ADC_INJ_TRIGGER_CNT                /**< Count of injected group trigger sources */
}   adc_InjTriggerId_t;


/** \brief List of regular channels trigger modes */
typedef enum
{
    ADC_REG_TRIGGER_MODE_SINGLE = 0u, /**< Trigger event is needed to start each conversion list                   */
    ADC_REG_TRIGGER_MODE_CONTINUOUS,  /**< After first trigger, the conversion list will be continuously converted */
    ADC_REG_TRIGGER_MODE_CNT          /**< Count of available trigger modes                                        */
}   adc_RegTriggerMode_t;


/** \brief List of available external trigger edge detectors (common for regular and injected group) */
typedef enum
{
    ADC_TRIGGER_EDGE_RISING = 0u, /**< Conversion is triggered on rising edge of external trigger  */
    ADC_TRIGGER_EDGE_FALLING,     /**< Conversion is triggered on falling edge of external trigger */
    ADC_TRIGGER_EDGE_BOTH,        /**< Conversion is triggered on both edges of external trigger   */
    ADC_TRIGGER_EDGE_CNT          /**< Count of available trigger edges                            */
}   adc_TriggerEdge_t;


/** \brief Type representing sequencer length.
 * - Range: 1 - 16 conversions for ADC peripheral
 * - Step size: 1 conversion
 */
typedef uint8_t adc_RegSequenceLen_t;


/**
 * \brief Regular sequence order identification list.
 *
 */
typedef enum
{
    ADC_REG_SEQUENCE_1 = 0u, /**< Sequencer 1st channel ID   */
    ADC_REG_SEQUENCE_2,      /**< Sequencer 2nd channel ID   */
    ADC_REG_SEQUENCE_3,      /**< Sequencer 3rd channel ID   */
    ADC_REG_SEQUENCE_4,      /**< Sequencer 4th channel ID   */
    ADC_REG_SEQUENCE_5,      /**< Sequencer 5th channel ID   */
    ADC_REG_SEQUENCE_6,      /**< Sequencer 6th channel ID   */
    ADC_REG_SEQUENCE_7,      /**< Sequencer 7th channel ID   */
    ADC_REG_SEQUENCE_8,      /**< Sequencer 8th channel ID   */
    ADC_REG_SEQUENCE_9,      /**< Sequencer 9th channel ID   */
    ADC_REG_SEQUENCE_10,     /**< Sequencer 10th channel ID  */
    ADC_REG_SEQUENCE_11,     /**< Sequencer 11th channel ID  */
    ADC_REG_SEQUENCE_12,     /**< Sequencer 12th channel ID  */
    ADC_REG_SEQUENCE_13,     /**< Sequencer 13th channel ID  */
    ADC_REG_SEQUENCE_14,     /**< Sequencer 14th channel ID  */
    ADC_REG_SEQUENCE_15,     /**< Sequencer 15th channel ID  */
    ADC_REG_SEQUENCE_16,     /**< Sequencer 16th channel ID  */
    ADC_REG_SEQUENCE_CNT,    /**< Sequencer channel ID count */
}   adc_RegSequenceId_t;


/** \brief Type representing sequencer length.
 * - Range: 0 - 4 conversions for ADC peripheral
 * - Step size: 1 conversion
 *
 * \note If value is set to 0, injected channels will not be configured.
 */
typedef uint8_t adc_InjSequenceLen_t;


/**
 * \brief Injected sequence order identification list.
 *
 */
typedef enum
{
    ADC_INJ_SEQUENCE_1 = 0u, /**< Sequencer 1st channel ID  */
    ADC_INJ_SEQUENCE_2,      /**< Sequencer 2nd channel ID  */
    ADC_INJ_SEQUENCE_3,      /**< Sequencer 3rd channel ID  */
    ADC_INJ_SEQUENCE_4,      /**< Sequencer 4th channel ID  */
    ADC_INJ_SEQUENCE_CNT,    /**< Sequencer channel ID count */
}   adc_InjSequenceId_t;


/** \brief List of available resolutions */
typedef enum
{
    ADC_RESOLUTION_12BIT = 0u, /**< ADC resolution 12 bits      */
    ADC_RESOLUTION_10BIT,      /**< ADC resolution 10 bits      */
    ADC_RESOLUTION_8BIT,       /**< ADC resolution 8 bits       */
    ADC_RESOLUTION_6BIT,       /**< ADC resolution 6 bits       */
    ADC_RESOLUTION_CNT         /**< Count of resolution options */
}   adc_Resolution_t;


/** \brief List of injected channels trigger modes */
typedef enum
{
    ADC_INJ_TRIGGER_MODE_CONTINUOUS = 0u, /**< Trigger event starts whole sequence    */
    ADC_INJ_TRIGGER_MODE_SINGLE,          /**< Trigger event starts single conversion */
    ADC_INJ_TRIGGER_MODE_CNT              /**< Count of available trigger modes       */
}   adc_InjTriggerMode_t;

/* -------------------------------------------------------------------------- */
/* -------------------------- Channel configuration ------------------------- */
/* -------------------------------------------------------------------------- */

/** \brief List of Analog-to-Digital Converter (ADC) channels (channels with pins differ per
 *         peripheral, see README) */
typedef enum
{
    ADC_CHANNEL_0 = 0u, /**< Analog-to-Digital Converter (ADC) channel 0    */
    ADC_CHANNEL_1,     /**< Analog-to-Digital Converter (ADC) channel 1    */
    ADC_CHANNEL_2,     /**< Analog-to-Digital Converter (ADC) channel 2    */
    ADC_CHANNEL_3,     /**< Analog-to-Digital Converter (ADC) channel 3    */
    ADC_CHANNEL_4,     /**< Analog-to-Digital Converter (ADC) channel 4    */
    ADC_CHANNEL_5,     /**< Analog-to-Digital Converter (ADC) channel 5    */
    ADC_CHANNEL_6,     /**< Analog-to-Digital Converter (ADC) channel 6    */
    ADC_CHANNEL_7,     /**< Analog-to-Digital Converter (ADC) channel 7    */
    ADC_CHANNEL_8,     /**< Analog-to-Digital Converter (ADC) channel 8    */
    ADC_CHANNEL_9,     /**< Analog-to-Digital Converter (ADC) channel 9    */
    ADC_CHANNEL_10,    /**< Analog-to-Digital Converter (ADC) channel 10   */
    ADC_CHANNEL_11,    /**< Analog-to-Digital Converter (ADC) channel 11   */
    ADC_CHANNEL_12,    /**< Analog-to-Digital Converter (ADC) channel 12   */
    ADC_CHANNEL_13,    /**< Analog-to-Digital Converter (ADC) channel 13   */
    ADC_CHANNEL_14,    /**< Analog-to-Digital Converter (ADC) channel 14   */
    ADC_CHANNEL_15,    /**< Analog-to-Digital Converter (ADC) channel 15   */
    ADC_CHANNEL_16,    /**< Analog-to-Digital Converter (ADC) channel 16   */
    ADC_CHANNEL_17,    /**< Analog-to-Digital Converter (ADC) channel 17   */
    ADC_CHANNEL_18,    /**< Analog-to-Digital Converter (ADC) channel 18   */
    ADC_CHANNEL_CNT    /**< Count of Analog-to-Digital Converter (ADC) channels */
}   adc_ChannelId_t;


/** \brief List of available channels sampling options */
typedef enum
{
    ADC_CHANNEL_SAMPLING_2_5_CYCLES = 0u, /**< Channel will be sampled within 2.5 clock cycles   */
    ADC_CHANNEL_SAMPLING_6_5_CYCLES,      /**< Channel will be sampled within 6.5 clock cycles   */
    ADC_CHANNEL_SAMPLING_12_5_CYCLES,     /**< Channel will be sampled within 12.5 clock cycles  */
    ADC_CHANNEL_SAMPLING_24_5_CYCLES,     /**< Channel will be sampled within 24.5 clock cycles  */
    ADC_CHANNEL_SAMPLING_47_5_CYCLES,     /**< Channel will be sampled within 47.5 clock cycles  */
    ADC_CHANNEL_SAMPLING_92_5_CYCLES,     /**< Channel will be sampled within 92.5 clock cycles  */
    ADC_CHANNEL_SAMPLING_247_5_CYCLES,    /**< Channel will be sampled within 247.5 clock cycles */
    ADC_CHANNEL_SAMPLING_640_5_CYCLES,    /**< Channel will be sampled within 640.5 clock cycles */
    ADC_CHANNEL_SAMPLING_CNT              /**< Count of available sampling cycles                */
}   adc_ChannelSampling_t;


/** \brief List of available ADC channel signal inputs
 *
 * \note Internal signals per peripheral (an input not connected to the peripheral or to the
 *       requested channel is refused):
 *       - VREF: ADC1 channel 0
 *       - TEMP / VBAT: ADC1 / ADC3 channel 17 / 18
 *       - DAC1 / DAC2 (DAC1 output 1 / 2): ADC2 channel 17 / 18 and ADC3 channel 14 / 15 on devices
 *         with ADC2; ADC1 channel 17 / 18 on single ADC devices STM32L43x / L44x / L45x / L46x,
 *         shared with TEMP / VBAT - the DAC output is connected while the temperature sensor /
 *         VBAT path is disabled (CCR TSEN / VBATEN = 0). No DAC on STM32L41x / L42x, DAC1 output 2
 *         not on STM32L45x / L46x, DAC1 outputs not connected to the ADC on STM32L4R / L4S.
 *
 * The input is a part of the items of the channel list \ref adc_Channel_t (only the combinations that the device line
 * offers are in the list). */
typedef enum
{
    ADC_CHANNEL_INPUT_PIN_SINGLE = 0u, /**< External analog single-ended signal will be used as channel input (pin will be initialized)  */
    ADC_CHANNEL_INPUT_PIN_DIFF,        /**< External analog differential signal (channel i - channel i + 1), pins will be initialized  */
    ADC_CHANNEL_INPUT_TEMP,            /**< (ADC1 / ADC3 In 17) Temperature sensor is connected to the internal channel                 */
    ADC_CHANNEL_INPUT_VREF,            /**< (ADC1 In 0) Internal reference voltage is connected                                        */
    ADC_CHANNEL_INPUT_VBAT,            /**< (ADC1 / ADC3 In 18) Battery voltage divided by 3 is connected                               */
    ADC_CHANNEL_INPUT_DAC1,            /**< (ADC2 In 17 / ADC3 In 14, single ADC except L4R / L4S: ADC1 In 17) DAC1 output 1 is connected */
    ADC_CHANNEL_INPUT_DAC2,            /**< (ADC2 In 18 / ADC3 In 15, single ADC except L4R / L4S: ADC1 In 18) DAC1 output 2 is connected */
    ADC_CHANNEL_INPUT_CNT              /**< Count of available input options                                                            */
}   adc_ChannelInput_t;

/* -------------------------------------------------------------------------- */
/* ------------------ Channel list (peripheral, channel, input) ------------- */
/* -------------------------------------------------------------------------- */

/** Peripheral identification bit offset in encoded channel value */
#define ADC_CHANNEL_BIT_MASK_PERIPH_BIT_OFFSET  ( 16u )

/** Channel identification bit offset in encoded channel value */
#define ADC_CHANNEL_BIT_MASK_CHANNEL_BIT_OFFSET ( 8u )

/** Input identification bit offset in encoded channel value */
#define ADC_CHANNEL_BIT_MASK_INPUT_BIT_OFFSET   ( 0u )

/** Mask of one field (8 bits) in encoded channel value */
#define ADC_CHANNEL_BIT_MASK_FIELD              ( 0xFFu )

/**
 * \brief Encodes the channel (peripheral, channel number, input) into single value of \ref adc_Channel_t
 *
 * The macro defines the items of the channel list \ref adc_Channel_t, e.g. ADC1 channel 5 with the pin PA0 is
 * \ref ADC_CH_ADC1_IN5_PA0: ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_5, ADC_CHANNEL_INPUT_PIN_SINGLE )
 */
#define ADC_CHANNEL_ENCODE( PERIPH_ID, CHANNEL_ID, INPUT_ID )  ( (uint32_t)( ( (uint32_t)(PERIPH_ID)  << ADC_CHANNEL_BIT_MASK_PERIPH_BIT_OFFSET  ) | \
                                                                             ( (uint32_t)(CHANNEL_ID) << ADC_CHANNEL_BIT_MASK_CHANNEL_BIT_OFFSET ) | \
                                                                             ( (uint32_t)(INPUT_ID)   << ADC_CHANNEL_BIT_MASK_INPUT_BIT_OFFSET   )   ) )

/** Extract ADC peripheral ID (\ref adc_PeriphId_t) from encoded channel value */
#define ADC_CHANNEL_DECODE_PERIPH( CODED_VAL )   ( ( (uint32_t)(CODED_VAL) >> ADC_CHANNEL_BIT_MASK_PERIPH_BIT_OFFSET ) & ADC_CHANNEL_BIT_MASK_FIELD )

/** Extract channel number (\ref adc_ChannelId_t) from encoded channel value */
#define ADC_CHANNEL_DECODE_CHANNEL( CODED_VAL )  ( ( (uint32_t)(CODED_VAL) >> ADC_CHANNEL_BIT_MASK_CHANNEL_BIT_OFFSET ) & ADC_CHANNEL_BIT_MASK_FIELD )

/** Extract channel input (\ref adc_ChannelInput_t) from encoded channel value */
#define ADC_CHANNEL_DECODE_INPUT( CODED_VAL )    ( ( (uint32_t)(CODED_VAL) >> ADC_CHANNEL_BIT_MASK_INPUT_BIT_OFFSET ) & ADC_CHANNEL_BIT_MASK_FIELD )


/**
 * \brief List of the channels of the ADC peripherals - one item per valid combination of the ADC peripheral, the channel
 *        number and the input of the device line
 *
 * - External pin: ADC_CH_ADC<n>_IN<channel>_P<port><pin> (single-ended) and
 *   ADC_CH_ADC<n>_IN<channel>_DIFF_P<port><pin>_P<port><pin> (differential, positive and negative pin), the pins of the
 *   device line (STM32CubeMX database, union of the packages of the devices of the line).
 * - Internal signal: ADC_CH_ADC<n>_TEMP / VREF / VBAT / DAC1 / DAC2, the signal exists on the peripheral and is connected
 *   to the channel number of the item.
 *
 * The item shall belong to the peripheral of the configuration (\ref adc_PeriphConfig_t::PeriphId), otherwise the
 * configuration is refused.
 */
typedef enum
{
#if !defined(STM32L432xx) && \
    !defined(STM32L442xx)
    ADC_CH_ADC1_IN1_PC0            = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_1 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 1, single-ended input, pin PC0 */
    ADC_CH_ADC1_IN2_PC1            = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_2 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 2, single-ended input, pin PC1 */
    ADC_CH_ADC1_IN3_PC2            = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_3 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 3, single-ended input, pin PC2 */
    ADC_CH_ADC1_IN4_PC3            = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_4 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 4, single-ended input, pin PC3 */
#endif
    ADC_CH_ADC1_IN5_PA0            = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_5 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 5, single-ended input, pin PA0 */
    ADC_CH_ADC1_IN6_PA1            = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_6 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 6, single-ended input, pin PA1 */
    ADC_CH_ADC1_IN7_PA2            = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_7 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 7, single-ended input, pin PA2 */
    ADC_CH_ADC1_IN8_PA3            = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_8 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 8, single-ended input, pin PA3 */
    ADC_CH_ADC1_IN9_PA4            = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_9 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 9, single-ended input, pin PA4 */
    ADC_CH_ADC1_IN10_PA5           = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_10, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 10, single-ended input, pin PA5 */
    ADC_CH_ADC1_IN11_PA6           = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_11, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 11, single-ended input, pin PA6 */
    ADC_CH_ADC1_IN12_PA7           = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_12, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 12, single-ended input, pin PA7 */
#if !defined(STM32L432xx) && \
    !defined(STM32L442xx)
    ADC_CH_ADC1_IN13_PC4           = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_13, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 13, single-ended input, pin PC4 */
    ADC_CH_ADC1_IN14_PC5           = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_14, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 14, single-ended input, pin PC5 */
#endif
    ADC_CH_ADC1_IN15_PB0           = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_15, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 15, single-ended input, pin PB0 */
    ADC_CH_ADC1_IN16_PB1           = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_16, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC1 channel 16, single-ended input, pin PB1 */
#if !defined(STM32L432xx) && \
    !defined(STM32L442xx)
    ADC_CH_ADC1_IN1_DIFF_PC0_PC1   = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_1 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 1, differential input, pins PC0 / PC1 */
    ADC_CH_ADC1_IN2_DIFF_PC1_PC2   = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_2 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 2, differential input, pins PC1 / PC2 */
    ADC_CH_ADC1_IN3_DIFF_PC2_PC3   = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_3 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 3, differential input, pins PC2 / PC3 */
    ADC_CH_ADC1_IN4_DIFF_PC3_PA0   = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_4 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 4, differential input, pins PC3 / PA0 */
#endif
    ADC_CH_ADC1_IN5_DIFF_PA0_PA1   = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_5 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 5, differential input, pins PA0 / PA1 */
    ADC_CH_ADC1_IN6_DIFF_PA1_PA2   = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_6 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 6, differential input, pins PA1 / PA2 */
    ADC_CH_ADC1_IN7_DIFF_PA2_PA3   = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_7 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 7, differential input, pins PA2 / PA3 */
    ADC_CH_ADC1_IN8_DIFF_PA3_PA4   = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_8 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 8, differential input, pins PA3 / PA4 */
    ADC_CH_ADC1_IN9_DIFF_PA4_PA5   = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_9 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 9, differential input, pins PA4 / PA5 */
    ADC_CH_ADC1_IN10_DIFF_PA5_PA6  = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_10, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 10, differential input, pins PA5 / PA6 */
    ADC_CH_ADC1_IN11_DIFF_PA6_PA7  = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_11, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 11, differential input, pins PA6 / PA7 */
#if !defined(STM32L432xx) && \
    !defined(STM32L442xx)
    ADC_CH_ADC1_IN12_DIFF_PA7_PC4  = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_12, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 12, differential input, pins PA7 / PC4 */
    ADC_CH_ADC1_IN13_DIFF_PC4_PC5  = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_13, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 13, differential input, pins PC4 / PC5 */
    ADC_CH_ADC1_IN14_DIFF_PC5_PB0  = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_14, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 14, differential input, pins PC5 / PB0 */
#endif
    ADC_CH_ADC1_IN15_DIFF_PB0_PB1  = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_15, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC1 channel 15, differential input, pins PB0 / PB1 */
    ADC_CH_ADC1_TEMP               = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_17, ADC_CHANNEL_INPUT_TEMP ), /**< ADC1 channel 17, temperature sensor */
    ADC_CH_ADC1_VREF               = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_0 , ADC_CHANNEL_INPUT_VREF ), /**< ADC1 channel 0, internal reference voltage */
    ADC_CH_ADC1_VBAT               = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_18, ADC_CHANNEL_INPUT_VBAT ), /**< ADC1 channel 18, battery voltage divided by 3 */
#if defined(STM32L431xx) || \
    defined(STM32L432xx) || \
    defined(STM32L433xx) || \
    defined(STM32L442xx) || \
    defined(STM32L443xx) || \
    defined(STM32L451xx) || \
    defined(STM32L452xx) || \
    defined(STM32L462xx)
    ADC_CH_ADC1_DAC1               = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_17, ADC_CHANNEL_INPUT_DAC1 ), /**< ADC1 channel 17, DAC1 output 1 */
#endif
#if defined(STM32L431xx) || \
    defined(STM32L432xx) || \
    defined(STM32L433xx) || \
    defined(STM32L442xx) || \
    defined(STM32L443xx)
    ADC_CH_ADC1_DAC2               = ADC_CHANNEL_ENCODE( ADC_PERIPH_1, ADC_CHANNEL_18, ADC_CHANNEL_INPUT_DAC2 ), /**< ADC1 channel 18, DAC1 output 2 */
#endif

#if defined(ADC2)
    ADC_CH_ADC2_IN1_PC0            = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_1 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 1, single-ended input, pin PC0 */
    ADC_CH_ADC2_IN2_PC1            = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_2 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 2, single-ended input, pin PC1 */
    ADC_CH_ADC2_IN3_PC2            = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_3 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 3, single-ended input, pin PC2 */
    ADC_CH_ADC2_IN4_PC3            = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_4 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 4, single-ended input, pin PC3 */
#if !defined(STM32L412xx) && \
    !defined(STM32L422xx)
    ADC_CH_ADC2_IN5_PA0            = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_5 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 5, single-ended input, pin PA0 */
    ADC_CH_ADC2_IN6_PA1            = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_6 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 6, single-ended input, pin PA1 */
#endif
    ADC_CH_ADC2_IN7_PA2            = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_7 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 7, single-ended input, pin PA2 */
    ADC_CH_ADC2_IN8_PA3            = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_8 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 8, single-ended input, pin PA3 */
    ADC_CH_ADC2_IN9_PA4            = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_9 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 9, single-ended input, pin PA4 */
    ADC_CH_ADC2_IN10_PA5           = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_10, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 10, single-ended input, pin PA5 */
    ADC_CH_ADC2_IN11_PA6           = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_11, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 11, single-ended input, pin PA6 */
    ADC_CH_ADC2_IN12_PA7           = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_12, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 12, single-ended input, pin PA7 */
    ADC_CH_ADC2_IN13_PC4           = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_13, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 13, single-ended input, pin PC4 */
    ADC_CH_ADC2_IN14_PC5           = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_14, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 14, single-ended input, pin PC5 */
    ADC_CH_ADC2_IN15_PB0           = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_15, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 15, single-ended input, pin PB0 */
    ADC_CH_ADC2_IN16_PB1           = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_16, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC2 channel 16, single-ended input, pin PB1 */
    ADC_CH_ADC2_IN1_DIFF_PC0_PC1   = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_1 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 1, differential input, pins PC0 / PC1 */
    ADC_CH_ADC2_IN2_DIFF_PC1_PC2   = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_2 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 2, differential input, pins PC1 / PC2 */
    ADC_CH_ADC2_IN3_DIFF_PC2_PC3   = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_3 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 3, differential input, pins PC2 / PC3 */
#if !defined(STM32L412xx) && \
    !defined(STM32L422xx)
    ADC_CH_ADC2_IN4_DIFF_PC3_PA0   = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_4 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 4, differential input, pins PC3 / PA0 */
    ADC_CH_ADC2_IN5_DIFF_PA0_PA1   = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_5 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 5, differential input, pins PA0 / PA1 */
    ADC_CH_ADC2_IN6_DIFF_PA1_PA2   = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_6 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 6, differential input, pins PA1 / PA2 */
#endif
    ADC_CH_ADC2_IN7_DIFF_PA2_PA3   = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_7 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 7, differential input, pins PA2 / PA3 */
    ADC_CH_ADC2_IN8_DIFF_PA3_PA4   = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_8 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 8, differential input, pins PA3 / PA4 */
    ADC_CH_ADC2_IN9_DIFF_PA4_PA5   = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_9 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 9, differential input, pins PA4 / PA5 */
    ADC_CH_ADC2_IN10_DIFF_PA5_PA6  = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_10, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 10, differential input, pins PA5 / PA6 */
    ADC_CH_ADC2_IN11_DIFF_PA6_PA7  = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_11, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 11, differential input, pins PA6 / PA7 */
    ADC_CH_ADC2_IN12_DIFF_PA7_PC4  = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_12, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 12, differential input, pins PA7 / PC4 */
    ADC_CH_ADC2_IN13_DIFF_PC4_PC5  = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_13, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 13, differential input, pins PC4 / PC5 */
    ADC_CH_ADC2_IN14_DIFF_PC5_PB0  = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_14, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 14, differential input, pins PC5 / PB0 */
    ADC_CH_ADC2_IN15_DIFF_PB0_PB1  = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_15, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC2 channel 15, differential input, pins PB0 / PB1 */
#if !defined(STM32L412xx) && \
    !defined(STM32L422xx)
    ADC_CH_ADC2_DAC1               = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_17, ADC_CHANNEL_INPUT_DAC1 ), /**< ADC2 channel 17, DAC1 output 1 */
    ADC_CH_ADC2_DAC2               = ADC_CHANNEL_ENCODE( ADC_PERIPH_2, ADC_CHANNEL_18, ADC_CHANNEL_INPUT_DAC2 ), /**< ADC2 channel 18, DAC1 output 2 */
#endif
#endif /* ADC2 */

#if defined(ADC3)
    ADC_CH_ADC3_IN1_PC0            = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_1 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 1, single-ended input, pin PC0 */
    ADC_CH_ADC3_IN2_PC1            = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_2 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 2, single-ended input, pin PC1 */
    ADC_CH_ADC3_IN3_PC2            = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_3 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 3, single-ended input, pin PC2 */
    ADC_CH_ADC3_IN4_PC3            = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_4 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 4, single-ended input, pin PC3 */
#if !defined(STM32L475xx) && \
    !defined(STM32L485xx)
    ADC_CH_ADC3_IN6_PF3            = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_6 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 6, single-ended input, pin PF3 */
    ADC_CH_ADC3_IN7_PF4            = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_7 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 7, single-ended input, pin PF4 */
    ADC_CH_ADC3_IN8_PF5            = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_8 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 8, single-ended input, pin PF5 */
    ADC_CH_ADC3_IN9_PF6            = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_9 , ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 9, single-ended input, pin PF6 */
    ADC_CH_ADC3_IN10_PF7           = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_10, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 10, single-ended input, pin PF7 */
    ADC_CH_ADC3_IN11_PF8           = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_11, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 11, single-ended input, pin PF8 */
    ADC_CH_ADC3_IN12_PF9           = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_12, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 12, single-ended input, pin PF9 */
    ADC_CH_ADC3_IN13_PF10          = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_13, ADC_CHANNEL_INPUT_PIN_SINGLE ), /**< ADC3 channel 13, single-ended input, pin PF10 */
#endif
    ADC_CH_ADC3_IN1_DIFF_PC0_PC1   = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_1 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 1, differential input, pins PC0 / PC1 */
    ADC_CH_ADC3_IN2_DIFF_PC1_PC2   = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_2 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 2, differential input, pins PC1 / PC2 */
    ADC_CH_ADC3_IN3_DIFF_PC2_PC3   = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_3 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 3, differential input, pins PC2 / PC3 */
#if !defined(STM32L475xx) && \
    !defined(STM32L485xx)
    ADC_CH_ADC3_IN6_DIFF_PF3_PF4   = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_6 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 6, differential input, pins PF3 / PF4 */
    ADC_CH_ADC3_IN7_DIFF_PF4_PF5   = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_7 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 7, differential input, pins PF4 / PF5 */
    ADC_CH_ADC3_IN8_DIFF_PF5_PF6   = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_8 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 8, differential input, pins PF5 / PF6 */
    ADC_CH_ADC3_IN9_DIFF_PF6_PF7   = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_9 , ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 9, differential input, pins PF6 / PF7 */
    ADC_CH_ADC3_IN10_DIFF_PF7_PF8  = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_10, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 10, differential input, pins PF7 / PF8 */
    ADC_CH_ADC3_IN11_DIFF_PF8_PF9  = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_11, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 11, differential input, pins PF8 / PF9 */
    ADC_CH_ADC3_IN12_DIFF_PF9_PF10 = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_12, ADC_CHANNEL_INPUT_PIN_DIFF ), /**< ADC3 channel 12, differential input, pins PF9 / PF10 */
#endif
    ADC_CH_ADC3_TEMP               = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_17, ADC_CHANNEL_INPUT_TEMP ), /**< ADC3 channel 17, temperature sensor */
    ADC_CH_ADC3_VBAT               = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_18, ADC_CHANNEL_INPUT_VBAT ), /**< ADC3 channel 18, battery voltage divided by 3 */
    ADC_CH_ADC3_DAC1               = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_14, ADC_CHANNEL_INPUT_DAC1 ), /**< ADC3 channel 14, DAC1 output 1 */
    ADC_CH_ADC3_DAC2               = ADC_CHANNEL_ENCODE( ADC_PERIPH_3, ADC_CHANNEL_15, ADC_CHANNEL_INPUT_DAC2 ), /**< ADC3 channel 15, DAC1 output 2 */
#endif /* ADC3 */
}   adc_Channel_t;


/* -------------------------------------------------------------------------- */
/* ---------------------- Analog Watch-dog configuration -------------------- */
/* -------------------------------------------------------------------------- */

/** \brief List of available Analog Watch-dog components */
typedef enum
{
    ADC_AWD_1 = 0u, /**< Analog Watch-dog 1         */
    ADC_AWD_2,      /**< Analog Watch-dog 2         */
    ADC_AWD_3,      /**< Analog Watch-dog 3         */
    ADC_AWD_CNT     /**< Count of Analog Watch-dogs */
}   adc_AwdId_t;


/**
 * \brief Analog Watch-Dog 1 operation modes.
 *
 */
typedef enum
{
    ADC_AWD_MODE_ALL  = 0u,       /**< All channels will be monitored (regular & injected)     */
    ADC_AWD_MODE_ALL_REGULAR,     /**< All channels configured as regular will be monitored    */
    ADC_AWD_MODE_ALL_INJECTED,    /**< All channels configured as injected will be monitored   */
    ADC_AWD_MODE_SINGLE,          /**< Single channel (Regular & injected) will be monitored   */
    ADC_AWD_MODE_SINGLE_REGULAR,  /**< Single channel configured as regular will be monitored  */
    ADC_AWD_MODE_SINGLE_INJECTED, /**< Single channel configured as injected will be monitored */
    ADC_AWD_MODE_CNT,             /**< Count of Analog Watch-dog modes                         */
}   adc_AwdMode_t;


/**
 * \brief List of available AWD channels.
 *
 * \note AWD1 - Support only one channel at a time
 * \note AWD2 & AWD3 - Support multiple channels
 *
 */
typedef enum
{
    ADC_AWD_MODE_CHANNEL_0  = (1u << 0u),  /**< Channel 0 will be monitored       */
    ADC_AWD_MODE_CHANNEL_1  = (1u << 1u),  /**< Channel 1 will be monitored       */
    ADC_AWD_MODE_CHANNEL_2  = (1u << 2u),  /**< Channel 2 will be monitored       */
    ADC_AWD_MODE_CHANNEL_3  = (1u << 3u),  /**< Channel 3 will be monitored       */
    ADC_AWD_MODE_CHANNEL_4  = (1u << 4u),  /**< Channel 4 will be monitored       */
    ADC_AWD_MODE_CHANNEL_5  = (1u << 5u),  /**< Channel 5 will be monitored       */
    ADC_AWD_MODE_CHANNEL_6  = (1u << 6u),  /**< Channel 6 will be monitored       */
    ADC_AWD_MODE_CHANNEL_7  = (1u << 7u),  /**< Channel 7 will be monitored       */
    ADC_AWD_MODE_CHANNEL_8  = (1u << 8u),  /**< Channel 8 will be monitored       */
    ADC_AWD_MODE_CHANNEL_9  = (1u << 9u),  /**< Channel 9 will be monitored       */
    ADC_AWD_MODE_CHANNEL_10 = (1u << 10u), /**< Channel 10 will be monitored      */
    ADC_AWD_MODE_CHANNEL_11 = (1u << 11u), /**< Channel 11 will be monitored      */
    ADC_AWD_MODE_CHANNEL_12 = (1u << 12u), /**< Channel 12 will be monitored      */
    ADC_AWD_MODE_CHANNEL_13 = (1u << 13u), /**< Channel 13 will be monitored      */
    ADC_AWD_MODE_CHANNEL_14 = (1u << 14u), /**< Channel 14 will be monitored      */
    ADC_AWD_MODE_CHANNEL_15 = (1u << 15u), /**< Channel 15 will be monitored      */
    ADC_AWD_MODE_CHANNEL_16 = (1u << 16u), /**< Channel 16 will be monitored      */
    ADC_AWD_MODE_CHANNEL_17 = (1u << 17u), /**< Channel 17 will be monitored      */
    ADC_AWD_MODE_CHANNEL_18 = (1u << 18u), /**< Channel 18 will be monitored      */
}   adc_AwdChannelId_t;


/** \brief Type representing Analog Watch-dog threshold values (in RAW) */
typedef uint16_t adc_AwdThreshold_t;


/** \brief List of Analog Watch-dog event filtration samples count.
 *
 * \note STM32L4 ADC has no Analog Watch-dog event filter - only ADC_AWD_FILTER_NONE is accepted
 *       (the values are kept for the common interface).
 */
typedef enum
{
    ADC_AWD_FILTER_NONE = 0u, /**< Analog Watch-dog event inactive         */
    ADC_AWD_FILTER_2,         /**< Analog Watch-dog event filter 2 samples */
    ADC_AWD_FILTER_3,         /**< Analog Watch-dog event filter 3 samples */
    ADC_AWD_FILTER_4,         /**< Analog Watch-dog event filter 4 samples */
    ADC_AWD_FILTER_5,         /**< Analog Watch-dog event filter 5 samples */
    ADC_AWD_FILTER_6,         /**< Analog Watch-dog event filter 6 samples */
    ADC_AWD_FILTER_7,         /**< Analog Watch-dog event filter 7 samples */
    ADC_AWD_FILTER_8,         /**< Analog Watch-dog event filter 8 samples */
    ADC_AWD_FILTER_CNT,       /**< Count of available filter options       */
}   adc_AwdFilter_t;


/* -------------------------------------------------------------------------- */
/* ---------------------- Data handling configuration ----------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief List of regular group data transfer modes
 *
 * All modes store regular conversion results into the same user buffer and signal the same
 * events through the same callbacks - they differ only in the context moving the data:
 * - DMA:  DMA channel (callbacks from DMA interrupt)
 * - ISR:  ADC end of conversion interrupt (callbacks from ADC interrupt)
 * - POLL: Adc_Task() polling ADC flags (callbacks from Adc_Task() context)
 */
typedef enum
{
    ADC_TRANSFER_MODE_DMA = 0u, /**< Data are transferred by DMA                              */
    ADC_TRANSFER_MODE_ISR,      /**< Data are transferred by ADC interrupt service routine    */
    ADC_TRANSFER_MODE_POLL,     /**< Data are transferred by Adc_Task() (polling of ADC flags) */
    ADC_TRANSFER_MODE_CNT       /**< Count of available data transfer modes                   */
}   adc_TransferMode_t;


/** \brief List of data buffer handling modes */
typedef enum
{
    ADC_BUFFER_MODE_ONE_SHOT = 0u, /**< Transfer stops when the buffer is full                        */
    ADC_BUFFER_MODE_CIRCULAR,      /**< Transfer continues from the buffer start when buffer is full */
    ADC_BUFFER_MODE_CNT            /**< Count of available buffer modes                            */
}   adc_BufferMode_t;


/** \brief Type representing data buffer size in count of \ref adc_Data_t items */
typedef uint16_t adc_BufferSize_t;


/** \brief List of data transfer errors reported through \ref adc_ErrCallback_t */
typedef enum
{
    ADC_ERROR_OVERRUN = 0u,        /**< ADC overrun - conversion result was not read in time (OVR)  */
    ADC_ERROR_DMA_TRANSFER,        /**< DMA transfer error (bus error during transfer)             */
    ADC_ERROR_DMA_CONFIG,          /**< DMA configuration error - not reported by STM32L4          */
    ADC_ERROR_DMA_CONFIG_UPDATE,   /**< DMA configuration update error - not reported by STM32L4   */
    ADC_ERROR_DMA_TRIGGER_OVERRUN, /**< DMA trigger overrun - not reported by STM32L4              */
    ADC_ERROR_CNT                  /**< Count of data transfer errors                               */
}   adc_ErrorId_t;


/** \brief Encoded DMA channel (value of \ref adc_Dma_t) */
typedef uint32_t adc_DmaCode_t;


/** DMA peripherals enumeration list
 *
 * \note The DMA channels usable by the ADC peripherals are given by the list \ref adc_Dma_t. */
typedef enum
{
    ADC_DMA_PERIPH_1 = DMA_PERIPH_1, /**< DMA peripheral 1 identification */
#if defined(DMA2)
    ADC_DMA_PERIPH_2 = DMA_PERIPH_2, /**< DMA peripheral 2 identification */
#endif
    ADC_DMA_PERIPH_CNT               /**< Count of DMA peripherals        */
}   adc_DmaPeriphId_t;


/**
 * \brief Enumeration of available channels of DMA peripheral
 *
 * STM32L4 without DMAMUX1: the channel has to be one of the channels connected to the ADC request
 * (DMA request mapping of the reference manual, e.g. ADC1: DMA1 channel 1 or DMA2 channel 3),
 * STM32L4+ routes the request to any channel. The channels usable by the ADC peripherals are given
 * by the list \ref adc_Dma_t.
 */
typedef enum
{
    ADC_DMA_CHANNEL_1 = DMA_CHANNEL_1, /**< DMA channel 1 */
    ADC_DMA_CHANNEL_2 = DMA_CHANNEL_2, /**< DMA channel 2 */
    ADC_DMA_CHANNEL_3 = DMA_CHANNEL_3, /**< DMA channel 3 */
    ADC_DMA_CHANNEL_4 = DMA_CHANNEL_4, /**< DMA channel 4 */
    ADC_DMA_CHANNEL_5 = DMA_CHANNEL_5, /**< DMA channel 5 */
    ADC_DMA_CHANNEL_6 = DMA_CHANNEL_6, /**< DMA channel 6 */
#if defined(DMA1_Channel7)
    ADC_DMA_CHANNEL_7 = DMA_CHANNEL_7, /**< DMA channel 7 */
#endif
#if defined(DMA1_Channel8)
    ADC_DMA_CHANNEL_8 = DMA_CHANNEL_8, /**< DMA channel 8 */
#endif
    ADC_DMA_CHANNEL_CNT                /**< Count of DMA channels */
}   adc_DmaChannelId_t;


/**
 * \brief List of DMA channels able to serve the ADC regular group of the peripherals (STM32CubeMX database / reference
 *        manual DMA request mapping, the request selection (DMA_CSELR) of the channel is part of the value;
 *        STM32L4+ devices with DMAMUX1 route the request to every channel of DMA1 / DMA2)
 */
typedef enum
{
#if !defined(DMAMUX1)
    ADC_DMA_ADC1_DMA1_CHANNEL1         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_1, 0u ), /**< ADC1 request on DMA1 channel 1 (request selection 0) */
    ADC_DMA_ADC1_DMA2_CHANNEL3         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_3, 0u ), /**< ADC1 request on DMA2 channel 3 (request selection 0) */
#else
    ADC_DMA_ADC1_DMA1_CHANNEL1         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_1, 0u ), /**< ADC1 request on DMA1 channel 1 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA1_CHANNEL2         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_2, 0u ), /**< ADC1 request on DMA1 channel 2 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA1_CHANNEL3         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_3, 0u ), /**< ADC1 request on DMA1 channel 3 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA1_CHANNEL4         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_4, 0u ), /**< ADC1 request on DMA1 channel 4 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA1_CHANNEL5         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_5, 0u ), /**< ADC1 request on DMA1 channel 5 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA1_CHANNEL6         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_6, 0u ), /**< ADC1 request on DMA1 channel 6 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA1_CHANNEL7         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_7, 0u ), /**< ADC1 request on DMA1 channel 7 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA2_CHANNEL1         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_1, 0u ), /**< ADC1 request on DMA2 channel 1 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA2_CHANNEL2         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_2, 0u ), /**< ADC1 request on DMA2 channel 2 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA2_CHANNEL3         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_3, 0u ), /**< ADC1 request on DMA2 channel 3 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA2_CHANNEL4         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_4, 0u ), /**< ADC1 request on DMA2 channel 4 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA2_CHANNEL5         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_5, 0u ), /**< ADC1 request on DMA2 channel 5 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA2_CHANNEL6         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_6, 0u ), /**< ADC1 request on DMA2 channel 6 (routed by DMAMUX1) */
    ADC_DMA_ADC1_DMA2_CHANNEL7         = ADC_DMA_ENCODE( ADC_PERIPH_1, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_7, 0u ), /**< ADC1 request on DMA2 channel 7 (routed by DMAMUX1) */
#endif /* DMAMUX1 */
#if defined(ADC2)
#if !defined(DMAMUX1)
    ADC_DMA_ADC2_DMA1_CHANNEL2         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_2, 0u ), /**< ADC2 request on DMA1 channel 2 (request selection 0) */
    ADC_DMA_ADC2_DMA2_CHANNEL4         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_4, 0u ), /**< ADC2 request on DMA2 channel 4 (request selection 0) */
#else
    ADC_DMA_ADC2_DMA1_CHANNEL1         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_1, 0u ), /**< ADC2 request on DMA1 channel 1 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA1_CHANNEL2         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_2, 0u ), /**< ADC2 request on DMA1 channel 2 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA1_CHANNEL3         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_3, 0u ), /**< ADC2 request on DMA1 channel 3 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA1_CHANNEL4         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_4, 0u ), /**< ADC2 request on DMA1 channel 4 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA1_CHANNEL5         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_5, 0u ), /**< ADC2 request on DMA1 channel 5 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA1_CHANNEL6         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_6, 0u ), /**< ADC2 request on DMA1 channel 6 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA1_CHANNEL7         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_7, 0u ), /**< ADC2 request on DMA1 channel 7 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA2_CHANNEL1         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_1, 0u ), /**< ADC2 request on DMA2 channel 1 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA2_CHANNEL2         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_2, 0u ), /**< ADC2 request on DMA2 channel 2 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA2_CHANNEL3         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_3, 0u ), /**< ADC2 request on DMA2 channel 3 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA2_CHANNEL4         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_4, 0u ), /**< ADC2 request on DMA2 channel 4 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA2_CHANNEL5         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_5, 0u ), /**< ADC2 request on DMA2 channel 5 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA2_CHANNEL6         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_6, 0u ), /**< ADC2 request on DMA2 channel 6 (routed by DMAMUX1) */
    ADC_DMA_ADC2_DMA2_CHANNEL7         = ADC_DMA_ENCODE( ADC_PERIPH_2, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_7, 0u ), /**< ADC2 request on DMA2 channel 7 (routed by DMAMUX1) */
#endif /* DMAMUX1 */
#endif /* ADC2 */
#if defined(ADC3)
#if !defined(DMAMUX1)
    ADC_DMA_ADC3_DMA1_CHANNEL3         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_3, 0u ), /**< ADC3 request on DMA1 channel 3 (request selection 0) */
    ADC_DMA_ADC3_DMA2_CHANNEL5         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_5, 0u ), /**< ADC3 request on DMA2 channel 5 (request selection 0) */
#else
    ADC_DMA_ADC3_DMA1_CHANNEL1         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_1, 0u ), /**< ADC3 request on DMA1 channel 1 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA1_CHANNEL2         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_2, 0u ), /**< ADC3 request on DMA1 channel 2 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA1_CHANNEL3         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_3, 0u ), /**< ADC3 request on DMA1 channel 3 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA1_CHANNEL4         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_4, 0u ), /**< ADC3 request on DMA1 channel 4 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA1_CHANNEL5         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_5, 0u ), /**< ADC3 request on DMA1 channel 5 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA1_CHANNEL6         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_6, 0u ), /**< ADC3 request on DMA1 channel 6 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA1_CHANNEL7         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_1, ADC_DMA_CHANNEL_7, 0u ), /**< ADC3 request on DMA1 channel 7 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA2_CHANNEL1         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_1, 0u ), /**< ADC3 request on DMA2 channel 1 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA2_CHANNEL2         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_2, 0u ), /**< ADC3 request on DMA2 channel 2 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA2_CHANNEL3         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_3, 0u ), /**< ADC3 request on DMA2 channel 3 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA2_CHANNEL4         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_4, 0u ), /**< ADC3 request on DMA2 channel 4 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA2_CHANNEL5         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_5, 0u ), /**< ADC3 request on DMA2 channel 5 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA2_CHANNEL6         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_6, 0u ), /**< ADC3 request on DMA2 channel 6 (routed by DMAMUX1) */
    ADC_DMA_ADC3_DMA2_CHANNEL7         = ADC_DMA_ENCODE( ADC_PERIPH_3, ADC_DMA_PERIPH_2, ADC_DMA_CHANNEL_7, 0u ), /**< ADC3 request on DMA2 channel 7 (routed by DMAMUX1) */
#endif /* DMAMUX1 */
#endif /* ADC3 */
    ADC_DMA_UNUSED                     = ADC_DMA_CODE_UNUSED  /**< DMA channel is not selected */
}   adc_Dma_t;


/** DMA channel priority options enumeration */
typedef enum
{
    ADC_DMA_PRIORITY_LOW      = DMA_PRIORITY_LOW     , /**< Priority level : Low       */
    ADC_DMA_PRIORITY_MEDIUM   = DMA_PRIORITY_MEDIUM  , /**< Priority level : Medium    */
    ADC_DMA_PRIORITY_HIGH     = DMA_PRIORITY_HIGH    , /**< Priority level : High      */
    ADC_DMA_PRIORITY_VERYHIGH = DMA_PRIORITY_VERYHIGH, /**< Priority level : Very high */
}   adc_DmaPriority_t;


/** \brief Type representing interrupt priority (ADC interrupt in ISR mode, DMA interrupt in DMA mode) */
typedef uint32_t adc_IrqPrio_t;


/** \brief Data transfer event callback (half transfer, transfer complete, injected sequence complete) */
typedef void ( adc_Callback_t )( void );


/** \brief Data transfer error callback, error identification is given as parameter */
typedef void ( adc_ErrCallback_t )( adc_ErrorId_t errorId );

/* -------------------------------------------------------------------------- */
/* ------------------------ Configuration structure ------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Regular group data transfer configuration (common for DMA, ISR and POLL mode)
 *
 * Callback events (equivalent in all modes):
 * - HalfTransferCallback:     BufferSize / 2 results were stored into DataBuffer
 * - TransferCompleteCallback: BufferSize results were stored into DataBuffer (in circular mode
 *                             the next result is stored to DataBuffer[ 0 ])
 * - ErrorCallback:            ADC overrun or DMA error (\ref adc_ErrorId_t)
 * - InjCompleteCallback:      injected sequence converted (JEOS), results are read by Adc_Get_InjData()
 *
 * Unused callback shall be set to ADC_NULL_PTR (related interrupt is not activated).
 * Dma / DmaPriority are used only in ADC_TRANSFER_MODE_DMA (Dma is an item of the list \ref adc_Dma_t of the
 * configured ADC peripheral - channels of the STM32L4 request mapping, any DMA1 / DMA2 channel through
 * DMAMUX1 on STM32L4+; ADC_DMA_UNUSED in other modes), IrqPriority is not used in ADC_TRANSFER_MODE_POLL.
 */
typedef struct
{
    adc_TransferMode_t  TransferMode;             /**< Data transfer mode (DMA / ISR / POLL)                         */
    adc_Data_t         *DataBuffer;               /**< Buffer for regular conversion results. Must not be NULL.      */
    adc_BufferSize_t    BufferSize;               /**< Buffer size in count of adc_Data_t items (> 0)                */
    adc_BufferMode_t    BufferMode;               /**< One shot / circular buffer handling                           */

    adc_Dma_t           Dma;                      /**< DMA channel (DMA mode only)                                   */
    adc_DmaPriority_t   DmaPriority;              /**< DMA channel priority (DMA mode only)                          */

    adc_IrqPrio_t       IrqPriority;              /**< ADC (ISR mode) / DMA (DMA mode) interrupt priority            */

    adc_Callback_t     *HalfTransferCallback;     /**< Buffer half filled. ADC_NULL_PTR if not used.                 */
    adc_Callback_t     *TransferCompleteCallback; /**< Buffer filled. ADC_NULL_PTR if not used.                      */
    adc_ErrCallback_t  *ErrorCallback;            /**< Transfer error. ADC_NULL_PTR if not used.                     */
    adc_Callback_t     *InjCompleteCallback;      /**< Injected sequence converted (JEOS). ADC_NULL_PTR if not used. */
}   adc_DataConfig_t;

/** \brief Analog Watch-dog configuration structure */
typedef struct
{
    adc_AwdId_t        AwdId;            /**< Analog Watch-dog identification                 */
    adc_AwdMode_t      AwdMode;          /**< Monitored channel group                         */
    adc_AwdThreshold_t AwdLowThreshold;  /**< Lower threshold (RAW value)                     */
    adc_AwdThreshold_t AwdHighThreshold; /**< Upper threshold (RAW value)                     */
    adc_AwdFilter_t    AwdFilter;        /**< Event filtering (other than NONE only for AWD1) */
}   adc_AwdConfig_t;


/** \brief Channel configuration structure */
typedef struct
{
    adc_Channel_t         Channel;         /**< Channel (item of the peripheral PeriphId from the list \ref adc_Channel_t) */
    adc_ChannelSampling_t ChannelSampling; /**< Channel sampling time (internal inputs have a minimum)  */
}   adc_ChannelConfig_t;


/** \brief Peripheral configuration structure */
typedef struct
{
    adc_PeriphId_t       PeriphId;                              /**< ADC peripheral identification                            */
    adc_Resolution_t     Resolution;                            /**< ADC resolution (common for all channels)                 */

    adc_DataConfig_t     DataConfig;                            /**< Data transfer configuration                              */

    adc_RegTriggerMode_t RegTriggerMode;                        /**< Regular group conversion mode (single / continuous)      */
    adc_TriggerEdge_t    RegTriggerEdge;                        /**< Regular group external trigger edge (ignored for SW)     */
    adc_RegTriggerId_t   RegTriggerId;                          /**< Regular group trigger source                             */
    adc_RegSequenceLen_t RegChannelsCnt;                        /**< Regular sequence length (0 - 16, 0 = group not used)     */
    adc_ChannelConfig_t  RegChannels[ ADC_REG_SEQUENCE_CNT ];   /**< Regular sequence, array index = rank (index 0 = rank 1)  */

    adc_InjTriggerMode_t InjTriggerMode;                        /**< Injected group conversion mode (single / continuous)     */
    adc_InjTriggerId_t   InjTriggerId;                          /**< Injected group trigger source                            */
    adc_TriggerEdge_t    InjTriggerEdge;                        /**< Injected group external trigger edge (ignored for SW)    */
    adc_InjSequenceLen_t InjChannelsCnt;                        /**< Injected sequence length (0 - 4, 0 = group not used)     */
    adc_ChannelConfig_t  InjChannels[ ADC_INJ_SEQUENCE_CNT ];   /**< Injected sequence, array index = rank (index 0 = rank 1) */
}   adc_PeriphConfig_t;


/** \brief Module configuration representing top layer configuration structure */
typedef struct
{
    adc_ClkSrc_t       ClockSource;                    /**< ADC kernel clock source (common for all ADC peripherals)      */
    adc_ClkDiv_t       ClockDivider;                   /**< ADC kernel clock divider (common for all ADC peripherals)     */
    adc_PeriphConfig_t PeriphConfig[ ADC_PERIPH_CNT ]; /**< Peripheral configuration, indexed by \ref adc_PeriphId_t    */
}   adc_Config_t;


#endif /* ADC_ADC_TYPES_H */

