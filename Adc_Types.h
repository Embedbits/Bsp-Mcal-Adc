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
 * \note STM32G4 family: configuration structures and type names are common for
 *       all families, enumerations with hardware specific values (clock, triggers,
 *       peripherals, channels, channel inputs, DMA) follow STM32G4 ADC.
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

/* ========================== EXPORTED MACROS =============================== */

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
 * \note The clock source is common for all ADC peripherals - it is applied to the ADC1 / ADC2
 *       clock group (ADC12SEL) and to the ADC3 / ADC4 / ADC5 group (ADC345SEL) on devices with
 *       ADC3. HCLK selects the synchronous clock mode (CKMODE, no kernel clock in RCC). */
typedef enum
{
    ADC_CLK_SRC_HCLK = 0u, /**< ADC clocked synchronously by AHB clock (HCLK / 1, 2, 4)       */
    ADC_CLK_SRC_SYSCLK,    /**< ADC kernel clock from system clock (SYSCLK, asynchronous mode) */
    ADC_CLK_SRC_PLLP,      /**< ADC kernel clock from PLL output P (asynchronous mode)          */
    ADC_CLK_SRC_CNT        /**< Count of ADC clock sources                                     */
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
#if defined (ADC4)
    ADC_PERIPH_4,      /**< Analog-to-Digital Converter (ADC) peripheral 4 */
#endif
#if defined (ADC5)
    ADC_PERIPH_5,      /**< Analog-to-Digital Converter (ADC) peripheral 5 */
#endif
    ADC_PERIPH_CNT     /**< Count of Analog-to-Digital Converter (ADC) peripherals */
}   adc_PeriphId_t;


/** \brief List of available regular group conversion trigger sources
 *
 * \note Trigger connection differs between ADC1 / ADC2 and ADC3 / ADC4 / ADC5 - a trigger not
 *       connected to the peripheral is refused with \ref ADC_REQUEST_ERROR. */
typedef enum
{
    ADC_REG_TRIGGER_SOFTWARE = 0u,     /**< Software start (Adc_Set_RegStart())                      */
    ADC_REG_TRIGGER_EXT_TIM1_TRGO,     /**< External trigger TIM1 TRGO                               */
    ADC_REG_TRIGGER_EXT_TIM1_TRGO2,    /**< External trigger TIM1 TRGO2                              */
    ADC_REG_TRIGGER_EXT_TIM1_CH1,      /**< External trigger TIM1 channel 1 (ADC1 / ADC2)            */
    ADC_REG_TRIGGER_EXT_TIM1_CH2,      /**< External trigger TIM1 channel 2 (ADC1 / ADC2)            */
    ADC_REG_TRIGGER_EXT_TIM1_CH3,      /**< External trigger TIM1 channel 3                          */
    ADC_REG_TRIGGER_EXT_TIM2_TRGO,     /**< External trigger TIM2 TRGO                               */
    ADC_REG_TRIGGER_EXT_TIM2_CH1,      /**< External trigger TIM2 channel 1 (ADC3 / ADC4 / ADC5)     */
    ADC_REG_TRIGGER_EXT_TIM2_CH2,      /**< External trigger TIM2 channel 2 (ADC1 / ADC2)            */
    ADC_REG_TRIGGER_EXT_TIM2_CH3,      /**< External trigger TIM2 channel 3 (ADC3 / ADC4 / ADC5)     */
    ADC_REG_TRIGGER_EXT_TIM3_TRGO,     /**< External trigger TIM3 TRGO                               */
    ADC_REG_TRIGGER_EXT_TIM3_CH1,      /**< External trigger TIM3 channel 1 (ADC3 / ADC4 / ADC5)     */
    ADC_REG_TRIGGER_EXT_TIM3_CH4,      /**< External trigger TIM3 channel 4 (ADC1 / ADC2)            */
    ADC_REG_TRIGGER_EXT_TIM4_TRGO,     /**< External trigger TIM4 TRGO                               */
    ADC_REG_TRIGGER_EXT_TIM4_CH1,      /**< External trigger TIM4 channel 1 (ADC3 / ADC4 / ADC5)     */
    ADC_REG_TRIGGER_EXT_TIM4_CH4,      /**< External trigger TIM4 channel 4 (ADC1 / ADC2)            */
    ADC_REG_TRIGGER_EXT_TIM6_TRGO,     /**< External trigger TIM6 TRGO                               */
    ADC_REG_TRIGGER_EXT_TIM7_TRGO,     /**< External trigger TIM7 TRGO                               */
    ADC_REG_TRIGGER_EXT_TIM8_TRGO,     /**< External trigger TIM8 TRGO                               */
    ADC_REG_TRIGGER_EXT_TIM8_TRGO2,    /**< External trigger TIM8 TRGO2                              */
    ADC_REG_TRIGGER_EXT_TIM8_CH1,      /**< External trigger TIM8 channel 1 (ADC3 / ADC4 / ADC5)     */
    ADC_REG_TRIGGER_EXT_TIM15_TRGO,    /**< External trigger TIM15 TRGO                              */
#if defined(TIM20)
    ADC_REG_TRIGGER_EXT_TIM20_TRGO,    /**< External trigger TIM20 TRGO                              */
    ADC_REG_TRIGGER_EXT_TIM20_TRGO2,   /**< External trigger TIM20 TRGO2                             */
    ADC_REG_TRIGGER_EXT_TIM20_CH1,     /**< External trigger TIM20 channel 1                         */
    ADC_REG_TRIGGER_EXT_TIM20_CH2,     /**< External trigger TIM20 channel 2 (ADC1 / ADC2)           */
    ADC_REG_TRIGGER_EXT_TIM20_CH3,     /**< External trigger TIM20 channel 3 (ADC1 / ADC2)           */
#endif /* TIM20 */
#if defined(HRTIM1)
    ADC_REG_TRIGGER_EXT_HRTIM_TRG1,    /**< External trigger HRTIM ADC trigger 1                     */
    ADC_REG_TRIGGER_EXT_HRTIM_TRG2,    /**< External trigger HRTIM ADC trigger 2 (ADC3 / ADC4 / ADC5) */
    ADC_REG_TRIGGER_EXT_HRTIM_TRG3,    /**< External trigger HRTIM ADC trigger 3                     */
    ADC_REG_TRIGGER_EXT_HRTIM_TRG4,    /**< External trigger HRTIM ADC trigger 4 (ADC3 / ADC4 / ADC5) */
    ADC_REG_TRIGGER_EXT_HRTIM_TRG5,    /**< External trigger HRTIM ADC trigger 5                     */
    ADC_REG_TRIGGER_EXT_HRTIM_TRG6,    /**< External trigger HRTIM ADC trigger 6                     */
    ADC_REG_TRIGGER_EXT_HRTIM_TRG7,    /**< External trigger HRTIM ADC trigger 7                     */
    ADC_REG_TRIGGER_EXT_HRTIM_TRG8,    /**< External trigger HRTIM ADC trigger 8                     */
    ADC_REG_TRIGGER_EXT_HRTIM_TRG9,    /**< External trigger HRTIM ADC trigger 9                     */
    ADC_REG_TRIGGER_EXT_HRTIM_TRG10,   /**< External trigger HRTIM ADC trigger 10                    */
#endif /* HRTIM1 */
    ADC_REG_TRIGGER_EXT_EXTI_LINE11,   /**< External trigger EXTI line 11 (ADC1 / ADC2)              */
    ADC_REG_TRIGGER_EXT_EXTI_LINE2,    /**< External trigger EXTI line 2 (ADC3 / ADC4 / ADC5)        */
    ADC_REG_TRIGGER_EXT_LPTIM1_OUT,    /**< External trigger LPTIM1 output                           */
    ADC_REG_TRIGGER_CNT                /**< Count of regular group trigger sources                   */
}   adc_RegTriggerId_t;


/** \brief List of available injected group conversion trigger sources
 *
 * \note Trigger connection differs between ADC1 / ADC2 and ADC3 / ADC4 / ADC5 - a trigger not
 *       connected to the peripheral is refused with \ref ADC_REQUEST_ERROR. */
typedef enum
{
    ADC_INJ_TRIGGER_SOFTWARE = 0u,     /**< Software start (Adc_Set_InjStart())                      */
    ADC_INJ_TRIGGER_AUTO,              /**< Automatic trigger (after regular group)                  */
    ADC_INJ_TRIGGER_EXT_TIM1_TRGO,     /**< External trigger TIM1 TRGO                               */
    ADC_INJ_TRIGGER_EXT_TIM1_TRGO2,    /**< External trigger TIM1 TRGO2                              */
    ADC_INJ_TRIGGER_EXT_TIM1_CH3,      /**< External trigger TIM1 channel 3 (ADC3 / ADC4 / ADC5)     */
    ADC_INJ_TRIGGER_EXT_TIM1_CH4,      /**< External trigger TIM1 channel 4                          */
    ADC_INJ_TRIGGER_EXT_TIM2_TRGO,     /**< External trigger TIM2 TRGO                               */
    ADC_INJ_TRIGGER_EXT_TIM2_CH1,      /**< External trigger TIM2 channel 1 (ADC1 / ADC2)            */
    ADC_INJ_TRIGGER_EXT_TIM3_TRGO,     /**< External trigger TIM3 TRGO                               */
    ADC_INJ_TRIGGER_EXT_TIM3_CH1,      /**< External trigger TIM3 channel 1 (ADC1 / ADC2)            */
    ADC_INJ_TRIGGER_EXT_TIM3_CH3,      /**< External trigger TIM3 channel 3 (ADC1 / ADC2)            */
    ADC_INJ_TRIGGER_EXT_TIM3_CH4,      /**< External trigger TIM3 channel 4 (ADC1 / ADC2)            */
    ADC_INJ_TRIGGER_EXT_TIM4_TRGO,     /**< External trigger TIM4 TRGO                               */
    ADC_INJ_TRIGGER_EXT_TIM4_CH3,      /**< External trigger TIM4 channel 3 (ADC3 / ADC4 / ADC5)     */
    ADC_INJ_TRIGGER_EXT_TIM4_CH4,      /**< External trigger TIM4 channel 4 (ADC3 / ADC4 / ADC5)     */
    ADC_INJ_TRIGGER_EXT_TIM6_TRGO,     /**< External trigger TIM6 TRGO                               */
    ADC_INJ_TRIGGER_EXT_TIM7_TRGO,     /**< External trigger TIM7 TRGO                               */
    ADC_INJ_TRIGGER_EXT_TIM8_TRGO,     /**< External trigger TIM8 TRGO                               */
    ADC_INJ_TRIGGER_EXT_TIM8_TRGO2,    /**< External trigger TIM8 TRGO2                              */
    ADC_INJ_TRIGGER_EXT_TIM8_CH2,      /**< External trigger TIM8 channel 2 (ADC3 / ADC4 / ADC5)     */
    ADC_INJ_TRIGGER_EXT_TIM8_CH4,      /**< External trigger TIM8 channel 4                          */
    ADC_INJ_TRIGGER_EXT_TIM15_TRGO,    /**< External trigger TIM15 TRGO                              */
    ADC_INJ_TRIGGER_EXT_TIM16_CH1,     /**< External trigger TIM16 channel 1 (ADC1 / ADC2)           */
#if defined(TIM20)
    ADC_INJ_TRIGGER_EXT_TIM20_TRGO,    /**< External trigger TIM20 TRGO                              */
    ADC_INJ_TRIGGER_EXT_TIM20_TRGO2,   /**< External trigger TIM20 TRGO2                             */
    ADC_INJ_TRIGGER_EXT_TIM20_CH2,     /**< External trigger TIM20 channel 2 (ADC3 / ADC4 / ADC5)    */
    ADC_INJ_TRIGGER_EXT_TIM20_CH4,     /**< External trigger TIM20 channel 4 (ADC1 / ADC2)           */
#endif /* TIM20 */
#if defined(HRTIM1)
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG1,    /**< External trigger HRTIM ADC trigger 1 (ADC3 / ADC4 / ADC5) */
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG2,    /**< External trigger HRTIM ADC trigger 2                     */
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG3,    /**< External trigger HRTIM ADC trigger 3 (ADC3 / ADC4 / ADC5) */
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG4,    /**< External trigger HRTIM ADC trigger 4                     */
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG5,    /**< External trigger HRTIM ADC trigger 5                     */
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG6,    /**< External trigger HRTIM ADC trigger 6                     */
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG7,    /**< External trigger HRTIM ADC trigger 7                     */
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG8,    /**< External trigger HRTIM ADC trigger 8                     */
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG9,    /**< External trigger HRTIM ADC trigger 9                     */
    ADC_INJ_TRIGGER_EXT_HRTIM_TRG10,   /**< External trigger HRTIM ADC trigger 10                    */
#endif /* HRTIM1 */
    ADC_INJ_TRIGGER_EXT_EXTI_LINE3,    /**< External trigger EXTI line 3 (ADC3 / ADC4 / ADC5)        */
    ADC_INJ_TRIGGER_EXT_EXTI_LINE15,   /**< External trigger EXTI line 15 (ADC1 / ADC2)              */
    ADC_INJ_TRIGGER_EXT_LPTIM1_OUT,    /**< External trigger LPTIM1 output                           */
    ADC_INJ_TRIGGER_CNT                /**< Count of injected group trigger sources                  */
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
 * \note OPAMPx inputs only select the ADC channel connected to the internal output of the
 *       operational amplifier - the amplifier (and its internal output) is configured by the
 *       application. */
typedef enum
{
    ADC_CHANNEL_INPUT_PIN_SINGLE = 0u, /**< External analog single-ended signal will be used as channel input (pin will be initialized)   */
    ADC_CHANNEL_INPUT_PIN_DIFF,        /**< External analog differential signal (channel i - channel i + 1), pins will be initialized   */
    ADC_CHANNEL_INPUT_TEMP,            /**< (ADC1 In 16 / ADC5 In 4) Temperature sensor is connected to the internal channel             */
    ADC_CHANNEL_INPUT_VREF,            /**< (ADC1 / ADC3 / ADC4 / ADC5 In 18) Internal reference voltage is connected                   */
    ADC_CHANNEL_INPUT_VBAT,            /**< (ADC1 / ADC3 / ADC5 In 17) Battery voltage divided by 3 is connected                         */
#if defined (OPAMP1)
    ADC_CHANNEL_INPUT_OPAMP1,          /**< (ADC1 In 13) OPAMP1 internal output is connected                                             */
#endif
#if defined (OPAMP2)
    ADC_CHANNEL_INPUT_OPAMP2,          /**< (ADC2 In 16) OPAMP2 internal output is connected                                             */
#endif
#if defined (OPAMP3)
    ADC_CHANNEL_INPUT_OPAMP3,          /**< (ADC2 In 18 / ADC3 In 13) OPAMP3 internal output is connected                                */
#endif
#if defined (OPAMP4)
    ADC_CHANNEL_INPUT_OPAMP4,          /**< (ADC5 In 5) OPAMP4 internal output is connected                                              */
#endif
#if defined (OPAMP5)
    ADC_CHANNEL_INPUT_OPAMP5,          /**< (ADC5 In 3) OPAMP5 internal output is connected                                              */
#endif
#if defined (OPAMP6)
    ADC_CHANNEL_INPUT_OPAMP6,          /**< (ADC4 In 17) OPAMP6 internal output is connected                                             */
#endif
    ADC_CHANNEL_INPUT_CNT              /**< Count of available input options                                                             */
}   adc_ChannelInput_t;

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
 * \note Only the Analog Watch-dog 1 event can be filtered to raise after n samples.
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
 * - DMA:  DMA channel, request routed by DMAMUX (callbacks from DMA interrupt)
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
    ADC_ERROR_DMA_CONFIG,          /**< DMA configuration error - not reported by STM32G4          */
    ADC_ERROR_DMA_CONFIG_UPDATE,   /**< DMA configuration update error - not reported by STM32G4   */
    ADC_ERROR_DMA_TRIGGER_OVERRUN, /**< DMA trigger overrun - not reported by STM32G4              */
    ADC_ERROR_CNT                  /**< Count of data transfer errors                               */
}   adc_ErrorId_t;


/** DMA peripherals enumeration list (ADC requests are routed to any channel by DMAMUX1) */
typedef enum
{
    ADC_DMA_PERIPH_1 = DMA_PERIPH_1, /**< DMA peripheral 1 identification */
#if defined(DMA2)
    ADC_DMA_PERIPH_2 = DMA_PERIPH_2, /**< DMA peripheral 2 identification */
#endif
    ADC_DMA_PERIPH_CNT               /**< Count of DMA peripherals        */
}   adc_DmaPeriphId_t;


/** Enumeration of available channels of DMA peripheral */
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
 * DmaPeriphId / DmaChannelId / DmaPriority are used only in ADC_TRANSFER_MODE_DMA,
 * IrqPriority is not used in ADC_TRANSFER_MODE_POLL.
 */
typedef struct
{
    adc_TransferMode_t  TransferMode;             /**< Data transfer mode (DMA / ISR / POLL)                         */
    adc_Data_t         *DataBuffer;               /**< Buffer for regular conversion results. Must not be NULL.      */
    adc_BufferSize_t    BufferSize;               /**< Buffer size in count of adc_Data_t items (> 0)                */
    adc_BufferMode_t    BufferMode;               /**< One shot / circular buffer handling                           */

    adc_DmaPeriphId_t   DmaPeriphId;              /**< DMA peripheral (DMA mode only)                                */
    adc_DmaChannelId_t  DmaChannelId;             /**< DMA channel (DMA mode only)                                   */
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
    adc_ChannelId_t       ChannelId;       /**< Channel identification                                  */
    adc_ChannelInput_t    ChannelInput;    /**< Channel input selection                                 */
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

