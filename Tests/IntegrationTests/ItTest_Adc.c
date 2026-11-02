/**
 * \author Mr.Nobody
 * \file ItTest_Adc.c
 * \ingroup Adc
 * \brief Integration tests of Analog-to-digital converter (ADC) module on target (STM32L4 / STM32L4+).
 *
 * Adc module runs on the MCU together with real RCC, NVIC, GPIO and DMA modules
 * and hardware. Tests verify behavior which cannot be verified by unit tests
 * (emulated registers): calibration, conversion of internal channels, resolution,
 * regular sequence, injected group, data transfer in polling, interrupt and DMA
 * mode, analog watchdog, start / stop of continuous conversion and clock
 * configuration.
 *
 * No external wiring and no pin is used (except the divider tests below):
 * - VREFINT (ADC1 channel 0) - analog supply VDDA is calculated with the factory
 *   calibration value VREFINT_CAL (Nucleo boards: VDDA = 3.3 V).
 * - temperature sensor (ADC1 / ADC3 channel 17) - temperature calculated with the factory
 *   calibration values TS_CAL1 / TS_CAL2 is within room range.
 * - VBAT / 3 (ADC1 / ADC3 channel 18) - VBAT is supplied from VDD on the Nucleo boards
 *   (VBAT pin connected to VDD, internally bonded on packages without VBAT pin).
 * - DAC1 outputs (Dac module, output connected to on-chip peripherals only) - ADC2 channel
 *   17 / 18 and ADC3 channel 14 / 15, ADC1 channel 17 / 18 on single ADC MCUs (shared with the
 *   temperature sensor / VBAT). MCUs without DAC (STM32L41x / L42x): test ignored.
 *
 * Divider tests: two resistor dividers from +3V3 (= VDDA of the board) to GND on external channels
 * of ADC1 - PA0 (channel 5): 10 kOhm to +3V3 / 4.7 kOhm to GND, PC0 (channel 1): 4.7 kOhm to
 * +3V3 / 10 kOhm to GND. The ratio of the conversion result to the full scale equals the divider
 * ratio (independent of VDDA). Fixture of the board (EmBi_Platform IntegrationTesting, "Board
 * fixtures"): feature ADC_DIVIDERS - the divider tests are ignored on a board with a fixture
 * without it (a board without a fixture is expected to have the dividers) and on a board without
 * dividers known to the test set. The only STM32L4 board of the inventory is NUCLEO-L4R5ZI.
 *
 * Default clock of StartUp: HCLK = 80 MHz (STM32L4) / 120 MHz (STM32L4+), ADC clock
 * HCLK / 4 = 20 MHz / 30 MHz.
 *
 * Used resources: ADC1, ADC2 / ADC3 (MCUs with the peripheral), DAC1, DMA1 channel 1 (DMA mode).
 *
 * \note Boards are named by the MCU detected by integration testing - the tests of the internal
 *       signals are the same for all 13 STM32L4 / STM32L4+ Nucleo boards.
 * \note STM32L4 errata sheets not reviewed yet (errata workarounds of the STM32G4 module).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Adc_Port.h"                       /* Module under test              */
#include "Dac_Port.h"                       /* DAC outputs measured by ADC    */
/* ============================= TYPEDEFS =================================== */

/** ADC channel measuring a DAC1 output */
typedef struct
{
    adc_PeriphId_t     AdcPeriphId;    /**< ADC peripheral                      */
    adc_Channel_t      AdcChannel;     /**< ADC channel of the DAC output (item of the channel list) */
    dac_ChannelId_t    DacChannelId;   /**< DAC1 channel of the output          */
}   it_AdcDacInput_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void         ItAdc_Get_Config        ( adc_Config_t * const config, adc_TransferMode_t transferMode,
                                              adc_BufferSize_t bufferSize, adc_BufferMode_t bufferMode,
                                              adc_RegTriggerMode_t triggerMode );
static adc_Dma_t    ItAdc_Get_Dma           ( adc_PeriphId_t periphId );
static void         ItAdc_Add_RegChannel    ( adc_Config_t * const config, adc_PeriphId_t periphId, adc_Channel_t channel );
static void         ItAdc_Wait              ( volatile const uint32_t * const counter, uint32_t expectedCnt );
static void         ItAdc_Wait_Sample       ( uint32_t sampleIdx );
static void         ItAdc_Poll              ( uint32_t loopCnt );
static uint32_t     ItAdc_Measure_Vdda_mV   ( void );
static uint32_t     ItAdc_Get_Vdda_mV       ( adc_Data_t vrefRaw12 );
static int32_t      ItAdc_Get_Temp_C        ( adc_Data_t tempRaw12, uint32_t vdda_mV );
static uint32_t     ItAdc_Get_Vbat_mV       ( adc_Data_t vbatRaw12, uint32_t vdda_mV );

static void         ItAdc_HalfCallback      ( void );
static void         ItAdc_CompleteCallback  ( void );
static void         ItAdc_InjCallback       ( void );
static void         ItAdc_ErrorCallback     ( adc_ErrorId_t errorId );
static void         ItAdc_RequireDividers   ( void );
static void         ItAdc_Check_Divider     ( adc_Data_t raw12,
                                              uint32_t topOhm,
                                              uint32_t bottomOhm,
                                              const char * const message );
static uint32_t     ItAdc_Get_Voltage_mV    ( adc_Data_t raw12, uint32_t vdda_mV );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection). The tests of the
 * internal signals are the same for every board, the resistor dividers are wired on NUCLEO-L4R5ZI only
 * (inventory of EmBi_Tester, fixture feature ADC_DIVIDERS). */
#if defined(IT_BOARD_STM32L4R5xI)
    /** NUCLEO-L4R5ZI: dividers on PA0 (ADC1 channel 5) and PC0 (ADC1 channel 1) */
    #define IT_ADC_DIV_AVAILABLE            ( 1u )
    #define IT_ADC_CH_DIV_PA0               ( ADC_CH_ADC1_IN5_PA0 )
    #define IT_ADC_CH_DIV_PC0               ( ADC_CH_ADC1_IN1_PC0 )
#else
    /** No dividers known for the board (placeholder channels - PC0 does not exist on all devices) */
    #define IT_ADC_DIV_AVAILABLE            ( 0u )
    #define IT_ADC_CH_DIV_PA0               ( ADC_CH_ADC1_VREF )
    #define IT_ADC_CH_DIV_PC0               ( ADC_CH_ADC1_VREF )
#endif /* IT_BOARD_STM32L4R5xI */

/** Resistors of the dividers [Ohm]: PA0 10k to +3V3 / 4.7k to GND, PC0 4.7k to +3V3 / 10k to GND */
#define IT_ADC_DIV_R_10K                    ( 10000u )
#define IT_ADC_DIV_R_4K7                    ( 4700u )

/** Tolerance of the divider ratio [% of the expected result] (resistors 1 %, ADC offset / gain) */
#define IT_ADC_DIV_TOL_PCT                  ( 3u )

/** Tolerance of the divider voltage calculated with the VDDA of the VREFINT conversion [% of the expected voltage] */
#define IT_ADC_DIV_VOLT_TOL_PCT             ( 8u )

/** Samples of the divider sequences: PA0, PC0 (+ VREFINT) */
#define IT_ADC_DIV_SAMPLES                  ( 2u )
#define IT_ADC_DIV_VREF_SAMPLES             ( 3u )

/** Count of percent in one unit */
#define IT_ADC_PERCENT                      ( 100u )

/** ADC peripheral under test */
#define IT_ADC_PERIPH                       ( ADC_PERIPH_1 )

/** Internal channels of ADC1 (temperature sensor / VBAT also of ADC3) */
#define IT_ADC_CH_VREF                      ( ADC_CHANNEL_0 )
#define IT_ADC_CH_TEMP                      ( ADC_CHANNEL_17 )
#define IT_ADC_CH_VBAT                      ( ADC_CHANNEL_18 )

/** Channel built from the parts - the combinations that are not in the list adc_Channel_t are built with it */
#define IT_ADC_CH( PERIPH_ID, CHANNEL_ID, INPUT_ID )   ( (adc_Channel_t)ADC_CHANNEL_ENCODE( (PERIPH_ID), (CHANNEL_ID), (INPUT_ID) ) )

/** Sampling time satisfying minimum of internal channels (VREF 4 us, TEMP 5 us, VBAT 12 us,
 *  640.5 cycles = 21 us at 30 MHz) */
#define IT_ADC_SAMPLING                     ( ADC_CHANNEL_SAMPLING_640_5_CYCLES )

/** ADC clock: HCLK / 4 (20 MHz / 30 MHz, within 0.14 - 80 MHz) */
#define IT_ADC_CLK_SRC                      ( ADC_CLK_SRC_HCLK )
#define IT_ADC_CLK_DIV                      ( ADC_CLK_DIV_4 )

/** Expected analog supply of the Nucleo boards [mV] and tolerance */
#define IT_ADC_VDDA_MV                      ( 3300u )
#define IT_ADC_VDDA_TOL_MV                  ( 150u )

/** Expected VBAT (supplied from VDD on the Nucleo boards) [mV] and tolerance (VBAT / 3 bridge) */
#define IT_ADC_VBAT_MV                      ( 3300u )
#define IT_ADC_VBAT_TOL_MV                  ( 400u )

/** Ratio of the VBAT bridge (VBAT / 3 is converted) */
#define IT_ADC_VBAT_RATIO                   ( 3u )

/** Full scale of 12-bit conversion */
#define IT_ADC_FULL_SCALE_12B               ( 4095u )

/** Plausible temperature of the device during the test [degree C] */
#define IT_ADC_TEMP_MIN_C                   ( 5 )
#define IT_ADC_TEMP_MAX_C                   ( 70 )

/** Maximal difference of repeated conversions of the same signal [LSB, 12-bit] */
#define IT_ADC_NOISE_LSB                    ( 40u )

/** DAC1 output values measured by the ADC (different per channel) and maximal difference [LSB, 12-bit] */
#define IT_ADC_DAC1_VALUE                   ( 1024u )
#define IT_ADC_DAC2_VALUE                   ( 3072u )
#define IT_ADC_DAC_TOL_LSB                  ( 60u )

/** DAC1 outputs not connected to the ADC - STM32L4R / L4S (Bug AB#1282) */
#if defined(STM32L4R5xx) || \
    defined(STM32L4R7xx) || \
    defined(STM32L4R9xx) || \
    defined(STM32L4S5xx) || \
    defined(STM32L4S7xx) || \
    defined(STM32L4S9xx)
#define IT_ADC_DAC_NOT_CONNECTED
#endif /* STM32L4R / L4S */

/** Data buffer size */
#define IT_ADC_BUF_SIZE                     ( 8u )

/** Maximal count of wait loop iterations (Adc_Task called) */
#define IT_ADC_WAIT_LOOPS                   ( 1000000u )

/** Count of Adc_Task calls after a discontinuous subgroup - takes far longer than several conversions */
#define IT_ADC_IDLE_LOOPS                   ( 20000u )

/** Value of a data buffer item which was not written by the module */
#define IT_ADC_BUF_EMPTY                    ( 0xFFFFu )

/** Minimal difference of the VREFINT and the temperature sensor result for the analog watch-dog
 *  window between them [LSB, 12-bit] - AWD2 / AWD3 compare the 8 MSB (16 LSB steps) */
#define IT_ADC_AWD_MARGIN_LSB               ( 64u )

/** Interrupt priority of the ADC / DMA interrupts */
#define IT_ADC_IRQ_PRIO                     ( 5u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Conversion results */
static adc_Data_t               itAdc_Buffer[ IT_ADC_BUF_SIZE ];

/** Counts of callback calls */
static volatile uint32_t        itAdc_HalfCnt;
static volatile uint32_t        itAdc_CompleteCnt;
static volatile uint32_t        itAdc_InjCnt;
static volatile uint32_t        itAdc_ErrorCnt;

#if defined(DAC1) && \
    !defined(IT_ADC_DAC_NOT_CONNECTED)
/** ADC channels connected to the DAC1 outputs (Adc_Types.h, adc_Channel_t) */
static const it_AdcDacInput_t   itAdc_DacInputLut[ ] =
{
#if defined(ADC2)
    { ADC_PERIPH_2, ADC_CH_ADC2_DAC1, DAC_CHANNEL_1 },
    { ADC_PERIPH_2, ADC_CH_ADC2_DAC2, DAC_CHANNEL_2 },
#if defined(ADC3)
    { ADC_PERIPH_3, ADC_CH_ADC3_DAC1, DAC_CHANNEL_1 },
    { ADC_PERIPH_3, ADC_CH_ADC3_DAC2, DAC_CHANNEL_2 },
#endif /* ADC3 */
#else
    { ADC_PERIPH_1, ADC_CH_ADC1_DAC1, DAC_CHANNEL_1 },
#if defined(DAC_CHANNEL2_SUPPORT)
    { ADC_PERIPH_1, ADC_CH_ADC1_DAC2, DAC_CHANNEL_2 },
#endif /* DAC_CHANNEL2_SUPPORT */
#endif /* ADC2 */
};

/** Expected DAC1 output value, indexed by dac_ChannelId_t */
static const dac_Data_t         itAdc_DacValueLut[ DAC_CHANNEL_CNT ] =
{
    [DAC_CHANNEL_1] = IT_ADC_DAC1_VALUE,
#if defined(DAC_CHANNEL2_SUPPORT)
    [DAC_CHANNEL_2] = IT_ADC_DAC2_VALUE,
#endif /* DAC_CHANNEL2_SUPPORT */
};
#endif /* DAC1 / IT_ADC_DAC_NOT_CONNECTED */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itAdc_HalfCnt     = 0u;
    itAdc_CompleteCnt = 0u;
    itAdc_InjCnt      = 0u;
    itAdc_ErrorCnt    = 0u;

    for( uint32_t dataIdx = 0u; IT_ADC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        itAdc_Buffer[ dataIdx ] = IT_ADC_BUF_EMPTY;
    }
}


void tearDown( void )
{
    /* Every test case runs after system reset */
}

/* =============================== TESTS ==================================== */

/*------------------------------ Configuration -------------------------------*/

/**
 * \brief   Adc_Init() on target configures ADC clock.
 *
 * \details Initializes the module with clock HCLK / 4 and one regular VREF channel
 *          of ADC1 (polling mode), then reads the clock configuration back.
 *
 * \par Expected results
 * - Adc_Init() returns ADC_REQUEST_OK.
 * - Clock source reads back HCLK, clock divider reads back 4.
 */
void It_Adc_Init_ClockConfiguration_ReadBack( void )
{
    adc_Config_t config;
    adc_ClkSrc_t clkSrc = ADC_CLK_SRC_CNT;
    adc_ClkDiv_t clkDiv = ADC_CLK_DIV_CNT;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( IT_ADC_CLK_SRC, clkSrc );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockDivider( &clkDiv ) );
    TEST_ASSERT_EQUAL( IT_ADC_CLK_DIV, clkDiv );
}


/**
 * \brief   Adc_Init() on target rejects invalid configurations.
 *
 * \details Calls Adc_Init() with:
 * - NULL configuration,
 * - VREF channel with 47.5 cycles sampling time (2.4 us / 1.6 us, below VREFINT minimum 4 us),
 * - clock source PLLSAI1 R (output not enabled by Rcc_Init; the source is not in the list of
 *   STM32L41x / L42x - the case is not built there),
 * - VREF input on channel 3 (channel without internal connection),
 * - invalid regular trigger ADC_REG_TRIGGER_CNT.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 * - ADC1 stays usable - valid configuration is initialized afterwards.
 */
void It_Adc_Init_InvalidConfig_ReturnsError( void )
{
    adc_Config_t config;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( ADC_NULL_PTR ) );

    /* Sampling time below minimum of VREFINT */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    config.PeriphConfig[ IT_ADC_PERIPH ].RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_47_5_CYCLES;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

#if defined(RCC_CR_PLLSAI1ON)
    /* Clock source without clock */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    config.ClockSource = ADC_CLK_SRC_PLLSAI1R;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
#endif /* RCC_CR_PLLSAI1ON */

    /* Internal signal on channel without the connection */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, IT_ADC_CH( IT_ADC_PERIPH, ADC_CHANNEL_3, ADC_CHANNEL_INPUT_VREF ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* Invalid trigger */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    config.PeriphConfig[ IT_ADC_PERIPH ].RegTriggerId = ADC_REG_TRIGGER_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* Valid configuration */
    config.PeriphConfig[ IT_ADC_PERIPH ].RegTriggerId = ADC_REG_TRIGGER_SOFTWARE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );
}


/**
 * \brief   Adc_Deinit() on target allows new initialization of the peripheral.
 *
 * \details
 * 1. Initializes ADC1 (VREF, polling mode).
 * 2. Initializes it again while enabled.
 * 3. Deinitializes ADC1 and initializes it again.
 * 4. Starts regular conversion and waits for transfer complete (Adc_Task polled).
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK.
 * 2. ADC_REQUEST_ERROR - clock configuration is refused for enabled ADC.
 * 3. ADC_REQUEST_OK for both calls.
 * 4. Transfer complete callback is called once.
 */
void It_Adc_Deinit_InitializedPeripheral_InitializationPossibleAgain( void )
{
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* Peripheral is enabled - clock configuration is refused */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( IT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
}


/**
 * \brief   ADC clock source can be changed after reinitialization (kernel clock multiplexer
 *          released by the module).
 *
 * \details
 * 1. Initializes ADC1 with SYSCLK / 4 (asynchronous clock), converts VREFINT.
 * 2. Deinitializes ADC1, initializes it with HCLK / 4 (synchronous clock), converts VREFINT.
 *
 * \note    Regression of the STM32H5 module bug AB#1043 (previous source never released, RCC
 *          changes the multiplexer of a released clock only).
 *
 * \par Expected results
 * - Both initializations ADC_REQUEST_OK, clock source reads back SYSCLK / HCLK.
 * - Both conversions give VDDA = 3300 mV +- 150 mV.
 */
void It_Adc_Set_ClockSource_SysclkThenHclk_ConversionsValid( void )
{
    adc_Config_t config;
    adc_ClkSrc_t clkSrc = ADC_CLK_SRC_CNT;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    config.ClockSource = ADC_CLK_SRC_SYSCLK;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( ADC_CLK_SRC_SYSCLK, clkSrc );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( IT_ADC_PERIPH ) );

    config.ClockSource  = ADC_CLK_SRC_HCLK;
    itAdc_Buffer[ 0u ]  = 0xFFFFu;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( ADC_CLK_SRC_HCLK, clkSrc );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 2u );
    TEST_ASSERT_EQUAL_UINT32( 2u, itAdc_CompleteCnt );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] ) );
}

/*-------------------------- Conversion of signals ---------------------------*/

/**
 * \brief   Polling conversion of VREFINT corresponds to board analog supply.
 *
 * \details Initializes ADC1 with VREF channel in polling mode (12-bit, one-shot
 *          buffer of 1 sample), starts conversion and polls Adc_Task() until
 *          transfer complete. VDDA is calculated with the factory VREFINT_CAL.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - VDDA = 3300 mV +- 150 mV.
 */
void It_Adc_Set_RegStart_VrefPoll_VddaMatchesBoardSupply( void )
{
    /* Initialization, start and wait for transfer complete are asserted by the helper */
    const uint32_t vdda_mV = ItAdc_Measure_Vdda_mV( );

    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, vdda_mV );
}


/**
 * \brief   Manual polling: software conversion started repeatedly, result read by Adc_Get_RegData().
 *
 * \details Polling mode without buffer, 3x: start, wait for EOC flag, read data.
 *
 * \par Expected results
 * - EOC flag active after every start, data give VDDA = 3300 mV +- 150 mV.
 * - EOC flag cleared by reading of the data.
 */
void It_Adc_Get_RegData_ManualPolling_RepeatedSoftwareStart( void )
{
    adc_Config_t    config;
    adc_FlagState_t eocFlag = ADC_FLAG_INACTIVE;
    adc_Data_t      data    = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 0u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    config.PeriphConfig[ IT_ADC_PERIPH ].DataConfig.DataBuffer = ADC_NULL_PTR;
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    for( uint32_t convIdx = 0u; 3u > convIdx; convIdx++ )
    {
        eocFlag = ADC_FLAG_INACTIVE;

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );

        for( uint32_t loopIdx = 0u; IT_ADC_WAIT_LOOPS > loopIdx; loopIdx++ )
        {
            TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOC, &eocFlag ) );

            if( ADC_FLAG_ACTIVE == eocFlag )
            {
                break;
            }
            else
            {
                /* Conversion is running */
            }
        }

        TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, eocFlag );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_RegData( IT_ADC_PERIPH, &data ) );
        TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( data ) );

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOC, &eocFlag ) );
        TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, eocFlag );
    }
}


/**
 * \brief   8-bit resolution on target scales the conversion result.
 *
 * \details Initializes ADC1 with VREF channel and 8-bit resolution, reads the
 *          resolution back and converts VREFINT in polling mode.
 *
 * \par Expected results
 * - Resolution reads back 8-bit.
 * - Result is below 256, VDDA calculated from result << 4 is 3300 mV +- 300 mV.
 */
void It_Adc_Set_Resolution_8Bit_ResultScaled( void )
{
    adc_Config_t     config;
    adc_Resolution_t resolution = ADC_RESOLUTION_12BIT;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    config.PeriphConfig[ IT_ADC_PERIPH ].Resolution = ADC_RESOLUTION_8BIT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Resolution( IT_ADC_PERIPH, &resolution ) );
    TEST_ASSERT_EQUAL( ADC_RESOLUTION_8BIT, resolution );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_LESS_THAN_UINT16( 256u, itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT32_WITHIN( 2u * IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( (adc_Data_t)( itAdc_Buffer[ 0u ] << 4u ) ) );
}


/**
 * \brief   Regular sequence VREF + temperature sensor + VBAT in ISR mode.
 *
 * \details Initializes ADC1 in ISR mode with buffer of 3 samples and regular
 *          sequence VREFINT, temperature sensor, VBAT / 3, starts conversion and waits for
 *          transfer complete.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - 1st sample: VDDA = 3300 mV +- 150 mV.
 * - 2nd sample: temperature (TS_CAL1 / TS_CAL2) within 5 - 70 degree C.
 * - 3rd sample: VBAT = 3300 mV +- 400 mV (VBAT supplied from VDD).
 */
void It_Adc_Set_RegStart_VrefTempVbatSequenceIsr_ValuesPlausible( void )
{
    adc_Config_t config;
    uint32_t     vdda_mV = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 3u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_TEMP );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VBAT );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    vdda_mV = ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, vdda_mV );
    TEST_ASSERT_INT32_WITHIN( ( IT_ADC_TEMP_MAX_C - IT_ADC_TEMP_MIN_C ) / 2, ( IT_ADC_TEMP_MAX_C + IT_ADC_TEMP_MIN_C ) / 2,
                              ItAdc_Get_Temp_C( itAdc_Buffer[ 1u ], vdda_mV ) );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VBAT_TOL_MV, IT_ADC_VBAT_MV, ItAdc_Get_Vbat_mV( itAdc_Buffer[ 2u ], vdda_mV ) );
}


/**
 * \brief   Continuous conversion with circular DMA buffer.
 *
 * \details Initializes ADC1 in DMA mode (DMA1 channel 1 - request mapping of ADC1 on STM32L4,
 *          DMAMUX1 on STM32L4+) with circular buffer of 8 samples and continuous VREF conversion,
 *          starts it, waits for 3 buffer completions and stops the conversion.
 *
 * \par Expected results
 * - Complete and half transfer callbacks called at least 3x, no error callback.
 * - All samples differ from the 1st one by at most 40 LSB (noise).
 * - VDDA = 3300 mV +- 150 mV.
 * - After stop no further transfer complete is reported.
 */
void It_Adc_Set_RegStart_ContinuousDmaCircular_HalfAndCompleteCallbacks( void )
{
    adc_Config_t config;
    uint32_t     completeCnt = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_DMA, IT_ADC_BUF_SIZE, ADC_BUFFER_MODE_CIRCULAR, ADC_REG_TRIGGER_MODE_CONTINUOUS );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 3u );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( IT_ADC_PERIPH ) );

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 3u, itAdc_CompleteCnt );
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 3u, itAdc_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    for( uint32_t dataIdx = 1u; IT_ADC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        TEST_ASSERT_UINT16_WITHIN( IT_ADC_NOISE_LSB, itAdc_Buffer[ 0u ], itAdc_Buffer[ dataIdx ] );
    }

    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] ) );

    /* Stopped - no more transfers (wait of a few buffer periods) */
    completeCnt = itAdc_CompleteCnt;
    ItAdc_Wait( &itAdc_CompleteCnt, completeCnt + 1u );
    TEST_ASSERT_EQUAL_UINT32( completeCnt, itAdc_CompleteCnt );
}


/**
 * \brief   One shot DMA buffer can be filled again after restart, the first sample after the
 *          restart is valid.
 *
 * \details DMA mode, one shot buffer of 8 samples, continuous VREF conversion.
 *          Starts the conversion, waits for transfer complete (conversion stopped by
 *          the module), clears the buffer, restarts it and waits again.
 *
 * \note    STM32G4 device errata ES0430 2.7.10 / ES0431 2.5.10 / ES0523 2.6.8 - the first
 *          conversion after the end of a one shot DMA transfer / a software stop may convert
 *          channel 0 (result 0); the module executes a hardware dummy conversion at the next
 *          start (workaround kept, STM32L4 errata sheets not reviewed yet). On STM32L4 channel 0
 *          is VREFINT - a wrong first sample would still be plausible, the test verifies the
 *          restart of the transfer.
 *
 * \par Expected results
 * - Transfer complete callback 1x after each start, no error callback (no overrun after
 *   the automatic stop).
 * - All samples of the second buffer (incl. the first one) differ from the last sample by at
 *   most 40 LSB, VDDA = 3300 mV +- 150 mV.
 */
void It_Adc_Set_RegStart_DmaOneShotRestart_BufferFilledTwice( void )
{
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_DMA, IT_ADC_BUF_SIZE, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_CONTINUOUS );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );

    for( uint32_t dataIdx = 0u; IT_ADC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        itAdc_Buffer[ dataIdx ] = 0xFFFFu;
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 2u );
    TEST_ASSERT_EQUAL_UINT32( 2u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    for( uint32_t dataIdx = 0u; IT_ADC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        TEST_ASSERT_UINT16_WITHIN( IT_ADC_NOISE_LSB, itAdc_Buffer[ IT_ADC_BUF_SIZE - 1u ], itAdc_Buffer[ dataIdx ] );
    }

    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( itAdc_Buffer[ IT_ADC_BUF_SIZE - 1u ] ) );
}


/**
 * \brief   Continuous conversion stopped by software and started again delivers valid first
 *          sample.
 *
 * \details ISR mode, circular buffer of 8 samples, continuous temperature sensor conversion
 *          (channel 17, channel 0 would be VREFINT): start, wait for one buffer, software stop
 *          (ADSTP), buffer cleared, start, wait for half buffer.
 *
 * \note    STM32G4 device errata ES0430 2.7.10 / ES0431 2.5.10 / ES0523 2.6.8 - the first
 *          conversion after a software stop may convert channel 0; the module executes a
 *          hardware dummy conversion at the next start (workaround kept, STM32L4 errata sheets
 *          not reviewed yet).
 *
 * \par Expected results
 * - First sample after the restart differs from the last sample before the stop by at most
 *   40 LSB (temperature sensor, not channel 0), no error.
 */
void It_Adc_Set_RegStart_AfterSoftwareStop_FirstSampleValid( void )
{
    adc_Config_t config;
    adc_Data_t   lastSample = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, IT_ADC_BUF_SIZE, ADC_BUFFER_MODE_CIRCULAR, ADC_REG_TRIGGER_MODE_CONTINUOUS );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_TEMP );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( IT_ADC_PERIPH ) );
    lastSample = itAdc_Buffer[ IT_ADC_BUF_SIZE - 1u ];

    for( uint32_t dataIdx = 0u; IT_ADC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        itAdc_Buffer[ dataIdx ] = 0xFFFFu;
    }

    itAdc_HalfCnt = 0u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_HalfCnt, 1u );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( IT_ADC_PERIPH ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_NOISE_LSB, lastSample, itAdc_Buffer[ 0u ] );
}


/**
 * \brief   Injected conversion of VREFINT and temperature sensor on target.
 *
 * \details Initializes ADC1 with injected sequence VREF, temperature sensor (ISR mode),
 *          starts the injected group by software and waits for injected complete callback.
 *
 * \par Expected results
 * - Injected complete callback 1x.
 * - Rank 1 gives VDDA = 3300 mV +- 150 mV, rank 2 temperature within 5 - 70 degree C.
 */
void It_Adc_Set_InjStart_VrefTempInjected_InjCallbackAndData( void )
{
    adc_Config_t config;
    adc_Data_t   injData = 0u;
    uint32_t     vdda_mV = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannelsCnt                      = 2u;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 0u ].Channel           = ADC_CH_ADC1_VREF;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 0u ].ChannelSampling   = IT_ADC_SAMPLING;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 1u ].Channel           = ADC_CH_ADC1_TEMP;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 1u ].ChannelSampling   = IT_ADC_SAMPLING;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_InjCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_InjCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_InjData( IT_ADC_PERIPH, ADC_INJ_SEQUENCE_1, &injData ) );
    vdda_mV = ItAdc_Get_Vdda_mV( injData );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, vdda_mV );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_InjData( IT_ADC_PERIPH, ADC_INJ_SEQUENCE_2, &injData ) );
    TEST_ASSERT_INT32_WITHIN( ( IT_ADC_TEMP_MAX_C - IT_ADC_TEMP_MIN_C ) / 2, ( IT_ADC_TEMP_MAX_C + IT_ADC_TEMP_MIN_C ) / 2,
                              ItAdc_Get_Temp_C( injData, vdda_mV ) );
}

/*--------------------------- Other ADC peripherals --------------------------*/

/**
 * \brief   ADC3 (own interrupt line) converts temperature sensor and VBAT.
 *
 * \details VDDA measured by ADC1 (VREFINT, polling), ADC1 deinitialized. ADC3 in ISR mode,
 *          regular sequence temperature sensor (channel 17), VBAT / 3 (channel 18), buffer of
 *          2 samples. MCUs without ADC3: test ignored.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - Temperature within 5 - 70 degree C, VBAT = 3300 mV +- 400 mV.
 */
void It_Adc_Adc3_TempVbatSequenceIsr_ValuesPlausible( void )
{
#if defined(ADC3)
    adc_Config_t config;
    uint32_t     vdda_mV = ItAdc_Measure_Vdda_mV( );

    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, vdda_mV );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( IT_ADC_PERIPH ) );

    itAdc_CompleteCnt = 0u;
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, ADC_PERIPH_3, ADC_CH_ADC3_TEMP );
    ItAdc_Add_RegChannel( &config, ADC_PERIPH_3, ADC_CH_ADC3_VBAT );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( ADC_PERIPH_3 ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    TEST_ASSERT_INT32_WITHIN( ( IT_ADC_TEMP_MAX_C - IT_ADC_TEMP_MIN_C ) / 2, ( IT_ADC_TEMP_MAX_C + IT_ADC_TEMP_MIN_C ) / 2,
                              ItAdc_Get_Temp_C( itAdc_Buffer[ 0u ], vdda_mV ) );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VBAT_TOL_MV, IT_ADC_VBAT_MV, ItAdc_Get_Vbat_mV( itAdc_Buffer[ 1u ], vdda_mV ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without ADC3" );
#endif /* ADC3 */
}


/**
 * \brief   Internal signals of ADC1 are refused on ADC2.
 *
 * \details MCUs with ADC2: ADC2 initialized with VREF input on channel 0 and with the
 *          temperature sensor input on channel 17 (both connected to ADC1 / ADC3 only).
 *          MCUs with one ADC: test ignored.
 *
 * \note    VREFINT connection to ADC1 only is taken from the STM32CubeMX database and the LL
 *          channel documentation (LL helper __LL_ADC_IS_CHANNEL_INTERNAL_AVAILABLE lists it for
 *          ADC2 / ADC3 as well) - the module refuses it on ADC2 / ADC3.
 *
 * \par Expected results
 * - ADC2 VREF / temperature sensor inputs: ADC_REQUEST_ERROR.
 */
void It_Adc_Adc2_InternalSignalsOfAdc1_Refused( void )
{
#if defined(ADC2)
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, ADC_PERIPH_2, IT_ADC_CH( ADC_PERIPH_2, ADC_CHANNEL_0, ADC_CHANNEL_INPUT_VREF ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, ADC_PERIPH_2, IT_ADC_CH( ADC_PERIPH_2, IT_ADC_CH_TEMP, ADC_CHANNEL_INPUT_TEMP ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( "MCU has only one ADC" );
#endif /* ADC2 */
}


/**
 * \brief   DAC1 outputs (internal connection) are measured by the ADC channels connected to them.
 *
 * \details DAC1 channel 1 / 2 (channel 2 where available) initialized by Dac module with output
 *          connected to on-chip peripherals only (no pin), values 1024 / 3072. Every ADC channel
 *          of the DAC outputs (ADC2 channel 17 / 18, ADC3 channel 14 / 15; ADC1 channel 17 / 18
 *          on single ADC MCUs) is converted in polling mode, ADC deinitialized after each
 *          conversion. Both converters use VREF+ - values are compared directly.
 *          MCUs without DAC: test ignored.
 * \note    Bug AB#1282: the DAC1 outputs of STM32L4R / L4S are not connected to the ADC (ADC1
 *          channel 17 / 18 do not follow the DAC) - Adc_Init() with the DAC inputs is refused and
 *          the test is ignored.
 *
 * \par Expected results
 * - Dac_Init() / Adc_Init() return OK.
 * - Every ADC result differs from the value of its DAC channel by at most 60 LSB (different
 *   values detect swapped outputs).
 * - STM32L4R / L4S: Adc_Init() with DAC1 output 1 / 2 on ADC1 channel 17 / 18 returns
 *   ADC_REQUEST_ERROR.
 */
void It_Adc_DacOutputs_InternalChannels_MeasuredValuesMatch( void )
{
#if defined(IT_ADC_DAC_NOT_CONNECTED)
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, ADC_PERIPH_1, IT_ADC_CH( ADC_PERIPH_1, ADC_CHANNEL_17, ADC_CHANNEL_INPUT_DAC1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, ADC_PERIPH_1, IT_ADC_CH( ADC_PERIPH_1, ADC_CHANNEL_18, ADC_CHANNEL_INPUT_DAC2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    TEST_IGNORE_MESSAGE( "DAC1 outputs not connected to the ADC (STM32L4R / L4S) - DAC inputs refused" );
#elif defined(DAC1)
    adc_Config_t config;
    dac_Config_t dacConfig;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DefaultConfig( &dacConfig ) );
    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_1 ].OutputMode = DAC_OUTPUT_INTERNAL;
    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_1 ].InitValue  = itAdc_DacValueLut[ DAC_CHANNEL_1 ];
#if defined(DAC_CHANNEL2_SUPPORT)
    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_2 ].OutputMode  = DAC_OUTPUT_INTERNAL;
    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_2 ].InitValue   = itAdc_DacValueLut[ DAC_CHANNEL_2 ];
#endif /* DAC_CHANNEL2_SUPPORT */
    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &dacConfig ) );

    for( uint32_t inputIdx = 0u; ( sizeof( itAdc_DacInputLut ) / sizeof( itAdc_DacInputLut[ 0u ] ) ) > inputIdx; inputIdx++ )
    {
        const it_AdcDacInput_t * const dacInput = &itAdc_DacInputLut[ inputIdx ];

        itAdc_CompleteCnt  = 0u;
        itAdc_Buffer[ 0u ] = 0xFFFFu;

        ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
        ItAdc_Add_RegChannel( &config, dacInput->AdcPeriphId, dacInput->AdcChannel );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( dacInput->AdcPeriphId ) );
        ItAdc_Wait( &itAdc_CompleteCnt, 1u );

        TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
        TEST_ASSERT_UINT16_WITHIN( IT_ADC_DAC_TOL_LSB, itAdc_DacValueLut[ dacInput->DacChannelId ], itAdc_Buffer[ 0u ] );

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( dacInput->AdcPeriphId ) );
    }
#else
    TEST_IGNORE_MESSAGE( "MCU without DAC" );
#endif /* IT_ADC_DAC_NOT_CONNECTED / DAC1 */
}

/*----------------------------- Analog watchdog ------------------------------*/

/**
 * \brief   Analog watchdog detects VREFINT above the window.
 *
 * \details Initializes ADC1 with VREF channel and AWD1 on all regular channels
 *          with window 0 - 100 LSB (VREFINT ~1500 LSB), clears AWD1 flag and
 *          converts VREFINT.
 *
 * \par Expected results
 * - Thresholds read back 0 / 100.
 * - AWD1 flag is ACTIVE after conversion.
 */
void It_Adc_AwdInit_VrefAboveHighThreshold_AwdFlagActive( void )
{
    adc_Config_t       config;
    adc_AwdConfig_t    awdConfig;
    adc_FlagState_t    awdFlag  = ADC_FLAG_INACTIVE;
    adc_AwdThreshold_t lowThr   = 0xFFFFu;
    adc_AwdThreshold_t highThr  = 0xFFFFu;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    awdConfig.AwdId            = ADC_AWD_1;
    awdConfig.AwdMode          = ADC_AWD_MODE_ALL_REGULAR;
    awdConfig.AwdLowThreshold  = 0u;
    awdConfig.AwdHighThreshold = 100u;
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( IT_ADC_PERIPH, ADC_AWD_1, &lowThr, &highThr ) );
    TEST_ASSERT_EQUAL_UINT16( 0u, lowThr );
    TEST_ASSERT_EQUAL_UINT16( 100u, highThr );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, awdFlag );
}


/**
 * \brief   Analog watchdog does not trigger for VREFINT inside the window, event filter is not
 *          available.
 *
 * \details Initializes ADC1 with VREF channel and AWD1 on all regular channels
 *          with window 100 - 4000 LSB, clears AWD1 flag and converts VREFINT. Requests AWD1
 *          event filter of 2 samples.
 *
 * \par Expected results
 * - AWD1 flag is INACTIVE after conversion.
 * - Filter request: ADC_REQUEST_ERROR (STM32L4 ADC has no event filter), filter reads back
 *   "no filter".
 */
void It_Adc_AwdInit_VrefInsideWindow_AwdFlagInactive( void )
{
    adc_Config_t    config;
    adc_AwdConfig_t awdConfig;
    adc_FlagState_t awdFlag   = ADC_FLAG_ACTIVE;
    adc_AwdFilter_t awdFilter = ADC_AWD_FILTER_CNT;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    awdConfig.AwdId            = ADC_AWD_1;
    awdConfig.AwdMode          = ADC_AWD_MODE_ALL_REGULAR;
    awdConfig.AwdLowThreshold  = 100u;
    awdConfig.AwdHighThreshold = 4000u;
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, awdFlag );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_AwdFilter( IT_ADC_PERIPH, ADC_AWD_1, ADC_AWD_FILTER_2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdFilter( IT_ADC_PERIPH, ADC_AWD_1, &awdFilter ) );
    TEST_ASSERT_EQUAL( ADC_AWD_FILTER_NONE, awdFilter );
}

/**
 * \brief   Regular discontinuous mode converts 2 ranks per software start.
 *
 * \details Initializes ADC1 in polling mode with one-shot buffer of 3 samples and regular
 *          sequence VREFINT, temperature sensor, VREFINT converted 2 ranks per trigger
 *          (ADC_REG_TRIGGER_MODE_DISCONT_2RANKS, software trigger), clears the end of
 *          sequence flag and:
 *          1. starts the regular group, waits for the 2nd sample and calls Adc_Task() for
 *             IT_ADC_IDLE_LOOPS more iterations (far longer than one conversion),
 *          2. starts the regular group again and waits for transfer complete.
 * \note    Task AB#1820: the regular discontinuous mode was not available. Without DISCEN the
 *          1st start converts the whole sequence (3rd sample written, EOS set).
 *
 * \par Expected results
 * - Conversion mode reads back ADC_REG_TRIGGER_MODE_DISCONT_2RANKS.
 * 1. 2 samples are stored, the 3rd buffer item is not written, no transfer complete, end of
 *    sequence flag INACTIVE.
 * 2. The start is accepted (ADSTART was cleared by HW after the 1st subgroup), the last
 *    subgroup has 1 rank: transfer complete 1x, end of sequence flag ACTIVE, no error.
 * - VREFINT samples: VDDA = 3300 mV +- 150 mV, temperature sensor sample within 5 - 70 degree C (factory calibration).
 */
void It_Adc_Set_RegStart_DiscontinuousTwoRanks_EachStartConvertsSubgroup( void )
{
    adc_Config_t         config;
    adc_RegTriggerMode_t triggerMode = ADC_REG_TRIGGER_MODE_CNT;
    adc_FlagState_t      eosFlag     = ADC_FLAG_ACTIVE;
    uint32_t             vdda_mV     = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 3u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_DISCONT_2RANKS );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_TEMP );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerMode( IT_ADC_PERIPH, &triggerMode ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_MODE_DISCONT_2RANKS, triggerMode );

    /* 1st subgroup: ranks 1 and 2 */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOS ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait_Sample( 1u );
    ItAdc_Poll( IT_ADC_IDLE_LOOPS );

    TEST_ASSERT_NOT_EQUAL( IT_ADC_BUF_EMPTY, itAdc_Buffer[ 0u ] );
    TEST_ASSERT_NOT_EQUAL( IT_ADC_BUF_EMPTY, itAdc_Buffer[ 1u ] );
    TEST_ASSERT_EQUAL_HEX16( IT_ADC_BUF_EMPTY, itAdc_Buffer[ 2u ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOS, &eosFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, eosFlag );

    /* 2nd subgroup: rank 3 (the last subgroup is shorter, no rollover) */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOS, &eosFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, eosFlag );

    vdda_mV = ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, vdda_mV );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( itAdc_Buffer[ 2u ] ) );
    TEST_ASSERT_INT32_WITHIN( ( IT_ADC_TEMP_MAX_C - IT_ADC_TEMP_MIN_C ) / 2, ( IT_ADC_TEMP_MAX_C + IT_ADC_TEMP_MIN_C ) / 2,
                              ItAdc_Get_Temp_C( itAdc_Buffer[ 1u ], vdda_mV ) );
}

/**
 * \brief   Analog watchdog 1 monitors only the selected channel.
 *
 * \details Initializes ADC1 with regular sequence VREFINT (channel 0), temperature sensor
 *          (channel 17) in polling mode (buffer of 2 samples), converts the sequence once and
 *          sets the upper threshold to the middle between both results (lower threshold 0), then:
 *          1. AWD1 on the single regular temperature sensor channel, converts the sequence,
 *          2. AWD1 on the single regular VREFINT channel, converts the sequence again.
 *          The window contains the temperature sensor voltage, VREFINT is above it.
 * \note    Task AB#1821: the single channel modes were rejected (no channel in adc_AwdConfig_t).
 *
 * \par Expected results
 * - VREFINT result is above the temperature sensor result by more than IT_ADC_AWD_MARGIN_LSB.
 * 1. ADC_REQUEST_OK, AWD1 flag INACTIVE (VREFINT is outside the window, but not monitored).
 * 2. ADC_REQUEST_OK, AWD1 flag ACTIVE.
 * - Transfer complete 3x, no error.
 */
void It_Adc_AwdInit_Awd1SingleChannel_OnlySelectedChannelMonitored( void )
{
    adc_Config_t    config;
    adc_AwdConfig_t awdConfig;
    adc_FlagState_t awdFlag = ADC_FLAG_ACTIVE;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_TEMP );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* Threshold between temperature sensor and VREFINT */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_GREATER_THAN_UINT32( (uint32_t)itAdc_Buffer[ 1u ] + IT_ADC_AWD_MARGIN_LSB, itAdc_Buffer[ 0u ] );

    /* 1. Temperature sensor only - inside the window */
    awdConfig.AwdId            = ADC_AWD_1;
    awdConfig.AwdMode          = ADC_AWD_MODE_SINGLE_REGULAR;
    awdConfig.AwdChannelId     = IT_ADC_CH_TEMP;
    awdConfig.AwdChannelMask   = ADC_AWD_CHANNEL_MASK_NONE;
    awdConfig.AwdLowThreshold  = 0u;
    awdConfig.AwdHighThreshold = (adc_AwdThreshold_t)( ( (uint32_t)itAdc_Buffer[ 0u ] + itAdc_Buffer[ 1u ] ) / 2u );
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 2u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, awdFlag );

    /* 2. VREFINT only - above the window */
    awdConfig.AwdChannelId = IT_ADC_CH_VREF;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 3u );

    TEST_ASSERT_EQUAL_UINT32( 3u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, awdFlag );
}


/**
 * \brief   Analog watchdog 2 monitors only the channels of the channel set.
 *
 * \details Initializes ADC1 with regular sequence VREFINT (channel 0), temperature sensor
 *          (channel 17) in polling mode (buffer of 2 samples), converts the sequence once and
 *          sets the upper threshold to the middle between both results (lower threshold 0), then:
 *          1. AWD2 in single mode with channel set {17}, converts the sequence,
 *          2. AWD2 with channel set {17, 0}, converts the sequence again.
 * \note    Task AB#1821: the channel set of AWD2 / AWD3 was not configurable.
 *
 * \par Expected results
 * - VREFINT result is above the temperature sensor result by more than IT_ADC_AWD_MARGIN_LSB.
 * 1. ADC_REQUEST_OK, AWD2 flag INACTIVE (VREFINT is outside the window, but not monitored).
 * 2. ADC_REQUEST_OK, AWD2 flag ACTIVE.
 * - Transfer complete 3x, no error.
 */
void It_Adc_AwdInit_Awd2ChannelMask_OnlySelectedChannelsMonitored( void )
{
    adc_Config_t    config;
    adc_AwdConfig_t awdConfig;
    adc_FlagState_t awdFlag = ADC_FLAG_ACTIVE;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_TEMP );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* Threshold between temperature sensor and VREFINT */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_GREATER_THAN_UINT32( (uint32_t)itAdc_Buffer[ 1u ] + IT_ADC_AWD_MARGIN_LSB, itAdc_Buffer[ 0u ] );

    /* 1. Temperature sensor only - inside the window */
    awdConfig.AwdId            = ADC_AWD_2;
    awdConfig.AwdMode          = ADC_AWD_MODE_SINGLE;
    awdConfig.AwdChannelId     = IT_ADC_CH_TEMP;
    awdConfig.AwdChannelMask   = ADC_AWD_CHANNEL_MASK_17;
    awdConfig.AwdLowThreshold  = 0u;
    awdConfig.AwdHighThreshold = (adc_AwdThreshold_t)( ( (uint32_t)itAdc_Buffer[ 0u ] + itAdc_Buffer[ 1u ] ) / 2u );
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 2u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, awdFlag );

    /* 2. Temperature sensor and VREFINT - VREFINT above the window */
    awdConfig.AwdChannelMask = ADC_AWD_CHANNEL_MASK_17 | ADC_AWD_CHANNEL_MASK_0;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 3u );

    TEST_ASSERT_EQUAL_UINT32( 3u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, awdFlag );
}

/*----------------------------- Dividers (ADC_DIVIDERS) ---------------------*/

/**
 * \brief   Divider voltages on PA0 and PC0 converted in polling mode.
 *
 * \details Polling mode, one-shot buffer of 2 samples, regular sequence of ADC1 channel 5 (PA0)
 *          and channel 1 (PC0).
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - PA0: 4095 x 4.7k / 14.7k = 1309 LSB +- 3 %, PC0: 4095 x 10k / 14.7k = 2786 LSB +- 3 %.
 */
void It_Adc_Dividers_PollSequence_RatiosMatchResistors( void )
{
    adc_Config_t config;

    ItAdc_RequireDividers();

    ItAdc_Get_Config( &config,
                      ADC_TRANSFER_MODE_POLL,
                      IT_ADC_DIV_SAMPLES,
                      ADC_BUFFER_MODE_ONE_SHOT,
                      ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, IT_ADC_CH_DIV_PA0 );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, IT_ADC_CH_DIV_PC0 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    ItAdc_Check_Divider( itAdc_Buffer[ 0u ], IT_ADC_DIV_R_10K, IT_ADC_DIV_R_4K7, "PA0 divider (channel 5)" );
    ItAdc_Check_Divider( itAdc_Buffer[ 1u ], IT_ADC_DIV_R_4K7, IT_ADC_DIV_R_10K, "PC0 divider (channel 1)" );
}


/**
 * \brief   Divider voltages and VREFINT converted in DMA mode.
 *
 * \details DMA mode (DMA1 channel 1), one-shot buffer of 3 samples, regular sequence ADC1 channel 5
 *          (PA0), channel 1 (PC0), VREFINT. The divider voltages are calculated with VDDA of the
 *          VREFINT conversion (factory calibration VREFINT_CAL).
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - Divider ratios as in polling mode (+- 3 %).
 * - PA0 = VDDA x 4.7k / 14.7k, PC0 = VDDA x 10k / 14.7k with VDDA 3300 mV (+- 8 %).
 */
void It_Adc_Dividers_DmaSequenceWithVref_VoltagesMatchResistors( void )
{
    adc_Config_t config;

    ItAdc_RequireDividers();

    ItAdc_Get_Config( &config,
                      ADC_TRANSFER_MODE_DMA,
                      IT_ADC_DIV_VREF_SAMPLES,
                      ADC_BUFFER_MODE_ONE_SHOT,
                      ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, IT_ADC_CH_DIV_PA0 );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, IT_ADC_CH_DIV_PC0 );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    ItAdc_Check_Divider( itAdc_Buffer[ 0u ], IT_ADC_DIV_R_10K, IT_ADC_DIV_R_4K7, "PA0 divider (channel 5)" );
    ItAdc_Check_Divider( itAdc_Buffer[ 1u ], IT_ADC_DIV_R_4K7, IT_ADC_DIV_R_10K, "PC0 divider (channel 1)" );

    const uint32_t vdda_mV   = ItAdc_Get_Vdda_mV( itAdc_Buffer[ 2u ] );
    const uint32_t pa0Expect = ( IT_ADC_VDDA_MV * IT_ADC_DIV_R_4K7 ) / ( IT_ADC_DIV_R_10K + IT_ADC_DIV_R_4K7 );
    const uint32_t pc0Expect = ( IT_ADC_VDDA_MV * IT_ADC_DIV_R_10K ) / ( IT_ADC_DIV_R_10K + IT_ADC_DIV_R_4K7 );

    TEST_ASSERT_UINT32_WITHIN_MESSAGE( ( pa0Expect * IT_ADC_DIV_VOLT_TOL_PCT ) / IT_ADC_PERCENT,
                                       pa0Expect,
                                       ItAdc_Get_Voltage_mV( itAdc_Buffer[ 0u ], vdda_mV ),
                                       "PA0 divider voltage [mV]" );
    TEST_ASSERT_UINT32_WITHIN_MESSAGE( ( pc0Expect * IT_ADC_DIV_VOLT_TOL_PCT ) / IT_ADC_PERCENT,
                                       pc0Expect,
                                       ItAdc_Get_Voltage_mV( itAdc_Buffer[ 1u ], vdda_mV ),
                                       "PC0 divider voltage [mV]" );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Returns the DMA channel of the ADC peripheral (DMA1 channel 1 / 2 / 3 of ADC1 / ADC2 / ADC3 -
 *        STM32L4 request mapping, the channel is valid on STM32L4+ with DMAMUX1 too).
 *
 * \param periphId [in]: ADC peripheral
 *
 * \return Item of the list \ref adc_Dma_t of the peripheral
 */
static adc_Dma_t ItAdc_Get_Dma( adc_PeriphId_t periphId )
{
    adc_Dma_t dma = ADC_DMA_ADC1_DMA1_CHANNEL1;

#if defined(ADC2)
    if( ADC_PERIPH_2 == periphId )
    {
        dma = ADC_DMA_ADC2_DMA1_CHANNEL2;
    }
    else
    {
        /* Other peripheral */
    }
#endif /* ADC2 */

#if defined(ADC3)
    if( ADC_PERIPH_3 == periphId )
    {
        dma = ADC_DMA_ADC3_DMA1_CHANNEL3;
    }
    else
    {
        /* Other peripheral */
    }
#endif /* ADC3 */

    (void)periphId;

    return ( dma );
}


/**
 * \brief Fills module configuration: all peripheral slots with software trigger, 12-bit,
 *        data handling with callbacks and no channel (peripheral not used until a channel
 *        is added), clock HCLK / 4.
 *
 * \param config       [out]: Module configuration
 * \param transferMode  [in]: Data transfer mode
 * \param bufferSize    [in]: Data buffer size
 * \param bufferMode    [in]: Data buffer mode
 * \param triggerMode   [in]: Regular group single / continuous conversion
 */
static void ItAdc_Get_Config( adc_Config_t * const config, adc_TransferMode_t transferMode,
                              adc_BufferSize_t bufferSize, adc_BufferMode_t bufferMode,
                              adc_RegTriggerMode_t triggerMode )
{
    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx++ )
    {
        adc_PeriphConfig_t * const periphCfg = &config->PeriphConfig[ periphIdx ];

        periphCfg->PeriphId       = periphIdx;
        periphCfg->Resolution     = ADC_RESOLUTION_12BIT;
        periphCfg->RegTriggerMode = triggerMode;
        periphCfg->RegTriggerEdge = ADC_TRIGGER_EDGE_RISING;
        periphCfg->RegTriggerId   = ADC_REG_TRIGGER_SOFTWARE;
        periphCfg->RegChannelsCnt = 0u;
        periphCfg->InjTriggerMode = ADC_INJ_TRIGGER_MODE_CONTINUOUS;
        periphCfg->InjTriggerId   = ADC_INJ_TRIGGER_SOFTWARE;
        periphCfg->InjTriggerEdge = ADC_TRIGGER_EDGE_RISING;
        periphCfg->InjChannelsCnt = 0u;

        periphCfg->DataConfig.TransferMode             = transferMode;
        periphCfg->DataConfig.DataBuffer               = itAdc_Buffer;
        periphCfg->DataConfig.BufferSize               = bufferSize;
        periphCfg->DataConfig.BufferMode               = bufferMode;
        periphCfg->DataConfig.Dma                      = ItAdc_Get_Dma( periphIdx );
        periphCfg->DataConfig.DmaPriority              = ADC_DMA_PRIORITY_HIGH;
        periphCfg->DataConfig.IrqPriority              = IT_ADC_IRQ_PRIO;
        periphCfg->DataConfig.HalfTransferCallback     = ItAdc_HalfCallback;
        periphCfg->DataConfig.TransferCompleteCallback = ItAdc_CompleteCallback;
        periphCfg->DataConfig.ErrorCallback            = ItAdc_ErrorCallback;
        periphCfg->DataConfig.InjCompleteCallback      = ItAdc_InjCallback;
    }

    config->ClockSource  = IT_ADC_CLK_SRC;
    config->ClockDivider = IT_ADC_CLK_DIV;
}


/**
 * \brief Appends channel to the regular sequence of a peripheral.
 *
 * \param config       [in,out]: Module configuration
 * \param periphId         [in]: ADC peripheral
 * \param channel          [in]: Channel (item of the list adc_Channel_t)
 */
static void ItAdc_Add_RegChannel( adc_Config_t * const config, adc_PeriphId_t periphId, adc_Channel_t channel )
{
    adc_PeriphConfig_t * const periphCfg = &config->PeriphConfig[ periphId ];

    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].Channel         = channel;
    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].ChannelSampling = IT_ADC_SAMPLING;
    periphCfg->RegChannelsCnt++;
}


/**
 * \brief Waits until counter reaches required value or timeout (Adc_Task called -
 *        data transfer in polling mode).
 *
 * \param counter     [in]: Callback call counter
 * \param expectedCnt [in]: Required count
 */
static void ItAdc_Wait( volatile const uint32_t * const counter, uint32_t expectedCnt )
{
    for( uint32_t loopIdx = 0u; IT_ADC_WAIT_LOOPS > loopIdx; loopIdx++ )
    {
        if( expectedCnt <= *counter )
        {
            break;
        }
        else
        {
            Adc_Task();
        }
    }
}


/**
 * \brief Waits until a data buffer item is written or timeout (Adc_Task called - data
 *        transfer in polling mode).
 *
 * \param sampleIdx [in]: Index of the data buffer item (< IT_ADC_BUF_SIZE)
 */
static void ItAdc_Wait_Sample( uint32_t sampleIdx )
{
    for( uint32_t loopIdx = 0u;
         ( IT_ADC_WAIT_LOOPS > loopIdx                   ) &&
         ( IT_ADC_BUF_EMPTY == itAdc_Buffer[ sampleIdx ] );
         loopIdx++ )
    {
        Adc_Task();
    }
}


/**
 * \brief Calls Adc_Task() a given count of times (data transfer in polling mode continues).
 *
 * \param loopCnt [in]: Count of Adc_Task() calls
 */
static void ItAdc_Poll( uint32_t loopCnt )
{
    for( uint32_t loopIdx = 0u; loopCnt > loopIdx; loopIdx++ )
    {
        Adc_Task();
    }
}


/**
 * \brief Measures analog supply voltage by ADC1 (VREFINT, polling mode) - ADC1 stays initialized.
 *
 * \return VDDA [mV], 0 if the conversion failed (reported by the assertions)
 */
static uint32_t ItAdc_Measure_Vdda_mV( void )
{
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_PERIPH, ADC_CH_ADC1_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    itAdc_CompleteCnt = 0u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );

    return ( ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] ) );
}


/**
 * \brief Calculates analog supply voltage from VREFINT conversion (factory calibration VREFINT_CAL).
 *
 * \param vrefRaw12 [in]: VREFINT conversion result (12-bit)
 *
 * \return VDDA [mV]
 */
static uint32_t ItAdc_Get_Vdda_mV( adc_Data_t vrefRaw12 )
{
    uint32_t vdda_mV = 0u;

    if( 0u != vrefRaw12 )
    {
        vdda_mV = __LL_ADC_CALC_VREFANALOG_VOLTAGE( vrefRaw12, LL_ADC_RESOLUTION_12B );
    }
    else
    {
        /* Invalid conversion result */
    }

    return ( vdda_mV );
}


/**
 * \brief Calculates temperature from temperature sensor conversion (factory calibration
 *        TS_CAL1 / TS_CAL2).
 *
 * \param tempRaw12 [in]: Temperature sensor conversion result (12-bit)
 * \param vdda_mV   [in]: Analog supply voltage [mV]
 *
 * \return Temperature [degree C]
 */
static int32_t ItAdc_Get_Temp_C( adc_Data_t tempRaw12, uint32_t vdda_mV )
{
    return ( (int32_t)__LL_ADC_CALC_TEMPERATURE( vdda_mV, tempRaw12, LL_ADC_RESOLUTION_12B ) );
}


/**
 * \brief Calculates VBAT from VBAT / 3 conversion.
 *
 * \param vbatRaw12 [in]: VBAT / 3 conversion result (12-bit)
 * \param vdda_mV   [in]: Analog supply voltage [mV]
 *
 * \return VBAT [mV]
 */
static uint32_t ItAdc_Get_Vbat_mV( adc_Data_t vbatRaw12, uint32_t vdda_mV )
{
    return ( ( (uint32_t)vbatRaw12 * vdda_mV * IT_ADC_VBAT_RATIO ) / IT_ADC_FULL_SCALE_12B );
}


/** \brief Data buffer half filled callback */
static void ItAdc_HalfCallback( void )
{
    itAdc_HalfCnt++;
}


/** \brief Data buffer filled callback */
static void ItAdc_CompleteCallback( void )
{
    itAdc_CompleteCnt++;
}


/** \brief Injected sequence converted callback */
static void ItAdc_InjCallback( void )
{
    itAdc_InjCnt++;
}


/**
 * \brief Data transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void ItAdc_ErrorCallback( adc_ErrorId_t errorId )
{
    (void)errorId;
    itAdc_ErrorCnt++;
}


/**
 * \brief   Ignores the test on a board without the dividers.
 *
 * \details Fixture of the board (EmBi_Platform, IntegrationTesting README "Board fixtures"):
 *          feature ADC_DIVIDERS. A board without a fixture (IT_FIXTURE not defined) is expected
 *          to have the dividers of its board block.
 */
static void ItAdc_RequireDividers( void )
{
#if ( 0u == IT_ADC_DIV_AVAILABLE )
    TEST_IGNORE_MESSAGE( "No ADC dividers known for the board" );
#elif defined(IT_FEATURE_ADC_DIVIDERS) || \
      !defined(IT_FIXTURE)
    /* Dividers on PA0 and PC0 */
#else
    TEST_IGNORE_MESSAGE( "Board without the fixture ADC_DIVIDERS (dividers on PA0 / PC0)" );
#endif
}


/**
 * \brief Checks a 12-bit conversion result of a divider against the ratio of its resistors.
 *
 * \param raw12     [in]: Conversion result (12-bit)
 * \param topOhm    [in]: Resistor to +3V3 (= VDDA) [Ohm]
 * \param bottomOhm [in]: Resistor to GND [Ohm]
 * \param message   [in]: Message of a failed check
 */
static void ItAdc_Check_Divider( adc_Data_t raw12,
                                 uint32_t topOhm,
                                 uint32_t bottomOhm,
                                 const char * const message )
{
    const uint32_t expected = ( IT_ADC_FULL_SCALE_12B * bottomOhm ) / ( topOhm + bottomOhm );

    TEST_ASSERT_UINT32_WITHIN_MESSAGE( ( expected * IT_ADC_DIV_TOL_PCT ) / IT_ADC_PERCENT,
                                       expected,
                                       (uint32_t)raw12,
                                       message );
}


/**
 * \brief Calculates a voltage from a 12-bit conversion result.
 *
 * \param raw12   [in]: Conversion result (12-bit)
 * \param vdda_mV [in]: Analog supply voltage [mV]
 *
 * \return Voltage [mV]
 */
static uint32_t ItAdc_Get_Voltage_mV( adc_Data_t raw12, uint32_t vdda_mV )
{
    return ( ( (uint32_t)raw12 * vdda_mV ) / IT_ADC_FULL_SCALE_12B );
}
