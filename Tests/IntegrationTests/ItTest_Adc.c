/**
 * \author Mr.Nobody
 * \file ItTest_Adc.c
 * \ingroup Adc
 * \brief Integration tests of Analog-to-digital converter (ADC) module on target.
 *
 * Adc module runs on the MCU together with real RCC, NVIC, GPIO and GPDMA
 * modules and hardware. Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): calibration, conversion of internal channels,
 * resolution, regular sequence, regular discontinuous mode, injected group, data
 * transfer in polling, interrupt and DMA mode, analog watchdog and clock configuration.
 *
 * No external wiring is used - internal channels of ADC1 are converted:
 * - VREFINT (channel 17) - analog supply VDDA is calculated with the typical
 *   VREFINT voltage (NUCLEO boards: VDDA = 3.3 V). Factory calibration values
 *   (0x08FFF810) are not used - the area is not readable with enabled
 *   instruction cache (bus fault).
 * - temperature sensor (channel 16) - sensor voltage within plausible range
 *
 * On MCUs with ADC3, VREFINT (ADC3 channel 17) is converted by ADC3 in ISR and
 * DMA mode as well.
 *
 * Divider tests: two resistor dividers from +3V3 (= VDDA of the board) to GND on
 * external channels of ADC1 - PA0 (channel 0, INP0 through the channel 0 switch
 * ADC_OR.OP0): 10 kOhm to +3V3 / 4.7 kOhm to GND, PC0 (channel 10): 4.7 kOhm to
 * +3V3 / 10 kOhm to GND. The ratio of the conversion result to the full scale equals
 * the divider ratio (independent of VDDA). Fixture of the board (EmBi_Platform
 * IntegrationTesting, "Board fixtures"): feature ADC_DIVIDERS - the divider tests are
 * ignored on a board with a fixture without it (a board without a fixture is
 * expected to have the dividers).
 * Used resources: ADC1, ADC3 (MCUs with ADC3), GPDMA1 channel 7 (DMA mode).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Adc_Port.h"                       /* Module under test              */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void         ItAdc_Get_Config        ( adc_Config_t * const config, adc_TransferMode_t transferMode,
                                              adc_BufferSize_t bufferSize, adc_BufferMode_t bufferMode,
                                              adc_RegTriggerMode_t triggerMode );
static void         ItAdc_Add_RegChannel   ( adc_Config_t * const config, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput );
static void         ItAdc_Add_PeriphRegChannel( adc_Config_t * const config, adc_PeriphId_t periphId,
                                                 adc_ChannelId_t channelId, adc_ChannelInput_t channelInput );
static void         ItAdc_Wait             ( volatile const uint32_t * const counter, uint32_t expectedCnt );
static void         ItAdc_Wait_Sample      ( uint32_t sampleIdx );
static void         ItAdc_Poll             ( uint32_t loopCnt );
static uint32_t     ItAdc_Get_Vdda_mV      ( adc_Data_t vrefRaw12 );
static uint32_t     ItAdc_Get_Voltage_mV   ( adc_Data_t raw12, uint32_t vdda_mV );
static adc_Data_t   ItAdc_Get_Average      ( uint32_t firstIdx, uint32_t cnt );
static void         ItAdc_Add_RegChannels  ( adc_Config_t * const config,
                                             adc_ChannelId_t channelId,
                                             adc_ChannelInput_t channelInput,
                                             uint32_t cnt );

static void         ItAdc_HalfCallback     ( void );
static void         ItAdc_CompleteCallback ( void );
static void         ItAdc_InjCallback      ( void );
static void         ItAdc_ErrorCallback    ( adc_ErrorId_t errorId );
static void         ItAdc_RequireDividers  ( void );
static void         ItAdc_Check_Divider    ( adc_Data_t raw12,
                                              uint32_t topOhm,
                                              uint32_t bottomOhm,
                                              const char * const message );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection) */
#if defined(IT_BOARD_STM32H503xB) || \
    defined(IT_BOARD_STM32H523xE) || \
    defined(IT_BOARD_STM32H533xE) || \
    defined(IT_BOARD_STM32H562xI) || \
    defined(IT_BOARD_STM32H563xI) || \
    defined(IT_BOARD_STM32H573xI)

    /** NUCLEO-H503RB / NUCLEO-H533RE / NUCLEO-H563ZI: dividers on PA0 (channel 0) and PC0 (channel 10) */
    #define IT_ADC_DIV_AVAILABLE            ( 1u )

#elif defined(IT_BOARD_STM32H5E5xJ)

    /** No dividers known for the board */
    #define IT_ADC_DIV_AVAILABLE            ( 0u )

#else
    #error "Board of Adc integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** ADC peripheral under test */
#define IT_ADC_PERIPH                       ( ADC_PERIPH_1 )

/** External channels of the dividers: PA0 (INP0) and PC0 (INP10) */
#define IT_ADC_CH_DIV_PA0                   ( ADC_CHANNEL_0 )
#define IT_ADC_CH_DIV_PC0                   ( ADC_CHANNEL_10 )

/** Resistors of the dividers [Ohm]: PA0 10k to +3V3 / 4.7k to GND, PC0 4.7k to +3V3 / 10k to GND */
#define IT_ADC_DIV_R_10K                    ( 10000u )
#define IT_ADC_DIV_R_4K7                    ( 4700u )

/** Tolerance of the divider ratio [% of the expected result] (resistors 1 %, ADC offset / gain) */
#define IT_ADC_DIV_TOL_PCT                  ( 3u )

/** Tolerance of the divider voltage calculated with the typical VREFINT [% of the expected voltage] */
#define IT_ADC_DIV_VOLT_TOL_PCT             ( 8u )

/** Conversions of a divider channel which are averaged by the divider tests. The pad PA0 of the NUCLEO-H563ZI picks up
 *  noise of about +- 50 LSB (measured 1253 ... 1356 for 1309 expected, the pad PC0 and the other boards +- 3 LSB with
 *  the same sampling time and source impedance), the mean of 8 conversions stays within the tolerance of the ratio */
#define IT_ADC_DIV_AVG_CNT                  ( 8u )

/** Conversions of the divider sequences: PA0 and PC0 (polling), PA0, PC0 and VREFINT (DMA) */
#define IT_ADC_DIV_SAMPLES                  ( 2u * IT_ADC_DIV_AVG_CNT )
#define IT_ADC_DIV_PC0_VREF_CNT             ( IT_ADC_DIV_AVG_CNT / 2u )
#define IT_ADC_DIV_VREF_SAMPLES             ( IT_ADC_DIV_AVG_CNT + 2u * IT_ADC_DIV_PC0_VREF_CNT )

/** Full scale of a 12-bit result [LSB] and count of percent in one unit */
#define IT_ADC_FULL_SCALE_12BIT             ( 4095u )
#define IT_ADC_PERCENT                      ( 100u )

/** Internal channels of ADC1 */
#define IT_ADC_CH_VREF                      ( ADC_CHANNEL_17 )
#define IT_ADC_CH_TEMP                      ( ADC_CHANNEL_16 )

/** Sampling time satisfying minimum of internal channels (VREF 4 us, TEMP 5 us) */
#define IT_ADC_SAMPLING                     ( ADC_CHANNEL_SAMPLING_640_5_CYCLES )

/** Kernel clock: HCLK / 4 (within 1.5 - 75 MHz) */
#define IT_ADC_CLK_SRC                      ( ADC_CLK_SRC_HCLK )
#define IT_ADC_CLK_DIV                      ( ADC_CLK_DIV_4 )

/** Expected analog supply of NUCLEO boards [mV] and tolerance */
#define IT_ADC_VDDA_MV                      ( 3300u )
#define IT_ADC_VDDA_TOL_MV                  ( 150u )

/** Typical VREFINT voltage [mV] (datasheet range 1.182 - 1.232 V) */
#define IT_ADC_VREFINT_MV                   ( 1212u )

/** Plausible temperature sensor voltage range at room temperature [mV] */
#define IT_ADC_TEMP_MIN_MV                  ( 500u )
#define IT_ADC_TEMP_MAX_MV                  ( 900u )

/** Maximal difference of repeated conversions of the same signal [LSB, 12-bit] */
#define IT_ADC_NOISE_LSB                    ( 40u )

/** Data buffer size (at least the conversions of the divider sequences) */
#define IT_ADC_BUF_SIZE                     ( 16u )

/** Maximal count of wait loop iterations (Adc_Task called) */
#define IT_ADC_WAIT_LOOPS                   ( 1000000u )

/** Count of Adc_Task calls after a discontinuous subgroup - takes far longer than several
 *  conversions (one VREFINT conversion: 653 ADC clock cycles = 2612 HCLK cycles) */
#define IT_ADC_IDLE_LOOPS                   ( 20000u )

/** Value of a data buffer item which was not written by the module */
#define IT_ADC_BUF_EMPTY                    ( 0xFFFFu )

/** Upper analog watch-dog threshold separating the temperature sensor from VREFINT [LSB, 12-bit]:
 *  temperature sensor <= 900 mV at VDDA >= 3150 mV: <= 1170 LSB, VREFINT >= 1182 mV at
 *  VDDA <= 3450 mV: >= 1403 LSB (AWD2 compares 8 MSB: 1300 -> 1296, VREFINT 1403 -> 87 > 81) */
#define IT_ADC_AWD_SPLIT_LSB                ( 1300u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Conversion results */
static adc_Data_t               itAdc_Buffer[ IT_ADC_BUF_SIZE ];

/** Counts of callback calls */
static volatile uint32_t        itAdc_HalfCnt;
static volatile uint32_t        itAdc_CompleteCnt;
static volatile uint32_t        itAdc_InjCnt;
static volatile uint32_t        itAdc_ErrorCnt;

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
 * \brief   Adc_Init() on target configures ADC kernel clock.
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
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
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
 * - VREF channel with 2.5 cycles sampling time (below VREFINT minimum),
 * - not divided HCLK (kernel clock above maximum),
 * - VREF input on channel 3 (channel without internal connection).
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 */
void It_Adc_Init_InvalidConfig_ReturnsError( void )
{
    adc_Config_t config;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( ADC_NULL_PTR ) );

    /* Sampling time below minimum of VREFINT */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    config.PeriphConfig[ IT_ADC_PERIPH ].RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_2_5_CYCLES;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* Kernel clock above maximum (HCLK not divided) */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    config.ClockDivider = ADC_CLK_DIV_1;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* Internal signal on channel without the connection */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, ADC_CHANNEL_3, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
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
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* Peripheral is enabled - clock configuration is refused */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( IT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
}

/*-------------------------- Conversion of signals ---------------------------*/

/**
 * \brief   Polling conversion of VREFINT corresponds to board analog supply.
 *
 * \details Initializes ADC1 with VREF channel in polling mode (12-bit, one-shot
 *          buffer of 1 sample), starts conversion and polls Adc_Task() until
 *          transfer complete. VDDA is calculated with typical VREFINT voltage.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - VDDA = 3300 mV +- 150 mV (NUCLEO board supply).
 */
void It_Adc_Set_RegStart_VrefPoll_VddaMatchesBoardSupply( void )
{
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] ) );
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
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
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
 * \brief   Regular sequence VREF + temperature sensor in ISR mode.
 *
 * \details Initializes ADC1 in ISR mode with buffer of 2 samples and regular
 *          sequence VREFINT (channel 17), temperature sensor (channel 16), starts
 *          conversion and waits for transfer complete.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - 1st sample: VDDA = 3300 mV +- 150 mV.
 * - 2nd sample: temperature sensor voltage within 500 - 900 mV (room temperature).
 */
void It_Adc_Set_RegStart_VrefTempSequenceIsr_ValuesPlausible( void )
{
    adc_Config_t config;
    uint32_t     vdda_mV = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_TEMP, ADC_CHANNEL_INPUT_TEMP );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    vdda_mV = ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, vdda_mV );
    TEST_ASSERT_UINT32_WITHIN( ( IT_ADC_TEMP_MAX_MV - IT_ADC_TEMP_MIN_MV ) / 2u, ( IT_ADC_TEMP_MAX_MV + IT_ADC_TEMP_MIN_MV ) / 2u,
                               ItAdc_Get_Voltage_mV( itAdc_Buffer[ 1u ], vdda_mV ) );
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
 * \note    Task AB#221: the regular discontinuous mode was not available. Without DISCEN the
 *          1st start converts the whole sequence (3rd sample written, EOS set).
 *
 * \par Expected results
 * - Conversion mode reads back ADC_REG_TRIGGER_MODE_DISCONT_2RANKS.
 * 1. 2 samples are stored, the 3rd buffer item is not written, no transfer complete, end of
 *    sequence flag INACTIVE.
 * 2. The start is accepted (ADSTART was cleared by HW after the 1st subgroup), the last
 *    subgroup has 1 rank: transfer complete 1x, end of sequence flag ACTIVE, no error.
 * - VREFINT samples: VDDA = 3300 mV +- 150 mV, temperature sensor sample within 500 - 900 mV.
 */
void It_Adc_Set_RegStart_DiscontinuousTwoRanks_EachStartConvertsSubgroup( void )
{
    adc_Config_t         config;
    adc_RegTriggerMode_t triggerMode = ADC_REG_TRIGGER_MODE_CNT;
    adc_FlagState_t      eosFlag     = ADC_FLAG_ACTIVE;
    uint32_t             vdda_mV     = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 3u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_DISCONT_2RANKS );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_TEMP, ADC_CHANNEL_INPUT_TEMP );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
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
    TEST_ASSERT_UINT32_WITHIN( ( IT_ADC_TEMP_MAX_MV - IT_ADC_TEMP_MIN_MV ) / 2u, ( IT_ADC_TEMP_MAX_MV + IT_ADC_TEMP_MIN_MV ) / 2u,
                               ItAdc_Get_Voltage_mV( itAdc_Buffer[ 1u ], vdda_mV ) );
}


/**
 * \brief   Continuous conversion with circular DMA buffer.
 *
 * \details Initializes ADC1 in DMA mode (GPDMA1 channel 7) with circular buffer of
 *          8 samples and continuous VREF conversion, starts it, waits for 3 buffer
 *          completions and stops the conversion.
 *
 * \par Expected results
 * - Complete and half transfer callbacks called at least 3x, no error callback.
 * - All samples differ from the 1st one by at most 40 LSB (noise).
 * - VDDA = 3300 mV +- 150 mV.
 */
void It_Adc_Set_RegStart_ContinuousDmaCircular_HalfAndCompleteCallbacks( void )
{
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_DMA, IT_ADC_BUF_SIZE, ADC_BUFFER_MODE_CIRCULAR, ADC_REG_TRIGGER_MODE_CONTINUOUS );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
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
}


/**
 * \brief   ADC3 conversion of VREFINT in ISR mode.
 *
 * \note    Task AB#224: ISR transfer mode was rejected for ADC3.
 *
 * \details Initializes ADC3 in ISR mode with buffer of 2 samples and regular
 *          sequence VREFINT (ADC3 channel 17) twice, starts conversion and waits
 *          for transfer complete.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - Both samples: VDDA = 3300 mV +- 150 mV.
 * - Ignored on MCUs without ADC3.
 */
void It_Adc_Set_RegStart_Adc3VrefIsr_VddaMatchesBoardSupply( void )
{
#if !defined(ADC3)
    TEST_IGNORE_MESSAGE( "MCU without ADC3" );
#else
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_PeriphRegChannel( &config, ADC_PERIPH_3, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    ItAdc_Add_PeriphRegChannel( &config, ADC_PERIPH_3, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( ADC_PERIPH_3 ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] ) );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( itAdc_Buffer[ 1u ] ) );
#endif
}


/**
 * \brief   ADC3 continuous conversion with circular DMA buffer.
 *
 * \note    Task AB#224: DMA transfer mode was rejected for ADC3 (GPDMA request ADC3, Task AB#229).
 *
 * \details Initializes ADC3 in DMA mode (GPDMA1 channel 7) with circular buffer of
 *          8 samples and continuous VREFINT conversion (ADC3 channel 17), starts it,
 *          waits for 3 buffer completions and stops the conversion.
 *
 * \par Expected results
 * - Complete and half transfer callbacks called at least 3x, no error callback.
 * - All samples differ from the 1st one by at most 40 LSB (noise).
 * - VDDA = 3300 mV +- 150 mV.
 * - Ignored on MCUs without ADC3.
 */
void It_Adc_Set_RegStart_Adc3ContinuousDmaCircular_HalfAndCompleteCallbacks( void )
{
#if !defined(ADC3)
    TEST_IGNORE_MESSAGE( "MCU without ADC3" );
#else
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_DMA, IT_ADC_BUF_SIZE, ADC_BUFFER_MODE_CIRCULAR, ADC_REG_TRIGGER_MODE_CONTINUOUS );
    ItAdc_Add_PeriphRegChannel( &config, ADC_PERIPH_3, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( ADC_PERIPH_3 ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 3u );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( ADC_PERIPH_3 ) );

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 3u, itAdc_CompleteCnt );
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 3u, itAdc_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    for( uint32_t dataIdx = 1u; IT_ADC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        TEST_ASSERT_UINT16_WITHIN( IT_ADC_NOISE_LSB, itAdc_Buffer[ 0u ], itAdc_Buffer[ dataIdx ] );
    }

    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( itAdc_Buffer[ 0u ] ) );
#endif
}


/**
 * \brief   Injected conversion of VREFINT on target.
 *
 * \details Initializes ADC1 with one injected VREF channel (ISR mode), starts the
 *          injected group by software and waits for injected complete callback.
 *
 * \par Expected results
 * - Injected complete callback 1x.
 * - Injected data of rank 1 gives VDDA = 3300 mV +- 150 mV.
 */
void It_Adc_Set_InjStart_VrefInjected_InjCallbackAndData( void )
{
    adc_Config_t config;
    adc_Data_t   injData = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannelsCnt                      = 1u;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 0u ].ChannelId         = IT_ADC_CH_VREF;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 0u ].ChannelInput      = ADC_CHANNEL_INPUT_VREF;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 0u ].ChannelSampling   = IT_ADC_SAMPLING;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_InjCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_InjCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_InjData( IT_ADC_PERIPH, ADC_INJ_SEQUENCE_1, &injData ) );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, ItAdc_Get_Vdda_mV( injData ) );
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
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* VREFINT (~1500 LSB) is above the window 0 - 100 */
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
 * \brief   Analog watchdog does not trigger for VREFINT inside the window.
 *
 * \details Initializes ADC1 with VREF channel and AWD1 on all regular channels
 *          with window 100 - 4000 LSB, clears AWD1 flag and converts VREFINT.
 *
 * \par Expected results
 * - AWD1 flag is INACTIVE after conversion.
 */
void It_Adc_AwdInit_VrefInsideWindow_AwdFlagInactive( void )
{
    adc_Config_t    config;
    adc_AwdConfig_t awdConfig;
    adc_FlagState_t awdFlag = ADC_FLAG_ACTIVE;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
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
}


/**
 * \brief   Analog watchdog 1 monitors only the selected channel.
 *
 * \details Initializes ADC1 with regular sequence VREFINT (channel 17), temperature sensor
 *          (channel 16) in polling mode (buffer of 2 samples) and:
 *          1. AWD1 on the single regular channel 16 with window 0 - 1300 LSB, converts the sequence,
 *          2. AWD1 on the single regular channel 17 with the same window, converts the sequence again.
 *          The window contains the temperature sensor voltage, VREFINT is above it.
 * \note    Task AB#220: the single channel modes were rejected (no channel in adc_AwdConfig_t).
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK, AWD1 flag INACTIVE (VREFINT is outside the window, but not monitored).
 * 2. ADC_REQUEST_OK, AWD1 flag ACTIVE.
 * - Transfer complete 2x, no error.
 */
void It_Adc_AwdInit_Awd1SingleChannel_OnlySelectedChannelMonitored( void )
{
    adc_Config_t    config;
    adc_AwdConfig_t awdConfig;
    adc_FlagState_t awdFlag = ADC_FLAG_ACTIVE;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_TEMP, ADC_CHANNEL_INPUT_TEMP );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* 1. Temperature sensor only - inside the window */
    awdConfig.AwdId            = ADC_AWD_1;
    awdConfig.AwdMode          = ADC_AWD_MODE_SINGLE_REGULAR;
    awdConfig.AwdChannelId     = IT_ADC_CH_TEMP;
    awdConfig.AwdChannelMask   = ADC_AWD_CHANNEL_MASK_NONE;
    awdConfig.AwdLowThreshold  = 0u;
    awdConfig.AwdHighThreshold = IT_ADC_AWD_SPLIT_LSB;
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, awdFlag );

    /* 2. VREFINT only - above the window */
    awdConfig.AwdChannelId = IT_ADC_CH_VREF;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 2u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, awdFlag );
}


/**
 * \brief   Analog watchdog 2 monitors only the channels of the channel set.
 *
 * \details Initializes ADC1 with regular sequence VREFINT (channel 17), temperature sensor
 *          (channel 16) in polling mode (buffer of 2 samples) and:
 *          1. AWD2 in single mode with channel set {16} and window 0 - 1300 LSB, converts the sequence,
 *          2. AWD2 with channel set {16, 17} and the same window, converts the sequence again.
 * \note    Task AB#220: the channel set of AWD2 / AWD3 was not configurable.
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK, AWD2 flag INACTIVE (VREFINT is outside the window, but not monitored).
 * 2. ADC_REQUEST_OK, AWD2 flag ACTIVE.
 * - Transfer complete 2x, no error.
 */
void It_Adc_AwdInit_Awd2ChannelMask_OnlySelectedChannelsMonitored( void )
{
    adc_Config_t    config;
    adc_AwdConfig_t awdConfig;
    adc_FlagState_t awdFlag = ADC_FLAG_ACTIVE;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_TEMP, ADC_CHANNEL_INPUT_TEMP );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* 1. Temperature sensor only - inside the window */
    awdConfig.AwdId            = ADC_AWD_2;
    awdConfig.AwdMode          = ADC_AWD_MODE_SINGLE;
    awdConfig.AwdChannelId     = IT_ADC_CH_TEMP;
    awdConfig.AwdChannelMask   = ADC_AWD_CHANNEL_MASK_16;
    awdConfig.AwdLowThreshold  = 0u;
    awdConfig.AwdHighThreshold = IT_ADC_AWD_SPLIT_LSB;
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, awdFlag );

    /* 2. Temperature sensor and VREFINT - VREFINT above the window */
    awdConfig.AwdChannelMask = ADC_AWD_CHANNEL_MASK_16 | ADC_AWD_CHANNEL_MASK_17;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 2u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, awdFlag );
}

/*----------------------------- Dividers (ADC_DIVIDERS) ---------------------*/

/**
 * \brief   Divider voltages on PA0 and PC0 converted in polling mode.
 *
 * \details Polling mode, one-shot buffer of 16 samples, regular sequence of 8 conversions of channel 0 (PA0, INP0
 *          through ADC_OR.OP0) followed by 8 conversions of channel 10 (PC0). The mean of the conversions of a
 *          channel is checked (noise of the pad PA0 of the NUCLEO-H563ZI, see IT_ADC_DIV_AVG_CNT).
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - PA0: 4095 x 4.7k / 14.7k = 1309 LSB +- 3 %, PC0: 4095 x 10k / 14.7k = 2785 LSB +- 3 % (means).
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
    ItAdc_Add_RegChannels( &config, IT_ADC_CH_DIV_PA0, ADC_CHANNEL_INPUT_PIN_SINGLE, IT_ADC_DIV_AVG_CNT );
    ItAdc_Add_RegChannels( &config, IT_ADC_CH_DIV_PC0, ADC_CHANNEL_INPUT_PIN_SINGLE, IT_ADC_DIV_AVG_CNT );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    ItAdc_Check_Divider( ItAdc_Get_Average( 0u, IT_ADC_DIV_AVG_CNT ),
                         IT_ADC_DIV_R_10K,
                         IT_ADC_DIV_R_4K7,
                         "PA0 divider (channel 0)" );
    ItAdc_Check_Divider( ItAdc_Get_Average( IT_ADC_DIV_AVG_CNT, IT_ADC_DIV_AVG_CNT ),
                         IT_ADC_DIV_R_4K7,
                         IT_ADC_DIV_R_10K,
                         "PC0 divider (channel 10)" );
}


/**
 * \brief   Divider voltages and VREFINT converted in DMA mode.
 *
 * \details DMA mode (GPDMA1 channel 7), one-shot buffer of 16 samples, regular sequence of
 *          8 conversions of channel 0 (PA0), 4 of channel 10 (PC0) and 4 of VREFINT. The means of the
 *          conversions of a channel are used (noise of the pad PA0 of the NUCLEO-H563ZI, see
 *          IT_ADC_DIV_AVG_CNT). The divider voltages are calculated with VDDA of the VREFINT
 *          conversion (typical VREFINT voltage).
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
    ItAdc_Add_RegChannels( &config, IT_ADC_CH_DIV_PA0, ADC_CHANNEL_INPUT_PIN_SINGLE, IT_ADC_DIV_AVG_CNT );
    ItAdc_Add_RegChannels( &config, IT_ADC_CH_DIV_PC0, ADC_CHANNEL_INPUT_PIN_SINGLE, IT_ADC_DIV_PC0_VREF_CNT );
    ItAdc_Add_RegChannels( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF, IT_ADC_DIV_PC0_VREF_CNT );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    const adc_Data_t pa0Raw  = ItAdc_Get_Average( 0u, IT_ADC_DIV_AVG_CNT );
    const adc_Data_t pc0Raw  = ItAdc_Get_Average( IT_ADC_DIV_AVG_CNT, IT_ADC_DIV_PC0_VREF_CNT );
    const adc_Data_t vrefRaw = ItAdc_Get_Average( IT_ADC_DIV_AVG_CNT + IT_ADC_DIV_PC0_VREF_CNT,
                                                  IT_ADC_DIV_PC0_VREF_CNT );

    ItAdc_Check_Divider( pa0Raw, IT_ADC_DIV_R_10K, IT_ADC_DIV_R_4K7, "PA0 divider (channel 0)" );
    ItAdc_Check_Divider( pc0Raw, IT_ADC_DIV_R_4K7, IT_ADC_DIV_R_10K, "PC0 divider (channel 10)" );

    const uint32_t vdda_mV   = ItAdc_Get_Vdda_mV( vrefRaw );
    const uint32_t pa0Expect = ( IT_ADC_VDDA_MV * IT_ADC_DIV_R_4K7 ) / ( IT_ADC_DIV_R_10K + IT_ADC_DIV_R_4K7 );
    const uint32_t pc0Expect = ( IT_ADC_VDDA_MV * IT_ADC_DIV_R_10K ) / ( IT_ADC_DIV_R_10K + IT_ADC_DIV_R_4K7 );

    TEST_ASSERT_UINT32_WITHIN_MESSAGE( ( pa0Expect * IT_ADC_DIV_VOLT_TOL_PCT ) / IT_ADC_PERCENT,
                                       pa0Expect,
                                       ItAdc_Get_Voltage_mV( pa0Raw, vdda_mV ),
                                       "PA0 divider voltage [mV]" );
    TEST_ASSERT_UINT32_WITHIN_MESSAGE( ( pc0Expect * IT_ADC_DIV_VOLT_TOL_PCT ) / IT_ADC_PERCENT,
                                       pc0Expect,
                                       ItAdc_Get_Voltage_mV( pc0Raw, vdda_mV ),
                                       "PC0 divider voltage [mV]" );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Fills module configuration: ADC1 used (software trigger, 12-bit), other
 *        peripherals unused, data handling with callbacks, no channel.
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
        periphCfg->DataConfig.DmaPeriphId              = ADC_DMA_PERIPH_1;
        periphCfg->DataConfig.DmaChannelId             = ADC_DMA_CHANNEL_7;
        periphCfg->DataConfig.DmaPriority              = ADC_DMA_PRIORITY_HIGH;
        periphCfg->DataConfig.IrqPriority              = 5u;
        periphCfg->DataConfig.HalfTransferCallback     = ItAdc_HalfCallback;
        periphCfg->DataConfig.TransferCompleteCallback = ItAdc_CompleteCallback;
        periphCfg->DataConfig.ErrorCallback            = ItAdc_ErrorCallback;
        periphCfg->DataConfig.InjCompleteCallback      = ItAdc_InjCallback;
    }

    config->ClockSource  = IT_ADC_CLK_SRC;
    config->ClockDivider = IT_ADC_CLK_DIV;
}


/**
 * \brief Appends channel to the regular sequence of ADC1.
 *
 * \param config       [in,out]: Module configuration
 * \param channelId        [in]: Channel
 * \param channelInput     [in]: Channel input
 */
static void ItAdc_Add_RegChannel( adc_Config_t * const config, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput )
{
    ItAdc_Add_PeriphRegChannel( config, IT_ADC_PERIPH, channelId, channelInput );
}


/**
 * \brief Appends channel to the regular sequence of a peripheral.
 *
 * \param config       [in,out]: Module configuration
 * \param periphId         [in]: ADC peripheral
 * \param channelId        [in]: Channel
 * \param channelInput     [in]: Channel input
 */
static void ItAdc_Add_PeriphRegChannel( adc_Config_t * const config, adc_PeriphId_t periphId,
                                         adc_ChannelId_t channelId, adc_ChannelInput_t channelInput )
{
    adc_PeriphConfig_t * const periphCfg = &config->PeriphConfig[ periphId ];

    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].ChannelId       = channelId;
    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].ChannelInput    = channelInput;
    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].ChannelSampling = IT_ADC_SAMPLING;
    periphCfg->RegChannelsCnt++;
}


/**
 * \brief Appends the same channel several times to the regular sequence of ADC1 (the conversions are averaged
 *        by \ref ItAdc_Get_Average).
 *
 * \param config       [in,out]: Module configuration
 * \param channelId        [in]: Channel
 * \param channelInput     [in]: Channel input
 * \param cnt              [in]: Count of the conversions in the sequence
 */
static void ItAdc_Add_RegChannels( adc_Config_t * const config,
                                   adc_ChannelId_t channelId,
                                   adc_ChannelInput_t channelInput,
                                   uint32_t cnt )
{
    for( uint32_t idx = 0u; cnt > idx; idx++ )
    {
        ItAdc_Add_RegChannel( config, channelId, channelInput );
    }
}


/**
 * \brief Calculates the mean of consecutive items of the data buffer.
 *
 * \param firstIdx [in]: Index of the first item
 * \param cnt      [in]: Count of the items (the mean of zero items is zero)
 *
 * \return Mean of the items (rounded down).
 */
static adc_Data_t ItAdc_Get_Average( uint32_t firstIdx, uint32_t cnt )
{
    uint32_t sum = 0u;

    for( uint32_t idx = 0u; cnt > idx; idx++ )
    {
        sum += (uint32_t)itAdc_Buffer[ firstIdx + idx ];
    }

    if( 0u < cnt )
    {
        sum /= cnt;
    }
    else
    {
        /* No item, the mean is zero */
        sum = 0u;
    }

    return ( (adc_Data_t)sum );
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
    for( uint32_t loopIdx = 0u;
         ( IT_ADC_WAIT_LOOPS > loopIdx ) &&
         ( expectedCnt > *counter );
         loopIdx++ )
    {
        Adc_Task();
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
 * \brief Calculates analog supply voltage from VREFINT conversion (typical VREFINT voltage).
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
        vdda_mV = ( IT_ADC_VREFINT_MV * 4095u ) / vrefRaw12;
    }
    else
    {
        /* Invalid conversion result */
    }

    return ( vdda_mV );
}


/**
 * \brief Calculates input voltage of a conversion.
 *
 * \param raw12   [in]: Conversion result (12-bit)
 * \param vdda_mV [in]: Analog supply voltage [mV]
 *
 * \return Input voltage [mV]
 */
static uint32_t ItAdc_Get_Voltage_mV( adc_Data_t raw12, uint32_t vdda_mV )
{
    return ( ( (uint32_t)raw12 * vdda_mV ) / 4095u );
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
    const uint32_t expected = ( IT_ADC_FULL_SCALE_12BIT * bottomOhm ) / ( topOhm + bottomOhm );

    TEST_ASSERT_UINT32_WITHIN_MESSAGE( ( expected * IT_ADC_DIV_TOL_PCT ) / IT_ADC_PERCENT,
                                       expected,
                                       (uint32_t)raw12,
                                       message );
}
