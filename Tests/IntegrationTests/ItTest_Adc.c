/**
 * \author Mr.Nobody
 * \file ItTest_Adc.c
 * \ingroup Adc
 * \brief Integration tests of Analog-to-digital converter (ADC) module on target.
 *
 * Adc module runs on the MCU together with real RCC, NVIC, GPIO and GPDMA
 * modules and hardware. Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): calibration, conversion of internal channels,
 * resolution, regular sequence, injected group, data transfer in polling,
 * interrupt and DMA mode, analog watchdog and clock configuration.
 *
 * No external wiring is used - internal channels of ADC1 are converted:
 * - VREFINT (channel 17) - analog supply VDDA is calculated with the typical
 *   VREFINT voltage (NUCLEO boards: VDDA = 3.3 V). Factory calibration values
 *   (0x08FFF810) are not used - the area is not readable with enabled
 *   instruction cache (bus fault).
 * - temperature sensor (channel 16) - sensor voltage within plausible range
 *
 * Used resources: ADC1, GPDMA1 channel 7 (DMA mode).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Adc_Port.h"                       /* Module under test              */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void         It_Adc_Get_Config        ( adc_Config_t * const config, adc_TransferMode_t transferMode,
                                              adc_BufferSize_t bufferSize, adc_BufferMode_t bufferMode,
                                              adc_RegTriggerMode_t triggerMode );
static void         It_Adc_Add_RegChannel   ( adc_Config_t * const config, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput );
static void         It_Adc_Wait             ( volatile const uint32_t * const counter, uint32_t expectedCnt );
static uint32_t     It_Adc_Get_Vdda_mV      ( adc_Data_t vrefRaw12 );
static uint32_t     It_Adc_Get_Voltage_mV   ( adc_Data_t raw12, uint32_t vdda_mV );

static void         It_Adc_HalfCallback     ( void );
static void         It_Adc_CompleteCallback ( void );
static void         It_Adc_InjCallback      ( void );
static void         It_Adc_ErrorCallback    ( adc_ErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** ADC peripheral under test */
#define IT_ADC_PERIPH                       ( ADC_PERIPH_1 )

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

/** Data buffer size */
#define IT_ADC_BUF_SIZE                     ( 8u )

/** Maximal count of wait loop iterations (Adc_Task called) */
#define IT_ADC_WAIT_LOOPS                   ( 1000000u )

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
        itAdc_Buffer[ dataIdx ] = 0xFFFFu;
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

    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
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
    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    config.PeriphConfig[ IT_ADC_PERIPH ].RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_2_5_CYCLES;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* Kernel clock above maximum (HCLK not divided) */
    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    config.ClockDivider = ADC_CLK_DIV_1;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* Internal signal on channel without the connection */
    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, ADC_CHANNEL_3, ADC_CHANNEL_INPUT_VREF );
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

    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* Peripheral is enabled - clock configuration is refused */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( IT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    It_Adc_Wait( &itAdc_CompleteCnt, 1u );
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

    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    It_Adc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, It_Adc_Get_Vdda_mV( itAdc_Buffer[ 0u ] ) );
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

    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    config.PeriphConfig[ IT_ADC_PERIPH ].Resolution = ADC_RESOLUTION_8BIT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Resolution( IT_ADC_PERIPH, &resolution ) );
    TEST_ASSERT_EQUAL( ADC_RESOLUTION_8BIT, resolution );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    It_Adc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_LESS_THAN_UINT16( 256u, itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT32_WITHIN( 2u * IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, It_Adc_Get_Vdda_mV( (adc_Data_t)( itAdc_Buffer[ 0u ] << 4u ) ) );
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

    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_TEMP, ADC_CHANNEL_INPUT_TEMP );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    It_Adc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    vdda_mV = It_Adc_Get_Vdda_mV( itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, vdda_mV );
    TEST_ASSERT_UINT32_WITHIN( ( IT_ADC_TEMP_MAX_MV - IT_ADC_TEMP_MIN_MV ) / 2u, ( IT_ADC_TEMP_MAX_MV + IT_ADC_TEMP_MIN_MV ) / 2u,
                               It_Adc_Get_Voltage_mV( itAdc_Buffer[ 1u ], vdda_mV ) );
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

    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_DMA, IT_ADC_BUF_SIZE, ADC_BUFFER_MODE_CIRCULAR, ADC_REG_TRIGGER_MODE_CONTINUOUS );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    It_Adc_Wait( &itAdc_CompleteCnt, 3u );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( IT_ADC_PERIPH ) );

    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 3u, itAdc_CompleteCnt );
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 3u, itAdc_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    for( uint32_t dataIdx = 1u; IT_ADC_BUF_SIZE > dataIdx; dataIdx++ )
    {
        TEST_ASSERT_UINT16_WITHIN( IT_ADC_NOISE_LSB, itAdc_Buffer[ 0u ], itAdc_Buffer[ dataIdx ] );
    }

    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, It_Adc_Get_Vdda_mV( itAdc_Buffer[ 0u ] ) );
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

    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannelsCnt                      = 1u;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 0u ].ChannelId         = IT_ADC_CH_VREF;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 0u ].ChannelInput      = ADC_CHANNEL_INPUT_VREF;
    config.PeriphConfig[ IT_ADC_PERIPH ].InjChannels[ 0u ].ChannelSampling   = IT_ADC_SAMPLING;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStart( IT_ADC_PERIPH ) );
    It_Adc_Wait( &itAdc_InjCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_InjCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_InjData( IT_ADC_PERIPH, ADC_INJ_SEQUENCE_1, &injData ) );
    TEST_ASSERT_UINT32_WITHIN( IT_ADC_VDDA_TOL_MV, IT_ADC_VDDA_MV, It_Adc_Get_Vdda_mV( injData ) );
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

    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
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
    It_Adc_Wait( &itAdc_CompleteCnt, 1u );

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

    It_Adc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    It_Adc_Add_RegChannel( &config, IT_ADC_CH_VREF, ADC_CHANNEL_INPUT_VREF );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    awdConfig.AwdId            = ADC_AWD_1;
    awdConfig.AwdMode          = ADC_AWD_MODE_ALL_REGULAR;
    awdConfig.AwdLowThreshold  = 100u;
    awdConfig.AwdHighThreshold = 4000u;
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    It_Adc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, awdFlag );
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
static void It_Adc_Get_Config( adc_Config_t * const config, adc_TransferMode_t transferMode,
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
        periphCfg->DataConfig.HalfTransferCallback     = It_Adc_HalfCallback;
        periphCfg->DataConfig.TransferCompleteCallback = It_Adc_CompleteCallback;
        periphCfg->DataConfig.ErrorCallback            = It_Adc_ErrorCallback;
        periphCfg->DataConfig.InjCompleteCallback      = It_Adc_InjCallback;
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
static void It_Adc_Add_RegChannel( adc_Config_t * const config, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput )
{
    adc_PeriphConfig_t * const periphCfg = &config->PeriphConfig[ IT_ADC_PERIPH ];

    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].ChannelId       = channelId;
    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].ChannelInput    = channelInput;
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
static void It_Adc_Wait( volatile const uint32_t * const counter, uint32_t expectedCnt )
{
    for( uint32_t loopIdx = 0u; ( IT_ADC_WAIT_LOOPS > loopIdx ) && ( expectedCnt > *counter ); loopIdx++ )
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
static uint32_t It_Adc_Get_Vdda_mV( adc_Data_t vrefRaw12 )
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
static uint32_t It_Adc_Get_Voltage_mV( adc_Data_t raw12, uint32_t vdda_mV )
{
    return ( ( (uint32_t)raw12 * vdda_mV ) / 4095u );
}


/** \brief Data buffer half filled callback */
static void It_Adc_HalfCallback( void )
{
    itAdc_HalfCnt++;
}


/** \brief Data buffer filled callback */
static void It_Adc_CompleteCallback( void )
{
    itAdc_CompleteCnt++;
}


/** \brief Injected sequence converted callback */
static void It_Adc_InjCallback( void )
{
    itAdc_InjCnt++;
}


/**
 * \brief Data transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void It_Adc_ErrorCallback( adc_ErrorId_t errorId )
{
    (void)errorId;
    itAdc_ErrorCnt++;
}
