/**
 * \author Mr.Nobody
 * \file Test_Adc.c
 * \ingroup Adc
 * \brief Unit tests of Analog-to-Digital Converter (ADC) module (STM32F4).
 *
 * Adc.c, Adc_Isr.c, Adc_Poll.c and Adc_Dma.c are compiled unchanged with real
 * LL drivers. ADC registers are emulated by RegMem, RCC, NVIC, GPIO and DMA
 * modules are mocked by CMock. The common ADC ISR registered in NVIC and the
 * DMA callbacks passed to Dma_Init() are captured by stubs and called directly
 * to test interrupt data handling.
 *
 * \note Emulated registers are plain memory - SR flags are not cleared by data
 *       register reads, flag clear (write of 0, other bits written with 1) sets
 *       all other SR bits. Tests preset SR before each step.
 *
 * \note Configuration kept by the module (trigger shadows, channel inputs, data
 *       handling) is static - setUp() releases it (Adc_Deinit with ignored mocks)
 *       and re-initializes the mocks.
 */

/* ============================= INCLUDES =================================== */
#include <string.h>                         /* memset                         */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Adc_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32_adc.h"                      /* ADC registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static rcc_RequestState_t   Ut_Adc_RccGetClkStub        ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static rcc_RequestState_t   Ut_Adc_RccGetStateStub      ( rcc_PeriphId_t periphId, rcc_FunctionState_t * const funcState, int callCnt );
static gpio_RequestState_t  Ut_Adc_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static nvic_RequestState_t  Ut_Adc_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static dma_RequestState_t   Ut_Adc_DmaDefaultStub       ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static dma_RequestState_t   Ut_Adc_DmaInitStub          ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static void                 Ut_Adc_Ignore_PeriphMocks   ( void );
static void                 Ut_Adc_Reset_Mocks          ( void );
static void                 Ut_Adc_Release              ( void );
static adc_PeriphConfig_t   Ut_Adc_Get_PeriphConfig     ( adc_TransferMode_t xferMode, adc_BufferSize_t bufferSize );
static adc_Config_t         Ut_Adc_Get_Config           ( void );
static void                 Ut_Adc_Set_ClockDiv4        ( void );
static void                 Ut_Adc_PeriphInit           ( adc_PeriphConfig_t * const periphConfig );
static void                 Ut_Adc_Call_Isr             ( uint32_t srFlags, uint32_t data );

static void                 Ut_Adc_HalfCallback         ( void );
static void                 Ut_Adc_CompleteCallback     ( void );
static void                 Ut_Adc_InjCallback          ( void );
static void                 Ut_Adc_ErrorCallback        ( adc_ErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** ADC peripheral used by tests (available on all supported MCUs) */
#define UT_ADC_PERIPH                       ( ADC_PERIPH_1 )
#define UT_ADC_REG                          ( ADC1 )
#define UT_ADC_COMMON                       ( __LL_ADC_COMMON_INSTANCE( ADC1 ) )
#define UT_ADC_RCC                          ( RCC_PERIPH_ADC1 )
#define UT_ADC_NVIC                         ( NVIC_PERIPH_IRQ_ADC )

/** Clock frequencies returned by RCC mock [Hz] */
#define UT_ADC_PCLK2_HZ                     ( 84000000u )
#define UT_ADC_HCLK_HZ                      ( 168000000u )

/** Internal channels of ADC1 */
#define UT_ADC_VREF_CHANNEL                 ( ADC_CHANNEL_17 )
#define UT_ADC_VBAT_CHANNEL                 ( ADC_CHANNEL_18 )

/** Sampling time satisfying minimum of internal channels at PCLK2 / 4 (21 MHz, 10 us = 210 cycles) */
#define UT_ADC_SAMPLING_INTERNAL            ( ADC_CHANNEL_SAMPLING_480_CYCLES )

/** Interrupt priority of test configurations */
#define UT_ADC_PRIO                         ( 7u )

/** Size of the test data buffer */
#define UT_ADC_BUF_SIZE                     ( 8u )

/** Value of SR after flag clear in emulated register (all other bits written with 1) */
#define UT_ADC_SR_CLEARED( flags )          ( ~(uint32_t)( flags ) )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC for the ADC */
static nvic_IsrCallback_t       utAdc_Isr;

/** GPIO configuration of the last Gpio_Init call and count of calls */
static gpio_Config_t            utAdc_GpioConfig;
static uint32_t                 utAdc_GpioInitCnt;

/** DMA configuration of the last Dma_Init call and count of calls */
static dma_ConfigStruct_t       utAdc_DmaConfig;
static uint32_t                 utAdc_DmaInitCnt;

/** Clock state returned by Rcc_Get_PeriphState stub */
static rcc_FunctionState_t      utAdc_RccClkState;

/** Conversion data buffer */
static adc_Data_t               utAdc_Buffer[ UT_ADC_BUF_SIZE ];

/** Counts of callback calls */
static uint32_t                 utAdc_HalfCnt;
static uint32_t                 utAdc_CompleteCnt;
static uint32_t                 utAdc_InjCnt;
static uint32_t                 utAdc_ErrorCnt;

/** Parameter of the last error callback */
static adc_ErrorId_t            utAdc_LastError;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    Ut_Adc_Release();

    utAdc_Isr         = NULL;
    utAdc_GpioInitCnt = 0u;
    utAdc_DmaInitCnt  = 0u;
    utAdc_RccClkState = RCC_FUNCTION_ACTIVE;
    utAdc_HalfCnt     = 0u;
    utAdc_CompleteCnt = 0u;
    utAdc_InjCnt      = 0u;
    utAdc_ErrorCnt    = 0u;
    utAdc_LastError   = ADC_ERROR_CNT;

    memset( &utAdc_GpioConfig, 0, sizeof( utAdc_GpioConfig ) );
    memset( &utAdc_DmaConfig, 0, sizeof( utAdc_DmaConfig ) );
    memset( utAdc_Buffer, 0, sizeof( utAdc_Buffer ) );
}


void tearDown( void )
{
    /* Mocks are verified by the framework */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Adc_Get_ModuleVersion() returns version of the module.
 *
 * \par Expected results
 * - Version 1.0.0.
 */
void Ut_Adc_Get_ModuleVersion_ReturnsVersion( void )
{
    const adc_ModuleVersion_t version = Adc_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Adc_Init() rejects NULL configuration and invalid clock source.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR, no RCC clock is enabled (no Rcc_Set_PeriphActive expected).
 */
void Ut_Adc_Init_NullOrInvalidClockSource_ReturnsError( void )
{
    adc_Config_t config = Ut_Adc_Get_Config();

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( NULL ) );

    config.ClockSource = ADC_CLK_SRC_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
}


/**
 * \brief   Adc_Init() rejects clock divider resulting in ADC clock above maximum.
 *
 * \details PCLK2 84 MHz / 2 = 42 MHz (maximum 36 MHz).
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR, no RCC clock is enabled, CCR is not modified.
 */
void Ut_Adc_Init_ClockAboveMaximum_ReturnsErrorWithoutRcc( void )
{
    adc_Config_t config = Ut_Adc_Get_Config();

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    config.ClockDivider = ADC_CLK_DIV_2;
    UT_ADC_COMMON->CCR  = LL_ADC_CLOCK_SYNC_PCLK_DIV8;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_SYNC_PCLK_DIV8, UT_ADC_COMMON->CCR );
}


/**
 * \brief   Adc_Init() without used peripheral configures clocks only.
 *
 * \details All ADC clocks are inactive in RCC.
 *
 * \par Expected results
 * - ADC_REQUEST_OK, RCC clock of every ADC peripheral is activated.
 * - CCR ADCPRE = PCLK2 / 4, clock divider reads back ADC_CLK_DIV_4.
 * - No ADC is enabled.
 */
void Ut_Adc_Init_NoPeripheralUsed_ClocksEnabledAndDividerSet( void )
{
    adc_Config_t config = Ut_Adc_Get_Config();
    adc_ClkDiv_t clkDiv = ADC_CLK_DIV_CNT;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Rcc_Get_PeriphState_StubWithCallback( Ut_Adc_RccGetStateStub );
    utAdc_RccClkState = RCC_FUNCTION_INACTIVE;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC1, RCC_REQUEST_OK );
#if defined (ADC2)
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC2, RCC_REQUEST_OK );
#endif
#if defined (ADC3)
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC3, RCC_REQUEST_OK );
#endif

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_SYNC_PCLK_DIV4, UT_ADC_COMMON->CCR & ADC_CCR_ADCPRE );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockDivider( &clkDiv ) );
    TEST_ASSERT_EQUAL( ADC_CLK_DIV_4, clkDiv );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR2 & ADC_CR2_ADON );
}


/**
 * \brief   Adc_Init() rejects used slot with different peripheral identification.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR, ADC1 is not enabled.
 */
void Ut_Adc_Init_PeripheralIdMismatch_ReturnsError( void )
{
    adc_Config_t config = Ut_Adc_Get_Config();

    Ut_Adc_Ignore_PeriphMocks();

    config.PeriphConfig[ UT_ADC_PERIPH ]          = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.PeriphConfig[ UT_ADC_PERIPH ].PeriphId = (adc_PeriphId_t)( ADC_PERIPH_CNT - 1u );

#if defined (ADC2)
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR2 & ADC_CR2_ADON );
#else
    TEST_IGNORE_MESSAGE( "Single ADC device" );
#endif
}


/**
 * \brief   Adc_Init() with VREF channel of ADC1 in polling mode enables the peripheral.
 *
 * \par Expected results
 * - ADC_REQUEST_OK, ADON = 1.
 * - SQR3 rank 1 = channel 17, sampling time of channel 17 = 480 cycles.
 * - TSVREFE enabled, scan mode, EOC per conversion, right alignment.
 * - Channel input reads back VREF.
 */
void Ut_Adc_Init_VrefPoll_PeripheralEnabled( void )
{
    adc_Config_t       config = Ut_Adc_Get_Config();
    adc_ChannelInput_t input  = ADC_CHANNEL_INPUT_CNT;

    Ut_Adc_Ignore_PeriphMocks();
    config.PeriphConfig[ UT_ADC_PERIPH ] = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_BITS_HIGH( ADC_CR2_ADON, UT_ADC_REG->CR2 );
    TEST_ASSERT_EQUAL_UINT32( UT_ADC_VREF_CHANNEL, UT_ADC_REG->SQR3 & ADC_SQR3_SQ1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->SQR1 & ADC_SQR1_L );
    TEST_ASSERT_EQUAL_HEX32( ADC_SMPR1_SMP17, UT_ADC_REG->SMPR1 & ADC_SMPR1_SMP17 );
    TEST_ASSERT_BITS_HIGH( ADC_CCR_TSVREFE, UT_ADC_COMMON->CCR );
    TEST_ASSERT_BITS_HIGH( ADC_CR1_SCAN, UT_ADC_REG->CR1 );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_EOCS, UT_ADC_REG->CR2 );
    TEST_ASSERT_BITS_LOW( ADC_CR2_ALIGN | ADC_CR2_EXTEN | ADC_CR2_CONT | ADC_CR2_DMA, UT_ADC_REG->CR2 );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ChannelInput( UT_ADC_PERIPH, UT_ADC_VREF_CHANNEL, &input ) );
    TEST_ASSERT_EQUAL( ADC_CHANNEL_INPUT_VREF, input );
}


/**
 * \brief   Adc_Init() is refused while an ADC peripheral is enabled.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR, CCR is not modified.
 */
void Ut_Adc_Init_PeripheralEnabled_ReturnsErrorWithoutChange( void )
{
    adc_Config_t config = Ut_Adc_Get_Config();

    Ut_Adc_Ignore_PeriphMocks();
    config.PeriphConfig[ UT_ADC_PERIPH ] = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    UT_ADC_REG->CR2    = ADC_CR2_ADON;
    UT_ADC_COMMON->CCR = LL_ADC_CLOCK_SYNC_PCLK_DIV8;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_SYNC_PCLK_DIV8, UT_ADC_COMMON->CCR );
}

/* =========================== CLOCK CONFIGURATION ========================== */

/**
 * \brief   Adc_Set_ClockSource() rejects invalid source and enabled ADC, Adc_Get_ClockSource()
 *          returns PCLK2.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR without RCC access for invalid source / enabled ADC.
 * - Clock source reads back ADC_CLK_SRC_PCLK2.
 */
void Ut_Adc_Set_ClockSource_InvalidOrEnabled_ReturnsError( void )
{
    adc_ClkSrc_t clkSrc = ADC_CLK_SRC_CNT;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockSource( ADC_CLK_SRC_CNT ) );

    UT_ADC_REG->CR2 = ADC_CR2_ADON;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockSource( ADC_CLK_SRC_PCLK2 ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_ClockSource( NULL ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( ADC_CLK_SRC_PCLK2, clkSrc );
}


/**
 * \brief   Adc_Set_ClockDivider() writes ADCPRE for dividers within the frequency limits.
 *
 * \details PCLK2 = 84 MHz: /4, /6, /8 are within 0.6 - 36 MHz, /2 is above.
 *
 * \par Expected results
 * - /4, /6, /8: ADC_REQUEST_OK, ADCPRE and read back match.
 * - /2 and invalid divider: ADC_REQUEST_ERROR, CCR not modified.
 * - Enabled ADC: ADC_REQUEST_ERROR, CCR not modified.
 */
void Ut_Adc_Set_ClockDivider_Options_RegisterAndReadBack( void )
{
    const adc_ClkDiv_t validDiv[]   = { ADC_CLK_DIV_4, ADC_CLK_DIV_6, ADC_CLK_DIV_8 };
    const uint32_t     validLlDiv[] = { LL_ADC_CLOCK_SYNC_PCLK_DIV4, LL_ADC_CLOCK_SYNC_PCLK_DIV6, LL_ADC_CLOCK_SYNC_PCLK_DIV8 };
    adc_ClkDiv_t       clkDiv       = ADC_CLK_DIV_CNT;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    for( uint32_t idx = 0u; ( sizeof( validDiv ) / sizeof( validDiv[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockDivider( validDiv[ idx ] ) );
        TEST_ASSERT_EQUAL_HEX32( validLlDiv[ idx ], UT_ADC_COMMON->CCR & ADC_CCR_ADCPRE );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockDivider( &clkDiv ) );
        TEST_ASSERT_EQUAL( validDiv[ idx ], clkDiv );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_SYNC_PCLK_DIV8, UT_ADC_COMMON->CCR & ADC_CCR_ADCPRE );

    UT_ADC_REG->CR2 = ADC_CR2_ADON;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_4 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_SYNC_PCLK_DIV8, UT_ADC_COMMON->CCR & ADC_CCR_ADCPRE );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_ClockDivider( NULL ) );
}

/* ======================== PERIPHERAL CONFIGURATION ======================== */

/**
 * \brief   Adc_PeriphInit() rejects invalid configurations before any register access.
 *
 * \details NULL, regular sequence length 17, no channel, invalid trigger, resolution,
 *          edge and injected sequence length 5.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR, ADC1 registers stay zero (no mock call expected).
 */
void Ut_Adc_PeriphInit_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
    adc_PeriphConfig_t config;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Ut_Adc_Set_ClockDiv4();

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( NULL ) );

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.RegChannelsCnt = ADC_REG_SEQUENCE_CNT + 1u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.RegChannelsCnt = 0u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.RegTriggerId = ADC_REG_TRIGGER_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.Resolution = ADC_RESOLUTION_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.RegTriggerEdge = ADC_TRIGGER_EDGE_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.InjChannelsCnt = ADC_INJ_SEQUENCE_CNT + 1u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, 0u );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR2 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->SQR3 );
}


/**
 * \brief   Adc_PeriphInit() rejects invalid channel slots before any register access.
 *
 * \details VREF on channel 3, temperature sensor on ADC2, pin input on channel 16 (no pin),
 *          VREF sampling 112 cycles (5.3 us < 10 us), same channel with different sampling.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR, no GPIO initialization, ADC1 not enabled.
 */
void Ut_Adc_PeriphInit_InvalidChannels_ReturnsErrorWithoutAccess( void )
{
    adc_PeriphConfig_t config;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Ut_Adc_Set_ClockDiv4();

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.RegChannels[ 0u ].ChannelId = ADC_CHANNEL_3;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

#if defined (ADC2)
    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.PeriphId                       = ADC_PERIPH_2;
    config.RegChannels[ 0u ].ChannelId    = ADC_CHANNEL_16;
    config.RegChannels[ 0u ].ChannelInput = ADC_CHANNEL_INPUT_TEMP;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );
#endif

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.RegChannels[ 0u ].ChannelId    = ADC_CHANNEL_16;
    config.RegChannels[ 0u ].ChannelInput = ADC_CHANNEL_INPUT_PIN_SINGLE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_112_CYCLES;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.RegChannelsCnt                    = 2u;
    config.RegChannels[ 1u ]                 = config.RegChannels[ 0u ];
    config.RegChannels[ 1u ].ChannelSampling = ADC_CHANNEL_SAMPLING_3_CYCLES;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_GpioInitCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR2 );
}


/**
 * \brief   Adc_PeriphInit() configures GPIO pins of external channels and the regular sequence.
 *
 * \details Regular sequence channel 0 (PA0), channel 8 (PB0), channel 15 (PC5).
 *
 * \par Expected results
 * - 3 GPIO initializations, the last one PC5 analog without pull.
 * - SQR3: rank 1 = 0, rank 2 = 8, rank 3 = 15, SQR1 L = 2 (3 conversions).
 * - Sampling time of channel 8 (SMPR2) = 3 cycles.
 */
void Ut_Adc_PeriphInit_PinChannels_GpioAnalogAndSequence( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    config.RegChannelsCnt = 3u;
    config.RegChannels[ 0u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_0,  ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_15_CYCLES };
    config.RegChannels[ 1u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_8,  ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_3_CYCLES  };
    config.RegChannels[ 2u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_15, ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_56_CYCLES };

    UT_ADC_REG->SMPR2 = ADC_SMPR2_SMP8;
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL_UINT32( 3u, utAdc_GpioInitCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_C, utAdc_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_5, utAdc_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ANALOG, utAdc_GpioConfig.PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE, utAdc_GpioConfig.PinPull );

    TEST_ASSERT_EQUAL_HEX32( ( 0u << ADC_SQR3_SQ1_Pos ) | ( 8u << ADC_SQR3_SQ2_Pos ) | ( 15u << ADC_SQR3_SQ3_Pos ), UT_ADC_REG->SQR3 );
    TEST_ASSERT_EQUAL_HEX32( 2u << ADC_SQR1_L_Pos, UT_ADC_REG->SQR1 & ADC_SQR1_L );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->SMPR2 & ADC_SMPR2_SMP8 );
}


/**
 * \brief   Adc_PeriphInit() with external trigger and continuous mode keeps them for the start.
 *
 * \details Regular trigger TIM2 TRGO, falling edge, continuous mode.
 *
 * \par Expected results
 * - EXTSEL = TIM2 TRGO, EXTEN = 0 and CONT = 0 (conversion not started).
 * - Trigger source, edge and mode read back the configuration.
 */
void Ut_Adc_PeriphInit_ExternalTriggerContinuous_KeptUntilStart( void )
{
    adc_PeriphConfig_t   config      = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    adc_RegTriggerId_t   triggerSrc  = ADC_REG_TRIGGER_CNT;
    adc_TriggerEdge_t    triggerEdge = ADC_TRIGGER_EDGE_CNT;
    adc_RegTriggerMode_t triggerMode = ADC_REG_TRIGGER_MODE_CNT;

    config.RegTriggerId   = ADC_REG_TRIGGER_EXT_TIM2_TRGO;
    config.RegTriggerEdge = ADC_TRIGGER_EDGE_FALLING;
    config.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_TIM2_TRGO & ADC_CR2_EXTSEL, UT_ADC_REG->CR2 & ADC_CR2_EXTSEL );
    TEST_ASSERT_BITS_LOW( ADC_CR2_EXTEN | ADC_CR2_CONT, UT_ADC_REG->CR2 );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerSrc( UT_ADC_PERIPH, &triggerSrc ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_EXT_TIM2_TRGO, triggerSrc );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerEdge( UT_ADC_PERIPH, &triggerEdge ) );
    TEST_ASSERT_EQUAL( ADC_TRIGGER_EDGE_FALLING, triggerEdge );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerMode( UT_ADC_PERIPH, &triggerMode ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_MODE_CONTINUOUS, triggerMode );
}


/**
 * \brief   Adc_PeriphInit() configures injected sequence aligned to the end of JSQR.
 *
 * \details Two injected channels 17 (rank 1) and 0 (rank 2), trigger TIM1 TRGO, single mode.
 *
 * \par Expected results
 * - JSQR: JL = 1, JSQ3 = 17, JSQ4 = 0, ranks read back.
 * - JEXTSEL = TIM1 TRGO, JEXTEN = 0, JDISCEN = 1, JAUTO = 0.
 */
void Ut_Adc_PeriphInit_InjectedSequence_JsqrConfigured( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    config.InjChannelsCnt    = 2u;
    config.InjChannels[ 0u ] = config.RegChannels[ 0u ];
    config.InjChannels[ 1u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_0, ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_3_CYCLES };
    config.InjTriggerId      = ADC_INJ_TRIGGER_EXT_TIM1_TRGO;
    config.InjTriggerMode    = ADC_INJ_TRIGGER_MODE_SINGLE;
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL_HEX32( ( 1u << ADC_JSQR_JL_Pos ) | ( 17u << ADC_JSQR_JSQ3_Pos ) | ( 0u << ADC_JSQR_JSQ4_Pos ), UT_ADC_REG->JSQR );
    TEST_ASSERT_EQUAL_UINT32( 17u, __LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_INJ_GetSequencerRanks( UT_ADC_REG, LL_ADC_INJ_RANK_1 ) ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ADC_CR2_JEXTSEL, UT_ADC_REG->CR2 & ADC_CR2_JEXTSEL );
    TEST_ASSERT_BITS_LOW( ADC_CR2_JEXTEN, UT_ADC_REG->CR2 );
    TEST_ASSERT_BITS_HIGH( ADC_CR1_JDISCEN, UT_ADC_REG->CR1 );
    TEST_ASSERT_BITS_LOW( ADC_CR1_JAUTO, UT_ADC_REG->CR1 );
}


/**
 * \brief   Adc_PeriphInit() with ADC_INJ_TRIGGER_AUTO sets auto-injected mode and rejects
 *          invalid auto-injected configurations.
 *
 * \par Expected results
 * - Auto-injected with regular group: ADC_REQUEST_OK, JAUTO = 1, JDISCEN = 0.
 * - Adc_Set_InjStart() is refused in auto-injected mode.
 * - Without regular group or with single injected mode: ADC_REQUEST_ERROR.
 */
void Ut_Adc_PeriphInit_AutoInjected_TrigAutoEnabled( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    config.InjChannelsCnt    = 1u;
    config.InjChannels[ 0u ] = config.RegChannels[ 0u ];
    config.InjTriggerId      = ADC_INJ_TRIGGER_AUTO;

    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ClockDiv4();

    config.RegChannelsCnt = 0u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config.RegChannelsCnt = 1u;
    config.InjTriggerMode = ADC_INJ_TRIGGER_MODE_SINGLE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config.InjTriggerMode = ADC_INJ_TRIGGER_MODE_CONTINUOUS;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( &config ) );

    TEST_ASSERT_BITS_HIGH( ADC_CR1_JAUTO, UT_ADC_REG->CR1 );
    TEST_ASSERT_BITS_LOW( ADC_CR1_JDISCEN, UT_ADC_REG->CR1 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_InjStart( UT_ADC_PERIPH ) );
}


/**
 * \brief   Adc_PeriphInit() refuses enabled peripheral without register change.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR, SQR3 not modified, ADC stays enabled.
 */
void Ut_Adc_PeriphInit_PeripheralEnabled_ReturnsErrorWithoutChange( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ClockDiv4();
    UT_ADC_REG->CR2 = ADC_CR2_ADON;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->SQR3 );
    TEST_ASSERT_EQUAL_HEX32( ADC_CR2_ADON, UT_ADC_REG->CR2 );
}


/**
 * \brief   Adc_Deinit() disables the peripheral and resets configuration kept by the module.
 *
 * \par Expected results
 * - ADC_REQUEST_OK, ADON = 0, NVIC interrupt disabled (ISR mode).
 * - Channel input reads back PIN_SINGLE, trigger source reads back SOFTWARE.
 */
void Ut_Adc_Deinit_InitializedPeripheral_DisabledAndShadowReset( void )
{
    adc_PeriphConfig_t config     = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, UT_ADC_BUF_SIZE );
    adc_ChannelInput_t input      = ADC_CHANNEL_INPUT_CNT;
    adc_RegTriggerId_t triggerSrc = ADC_REG_TRIGGER_CNT;

    config.RegTriggerId = ADC_REG_TRIGGER_EXT_TIM1_CH1;
    Ut_Adc_PeriphInit( &config );
    Ut_Adc_Reset_Mocks();

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_ADC_NVIC, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( UT_ADC_PERIPH ) );

    TEST_ASSERT_BITS_LOW( ADC_CR2_ADON, UT_ADC_REG->CR2 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ChannelInput( UT_ADC_PERIPH, UT_ADC_VREF_CHANNEL, &input ) );
    TEST_ASSERT_EQUAL( ADC_CHANNEL_INPUT_PIN_SINGLE, input );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerSrc( UT_ADC_PERIPH, &triggerSrc ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_SOFTWARE, triggerSrc );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Deinit( ADC_PERIPH_CNT ) );
}

/* =========================== PERIPHERAL CONTROL =========================== */

/**
 * \brief   Adc_Set_PeriphActive() / Adc_Set_PeriphInactive() control ADON.
 *
 * \par Expected results
 * - Active: ADON = 1, repeated activation refused.
 * - Inactive: ADON = 0, refused while continuous conversion runs.
 * - Active refused with ADC clock out of range (PCLK2 / 2).
 */
void Ut_Adc_Set_PeriphActive_EnableDisable_AdonControlled( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Ut_Adc_Set_ClockDiv4();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_PeriphActive( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_ADON, UT_ADC_REG->CR2 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_PeriphActive( UT_ADC_PERIPH ) );

    UT_ADC_REG->CR2 |= ADC_CR2_CONT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_PeriphInactive( UT_ADC_PERIPH ) );
    UT_ADC_REG->CR2 &= ~ADC_CR2_CONT;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_PeriphInactive( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_LOW( ADC_CR2_ADON, UT_ADC_REG->CR2 );

    UT_ADC_COMMON->CCR = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_PeriphActive( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_LOW( ADC_CR2_ADON, UT_ADC_REG->CR2 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_PeriphActive( ADC_PERIPH_CNT ) );
}


/**
 * \brief   Adc_Set_RegStart() requires enabled ADC.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR, SWSTART not set.
 */
void Ut_Adc_Set_RegStart_NotEnabled_ReturnsErrorWithoutStart( void )
{
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_LOW( ADC_CR2_SWSTART, UT_ADC_REG->CR2 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_RegStart( ADC_PERIPH_CNT ) );
}


/**
 * \brief   Software start with manual polling (POLL mode without buffer).
 *
 * \par Expected results
 * - Adc_Set_RegStart(): SWSTART set, CONT / EXTEN stay 0 (single software conversion).
 * - Adc_Get_RegData() returns DR value.
 * - Repeated start is accepted (single software conversion is not "ongoing").
 */
void Ut_Adc_Set_RegStart_ManualPolling_SwStartAndDataRead( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    adc_Data_t         data   = 0u;

    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_SWSTART, UT_ADC_REG->CR2 );
    TEST_ASSERT_BITS_LOW( ADC_CR2_CONT | ADC_CR2_EXTEN, UT_ADC_REG->CR2 );

    UT_ADC_REG->DR = 0x5A5u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_RegData( UT_ADC_PERIPH, &data ) );
    TEST_ASSERT_EQUAL_HEX16( 0x5A5u, data );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_RegData( UT_ADC_PERIPH, NULL ) );
}


/**
 * \brief   Continuous software conversion is ongoing until Adc_Set_RegStop().
 *
 * \par Expected results
 * - Start: CONT = 1 and SWSTART = 1.
 * - While running: resolution change and repeated start refused.
 * - Stop: CONT = 0, resolution change accepted.
 */
void Ut_Adc_Set_RegStart_Continuous_RunningUntilStop( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    config.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_CONT | ADC_CR2_SWSTART, UT_ADC_REG->CR2 );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_Resolution( UT_ADC_PERIPH, ADC_RESOLUTION_10BIT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_LOW( ADC_CR2_CONT, UT_ADC_REG->CR2 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_Resolution( UT_ADC_PERIPH, ADC_RESOLUTION_10BIT ) );
}


/**
 * \brief   External trigger is enabled by start and disabled by stop.
 *
 * \details Trigger TIM3 TRGO, both edges, single mode.
 *
 * \par Expected results
 * - Start: EXTEN = both edges, SWSTART not set, CONT = 0.
 * - Stop: EXTEN = 0, EXTSEL kept.
 */
void Ut_Adc_Set_RegStart_ExternalTrigger_ExtenControlled( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    config.RegTriggerId   = ADC_REG_TRIGGER_EXT_TIM3_TRGO;
    config.RegTriggerEdge = ADC_TRIGGER_EDGE_BOTH;
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_RISINGFALLING, UT_ADC_REG->CR2 & ADC_CR2_EXTEN );
    TEST_ASSERT_BITS_LOW( ADC_CR2_SWSTART | ADC_CR2_CONT, UT_ADC_REG->CR2 );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_LOW( ADC_CR2_EXTEN, UT_ADC_REG->CR2 );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_TIM3_TRGO & ADC_CR2_EXTSEL, UT_ADC_REG->CR2 & ADC_CR2_EXTSEL );
}


/**
 * \brief   Trigger source, edge and mode setters handle software / external triggers.
 *
 * \par Expected results
 * - Edge refused for software trigger, edge read back refused for software trigger.
 * - Change from software to external trigger uses rising edge.
 * - Change between external triggers keeps the edge.
 * - All setters refused while external trigger is running and for invalid values.
 */
void Ut_Adc_Set_Trigger_SoftwareExternal_EdgeAndModeHandled( void )
{
    adc_TriggerEdge_t  triggerEdge = ADC_TRIGGER_EDGE_CNT;
    adc_RegTriggerId_t triggerSrc  = ADC_REG_TRIGGER_CNT;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerEdge( UT_ADC_PERIPH, ADC_TRIGGER_EDGE_FALLING ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_TriggerEdge( UT_ADC_PERIPH, &triggerEdge ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_EXT_TIM1_CH1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerSrc( UT_ADC_PERIPH, &triggerSrc ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_EXT_TIM1_CH1, triggerSrc );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerEdge( UT_ADC_PERIPH, &triggerEdge ) );
    TEST_ASSERT_EQUAL( ADC_TRIGGER_EDGE_RISING, triggerEdge );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerEdge( UT_ADC_PERIPH, ADC_TRIGGER_EDGE_FALLING ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_EXT_EXTI_LINE11 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_EXTI_LINE11 & ADC_CR2_EXTSEL, UT_ADC_REG->CR2 & ADC_CR2_EXTSEL );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerEdge( UT_ADC_PERIPH, &triggerEdge ) );
    TEST_ASSERT_EQUAL( ADC_TRIGGER_EDGE_FALLING, triggerEdge );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_SOFTWARE ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerSrc( UT_ADC_PERIPH, &triggerSrc ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_SOFTWARE, triggerSrc );

    UT_ADC_REG->CR2 |= ADC_CR2_JEXTEN_0;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_EXT_TIM1_CH2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerMode( UT_ADC_PERIPH, ADC_REG_TRIGGER_MODE_CONTINUOUS ) );
    UT_ADC_REG->CR2 &= ~ADC_CR2_JEXTEN;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerMode( UT_ADC_PERIPH, ADC_REG_TRIGGER_MODE_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_TriggerSrc( UT_ADC_PERIPH, NULL ) );
}


/**
 * \brief   Injected start by software and by external trigger, injected stop.
 *
 * \par Expected results
 * - Software trigger: JSWSTART set.
 * - External trigger (TIM1 TRGO, falling): JEXTEN = falling, repeated start refused,
 *   Adc_Set_InjStop() clears JEXTEN.
 * - Injected data of rank 2 is read from JDR2.
 */
void Ut_Adc_Set_InjStart_SoftwareAndExternal_StartedAndStopped( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    adc_Data_t         data   = 0u;

    config.InjChannelsCnt    = 2u;
    config.InjChannels[ 0u ] = config.RegChannels[ 0u ];
    config.InjChannels[ 1u ] = config.RegChannels[ 0u ];
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_JSWSTART, UT_ADC_REG->CR2 );

    UT_ADC_REG->JDR2 = 0x321u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_InjData( UT_ADC_PERIPH, ADC_INJ_SEQUENCE_2, &data ) );
    TEST_ASSERT_EQUAL_HEX16( 0x321u, data );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_InjData( UT_ADC_PERIPH, ADC_INJ_SEQUENCE_CNT, &data ) );

    /* External trigger configured by a new initialization */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( UT_ADC_PERIPH ) );
    config.InjTriggerId   = ADC_INJ_TRIGGER_EXT_TIM1_TRGO;
    config.InjTriggerEdge = ADC_TRIGGER_EDGE_FALLING;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_INJ_TRIG_EXT_FALLING, UT_ADC_REG->CR2 & ADC_CR2_JEXTEN );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_InjStart( UT_ADC_PERIPH ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStop( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_LOW( ADC_CR2_JEXTEN, UT_ADC_REG->CR2 );
}

/* ============================= DATA TRANSFER ============================== */

/**
 * \brief   Polling mode with one shot buffer collects results, reports callbacks and stops.
 *
 * \details Buffer of 4 samples, continuous software conversion, EOC preset before every
 *          Adc_Task() call.
 *
 * \par Expected results
 * - Buffer contains DR values, half callback 1x, complete callback 1x.
 * - Conversion stopped (CONT = 0), Adc_Get_RegData() allowed again.
 */
void Ut_Adc_Task_PollOneShot_BufferFilledCallbacksAndStop( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 4u );
    adc_Data_t         data   = 0u;

    config.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_RegData( UT_ADC_PERIPH, &data ) );

    for( uint32_t sampleIdx = 0u; 4u > sampleIdx; sampleIdx++ )
    {
        UT_ADC_REG->SR = ADC_SR_EOC;
        UT_ADC_REG->DR = 0x100u + sampleIdx;
        Adc_Task();
    }

    TEST_ASSERT_EQUAL_HEX16( 0x100u, utAdc_Buffer[ 0u ] );
    TEST_ASSERT_EQUAL_HEX16( 0x103u, utAdc_Buffer[ 3u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_ErrorCnt );
    TEST_ASSERT_BITS_LOW( ADC_CR2_CONT, UT_ADC_REG->CR2 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_RegData( UT_ADC_PERIPH, &data ) );
}


/**
 * \brief   Polling mode with circular buffer wraps to the buffer start.
 *
 * \par Expected results
 * - 5th result is stored to DataBuffer[ 0 ], complete callback 1x, conversion keeps running.
 */
void Ut_Adc_Task_PollCircular_WrapsToBufferStart( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 4u );

    config.RegTriggerMode        = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    config.DataConfig.BufferMode = ADC_BUFFER_MODE_CIRCULAR;
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    for( uint32_t sampleIdx = 0u; 5u > sampleIdx; sampleIdx++ )
    {
        UT_ADC_REG->SR = ADC_SR_EOC;
        UT_ADC_REG->DR = 0x200u + sampleIdx;
        Adc_Task();
    }

    TEST_ASSERT_EQUAL_HEX16( 0x204u, utAdc_Buffer[ 0u ] );
    TEST_ASSERT_EQUAL_HEX16( 0x201u, utAdc_Buffer[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_CompleteCnt );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_CONT, UT_ADC_REG->CR2 );
}


/**
 * \brief   Polling mode reports overrun and injected end of sequence.
 *
 * \par Expected results
 * - Error callback with ADC_ERROR_OVERRUN, injected callback 1x.
 * - OVR / JEOC cleared by write of 0 (other emulated SR bits written with 1).
 */
void Ut_Adc_Task_PollOverrunAndInjected_Callbacks( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    config.InjChannelsCnt                 = 1u;
    config.InjChannels[ 0u ]              = config.RegChannels[ 0u ];
    config.DataConfig.InjCompleteCallback = Ut_Adc_InjCallback;
    Ut_Adc_PeriphInit( &config );

    UT_ADC_REG->SR = ADC_SR_OVR;
    Adc_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_ERROR_OVERRUN, utAdc_LastError );
    TEST_ASSERT_EQUAL_HEX32( UT_ADC_SR_CLEARED( ADC_SR_OVR ), UT_ADC_REG->SR );

    UT_ADC_REG->SR = ADC_SR_JEOC;
    Adc_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_InjCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_ADC_SR_CLEARED( ADC_SR_JEOC | ADC_SR_JSTRT ), UT_ADC_REG->SR );
}


/**
 * \brief   ISR mode enables EOC / OVR interrupts on start and stores results in ISR.
 *
 * \par Expected results
 * - Initialization registers the common ADC ISR in NVIC.
 * - Start: EOCIE and OVRIE set.
 * - 8 ISR calls fill the buffer, complete callback 1x, EOCIE / OVRIE cleared after the stop.
 */
void Ut_Adc_Isr_Transfer_InterruptsEnabledDataStored( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, UT_ADC_BUF_SIZE );

    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_NOT_NULL( utAdc_Isr );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_HIGH( ADC_CR1_EOCIE | ADC_CR1_OVRIE, UT_ADC_REG->CR1 );

    for( uint32_t sampleIdx = 0u; UT_ADC_BUF_SIZE > sampleIdx; sampleIdx++ )
    {
        Ut_Adc_Call_Isr( ADC_SR_EOC, 0x300u + sampleIdx );
    }

    TEST_ASSERT_EQUAL_HEX16( 0x307u, utAdc_Buffer[ 7u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_CompleteCnt );
    TEST_ASSERT_BITS_LOW( ADC_CR1_EOCIE | ADC_CR1_OVRIE, UT_ADC_REG->CR1 );
}


/**
 * \brief   ISR handles only enabled sources, overrun in ISR mode keeps the conversion running.
 *
 * \par Expected results
 * - JEOC without JEOCIE (no InjCompleteCallback): no callback.
 * - OVR: error callback ADC_ERROR_OVERRUN, CONT stays set.
 */
void Ut_Adc_Isr_OverrunAndDisabledSources_HandledByEnableMask( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, UT_ADC_BUF_SIZE );

    config.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    Ut_Adc_Call_Isr( ADC_SR_JEOC, 0u );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_InjCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_ErrorCnt );

    Ut_Adc_Call_Isr( ADC_SR_OVR, 0u );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_ERROR_OVERRUN, utAdc_LastError );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_CONT, UT_ADC_REG->CR2 );
}


/**
 * \brief   Common ADC interrupt is disabled in NVIC only when the last peripheral releases it.
 *
 * \par Expected results
 * - Deinit of ADC1 (ADC2 still in ISR mode): no Nvic_Set_PeriphIrq_Inactive call.
 * - Deinit of ADC2: Nvic_Set_PeriphIrq_Inactive called.
 */
void Ut_Adc_Isr_SharedInterrupt_DisabledByLastUser( void )
{
#if defined (ADC2)
    adc_PeriphConfig_t config1 = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, UT_ADC_BUF_SIZE );
    adc_PeriphConfig_t config2 = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, UT_ADC_BUF_SIZE );

    config2.PeriphId          = ADC_PERIPH_2;
    config2.RegChannels[ 0u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_1, ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_3_CYCLES };

    Ut_Adc_PeriphInit( &config1 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( &config2 ) );
    Ut_Adc_Reset_Mocks();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( ADC_PERIPH_1 ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_ADC_NVIC, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( ADC_PERIPH_2 ) );
#else
    TEST_IGNORE_MESSAGE( "Single ADC device" );
#endif
}


/**
 * \brief   DMA mode initialization configures ADC DMA requests and the DMA stream.
 *
 * \details ADC1, DMA2 stream 4, circular buffer of 8 samples.
 *
 * \par Expected results
 * - Dma_Init(): DMA2 stream 4, channel 0, peripheral to memory, circular, 16 bit, count 8,
 *   peripheral address of ADC1 DR, memory address of the buffer.
 * - ADC CR2 DMA = 1, DDS = 1.
 */
void Ut_Adc_Dma_Init_StreamConfigured( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    config.DataConfig.BufferMode   = ADC_BUFFER_MODE_CIRCULAR;
    config.DataConfig.DmaChannelId = ADC_DMA_CHANNEL_4;
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_DmaInitCnt );
    TEST_ASSERT_EQUAL( DMA_PERIPH_2, utAdc_DmaConfig.DmaPeriphId );
    TEST_ASSERT_EQUAL( DMA_STREAM_4, utAdc_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_0, utAdc_DmaConfig.PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_PERIPH_TO_MEMORY, utAdc_DmaConfig.Direction );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_CIRCULAR, utAdc_DmaConfig.TransferMode );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, utAdc_DmaConfig.PeriphTransferSize );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, utAdc_DmaConfig.MemoryTransferSize );
    TEST_ASSERT_EQUAL_UINT32( UT_ADC_BUF_SIZE, utAdc_DmaConfig.DataCount );
    TEST_ASSERT_EQUAL( (dma_PeriphAddr_t)(uintptr_t)&UT_ADC_REG->DR, utAdc_DmaConfig.PeriphAddress );
    TEST_ASSERT_EQUAL( (dma_MemoryAddr_t)(uintptr_t)utAdc_Buffer, utAdc_DmaConfig.MemoryAddress );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_DMA | ADC_CR2_DDS, UT_ADC_REG->CR2 );
}


/**
 * \brief   DMA mode rejects stream not connected to the ADC request.
 *
 * \par Expected results
 * - DMA1 stream 0 and DMA2 stream 7 for ADC1: ADC_REQUEST_ERROR, Dma_Init() not called.
 */
void Ut_Adc_Dma_Init_InvalidStream_ReturnsError( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Ut_Adc_Set_ClockDiv4();

    config.DataConfig.DmaPeriphId = ADC_DMA_PERIPH_1;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config.DataConfig.DmaPeriphId  = ADC_DMA_PERIPH_2;
    config.DataConfig.DmaChannelId = ADC_DMA_CHANNEL_7;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_DmaInitCnt );
}


/**
 * \brief   DMA callbacks registered in Dma_Init() report the transfer events.
 *
 * \details One shot buffer, continuous conversion started.
 *
 * \par Expected results
 * - Half transfer callback 1x, transfer error callback ADC_ERROR_DMA_TRANSFER.
 * - Transfer complete: complete callback 1x, conversion stopped (CONT = 0).
 */
void Ut_Adc_Dma_Callbacks_EventsReported( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    config.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_HIGH( ADC_CR1_OVRIE, UT_ADC_REG->CR1 );

    utAdc_DmaConfig.HalfTransferCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_HalfCnt );

    utAdc_DmaConfig.TransferErrorCallback();
    TEST_ASSERT_EQUAL( ADC_ERROR_DMA_TRANSFER, utAdc_LastError );

    utAdc_DmaConfig.TransferCompleteCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_CompleteCnt );
    TEST_ASSERT_BITS_LOW( ADC_CR2_CONT, UT_ADC_REG->CR2 );
    TEST_ASSERT_BITS_LOW( ADC_CR1_OVRIE, UT_ADC_REG->CR1 );
}


/**
 * \brief   Overrun in DMA mode stops the regular conversion and the DMA transfer.
 *
 * \par Expected results
 * - Error callback ADC_ERROR_OVERRUN, CONT = 0, OVRIE = 0.
 * - Conversion can be started again.
 */
void Ut_Adc_Isr_DmaOverrun_RegularGroupStopped( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    config.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    Ut_Adc_Call_Isr( ADC_SR_OVR, 0u );

    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_ERROR_OVERRUN, utAdc_LastError );
    TEST_ASSERT_BITS_LOW( ADC_CR2_CONT, UT_ADC_REG->CR2 );
    TEST_ASSERT_BITS_LOW( ADC_CR1_OVRIE, UT_ADC_REG->CR1 );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_CONT, UT_ADC_REG->CR2 );
}


/**
 * \brief   Adc_Set_DataConfig() changes the transfer mode, Adc_Get_DataConfig() returns it.
 *
 * \par Expected results
 * - POLL -> ISR: ADC_REQUEST_OK, NVIC interrupt configured, read back ISR mode.
 * - Refused while the transfer runs, for invalid configuration and NULL.
 */
void Ut_Adc_Set_DataConfig_ChangeModeAndInvalid_Handled( void )
{
    adc_PeriphConfig_t config     = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 4u );
    adc_DataConfig_t   dataConfig = config.DataConfig;
    adc_DataConfig_t   readConfig;

    Ut_Adc_PeriphInit( &config );

    dataConfig.TransferMode = ADC_TRANSFER_MODE_ISR;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_DataConfig( UT_ADC_PERIPH, &dataConfig ) );
    TEST_ASSERT_NOT_NULL( utAdc_Isr );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_DataConfig( UT_ADC_PERIPH, &readConfig ) );
    TEST_ASSERT_EQUAL( ADC_TRANSFER_MODE_ISR, readConfig.TransferMode );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_DataConfig( UT_ADC_PERIPH, &dataConfig ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( UT_ADC_PERIPH ) );

    dataConfig.DataBuffer = NULL;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_DataConfig( UT_ADC_PERIPH, &dataConfig ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_DataConfig( UT_ADC_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_DataConfig( UT_ADC_PERIPH, NULL ) );
}

/* ============================ FLAGS AND CHANNELS ========================== */

/**
 * \brief   Adc_Get_Flag() reports SR flags, Adc_Clear_Flag() clears by write of 0.
 *
 * \par Expected results
 * - Every flag is reported ACTIVE when its SR bit is set and INACTIVE otherwise.
 * - Clear writes inverted flag mask to SR.
 * - Invalid flag / peripheral / NULL: ADC_REQUEST_ERROR.
 */
void Ut_Adc_Get_Flag_AllFlags_ReportedAndCleared( void )
{
    const uint32_t  srBits[ ADC_FLAG_CNT ] = { ADC_SR_EOC, ADC_SR_OVR, ADC_SR_STRT, ADC_SR_JEOC, ADC_SR_JSTRT, ADC_SR_AWD };
    adc_FlagState_t flagState              = ADC_FLAG_INACTIVE;

    for( adc_FlagId_t flagId = ADC_FLAG_REG_EOC; ADC_FLAG_CNT > flagId; flagId++ )
    {
        UT_ADC_REG->SR = srBits[ flagId ];
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( UT_ADC_PERIPH, flagId, &flagState ) );
        TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, flagState );

        UT_ADC_REG->SR = ~srBits[ flagId ];
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( UT_ADC_PERIPH, flagId, &flagState ) );
        TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, flagState );

        UT_ADC_REG->SR = 0u;
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( UT_ADC_PERIPH, flagId ) );
        TEST_ASSERT_EQUAL_HEX32( ~srBits[ flagId ], UT_ADC_REG->SR );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_Flag( UT_ADC_PERIPH, ADC_FLAG_CNT, &flagState ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_Flag( UT_ADC_PERIPH, ADC_FLAG_AWD1, NULL ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Clear_Flag( ADC_PERIPH_CNT, ADC_FLAG_AWD1 ) );
}


/**
 * \brief   Adc_Set_Resolution() writes RES for every option and reads it back.
 *
 * \par Expected results
 * - RES field and read back match for all options, invalid option refused.
 */
void Ut_Adc_Set_Resolution_AllOptions_RegisterAndReadBack( void )
{
    const uint32_t   llRes[ ADC_RESOLUTION_CNT ] = { LL_ADC_RESOLUTION_12B, LL_ADC_RESOLUTION_10B, LL_ADC_RESOLUTION_8B, LL_ADC_RESOLUTION_6B };
    adc_Resolution_t resolution                  = ADC_RESOLUTION_CNT;

    for( adc_Resolution_t resIdx = ADC_RESOLUTION_12BIT; ADC_RESOLUTION_CNT > resIdx; resIdx++ )
    {
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_Resolution( UT_ADC_PERIPH, resIdx ) );
        TEST_ASSERT_EQUAL_HEX32( llRes[ resIdx ], UT_ADC_REG->CR1 & ADC_CR1_RES );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Resolution( UT_ADC_PERIPH, &resolution ) );
        TEST_ASSERT_EQUAL( resIdx, resolution );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_Resolution( UT_ADC_PERIPH, ADC_RESOLUTION_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_Resolution( UT_ADC_PERIPH, NULL ) );
}


/**
 * \brief   Adc_Set_SamplingTime() writes SMPR1 / SMPR2 for every option.
 *
 * \details Channel 5 (SMPR2) and channel 12 (SMPR1), pin inputs.
 *
 * \par Expected results
 * - SMPx field and read back match for all options.
 */
void Ut_Adc_Set_SamplingTime_AllOptions_RegisterAndReadBack( void )
{
    adc_ChannelSampling_t sampling = ADC_CHANNEL_SAMPLING_CNT;

    for( adc_ChannelSampling_t smpIdx = ADC_CHANNEL_SAMPLING_3_CYCLES; ADC_CHANNEL_SAMPLING_CNT > smpIdx; smpIdx++ )
    {
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_5, smpIdx ) );
        TEST_ASSERT_EQUAL_HEX32( (uint32_t)smpIdx << ADC_SMPR2_SMP5_Pos, UT_ADC_REG->SMPR2 & ADC_SMPR2_SMP5 );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_5, &sampling ) );
        TEST_ASSERT_EQUAL( smpIdx, sampling );

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_12, smpIdx ) );
        TEST_ASSERT_EQUAL_HEX32( (uint32_t)smpIdx << ADC_SMPR1_SMP12_Pos, UT_ADC_REG->SMPR1 & ADC_SMPR1_SMP12 );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_CNT, ADC_CHANNEL_SAMPLING_3_CYCLES ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_5, NULL ) );
}


/**
 * \brief   Internal channel inputs enable the measurement paths and require minimum sampling.
 *
 * \details VREF on channel 17, VBAT on channel 18 (PCLK2 / 4 = 21 MHz).
 *
 * \par Expected results
 * - VREF: TSVREFE set. Sampling 112 cycles (5.3 us) refused, 480 cycles accepted.
 * - VBAT: VBATE set, 112 cycles (5.3 us >= 5 us) accepted.
 * - VREF on channel 3 refused.
 */
void Ut_Adc_Set_ChannelInput_InternalInputs_PathsAndSampling( void )
{
    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ClockDiv4();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ChannelInput( UT_ADC_PERIPH, UT_ADC_VREF_CHANNEL, ADC_CHANNEL_INPUT_VREF ) );
    TEST_ASSERT_BITS_HIGH( ADC_CCR_TSVREFE, UT_ADC_COMMON->CCR );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_SamplingTime( UT_ADC_PERIPH, UT_ADC_VREF_CHANNEL, ADC_CHANNEL_SAMPLING_112_CYCLES ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_SamplingTime( UT_ADC_PERIPH, UT_ADC_VREF_CHANNEL, ADC_CHANNEL_SAMPLING_480_CYCLES ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ChannelInput( UT_ADC_PERIPH, UT_ADC_VBAT_CHANNEL, ADC_CHANNEL_INPUT_VBAT ) );
    TEST_ASSERT_BITS_HIGH( ADC_CCR_VBATE, UT_ADC_COMMON->CCR );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_SamplingTime( UT_ADC_PERIPH, UT_ADC_VBAT_CHANNEL, ADC_CHANNEL_SAMPLING_112_CYCLES ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ChannelInput( UT_ADC_PERIPH, ADC_CHANNEL_3, ADC_CHANNEL_INPUT_VREF ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ChannelInput( UT_ADC_PERIPH, ADC_CHANNEL_3, ADC_CHANNEL_INPUT_CNT ) );
}

/* ============================ ANALOG WATCH-DOG ============================ */

/**
 * \brief   Adc_AwdInit() configures watch-dog on all regular channels with thresholds.
 *
 * \par Expected results
 * - CR1 AWDEN = 1, JAWDEN = 0, AWDSGL = 0.
 * - HTR = 3000, LTR = 100, thresholds read back, filter reads back NONE.
 */
void Ut_Adc_AwdInit_AllRegular_ChannelsAndThresholds( void )
{
    adc_AwdConfig_t    awdConfig = { ADC_AWD_1, ADC_AWD_MODE_ALL_REGULAR, 100u, 3000u, ADC_AWD_FILTER_NONE };
    adc_AwdThreshold_t lowThr    = 0u;
    adc_AwdThreshold_t highThr   = 0u;
    adc_AwdFilter_t    filter    = ADC_AWD_FILTER_CNT;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_BITS_HIGH( ADC_CR1_AWDEN, UT_ADC_REG->CR1 );
    TEST_ASSERT_BITS_LOW( ADC_CR1_JAWDEN | ADC_CR1_AWDSGL, UT_ADC_REG->CR1 );
    TEST_ASSERT_EQUAL_UINT32( 3000u, UT_ADC_REG->HTR );
    TEST_ASSERT_EQUAL_UINT32( 100u, UT_ADC_REG->LTR );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, &lowThr, &highThr ) );
    TEST_ASSERT_EQUAL_UINT16( 100u, lowThr );
    TEST_ASSERT_EQUAL_UINT16( 3000u, highThr );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdFilter( UT_ADC_PERIPH, ADC_AWD_1, &filter ) );
    TEST_ASSERT_EQUAL( ADC_AWD_FILTER_NONE, filter );
}


/**
 * \brief   Watch-dog thresholds are converted to 12 bit format for lower resolution.
 *
 * \details 8 bit resolution, thresholds 10 / 200 RAW.
 *
 * \par Expected results
 * - LTR = 10 << 4, HTR = 200 << 4, thresholds read back in 8 bit RAW.
 * - Threshold above 255 refused.
 */
void Ut_Adc_Set_AwdThresholds_LowerResolution_Converted( void )
{
    adc_AwdThreshold_t lowThr  = 0u;
    adc_AwdThreshold_t highThr = 0u;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_Resolution( UT_ADC_PERIPH, ADC_RESOLUTION_8BIT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, 10u, 200u ) );

    TEST_ASSERT_EQUAL_UINT32( 10u << 4u, UT_ADC_REG->LTR );
    TEST_ASSERT_EQUAL_UINT32( 200u << 4u, UT_ADC_REG->HTR );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, &lowThr, &highThr ) );
    TEST_ASSERT_EQUAL_UINT16( 10u, lowThr );
    TEST_ASSERT_EQUAL_UINT16( 200u, highThr );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, 10u, 256u ) );
    TEST_ASSERT_EQUAL_UINT32( 200u << 4u, UT_ADC_REG->HTR );
}


/**
 * \brief   Adc_AwdInit() rejects single channel modes, filtering, invalid watch-dog and
 *          running conversion.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR, CR1 watch-dog bits not set.
 */
void Ut_Adc_AwdInit_UnsupportedModes_ReturnsError( void )
{
    adc_AwdConfig_t awdConfig = { ADC_AWD_1, ADC_AWD_MODE_SINGLE, 0u, 100u, ADC_AWD_FILTER_NONE };

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );

    awdConfig.AwdMode   = ADC_AWD_MODE_ALL;
    awdConfig.AwdFilter = ADC_AWD_FILTER_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_AwdFilter( UT_ADC_PERIPH, ADC_AWD_1, ADC_AWD_FILTER_CNT ) );

    awdConfig.AwdFilter = ADC_AWD_FILTER_NONE;
    awdConfig.AwdId     = ADC_AWD_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );

    awdConfig.AwdId  = ADC_AWD_1;
    UT_ADC_REG->CR2 = ADC_CR2_CONT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_AwdInit( UT_ADC_PERIPH, NULL ) );

    TEST_ASSERT_BITS_LOW( ADC_CR1_AWDEN | ADC_CR1_JAWDEN, UT_ADC_REG->CR1 );
}

/* ======================== DMA - RELEASE AND ERRORS ======================== */

/** Count of Dma_Set_InterruptInactive() calls (DMA stream released) */
static uint32_t utAdc_DmaIrqOffCnt;

/** Dma_Set_InterruptInactive() stub counting the calls */
static dma_RequestState_t Ut_Adc_DmaIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utAdc_DmaIrqOffCnt++;

    return ( DMA_REQUEST_OK );
}


/**
 * \brief   Adc_Deinit() releases the DMA stream of DMA mode.
 *
 * \details ADC1 in DMA mode (DMA2 stream 0), Adc_Deinit().
 *
 * \par Expected results
 * - ADC_REQUEST_OK, DMA stream interrupt disabled (1x), ADC DMA requests disabled (CR2 DMA, DDS 0).
 * - Second Adc_Deinit() does not touch the DMA stream again.
 */
void Ut_Adc_Dma_Deinit_StreamReleased( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_BITS_HIGH( ADC_CR2_DMA, UT_ADC_REG->CR2 );

    utAdc_DmaIrqOffCnt = 0u;
    Dma_Set_InterruptInactive_StubWithCallback( Ut_Adc_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_DmaIrqOffCnt );
    TEST_ASSERT_BITS_LOW( ADC_CR2_DMA | ADC_CR2_DDS, UT_ADC_REG->CR2 );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_DmaIrqOffCnt );
}


/**
 * \brief   DMA failures during initialization and start are reported.
 *
 * \details Dma_Init() fails, then initialization OK and Dma_Set_TransferActive() fails at
 *          the conversion start.
 *
 * \par Expected results
 * - Adc_PeriphInit(): ADC_REQUEST_ERROR.
 * - Adc_Set_RegStart(): ADC_REQUEST_ERROR.
 */
void Ut_Adc_Dma_InitAndStartFailure_ReturnsError( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ClockDiv4();
    Dma_Init_StubWithCallback( NULL );
    Dma_Init_IgnoreAndReturn( DMA_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    (void)Adc_Deinit( UT_ADC_PERIPH );
    Ut_Adc_PeriphInit( &config );

    /* First value of the ignore queue is returned by the next call */
    Ut_Adc_Reset_Mocks();
    Dma_Set_TransferActive_IgnoreAndReturn( DMA_REQUEST_ERROR );
    Ut_Adc_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_RegStart( UT_ADC_PERIPH ) );
}


/**
 * \brief   DMA callbacks of ADC2 and ADC3 report the events of their peripheral.
 *
 * \details ADC2 in DMA mode (DMA2 stream 2), ADC3 in DMA mode (DMA2 stream 1), callbacks
 *          captured from Dma_Init() are called.
 *          MCUs without ADC2 / ADC3: test ignored.
 *
 * \par Expected results
 * - Half / complete / error callback of the configuration called for every peripheral.
 */
void Ut_Adc_Dma_Adc2Adc3Callbacks_EventsReported( void )
{
#if defined(ADC2) && defined(ADC3)
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );
    const struct
    {
        adc_PeriphId_t       PeriphId;
        adc_DmaChannelId_t   Stream;
    }   periphLut[] =
    {
        { ADC_PERIPH_2, ADC_DMA_CHANNEL_2 },
        { ADC_PERIPH_3, ADC_DMA_CHANNEL_1 },
    };

    for( uint32_t idx = 0u; 2u > idx; idx++ )
    {
        config.PeriphId                = periphLut[ idx ].PeriphId;
        config.RegChannels[ 0u ]       = (adc_ChannelConfig_t){ ADC_CHANNEL_0, ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_15_CYCLES };
        config.DataConfig.DmaChannelId = periphLut[ idx ].Stream;

        Ut_Adc_PeriphInit( &config );

        utAdc_DmaConfig.HalfTransferCallback();
        utAdc_DmaConfig.TransferErrorCallback();
        utAdc_DmaConfig.TransferCompleteCallback();

        TEST_ASSERT_EQUAL_UINT32( idx + 1u, utAdc_HalfCnt );
        TEST_ASSERT_EQUAL_UINT32( idx + 1u, utAdc_CompleteCnt );
        TEST_ASSERT_EQUAL( ADC_ERROR_DMA_TRANSFER, utAdc_LastError );
    }
#else
    TEST_IGNORE_MESSAGE( "MCU without ADC2 / ADC3" );
#endif /* ADC2 AND ADC3 */
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief RCC clock stub - ADC peripherals PCLK2 84 MHz, SysTick (HCLK) 168 MHz.
 */
static rcc_RequestState_t Ut_Adc_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)callCnt;

    if( RCC_PERIPH_SYSTICK == periphId )
    {
        *periphClk = UT_ADC_HCLK_HZ;
    }
    else
    {
        *periphClk = UT_ADC_PCLK2_HZ;
    }

    return ( RCC_REQUEST_OK );
}


/**
 * \brief RCC clock state stub - returns utAdc_RccClkState.
 */
static rcc_RequestState_t Ut_Adc_RccGetStateStub( rcc_PeriphId_t periphId, rcc_FunctionState_t * const funcState, int callCnt )
{
    (void)periphId;
    (void)callCnt;

    *funcState = utAdc_RccClkState;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief GPIO initialization stub - stores the pin configuration.
 */
static gpio_RequestState_t Ut_Adc_GpioInitStub( gpio_Config_t *gpioConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( gpioConfig );

    utAdc_GpioConfig = *gpioConfig;
    utAdc_GpioInitCnt++;

    return ( GPIO_REQUEST_OK );
}


/**
 * \brief NVIC handler registration stub - stores the common ADC ISR.
 */
static nvic_RequestState_t Ut_Adc_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_ADC_NVIC, irqId );
    TEST_ASSERT_NOT_NULL( irqHandler );

    utAdc_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief DMA default configuration stub - zeroed configuration.
 */
static dma_RequestState_t Ut_Adc_DmaDefaultStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    (void)callCnt;

    memset( dmaConfig, 0, sizeof( dma_ConfigStruct_t ) );

    return ( DMA_REQUEST_OK );
}


/**
 * \brief DMA initialization stub - stores the stream configuration (incl. callbacks).
 */
static dma_RequestState_t Ut_Adc_DmaInitStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( dmaConfig );

    utAdc_DmaConfig = *dmaConfig;
    utAdc_DmaInitCnt++;

    return ( DMA_REQUEST_OK );
}


/**
 * \brief Ignores all calls of RCC / NVIC / GPIO / DMA functions used by the module.
 */
static void Ut_Adc_Ignore_PeriphMocks( void )
{
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphState_StubWithCallback( Ut_Adc_RccGetStateStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Gpio_Init_StubWithCallback( Ut_Adc_GpioInitStub );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Adc_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
    Dma_Get_DefaultConfig_StubWithCallback( Ut_Adc_DmaDefaultStub );
    Dma_Init_StubWithCallback( Ut_Adc_DmaInitStub );
    Dma_Set_TransferActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_DataCount_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferCompleteIrqActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferCompleteIrqInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_HalfTransferIrqActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_HalfTransferIrqInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_InterruptActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_InterruptInactive_IgnoreAndReturn( DMA_REQUEST_OK );
}


/**
 * \brief Removes all expectations and ignores of the mocks.
 */
static void Ut_Adc_Reset_Mocks( void )
{
    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockDma_Port_Destroy();
    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockDma_Port_Init();
}


/**
 * \brief Releases static module context of the previous test (data handling, interrupts,
 *        configuration kept by the module) and re-initializes the mocks. Registers must be zeroed.
 */
static void Ut_Adc_Release( void )
{
    Ut_Adc_Ignore_PeriphMocks();

    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx++ )
    {
        (void)Adc_Deinit( periphIdx );
    }

    Ut_Adc_Reset_Mocks();
}


/**
 * \brief Returns ADC1 configuration: one regular VREF channel, software trigger, single mode.
 *
 * \param xferMode   [in]: Data transfer mode
 * \param bufferSize [in]: Count of samples of data buffer (0 - no buffer)
 */
static adc_PeriphConfig_t Ut_Adc_Get_PeriphConfig( adc_TransferMode_t xferMode, adc_BufferSize_t bufferSize )
{
    adc_PeriphConfig_t periphConfig;

    memset( &periphConfig, 0, sizeof( periphConfig ) );

    periphConfig.PeriphId        = UT_ADC_PERIPH;
    periphConfig.Resolution      = ADC_RESOLUTION_12BIT;
    periphConfig.RegTriggerMode  = ADC_REG_TRIGGER_MODE_SINGLE;
    periphConfig.RegTriggerEdge  = ADC_TRIGGER_EDGE_RISING;
    periphConfig.RegTriggerId    = ADC_REG_TRIGGER_SOFTWARE;
    periphConfig.RegChannelsCnt  = 1u;
    periphConfig.RegChannels[ 0u ].ChannelId       = UT_ADC_VREF_CHANNEL;
    periphConfig.RegChannels[ 0u ].ChannelInput    = ADC_CHANNEL_INPUT_VREF;
    periphConfig.RegChannels[ 0u ].ChannelSampling = UT_ADC_SAMPLING_INTERNAL;
    periphConfig.InjTriggerMode  = ADC_INJ_TRIGGER_MODE_CONTINUOUS;
    periphConfig.InjTriggerId    = ADC_INJ_TRIGGER_SOFTWARE;
    periphConfig.InjTriggerEdge  = ADC_TRIGGER_EDGE_RISING;
    periphConfig.InjChannelsCnt  = 0u;

    periphConfig.DataConfig.TransferMode             = xferMode;
    periphConfig.DataConfig.DataBuffer               = ( 0u < bufferSize ) ? utAdc_Buffer : NULL;
    periphConfig.DataConfig.BufferSize               = bufferSize;
    periphConfig.DataConfig.BufferMode               = ADC_BUFFER_MODE_ONE_SHOT;
    periphConfig.DataConfig.DmaPeriphId              = ADC_DMA_PERIPH_2;
    periphConfig.DataConfig.DmaChannelId             = ADC_DMA_CHANNEL_0;
    periphConfig.DataConfig.DmaPriority              = ADC_DMA_PRIORITY_LOW;
    periphConfig.DataConfig.IrqPriority              = UT_ADC_PRIO;
    periphConfig.DataConfig.HalfTransferCallback     = Ut_Adc_HalfCallback;
    periphConfig.DataConfig.TransferCompleteCallback = Ut_Adc_CompleteCallback;
    periphConfig.DataConfig.ErrorCallback            = Ut_Adc_ErrorCallback;
    periphConfig.DataConfig.InjCompleteCallback      = NULL;

    return ( periphConfig );
}


/**
 * \brief Returns module configuration: PCLK2 / 4, no peripheral used.
 */
static adc_Config_t Ut_Adc_Get_Config( void )
{
    adc_Config_t config;

    memset( &config, 0, sizeof( config ) );

    config.ClockSource  = ADC_CLK_SRC_PCLK2;
    config.ClockDivider = ADC_CLK_DIV_4;

    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx++ )
    {
        config.PeriphConfig[ periphIdx ].PeriphId = periphIdx;
    }

    return ( config );
}


/**
 * \brief Presets ADC clock prescaler PCLK2 / 4 (21 MHz).
 */
static void Ut_Adc_Set_ClockDiv4( void )
{
    UT_ADC_COMMON->CCR = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
}


/**
 * \brief Initializes ADC1 with ignored RCC / GPIO / NVIC / DMA calls and PCLK2 / 4.
 *
 * \param periphConfig [in]: Peripheral configuration
 */
static void Ut_Adc_PeriphInit( adc_PeriphConfig_t * const periphConfig )
{
    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ClockDiv4();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( periphConfig ) );
}


/**
 * \brief Calls the captured ADC ISR with given SR flags and data register value.
 *
 * \param srFlags [in]: Value of the SR register
 * \param data    [in]: Value of the DR register
 */
static void Ut_Adc_Call_Isr( uint32_t srFlags, uint32_t data )
{
    TEST_ASSERT_NOT_NULL_MESSAGE( utAdc_Isr, "ADC ISR not registered" );

    UT_ADC_REG->DR = data;
    UT_ADC_REG->SR = srFlags;
    utAdc_Isr();
}


/** \brief Half transfer callback */
static void Ut_Adc_HalfCallback( void )
{
    utAdc_HalfCnt++;
}


/** \brief Transfer complete callback */
static void Ut_Adc_CompleteCallback( void )
{
    utAdc_CompleteCnt++;
}


/** \brief Injected sequence complete callback */
static void Ut_Adc_InjCallback( void )
{
    utAdc_InjCnt++;
}


/**
 * \brief Transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void Ut_Adc_ErrorCallback( adc_ErrorId_t errorId )
{
    utAdc_LastError = errorId;
    utAdc_ErrorCnt++;
}
