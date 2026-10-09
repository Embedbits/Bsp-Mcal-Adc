/**
 * \author Mr.Nobody
 * \file Test_Adc.c
 * \ingroup Adc
 * \brief Unit tests of Analog-to-Digital Converter (ADC) module (STM32G4).
 *
 * Adc.c, Adc_Isr.c, Adc_Poll.c and Adc_Dma.c are compiled unchanged with real
 * LL drivers. ADC registers are emulated by RegMem, RCC, NVIC, GPIO and DMA
 * modules are mocked by CMock. ADC ISR registered in NVIC and DMA handlers
 * registered by Dma_Init() are captured by stubs and called directly.
 *
 * Sequences where the ADC reacts on a CR write (calibration end, disable, stop
 * of conversion) are emulated for all ADC peripherals by HW model running in
 * background thread (Ut_Adc_HwModel). Tests without the model check register
 * values only.
 *
 * \note Emulated registers are plain memory - ISR flags are not cleared by
 *       data register reads, tests preset the flags handled by the module
 *       before each step. ADRDY is set by the module itself (write of ADRDY
 *       to ISR before enable).
 *
 * \note Clock source, channel inputs and data handling are kept in static
 *       module context - setUp() releases them (Adc_Deinit of all peripherals
 *       and clock source HCLK with ignored mocks) and re-initializes the mocks.
 */

/* ============================= INCLUDES =================================== */
#include <string.h>                         /* memset                         */
#include "unity.h"                          /* Unity testing framework        */
#include "UtCommon.h"                       /* Common test helpers            */
#include "RegMem.h"                         /* Register memory emulation      */
#include "CmsisHost.h"                      /* Core intrinsics emulation      */
#include "Adc_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32_adc.h"                      /* ADC registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 Ut_Adc_HwModel              ( void );
static void                 Ut_Adc_Set_ModelActive      ( void );
static rcc_RequestState_t   Ut_Adc_RccGetClkStub        ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static gpio_RequestState_t  Ut_Adc_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static nvic_RequestState_t  Ut_Adc_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static nvic_RequestState_t  Ut_Adc_NvicSetInactiveStub  ( nvic_PeriphIrqList_t irqId, int callCnt );
static dma_RequestState_t   Ut_Adc_DmaDefaultStub       ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static dma_RequestState_t   Ut_Adc_DmaInitStub          ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static dma_RequestState_t   Ut_Adc_DmaIrqOffStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static void                 Ut_Adc_Ignore_PeriphMocks   ( void );
static void                 Ut_Adc_Reset_Mocks          ( void );
static void                 Ut_Adc_Release              ( void );
static adc_PeriphConfig_t   Ut_Adc_Get_PeriphConfig     ( adc_TransferMode_t xferMode, adc_BufferSize_t bufferSize );
static void                 Ut_Adc_PeriphInit           ( adc_PeriphConfig_t * const periphConfig );
static void                 Ut_Adc_Set_Enabled          ( void );
static void                 Ut_Adc_Call_Isr             ( uint32_t isrFlags );

static void                 Ut_Adc_HalfCallback         ( void );
static void                 Ut_Adc_CompleteCallback     ( void );
static void                 Ut_Adc_InjCallback          ( void );
static void                 Ut_Adc_ErrorCallback        ( adc_ErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** ADC peripheral used by tests (available on all supported MCUs) */
#define UT_ADC_PERIPH                       ( ADC_PERIPH_1 )
#define UT_ADC_REG                          ( ADC1 )
#define UT_ADC_COMMON                       ( ADC12_COMMON )
#define UT_ADC_NVIC                         ( NVIC_PERIPH_IRQ_ADC1_2 )

/** Clock frequencies returned by RCC mock [Hz] */
#define UT_ADC_HCLK_HZ                      ( 48000000u )
#define UT_ADC_SYSCLK_HZ                    ( 170000000u )
#define UT_ADC_PLLP_HZ                      ( 500000u )

/** Internal channels of ADC1 */
#define UT_ADC_VREF_CHANNEL                 ( ADC_CHANNEL_18 )
#define UT_ADC_TEMP_CHANNEL                 ( ADC_CHANNEL_16 )

/** Interrupt priority of test configurations */
#define UT_ADC_PRIO                         ( 7u )

/** Size of the test data buffer */
#define UT_ADC_BUF_SIZE                     ( 8u )

/** ADC CR bits written by the module and handled by HW model */
#define UT_ADC_CR_HW_BITS                   ( ADC_CR_ADCAL | ADC_CR_ADDIS | ADC_CR_ADSTP | ADC_CR_JADSTP )

/** Count of ADC peripherals emulated by the HW model */
#define UT_ADC_MODEL_CNT                    ( sizeof( utAdc_ModelRegs ) / sizeof( utAdc_ModelRegs[ 0u ] ) )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** ADC peripherals emulated by the HW model, indexed by adc_PeriphId_t */
static ADC_TypeDef * const      utAdc_ModelRegs[ ] =
{
    ADC1,
    ADC2,
#if defined(ADC3)
    ADC3,
#endif /* ADC3 */
#if defined(ADC4)
    ADC4,
#endif /* ADC4 */
#if defined(ADC5)
    ADC5,
#endif /* ADC5 */
};

/** Count of disable requests (ADDIS) served by the HW model per peripheral */
static volatile uint32_t        utAdc_ModelDisableCnt[ ADC_PERIPH_CNT ];

/** HW model sets JEOS of ADC1 on request (end of the injected sequence) */
static volatile uint32_t        utAdc_ModelJeosRequest;

/** JEOS flag of ADC1 at the injected stop request (JADSTP) seen by the HW model */
static volatile uint32_t        utAdc_ModelJeosAtStop;

/** HCLK frequency returned by the RCC stub for the ADC HCLK sources */
static rcc_FreqHz_t             utAdc_HclkHz;

/** ISR registered in NVIC for the ADC, its line and count of line deactivations */
static nvic_IsrCallback_t       utAdc_Isr;
static nvic_PeriphIrqList_t     utAdc_IsrIrqId;
static uint32_t                 utAdc_NvicInactiveCnt;

/** GPIO configuration of the last Gpio_Init call and count of calls */
static gpio_Config_t            utAdc_GpioConfig;
static uint32_t                 utAdc_GpioInitCnt;

/** DMA configuration captured by Dma_Init stub, count of Dma_Init / Dma_Set_InterruptInactive calls */
static dma_ConfigStruct_t       utAdc_DmaConfig;
static uint32_t                 utAdc_DmaInitCnt;
static uint32_t                 utAdc_DmaIrqOffCnt;

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
    /* Stops HW model of previous test and clears registers */
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    Ut_Adc_Release();

    utAdc_Isr              = NULL;
    utAdc_IsrIrqId         = NVIC_PERIPH_IRQ_SIZE;
    utAdc_NvicInactiveCnt  = 0u;
    utAdc_GpioInitCnt      = 0u;
    utAdc_DmaInitCnt       = 0u;
    utAdc_DmaIrqOffCnt     = 0u;
    utAdc_HalfCnt          = 0u;
    utAdc_CompleteCnt      = 0u;
    utAdc_InjCnt           = 0u;
    utAdc_ErrorCnt         = 0u;
    utAdc_LastError        = ADC_ERROR_CNT;
    utAdc_ModelJeosRequest = 0u;
    utAdc_ModelJeosAtStop  = 0xFFFFFFFFu;

    (void)memset( &utAdc_DmaConfig, 0, sizeof( utAdc_DmaConfig ) );

    for( uint32_t idx = 0u; ADC_PERIPH_CNT > idx; idx++ )
    {
        utAdc_ModelDisableCnt[ idx ] = 0u;
    }

    for( uint32_t idx = 0u; UT_ADC_BUF_SIZE > idx; idx++ )
    {
        utAdc_Buffer[ idx ] = 0u;
    }
}


void tearDown( void )
{
    (void)RegMem_Set_ModelInactive();
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Adc_Get_ModuleVersion() returns version of the module.
 *
 * \details Reads the module version structure.
 *
 * \par Expected results
 * - Version is 1.0.0 (Major 1, Minor 0, Patch 0).
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
 * \brief   Adc_Init() rejects NULL configuration.
 *
 * \details Calls Adc_Init() with NULL pointer.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned.
 * - No other module is called (strict mocks without expectations).
 */
void Ut_Adc_Init_NullConfig_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( NULL ) );
}


/**
 * \brief   Adc_Init() rejects clock configurations out of ADC limits.
 *
 * \details Calls Adc_Init() with (RCC clock stub HCLK 48 MHz, SYSCLK 170 MHz, PLL P 500 kHz):
 * - synchronous HCLK clock with not supported divider 8,
 * - PLL P / 4 = 125 kHz, below minimal ADC clock frequency,
 * - SYSCLK / 2 = 85 MHz, above maximal ADC clock frequency,
 * - invalid clock source ADC_CLK_SRC_CNT.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 * - Clock source is not activated in RCC (no Rcc_Set_PeriphActive call expected).
 */
void Ut_Adc_Init_InvalidClock_ReturnsErrorWithoutClockSelection( void )
{
    adc_Config_t config = { 0 };

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    /* Synchronous clock (HCLK) supports dividers 1, 2, 4 only */
    config.ClockSource  = ADC_CLK_SRC_HCLK;
    config.ClockDivider = ADC_CLK_DIV_8;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* PLL P / 4 = 125 kHz is below minimal ADC clock */
    config.ClockSource  = ADC_CLK_SRC_PLLP;
    config.ClockDivider = ADC_CLK_DIV_4;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* SYSCLK / 2 = 85 MHz is above maximal ADC clock */
    config.ClockSource  = ADC_CLK_SRC_SYSCLK;
    config.ClockDivider = ADC_CLK_DIV_2;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    config.ClockSource  = ADC_CLK_SRC_CNT;
    config.ClockDivider = ADC_CLK_DIV_1;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
}


/**
 * \brief   Adc_Init() without used peripheral configures only the ADC clock.
 *
 * \details Calls Adc_Init() with clock HCLK / 2 and all peripheral slots empty
 *          (no regular and no injected channel).
 *
 * \par Expected results
 * - ADC_REQUEST_OK is returned, RCC_PERIPH_ADC12_HCLK (and RCC_PERIPH_ADC345_HCLK on MCUs with
 *   ADC3) is activated in RCC, the active source is not released.
 * - Common clock CKMODE = synchronous HCLK / 2 in all common register blocks.
 * - ADC1 is not powered up and not enabled (ADVREGEN = ADEN = 0).
 * - Adc_Get_ClockSource() / Adc_Get_ClockDivider() return HCLK and divider 2.
 */
void Ut_Adc_Init_NoPeripheralUsed_ClockConfiguredOnly( void )
{
    adc_Config_t config = { 0 };
    adc_ClkSrc_t clkSrc = ADC_CLK_SRC_CNT;
    adc_ClkDiv_t clkDiv = ADC_CLK_DIV_CNT;

    config.ClockSource  = ADC_CLK_SRC_HCLK;
    config.ClockDivider = ADC_CLK_DIV_2;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC12_HCLK, RCC_REQUEST_OK );
#if defined(ADC345_COMMON)
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC345_HCLK, RCC_REQUEST_OK );
#endif /* ADC345_COMMON */

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_SYNC_PCLK_DIV2, LL_ADC_GetCommonClock( UT_ADC_COMMON ) );
#if defined(ADC345_COMMON)
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_SYNC_PCLK_DIV2, LL_ADC_GetCommonClock( ADC345_COMMON ) );
#endif /* ADC345_COMMON */
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ( ADC_CR_ADEN | ADC_CR_ADVREGEN ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( ADC_CLK_SRC_HCLK, clkSrc );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockDivider( &clkDiv ) );
    TEST_ASSERT_EQUAL( ADC_CLK_DIV_2, clkDiv );
}


/**
 * \brief   Adc_Init() rejects peripheral slot with different PeriphId.
 *
 * \details Configuration slot of ADC1 is filled with valid configuration, but its
 *          PeriphId is set to ADC2. Peripheral mocks are ignored.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned.
 * - ADC1 is not enabled (ADEN = 0).
 * - Ignored on MCUs with one ADC (test function exists always, the runner collects
 *   test functions without preprocessor).
 */
void Ut_Adc_Init_PeripheralIdMismatch_ReturnsError( void )
{
#if !defined(ADC2)
    TEST_IGNORE_MESSAGE( "MCU has only one ADC" );
#else
    adc_Config_t config = { 0 };

    config.ClockSource  = ADC_CLK_SRC_HCLK;
    config.ClockDivider = ADC_CLK_DIV_1;
    config.PeriphConfig[ ADC_PERIPH_1 ] = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    config.PeriphConfig[ ADC_PERIPH_1 ].PeriphId = ADC_PERIPH_2;

    Ut_Adc_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ADC_CR_ADEN );
#endif /* ADC2 */
}


/**
 * \brief   Adc_Init() initializes and enables used peripheral.
 *
 * \details Calls Adc_Init() with clock HCLK / 1 and ADC1 slot configured for one
 *          regular VREF channel in polling mode. HW model handles calibration.
 *
 * \par Expected results
 * - ADC_REQUEST_OK is returned.
 * - ADC1 is out of deep power-down, voltage regulator and ADC are enabled
 *   (ADVREGEN = ADEN = 1, DEEPPWD = 0).
 */
void Ut_Adc_Init_VrefPeripheral_PeripheralEnabled( void )
{
    adc_Config_t config = { 0 };

    config.ClockSource  = ADC_CLK_SRC_HCLK;
    config.ClockDivider = ADC_CLK_DIV_1;
    config.PeriphConfig[ ADC_PERIPH_1 ] = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ModelActive();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( ADC_CR_ADEN | ADC_CR_ADVREGEN, UT_ADC_REG->CR & ( ADC_CR_ADEN | ADC_CR_ADVREGEN | ADC_CR_DEEPPWD ) );
}


/**
 * \brief   Adc_Set_ClockSource() releases the active source and activates the new one in all
 *          ADC clock groups.
 *
 * \details
 * 1. Changes HCLK (reset state) to SYSCLK.
 * 2. Changes SYSCLK to PLL P with failing RCC activation of the ADC1 / ADC2 group.
 * 3. Selects SYSCLK again (already selected).
 * 4. Calls Adc_Get_ClockSource() with NULL pointer.
 *
 * \note    Regression of the STM32H5 module bug AB#1043 (previous source never released, RCC
 *          changes the multiplexer of a released clock only).
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK, per group RCC_PERIPH_ADCxx_HCLK released and RCC_PERIPH_ADCxx_SYSCLK
 *    activated (strict order), read back source is SYSCLK.
 * 2. ADC_REQUEST_ERROR, read back source stays SYSCLK, second group not touched.
 * 3. ADC_REQUEST_OK, sources only activated (no release).
 * 4. ADC_REQUEST_ERROR.
 */
void Ut_Adc_Set_ClockSource_RccActivatedAndShadowUpdated( void )
{
    adc_ClkSrc_t clkSrc = ADC_CLK_SRC_CNT;

    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_ADC12_HCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC12_SYSCLK, RCC_REQUEST_OK );
#if defined(ADC345_COMMON)
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_ADC345_HCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC345_SYSCLK, RCC_REQUEST_OK );
#endif /* ADC345_COMMON */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockSource( ADC_CLK_SRC_SYSCLK ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( ADC_CLK_SRC_SYSCLK, clkSrc );

    /* RCC error - source is not changed */
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_ADC12_SYSCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC12_PLLP, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockSource( ADC_CLK_SRC_PLLP ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( ADC_CLK_SRC_SYSCLK, clkSrc );

    /* Same source - clock enable requested again, nothing released */
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC12_SYSCLK, RCC_REQUEST_OK );
#if defined(ADC345_COMMON)
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC345_SYSCLK, RCC_REQUEST_OK );
#endif /* ADC345_COMMON */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockSource( ADC_CLK_SRC_SYSCLK ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_ClockSource( NULL ) );
}


/**
 * \brief   Adc_Set_ClockSource() rejects invalid source and enabled ADC.
 *
 * \details
 * 1. Sets invalid clock source ADC_CLK_SRC_CNT.
 * 2. Presets ADEN (ADC enabled) and sets SYSCLK clock source.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in both cases.
 * - RCC is not called (no expectation on RCC mock).
 */
void Ut_Adc_Set_ClockSource_InvalidOrEnabled_ReturnsErrorWithoutRcc( void )
{
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockSource( ADC_CLK_SRC_CNT ) );

    UT_ADC_REG->CR = ADC_CR_ADEN;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockSource( ADC_CLK_SRC_SYSCLK ) );
}


/**
 * \brief   Adc_Set_ClockDivider() with synchronous HCLK source sets CKMODE.
 *
 * \details Sets dividers 1, 2 and 4 (clock source HCLK 48 MHz from setUp), then
 *          not supported divider 8, invalid divider and NULL read pointer.
 *
 * \par Expected results
 * - Dividers 1, 2, 4: ADC_REQUEST_OK, CCR CKMODE = HCLK / 1, / 2, / 4, read back
 *   divider equals the set one.
 * - Divider 8, ADC_CLK_DIV_CNT and Adc_Get_ClockDivider( NULL ): ADC_REQUEST_ERROR.
 */
void Ut_Adc_Set_ClockDivider_SyncHclk_CkModeAndReadBack( void )
{
    const adc_ClkDiv_t divs[]   = { ADC_CLK_DIV_1, ADC_CLK_DIV_2, ADC_CLK_DIV_4 };
    const uint32_t     llDivs[] = { LL_ADC_CLOCK_SYNC_PCLK_DIV1, LL_ADC_CLOCK_SYNC_PCLK_DIV2, LL_ADC_CLOCK_SYNC_PCLK_DIV4 };
    adc_ClkDiv_t       clkDiv   = ADC_CLK_DIV_CNT;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    for( uint32_t idx = 0u; ( sizeof( divs ) / sizeof( divs[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockDivider( divs[ idx ] ) );
        TEST_ASSERT_EQUAL_HEX32( llDivs[ idx ], LL_ADC_GetCommonClock( UT_ADC_COMMON ) );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockDivider( &clkDiv ) );
        TEST_ASSERT_EQUAL( divs[ idx ], clkDiv );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_8 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_ClockDivider( NULL ) );
}


/**
 * \brief   Adc_Set_ClockDivider() with asynchronous source sets PRESC and checks
 *          maximal ADC clock frequency.
 *
 * \details Selects SYSCLK (170 MHz) clock source, then:
 * 1. sets divider 16 (10.6 MHz),
 * 2. sets divider 2 (85 MHz, above maximal ADC clock).
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK, CCR of all common register blocks = asynchronous clock / 16, read
 *    back divider 16.
 * 2. ADC_REQUEST_ERROR, CCR keeps asynchronous clock / 16.
 */
void Ut_Adc_Set_ClockDivider_AsyncSource_PrescalerAndFrequencyLimits( void )
{
    adc_ClkDiv_t clkDiv = ADC_CLK_DIV_CNT;

    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockSource( ADC_CLK_SRC_SYSCLK ) );

    /* SYSCLK 170 MHz / 16 = 10.6 MHz */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockDivider( ADC_CLK_DIV_16 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_GetCommonClock( UT_ADC_COMMON ) );
#if defined(ADC345_COMMON)
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_GetCommonClock( ADC345_COMMON ) );
#endif /* ADC345_COMMON */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockDivider( &clkDiv ) );
    TEST_ASSERT_EQUAL( ADC_CLK_DIV_16, clkDiv );

    /* SYSCLK 170 MHz / 2 = 85 MHz is above maximal ADC clock - divider is not changed */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_2 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_GetCommonClock( UT_ADC_COMMON ) );
}


/**
 * \brief   Adc_Set_ClockDivider() is rejected while any ADC is enabled.
 *
 * \details Presets ADEN of ADC1 and sets divider 2.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned.
 * - Common CCR register is not written (stays 0).
 */
void Ut_Adc_Set_ClockDivider_PeripheralEnabled_ReturnsErrorWithoutWrite( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    UT_ADC_REG->CR = ADC_CR_ADEN;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_2 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_COMMON->CCR );
}


/**
 * \brief   Adc_PeriphInit() configures all registers of a polling VREF conversion.
 *
 * \details Initializes ADC1 by test configuration: one regular VREF channel
 *          (channel 18, 640.5 cycles), software trigger, single mode, 12-bit,
 *          polling without buffer. HW model handles calibration.
 *
 * \par Expected results
 * - ADC is powered up and enabled (ADVREGEN = ADEN = 1), no pending
 *   calibration / disable / stop request.
 * - Regular sequence length 1, rank 1 = channel 18, software trigger, single
 *   conversion mode, 12-bit resolution, overrun mode "data overwritten".
 * - VREFINT internal path enabled, channel 18 sampling time 640.5 cycles.
 * - Channel input reads back VREF, data configuration reads back polling mode.
 */
void Ut_Adc_PeriphInit_VrefPoll_RegistersConfigured( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    adc_ChannelInput_t input        = ADC_CHANNEL_INPUT_CNT;
    adc_DataConfig_t   readConfig;

    Ut_Adc_PeriphInit( &periphConfig );

    /* Power-up and enable */
    TEST_ASSERT_EQUAL_HEX32( ADC_CR_ADEN | ADC_CR_ADVREGEN, UT_ADC_REG->CR & ( ADC_CR_ADEN | ADC_CR_ADVREGEN | ADC_CR_DEEPPWD | UT_ADC_CR_HW_BITS ) );

    /* Regular sequence: rank 1 = VREF channel, software trigger, single conversion */
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_SEQ_SCAN_DISABLE, LL_ADC_REG_GetSequencerLength( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ADC_VREF_CHANNEL, __LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_REG_GetSequencerRanks( UT_ADC_REG, LL_ADC_REG_RANK_1 ) ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, LL_ADC_REG_IsTriggerSourceSWStart( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_CONV_SINGLE, LL_ADC_REG_GetContinuousMode( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_RESOLUTION_12B, LL_ADC_GetResolution( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_OVR_DATA_OVERWRITTEN, LL_ADC_REG_GetOverrun( UT_ADC_REG ) );

    /* Internal VREF path and sampling time */
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_PATH_INTERNAL_VREFINT, LL_ADC_GetCommonPathInternalCh( UT_ADC_COMMON ) & LL_ADC_PATH_INTERNAL_VREFINT );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_SAMPLINGTIME_640CYCLES_5,
                             LL_ADC_GetChannelSamplingTime( UT_ADC_REG, __LL_ADC_DECIMAL_NB_TO_CHANNEL( UT_ADC_VREF_CHANNEL ) ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ChannelInput( UT_ADC_PERIPH, UT_ADC_VREF_CHANNEL, &input ) );
    TEST_ASSERT_EQUAL( ADC_CHANNEL_INPUT_VREF, input );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_DataConfig( UT_ADC_PERIPH, &readConfig ) );
    TEST_ASSERT_EQUAL( ADC_TRANSFER_MODE_POLL, readConfig.TransferMode );
}


/**
 * \brief   Adc_PeriphInit() rejects invalid general configuration.
 *
 * \details Calls Adc_PeriphInit() with:
 * - NULL configuration,
 * - invalid peripheral ADC_PERIPH_CNT,
 * - no regular channel (RegChannelsCnt = 0, no injected channel),
 * - more regular channels than sequencer length,
 * - invalid resolution,
 * - invalid regular trigger.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 * - ADC CR register is not written (stays 0).
 */
void Ut_Adc_PeriphInit_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( NULL ) );

    periphConfig.PeriphId = ADC_PERIPH_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.RegChannelsCnt = 0u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.RegChannelsCnt = ADC_REG_SEQUENCE_CNT + 1u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.Resolution = ADC_RESOLUTION_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.RegTriggerId = ADC_REG_TRIGGER_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR );
}


/**
 * \brief   Adc_PeriphInit() rejects invalid channel configuration.
 *
 * \details Calls Adc_PeriphInit() with:
 * - VREF channel with 2.5 cycles sampling time (below required 4 us at 48 MHz),
 * - VREF input on channel 5 (VREF is connected to channel 18 only),
 * - pin input on channel 16 (channel without pin),
 * - the same channel twice with different sampling time.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 * - ADC CR register is not written, no GPIO pin is initialized.
 */
void Ut_Adc_PeriphInit_InvalidChannels_ReturnsErrorWithoutAccess( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    /* VREF needs at least 4 us sampling time (2.5 cycles at 48 MHz = 39 ns) */
    periphConfig.RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_2_5_CYCLES;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* VREF is not connected to channel 5 */
    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.RegChannels[ 0u ].ChannelId = ADC_CHANNEL_5;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* Channel 16 has no pin */
    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.RegChannels[ 0u ].ChannelId    = UT_ADC_TEMP_CHANNEL;
    periphConfig.RegChannels[ 0u ].ChannelInput = ADC_CHANNEL_INPUT_PIN_SINGLE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* Same channel twice with different sampling time */
    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.RegChannelsCnt  = 2u;
    periphConfig.RegChannels[ 1u ] = periphConfig.RegChannels[ 0u ];
    periphConfig.RegChannels[ 1u ].ChannelSampling = ADC_CHANNEL_SAMPLING_247_5_CYCLES;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* Same channel twice with the same configuration is allowed - checked by init in other test */
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_GpioInitCnt );
}


/**
 * \brief   Adc_PeriphInit() rejects invalid injected group and data configuration.
 *
 * \details Calls Adc_PeriphInit() with:
 * - automatic injected conversion without regular group,
 * - automatic injected conversion in single (discontinuous) mode,
 * - ISR transfer mode without data buffer and with zero buffer size,
 * - invalid transfer mode,
 * - DMA mode with invalid DMA channel and with invalid DMA priority (buffer size UINT16_MAX is
 *   valid - the DMA module counts items, Bug AB#1185).
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 * - ADC CR register is not written (stays 0).
 */
void Ut_Adc_PeriphInit_InvalidInjectedOrData_ReturnsErrorWithoutAccess( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    /* Automatic injected conversion needs regular group */
    periphConfig.RegChannelsCnt = 0u;
    periphConfig.InjChannelsCnt = 1u;
    periphConfig.InjChannels[ 0u ] = periphConfig.RegChannels[ 0u ];
    periphConfig.InjTriggerId   = ADC_INJ_TRIGGER_AUTO;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* Automatic injected conversion can not be discontinuous */
    periphConfig.RegChannelsCnt = 1u;
    periphConfig.InjTriggerMode = ADC_INJ_TRIGGER_MODE_SINGLE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* ISR mode needs data buffer */
    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, 0u );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, UT_ADC_BUF_SIZE );
    periphConfig.DataConfig.BufferSize = 0u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_CNT, UT_ADC_BUF_SIZE );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* DMA configuration */
    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );
    periphConfig.DataConfig.DmaChannelId = ADC_DMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );
    periphConfig.DataConfig.DmaPriority = (adc_DmaPriority_t)DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR );
}


/**
 * \brief   Adc_PeriphInit() is rejected for already enabled ADC.
 *
 * \details Presets ADEN and initializes ADC1 by valid configuration.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned.
 * - CR register keeps only ADEN.
 */
void Ut_Adc_PeriphInit_PeripheralEnabled_ReturnsErrorWithoutChange( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    UT_ADC_REG->CR = ADC_CR_ADEN;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    TEST_ASSERT_EQUAL_HEX32( ADC_CR_ADEN, UT_ADC_REG->CR );
}


/**
 * \brief   Adc_PeriphInit() configures GPIO pins of external channels and
 *          differential input.
 *
 * \details Initializes ADC1 with regular sequence: channel 3 differential (channel 3 - channel 4)
 *          and channel 1 single-ended (12.5 cycles). GPIO initialization is captured by stub.
 *
 * \par Expected results
 * - Gpio_Init() is called 3 times (PA2, PA3 for channel 3, PA0 for channel 1),
 *   the last configuration is PA0, analog mode, no pull.
 * - Channel 3 is differential, channel 1 single-ended.
 * - Regular sequence length 2, rank 1 = channel 3, rank 2 = channel 1.
 */
void Ut_Adc_PeriphInit_PinChannels_GpioAnalogAndDifferential( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    periphConfig.RegChannelsCnt = 2u;
    periphConfig.RegChannels[ 0u ].ChannelId       = ADC_CHANNEL_3;
    periphConfig.RegChannels[ 0u ].ChannelInput    = ADC_CHANNEL_INPUT_PIN_DIFF;
    periphConfig.RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_12_5_CYCLES;
    periphConfig.RegChannels[ 1u ].ChannelId       = ADC_CHANNEL_1;
    periphConfig.RegChannels[ 1u ].ChannelInput    = ADC_CHANNEL_INPUT_PIN_SINGLE;
    periphConfig.RegChannels[ 1u ].ChannelSampling = ADC_CHANNEL_SAMPLING_12_5_CYCLES;

    Ut_Adc_PeriphInit( &periphConfig );

    /* Channel 3 differential: PA2 (+) and PA3 (-), channel 1: PA0 */
    TEST_ASSERT_EQUAL_UINT32( 3u, utAdc_GpioInitCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_A,          utAdc_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_0,        utAdc_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ANALOG, utAdc_GpioConfig.PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE,   utAdc_GpioConfig.PinPull );

    TEST_ASSERT_NOT_EQUAL( 0u, LL_ADC_GetChannelSingleDiff( UT_ADC_REG, LL_ADC_CHANNEL_3 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, LL_ADC_GetChannelSingleDiff( UT_ADC_REG, LL_ADC_CHANNEL_1 ) );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS, LL_ADC_REG_GetSequencerLength( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_UINT32( 3u, __LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_REG_GetSequencerRanks( UT_ADC_REG, LL_ADC_REG_RANK_1 ) ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, __LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_REG_GetSequencerRanks( UT_ADC_REG, LL_ADC_REG_RANK_2 ) ) );
}


/**
 * \brief   Adc_PeriphInit() configures external regular trigger.
 *
 * \details Initializes ADC1 with TIM1 TRGO trigger, falling edge, continuous mode
 *          and 10-bit resolution.
 *
 * \par Expected results
 * - CFGR: trigger source TIM1 TRGO, falling edge, continuous mode, 10-bit resolution.
 * - Adc_Get_TriggerSrc() / Adc_Get_TriggerEdge() / Adc_Get_TriggerMode() read back
 *   the configured values.
 */
void Ut_Adc_PeriphInit_ExternalTriggerContinuous_TriggerConfigured( void )
{
    adc_PeriphConfig_t   periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    adc_RegTriggerId_t   triggerSrc   = ADC_REG_TRIGGER_CNT;
    adc_TriggerEdge_t    triggerEdge  = ADC_TRIGGER_EDGE_CNT;
    adc_RegTriggerMode_t triggerMode  = ADC_REG_TRIGGER_MODE_CNT;

    periphConfig.RegTriggerId   = ADC_REG_TRIGGER_EXT_TIM1_TRGO;
    periphConfig.RegTriggerEdge = ADC_TRIGGER_EDGE_FALLING;
    periphConfig.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    periphConfig.Resolution     = ADC_RESOLUTION_10BIT;

    Ut_Adc_PeriphInit( &periphConfig );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_TIM1_TRGO, LL_ADC_REG_GetTriggerSource( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_FALLING, LL_ADC_REG_GetTriggerEdge( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_CONV_CONTINUOUS, LL_ADC_REG_GetContinuousMode( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_RESOLUTION_10B, LL_ADC_GetResolution( UT_ADC_REG ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerSrc( UT_ADC_PERIPH, &triggerSrc ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_EXT_TIM1_TRGO, triggerSrc );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerEdge( UT_ADC_PERIPH, &triggerEdge ) );
    TEST_ASSERT_EQUAL( ADC_TRIGGER_EDGE_FALLING, triggerEdge );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerMode( UT_ADC_PERIPH, &triggerMode ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_MODE_CONTINUOUS, triggerMode );
}


/**
 * \brief   Adc_PeriphInit() configures injected sequence.
 *
 * \details Initializes ADC1 with injected group only: VREF (channel 18) and
 *          temperature sensor (channel 16), software trigger, continuous mode.
 *
 * \par Expected results
 * - Injected sequence length 2, rank 1 = channel 18, rank 2 = channel 16.
 * - Injected group triggered independently (JAUTO = 0).
 * - VREFINT and temperature sensor internal paths are enabled.
 */
void Ut_Adc_PeriphInit_InjectedSequence_QueueConfigured( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    periphConfig.RegChannelsCnt = 0u;
    periphConfig.InjChannelsCnt = 2u;
    periphConfig.InjTriggerId   = ADC_INJ_TRIGGER_SOFTWARE;
    periphConfig.InjTriggerMode = ADC_INJ_TRIGGER_MODE_CONTINUOUS;
    periphConfig.InjChannels[ 0u ].ChannelId       = UT_ADC_VREF_CHANNEL;
    periphConfig.InjChannels[ 0u ].ChannelInput    = ADC_CHANNEL_INPUT_VREF;
    periphConfig.InjChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_640_5_CYCLES;
    periphConfig.InjChannels[ 1u ].ChannelId       = UT_ADC_TEMP_CHANNEL;
    periphConfig.InjChannels[ 1u ].ChannelInput    = ADC_CHANNEL_INPUT_TEMP;
    periphConfig.InjChannels[ 1u ].ChannelSampling = ADC_CHANNEL_SAMPLING_640_5_CYCLES;

    Ut_Adc_PeriphInit( &periphConfig );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS, LL_ADC_INJ_GetSequencerLength( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ADC_VREF_CHANNEL, __LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_INJ_GetSequencerRanks( UT_ADC_REG, LL_ADC_INJ_RANK_1 ) ) );
    TEST_ASSERT_EQUAL_UINT32( UT_ADC_TEMP_CHANNEL, __LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_INJ_GetSequencerRanks( UT_ADC_REG, LL_ADC_INJ_RANK_2 ) ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_INJ_TRIG_INDEPENDENT, LL_ADC_INJ_GetTrigAuto( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR,
                             LL_ADC_GetCommonPathInternalCh( UT_ADC_COMMON ) & ( LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR ) );
}


/**
 * \brief   Adc_PeriphInit() configures automatic injected conversion.
 *
 * \details Initializes ADC1 with one regular and one injected VREF channel,
 *          injected trigger ADC_INJ_TRIGGER_AUTO, then starts injected group.
 *
 * \par Expected results
 * - Injected group is triggered from regular group (JAUTO = 1).
 * - Adc_Set_InjStart() returns ADC_REQUEST_ERROR (group is started by regular
 *   group only).
 */
void Ut_Adc_PeriphInit_AutoInjected_TrigAutoEnabled( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    periphConfig.InjChannelsCnt    = 1u;
    periphConfig.InjTriggerId      = ADC_INJ_TRIGGER_AUTO;
    periphConfig.InjTriggerMode    = ADC_INJ_TRIGGER_MODE_CONTINUOUS;
    periphConfig.InjChannels[ 0u ] = periphConfig.RegChannels[ 0u ];

    Ut_Adc_PeriphInit( &periphConfig );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_INJ_TRIG_FROM_GRP_REGULAR, LL_ADC_INJ_GetTrigAuto( UT_ADC_REG ) );

    /* Injected group started by regular group only */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_InjStart( UT_ADC_PERIPH ) );
}


/**
 * \brief   Adc_PeriphInit() in ISR transfer mode configures NVIC and interrupts.
 *
 * \details Initializes ADC1 in ISR mode with data buffer and injected complete
 *          callback. NVIC priority and activation are expected, handler is
 *          captured by stub.
 *
 * \par Expected results
 * - ADC_REQUEST_OK, NVIC priority UT_ADC_PRIO is set and IRQ activated.
 * - ISR is registered in NVIC.
 * - Overrun mode is "data overwritten", IER contains only JEOS (regular
 *   interrupts are enabled by conversion start).
 */
void Ut_Adc_PeriphInit_IsrMode_InterruptConfigured( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, UT_ADC_BUF_SIZE );

    periphConfig.DataConfig.InjCompleteCallback = Ut_Adc_InjCallback;

    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ModelActive();
    Nvic_Set_PeriphIrq_Prio_StopIgnore();
    Nvic_Set_PeriphIrq_Active_StopIgnore();
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_ADC_NVIC, UT_ADC_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_ADC_NVIC, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( &periphConfig ) );

    TEST_ASSERT_NOT_NULL( utAdc_Isr );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_OVR_DATA_OVERWRITTEN, LL_ADC_REG_GetOverrun( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_IT_JEOS, UT_ADC_REG->IER );
}


/**
 * \brief   Adc_Deinit() disables ADC and releases the module context.
 *
 * \details Initializes ADC1 in ISR mode, deinitializes it (NVIC IRQ deactivation
 *          expected), then deinitializes invalid peripheral.
 *
 * \par Expected results
 * - ADC_REQUEST_OK, NVIC IRQ deactivated, ADEN = ADVREGEN = 0.
 * - Channel input is reset to default pin single-ended input.
 * - Adc_Get_DataConfig() returns ADC_REQUEST_ERROR (no data handling).
 * - Adc_Deinit( ADC_PERIPH_CNT ) returns ADC_REQUEST_ERROR.
 */
void Ut_Adc_Deinit_InitializedPeripheral_DisabledAndInputsReset( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, UT_ADC_BUF_SIZE );
    adc_ChannelInput_t input        = ADC_CHANNEL_INPUT_CNT;
    adc_DataConfig_t   readConfig;

    Ut_Adc_PeriphInit( &periphConfig );
    Ut_Adc_Reset_Mocks();

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_ADC_NVIC, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( UT_ADC_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ( ADC_CR_ADEN | ADC_CR_ADVREGEN ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ChannelInput( UT_ADC_PERIPH, UT_ADC_VREF_CHANNEL, &input ) );
    TEST_ASSERT_EQUAL( ADC_CHANNEL_INPUT_PIN_SINGLE, input );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_DataConfig( UT_ADC_PERIPH, &readConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Deinit( ADC_PERIPH_CNT ) );
}


/**
 * \brief   Adc_Set_PeriphActive() calibrates and enables the ADC.
 *
 * \details Presets enabled voltage regulator, HW model ends calibration and the
 *          module enables ADC, then the function is called again.
 *
 * \par Expected results
 * - First call: ADC_REQUEST_OK, ADEN = 1, calibration finished (ADCAL = 0),
 *   ADRDY is set.
 * - Second call (already enabled): ADC_REQUEST_ERROR.
 */
void Ut_Adc_Set_PeriphActive_RegulatorOn_CalibratedAndEnabled( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Ut_Adc_Set_ModelActive();

    UT_ADC_REG->CR = ADC_CR_ADVREGEN;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_PeriphActive( UT_ADC_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( ADC_CR_ADEN, UT_ADC_REG->CR & ( ADC_CR_ADEN | ADC_CR_ADCAL ) );
    TEST_ASSERT_EQUAL_HEX32( ADC_ISR_ADRDY, UT_ADC_REG->ISR & ADC_ISR_ADRDY );

    /* Already enabled */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_PeriphActive( UT_ADC_PERIPH ) );
}


/**
 * \brief   Adc_Set_PeriphActive() is rejected without voltage regulator.
 *
 * \details Calls Adc_Set_PeriphActive() with disabled voltage regulator and for
 *          invalid peripheral.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in both cases.
 * - CR register is not written (stays 0).
 */
void Ut_Adc_Set_PeriphActive_NotReady_ReturnsErrorWithoutEnable( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    /* Voltage regulator not enabled */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_PeriphActive( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_PeriphActive( ADC_PERIPH_CNT ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR );
}


/**
 * \brief   Adc_Set_PeriphActive() reports calibration timeout.
 *
 * \details Presets enabled voltage regulator without HW model - ADCAL is never
 *          cleared by HW.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned (timeout).
 * - ADC is not enabled (ADEN = 0).
 */
void Ut_Adc_Set_PeriphActive_CalibrationTimeout_ReturnsErrorWithoutEnable( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    /* Without HW model the calibration never ends */
    UT_ADC_REG->CR = ADC_CR_ADVREGEN;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_PeriphActive( UT_ADC_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ADC_CR_ADEN );
}


/**
 * \brief   Adc_Set_PeriphInactive() disables enabled ADC.
 *
 * \details Presets enabled and ready ADC, HW model clears ADEN on disable request.
 *
 * \par Expected results
 * - ADC_REQUEST_OK is returned.
 * - ADEN = ADDIS = 0.
 */
void Ut_Adc_Set_PeriphInactive_EnabledPeripheral_Disabled( void )
{
    Ut_Adc_Set_ModelActive();
    Ut_Adc_Set_Enabled();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_PeriphInactive( UT_ADC_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ( ADC_CR_ADEN | ADC_CR_ADDIS ) );
}


/**
 * \brief   Adc_Set_PeriphInactive() is rejected during conversion.
 *
 * \details Presets enabled ADC with ongoing regular conversion (ADSTART = 1).
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned.
 * - ADC stays enabled (ADEN = 1, ADDIS = 0).
 */
void Ut_Adc_Set_PeriphInactive_ConversionOngoing_ReturnsErrorStaysEnabled( void )
{
    Ut_Adc_Set_Enabled();
    UT_ADC_REG->CR |= ADC_CR_ADSTART;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_PeriphInactive( UT_ADC_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( ADC_CR_ADEN, UT_ADC_REG->CR & ( ADC_CR_ADEN | ADC_CR_ADDIS ) );
}


/**
 * \brief   Adc_Set_Resolution() / Adc_Get_Resolution() handle all resolutions.
 *
 * \details Sets every resolution (12, 10, 8, 6 bit), then invalid resolution,
 *          NULL read pointer and change of resolution during conversion.
 *
 * \par Expected results
 * - Every valid resolution: ADC_REQUEST_OK, CFGR RES matches, read back equals.
 * - Invalid value and NULL pointer: ADC_REQUEST_ERROR.
 * - During conversion (ADSTART = 1): ADC_REQUEST_ERROR, resolution stays 6-bit.
 */
void Ut_Adc_Set_Resolution_AllOptions_RegisterAndReadBack( void )
{
    const uint32_t   llRes[] = { LL_ADC_RESOLUTION_12B, LL_ADC_RESOLUTION_10B, LL_ADC_RESOLUTION_8B, LL_ADC_RESOLUTION_6B };
    adc_Resolution_t res     = ADC_RESOLUTION_CNT;

    for( adc_Resolution_t idx = ADC_RESOLUTION_12BIT; ADC_RESOLUTION_CNT > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_Resolution( UT_ADC_PERIPH, idx ) );
        TEST_ASSERT_EQUAL_HEX32( llRes[ idx ], LL_ADC_GetResolution( UT_ADC_REG ) );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Resolution( UT_ADC_PERIPH, &res ) );
        TEST_ASSERT_EQUAL( idx, res );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_Resolution( UT_ADC_PERIPH, ADC_RESOLUTION_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_Resolution( UT_ADC_PERIPH, NULL ) );

    /* Conversion ongoing */
    UT_ADC_REG->CR = ADC_CR_ADSTART;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_Resolution( UT_ADC_PERIPH, ADC_RESOLUTION_12BIT ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_RESOLUTION_6B, LL_ADC_GetResolution( UT_ADC_REG ) );
}


/**
 * \brief   Adc_Set_SamplingTime() / Adc_Get_SamplingTime() handle all sampling times.
 *
 * \details Sets every sampling time of external channel 12, then invalid channel,
 *          invalid sampling time and NULL read pointer.
 *
 * \par Expected results
 * - Every valid sampling time: ADC_REQUEST_OK and read back value equals.
 * - Invalid channel, invalid sampling time, NULL pointer: ADC_REQUEST_ERROR.
 */
void Ut_Adc_Set_SamplingTime_AllOptions_RegisterAndReadBack( void )
{
    adc_ChannelSampling_t sampling = ADC_CHANNEL_SAMPLING_CNT;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    for( adc_ChannelSampling_t idx = ADC_CHANNEL_SAMPLING_2_5_CYCLES; ADC_CHANNEL_SAMPLING_CNT > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_12, idx ) );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_12, &sampling ) );
        TEST_ASSERT_EQUAL( idx, sampling );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_CNT, ADC_CHANNEL_SAMPLING_2_5_CYCLES ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_12, ADC_CHANNEL_SAMPLING_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_12, NULL ) );
}


/**
 * \brief   Adc_Set_SamplingTime() rejects too short sampling of internal channel.
 *
 * \details Initializes VREF channel, disables ADC and sets 47.5 cycles sampling
 *          time of VREF channel (shorter than required minimum).
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned.
 * - Sampling time of the channel stays 640.5 cycles.
 */
void Ut_Adc_Set_SamplingTime_InternalChannelTooShort_ReturnsError( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    Ut_Adc_PeriphInit( &periphConfig );
    UT_ADC_REG->CR &= ~ADC_CR_ADEN;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_SamplingTime( UT_ADC_PERIPH, UT_ADC_VREF_CHANNEL, ADC_CHANNEL_SAMPLING_47_5_CYCLES ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_SAMPLINGTIME_640CYCLES_5,
                             LL_ADC_GetChannelSamplingTime( UT_ADC_REG, __LL_ADC_DECIMAL_NB_TO_CHANNEL( UT_ADC_VREF_CHANNEL ) ) );
}


/**
 * \brief   Adc_ChannelInit() is rejected for enabled ADC and NULL configuration.
 *
 * \details Presets ADEN and initializes pin channel 3, then calls the function
 *          with NULL configuration.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in both cases.
 * - GPIO is not initialized (no Gpio_Init call expected).
 */
void Ut_Adc_ChannelInit_PeripheralEnabled_ReturnsErrorWithoutGpio( void )
{
    adc_ChannelConfig_t channelConfig = { .ChannelId = ADC_CHANNEL_3, .ChannelInput = ADC_CHANNEL_INPUT_PIN_SINGLE, .ChannelSampling = ADC_CHANNEL_SAMPLING_12_5_CYCLES };

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    UT_ADC_REG->CR = ADC_CR_ADEN;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_ChannelInit( UT_ADC_PERIPH, &channelConfig ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_ChannelInit( UT_ADC_PERIPH, NULL ) );
}


/**
 * \brief   STM32G4 channel inputs: pins and internal signals per peripheral.
 *
 * \details Adc_Set_ChannelInput() of disabled peripherals:
 * 1. ADC1 channel 0 (no pin), channel 12 / 15 differential (pin of channel 13 / 16 missing).
 * 2. ADC1 VBAT on channel 17, OPAMP1 on channel 13 (MCUs with OPAMP1).
 * 3. ADC2 VREF (not connected to ADC2), OPAMP2 on channel 16 (MCUs with OPAMP2), pin channel 17 (PA4).
 * 4. ADC5 temperature sensor on channel 4 (MCUs with ADC5).
 *
 * \par Expected results
 * 1. ADC_REQUEST_ERROR, GPIO not initialized.
 * 2. ADC_REQUEST_OK, VBATSEL set in ADC12_COMMON, inputs read back.
 * 3. VREF refused, OPAMP2 accepted, PA4 configured as analog.
 * 4. ADC_REQUEST_OK, VSENSESEL set in ADC345_COMMON.
 */
void Ut_Adc_Set_ChannelInput_G4Inputs_PinsAndInternalSignals( void )
{
    adc_ChannelInput_t input = ADC_CHANNEL_INPUT_CNT;

    Ut_Adc_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ChannelInput( UT_ADC_PERIPH, ADC_CHANNEL_0, ADC_CHANNEL_INPUT_PIN_SINGLE ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ChannelInput( UT_ADC_PERIPH, ADC_CHANNEL_12, ADC_CHANNEL_INPUT_PIN_DIFF ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ChannelInput( UT_ADC_PERIPH, ADC_CHANNEL_15, ADC_CHANNEL_INPUT_PIN_DIFF ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_GpioInitCnt );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ChannelInput( UT_ADC_PERIPH, ADC_CHANNEL_17, ADC_CHANNEL_INPUT_VBAT ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_PATH_INTERNAL_VBAT, LL_ADC_GetCommonPathInternalCh( UT_ADC_COMMON ) & LL_ADC_PATH_INTERNAL_VBAT );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ChannelInput( UT_ADC_PERIPH, ADC_CHANNEL_17, &input ) );
    TEST_ASSERT_EQUAL( ADC_CHANNEL_INPUT_VBAT, input );
#if defined(OPAMP1)
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ChannelInput( UT_ADC_PERIPH, ADC_CHANNEL_13, ADC_CHANNEL_INPUT_OPAMP1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ChannelInput( UT_ADC_PERIPH, ADC_CHANNEL_14, ADC_CHANNEL_INPUT_OPAMP1 ) );
#endif /* OPAMP1 */

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ChannelInput( ADC_PERIPH_2, ADC_CHANNEL_18, ADC_CHANNEL_INPUT_VREF ) );
#if defined(OPAMP2)
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ChannelInput( ADC_PERIPH_2, ADC_CHANNEL_16, ADC_CHANNEL_INPUT_OPAMP2 ) );
#endif /* OPAMP2 */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ChannelInput( ADC_PERIPH_2, ADC_CHANNEL_17, ADC_CHANNEL_INPUT_PIN_SINGLE ) );
    TEST_ASSERT_EQUAL( GPIO_PORT_A, utAdc_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_4, utAdc_GpioConfig.PinId );

#if defined(ADC5)
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ChannelInput( ADC_PERIPH_5, ADC_CHANNEL_4, ADC_CHANNEL_INPUT_TEMP ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_PATH_INTERNAL_TEMPSENSOR, LL_ADC_GetCommonPathInternalCh( ADC345_COMMON ) & LL_ADC_PATH_INTERNAL_TEMPSENSOR );
#endif /* ADC5 */
}


/**
 * \brief   Adc_Set_TriggerSrc() handles external and software trigger and its edge.
 *
 * \details
 * 1. Sets external trigger TIM2 TRGO.
 * 2. Sets both edges, then changes source to TIM6 TRGO.
 * 3. Sets software trigger, then tries to set falling edge.
 *
 * \par Expected results
 * 1. Trigger source TIM2 TRGO with default rising edge.
 * 2. Edge "rising and falling" is kept, source reads back TIM6 TRGO.
 * 3. Software start is selected and reads back, Adc_Set_TriggerEdge() returns
 *    ADC_REQUEST_ERROR.
 */
void Ut_Adc_Set_TriggerSrc_ExternalAndSoftware_EdgeHandled( void )
{
    adc_RegTriggerId_t triggerSrc = ADC_REG_TRIGGER_CNT;

    /* External trigger - default rising edge */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_EXT_TIM2_TRGO ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_TIM2_TRGO, LL_ADC_REG_GetTriggerSource( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_RISING, LL_ADC_REG_GetTriggerEdge( UT_ADC_REG ) );

    /* Edge is kept when the external source changes */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerEdge( UT_ADC_PERIPH, ADC_TRIGGER_EDGE_BOTH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_EXT_TIM6_TRGO ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_RISINGFALLING, LL_ADC_REG_GetTriggerEdge( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerSrc( UT_ADC_PERIPH, &triggerSrc ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_EXT_TIM6_TRGO, triggerSrc );

    /* Software trigger - external trigger disabled, edge can not be set */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_SOFTWARE ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, LL_ADC_REG_IsTriggerSourceSWStart( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerSrc( UT_ADC_PERIPH, &triggerSrc ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_SOFTWARE, triggerSrc );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerEdge( UT_ADC_PERIPH, ADC_TRIGGER_EDGE_FALLING ) );
}


/**
 * \brief   Trigger setters reject invalid values and change during conversion.
 *
 * \details
 * 1. Sets invalid trigger source, mode and edge, reads source to NULL pointer.
 * 2. Presets ongoing conversion (ADSTART) and sets valid source and mode.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 * - CFGR register is not written (stays 0).
 */
void Ut_Adc_Set_Trigger_InvalidOrConversionOngoing_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerMode( UT_ADC_PERIPH, ADC_REG_TRIGGER_MODE_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerEdge( UT_ADC_PERIPH, ADC_TRIGGER_EDGE_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_TriggerSrc( UT_ADC_PERIPH, NULL ) );

    UT_ADC_REG->CR = ADC_CR_ADSTART;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_EXT_TIM1_TRGO ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerMode( UT_ADC_PERIPH, ADC_REG_TRIGGER_MODE_CONTINUOUS ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CFGR );
}


/**
 * \brief   Adc_Set_TriggerMode() switches single and continuous conversion.
 *
 * \details Sets continuous mode, then single mode.
 *
 * \par Expected results
 * - CFGR CONT matches the set mode and Adc_Get_TriggerMode() reads it back.
 */
void Ut_Adc_Set_TriggerMode_SingleContinuous_RegisterAndReadBack( void )
{
    adc_RegTriggerMode_t triggerMode = ADC_REG_TRIGGER_MODE_CNT;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerMode( UT_ADC_PERIPH, ADC_REG_TRIGGER_MODE_CONTINUOUS ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_CONV_CONTINUOUS, LL_ADC_REG_GetContinuousMode( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerMode( UT_ADC_PERIPH, &triggerMode ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_MODE_CONTINUOUS, triggerMode );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerMode( UT_ADC_PERIPH, ADC_REG_TRIGGER_MODE_SINGLE ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_CONV_SINGLE, LL_ADC_REG_GetContinuousMode( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerMode( UT_ADC_PERIPH, &triggerMode ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_MODE_SINGLE, triggerMode );
}


/**
 * \brief   Trigger connection depends on the ADC group (ADC1 / ADC2 or ADC3 / ADC4 / ADC5).
 *
 * \details
 * 1. ADC1: TIM1 channel 1 (ADC1 / ADC2 only), TIM2 channel 1 (ADC3 / ADC4 / ADC5 only).
 * 2. ADC1 initialization with regular trigger TIM8 channel 1 and with injected trigger
 *    TIM4 channel 3 (ADC3 / ADC4 / ADC5 only).
 * 3. ADC3 (MCUs with ADC3): TIM2 channel 1 accepted, TIM1 channel 1 refused.
 *
 * \par Expected results
 * 1. TIM1 channel 1: ADC_REQUEST_OK, read back; TIM2 channel 1: ADC_REQUEST_ERROR, trigger kept.
 * 2. Adc_PeriphInit(): ADC_REQUEST_ERROR without register access (ADC stays powered down).
 * 3. ADC3: TIM2 channel 1 read back, TIM1 channel 1 refused.
 */
void Ut_Adc_Set_TriggerSrc_GroupConnection_UnconnectedRefused( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    adc_RegTriggerId_t triggerSrc   = ADC_REG_TRIGGER_CNT;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_EXT_TIM1_CH1 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_TIM1_CH1, LL_ADC_REG_GetTriggerSource( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerSrc( UT_ADC_PERIPH, &triggerSrc ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_EXT_TIM1_CH1, triggerSrc );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerSrc( UT_ADC_PERIPH, ADC_REG_TRIGGER_EXT_TIM2_CH1 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_TRIG_EXT_TIM1_CH1, LL_ADC_REG_GetTriggerSource( UT_ADC_REG ) );

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    periphConfig.RegTriggerId = ADC_REG_TRIGGER_EXT_TIM8_CH1;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.InjChannelsCnt = 1u;
    periphConfig.InjChannels[ 0u ] = periphConfig.RegChannels[ 0u ];
    periphConfig.InjTriggerId   = ADC_INJ_TRIGGER_EXT_TIM4_CH3;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ( ADC_CR_ADVREGEN | ADC_CR_ADEN ) );

#if defined(ADC3)
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_TriggerSrc( ADC_PERIPH_3, ADC_REG_TRIGGER_EXT_TIM2_CH1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerSrc( ADC_PERIPH_3, &triggerSrc ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_EXT_TIM2_CH1, triggerSrc );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_TriggerSrc( ADC_PERIPH_3, ADC_REG_TRIGGER_EXT_TIM1_CH1 ) );
#endif /* ADC3 */
}


/**
 * \brief   Adc_Set_RegStart() is rejected for not ready ADC.
 *
 * \details Starts regular conversion of disabled ADC, of invalid peripheral and
 *          of enabled ADC without ADRDY.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 * - Conversion is not started (ADSTART = 0).
 */
void Ut_Adc_Set_RegStart_NotReady_ReturnsErrorWithoutStart( void )
{
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_RegStart( ADC_PERIPH_CNT ) );

    /* Enabled but not ready */
    UT_ADC_REG->CR = ADC_CR_ADEN;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ADC_CR_ADSTART );
}


/**
 * \brief   Regular conversion without data buffer - start, user read, stop.
 *
 * \details Initializes polling mode without buffer, starts conversion twice,
 *          presets DR = 0x5A5, reads data and stops the conversion (HW model
 *          handles stop request).
 *
 * \par Expected results
 * - First start: ADC_REQUEST_OK, ADSTART = 1. Second start: ADC_REQUEST_ERROR.
 * - Adc_Get_RegData() returns 0x5A5.
 * - Adc_Set_RegStop(): ADC_REQUEST_OK, ADSTART = ADSTP = 0.
 */
void Ut_Adc_Set_RegStart_ManualPolling_StartedAndDataRead( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    adc_Data_t         data         = 0u;

    Ut_Adc_PeriphInit( &periphConfig );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( ADC_CR_ADSTART, UT_ADC_REG->CR & ADC_CR_ADSTART );

    /* Second start while converting */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    /* Without data buffer the result is read by the user */
    UT_ADC_REG->DR = 0x5A5u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_RegData( UT_ADC_PERIPH, &data ) );
    TEST_ASSERT_EQUAL_HEX16( 0x5A5u, data );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ( ADC_CR_ADSTART | ADC_CR_ADSTP ) );
}


/**
 * \brief   Injected conversion start, data read and stop.
 *
 * \details Presets enabled and ready ADC, starts injected group twice, presets
 *          JDR2 = 0x321, reads injected data of ranks 2 and invalid rank, then
 *          stops the group (HW model handles stop request).
 *
 * \par Expected results
 * - First start: ADC_REQUEST_OK, JADSTART = 1. Second start: ADC_REQUEST_ERROR.
 * - Rank 2 data is 0x321, invalid rank returns ADC_REQUEST_ERROR.
 * - Adc_Set_InjStop(): ADC_REQUEST_OK, JADSTART = JADSTP = 0.
 */
void Ut_Adc_Set_InjStart_ReadyPeripheral_StartedAndStopped( void )
{
    adc_Data_t data = 0u;

    /* Injected stop evaluates the AHB / ADC clock ratio (device errata ES0430 2.7.5) */
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Ut_Adc_Set_ModelActive();
    Ut_Adc_Set_Enabled();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( ADC_CR_JADSTART, UT_ADC_REG->CR & ADC_CR_JADSTART );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_InjStart( UT_ADC_PERIPH ) );

    UT_ADC_REG->JDR2 = 0x321u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_InjData( UT_ADC_PERIPH, ADC_INJ_SEQUENCE_2, &data ) );
    TEST_ASSERT_EQUAL_HEX16( 0x321u, data );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_InjData( UT_ADC_PERIPH, ADC_INJ_SEQUENCE_CNT, &data ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStop( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ( ADC_CR_JADSTART | ADC_CR_JADSTP ) );
}


/**
 * \brief   Initialization disables the injected context queue.
 *
 * \details Emulated CFGR is 0 (JQDIS = 0, queue enabled) before Adc_PeriphInit().
 *
 * \note    Device errata ES0430 2.7.1 / 2.7.2 (ES0431 2.5.1 / 2.5.2, ES0523 2.6.1 / 2.6.2) - new
 *          injected context started without trigger / merged with JQDIS = 0 and JQM = 0.
 *
 * \par Expected results
 * - JQDIS = 1 after initialization (queue disabled).
 */
void Ut_Adc_PeriphInit_InjectedQueue_Disabled( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CFGR & ADC_CFGR_JQDIS );

    Ut_Adc_PeriphInit( &periphConfig );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_INJ_QUEUE_DISABLE, LL_ADC_INJ_GetQueueMode( UT_ADC_REG ) );
}


/**
 * \brief   Conversion start after a software stop executes the hardware dummy conversion.
 *
 * \details ADC1 polling, continuous mode:
 * 1. Start, stop (ADSTP), start.
 * 2. Stop, injected conversion running (JADSTART preset), start.
 *
 * \note    Device errata ES0430 2.7.10 / ES0431 2.5.10 / ES0523 2.6.8 - the first conversion after a
 *          software stop may convert channel 0 (workaround: ADDIS + ADEN).
 *
 * \note    Emulated CR is plain memory - the LL writes of ADSTART / ADSTP (MODIFY_REG over all "rs"
 *          bits) clear ADEN, which has no effect on HW. ADEN is set again by the test after each
 *          start / stop.
 *
 * \par Expected results
 * 1. First start without disable request, start after the stop: ADC disabled once by ADDIS and
 *    enabled again, conversion started (ADSTART = 1).
 * 2. ADC can not be disabled with running injected conversion - start without disable request.
 */
void Ut_Adc_Set_RegStart_AfterSoftwareStop_DummyConversion( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    periphConfig.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    Ut_Adc_PeriphInit( &periphConfig );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_ModelDisableCnt[ UT_ADC_PERIPH ] );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( UT_ADC_PERIPH ) );
    UT_ADC_REG->CR |= ADC_CR_ADEN;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_ModelDisableCnt[ UT_ADC_PERIPH ] );
    TEST_ASSERT_BITS_HIGH( ADC_CR_ADSTART, UT_ADC_REG->CR );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStop( UT_ADC_PERIPH ) );
    UT_ADC_REG->CR |= ADC_CR_ADEN | ADC_CR_JADSTART;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_ModelDisableCnt[ UT_ADC_PERIPH ] );
}


/**
 * \brief   Injected stop with AHB / ADC clock ratio above 10 waits for the end of the sequence.
 *
 * \details ADC clock SYSCLK 170 MHz / 16 = 10.6 MHz (AHB 48 MHz in the RCC stub is not used for
 *          the ratio - HCLK 48 MHz / 10.6 MHz < 10), injected group started:
 * 1. AHB clock stub 170 MHz: HW model sets JEOS on request, injected stop requested.
 * 2. Ratio below 10 (default HCLK / 4): injected stop without JEOS.
 *
 * \note    Device errata ES0430 2.7.5 / ES0431 2.5.5 / ES0523 2.6.5 - injected data stored in the
 *          wrong ADC_JDRx if JADSTP coincides with the end of conversion (workaround: JADSTP after JEOS).
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK, JEOS was set when the HW model received JADSTP.
 * 2. ADC_REQUEST_OK, JADSTP issued without JEOS (no wait).
 */
void Ut_Adc_Set_InjStop_HighClockRatio_StopAfterSequenceEnd( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    periphConfig.RegChannelsCnt = 0u;
    periphConfig.InjChannelsCnt = 1u;
    periphConfig.InjChannels[ 0u ].ChannelId       = UT_ADC_VREF_CHANNEL;
    periphConfig.InjChannels[ 0u ].ChannelInput    = ADC_CHANNEL_INPUT_VREF;
    periphConfig.InjChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_640_5_CYCLES;

    /* AHB 170 MHz, ADC SYSCLK 170 MHz / 16 = 10.6 MHz - ratio 16 */
    Ut_Adc_Ignore_PeriphMocks();
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockSource( ADC_CLK_SRC_SYSCLK ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockDivider( ADC_CLK_DIV_16 ) );
    Ut_Adc_PeriphInit( &periphConfig );
    utAdc_HclkHz = UT_ADC_SYSCLK_HZ;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStart( UT_ADC_PERIPH ) );
    utAdc_ModelJeosRequest = 1u;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStop( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( ADC_ISR_JEOS, utAdc_ModelJeosAtStop );

    /* Ratio 48 MHz / 12 MHz = 4 */
    utAdc_ModelJeosRequest = 0u;
    UT_ADC_REG->ISR        = 0u;
    utAdc_HclkHz           = UT_ADC_HCLK_HZ;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockSource( ADC_CLK_SRC_HCLK ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockDivider( ADC_CLK_DIV_4 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( &periphConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStop( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, utAdc_ModelJeosAtStop );
}


/**
 * \brief   Adc_Task() in polling mode fills one-shot buffer and stops conversion.
 *
 * \details Initializes polling mode with 4 samples one-shot buffer and starts
 *          conversion. For every sample DR = 0x100 + index and EOC flag are
 *          preset and Adc_Task() is called. After the transfer one more EOC
 *          is processed.
 *
 * \par Expected results
 * - Adc_Get_RegData() is rejected while the transfer is running.
 * - Half transfer callback after 2nd sample, complete callback after 4th sample.
 * - Buffer contains 0x100 - 0x103, 5th item is untouched.
 * - Conversion is stopped (ADSTART = 0), next EOC is not stored, user data read
 *   is allowed again.
 */
void Ut_Adc_Task_PollOneShot_BufferFilledCallbacksAndStop( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 4u );
    adc_Data_t         data         = 0u;

    Ut_Adc_PeriphInit( &periphConfig );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    /* Result belongs to the running transfer */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_RegData( UT_ADC_PERIPH, &data ) );

    for( uint32_t idx = 0u; 4u > idx; idx++ )
    {
        UT_ADC_REG->DR  = 0x100u + idx;
        UT_ADC_REG->ISR = ADC_ISR_EOC;
        Adc_Task();

        if( 1u == idx )
        {
            TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_HalfCnt );
            TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_CompleteCnt );
        }
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_HEX16( 0x100u, utAdc_Buffer[ 0u ] );
    TEST_ASSERT_EQUAL_HEX16( 0x103u, utAdc_Buffer[ 3u ] );
    TEST_ASSERT_EQUAL_HEX16( 0x000u, utAdc_Buffer[ 4u ] );

    /* One shot buffer - conversion stopped */
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ADC_CR_ADSTART );
    UT_ADC_REG->ISR = ADC_ISR_EOC;
    Adc_Task();
    TEST_ASSERT_EQUAL_HEX16( 0x000u, utAdc_Buffer[ 4u ] );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_RegData( UT_ADC_PERIPH, &data ) );
}


/**
 * \brief   Adc_Task() in polling mode with circular buffer wraps to buffer start.
 *
 * \details Initializes circular buffer of 2 samples, starts conversion and
 *          processes 3 samples (DR = 0x200 + index, EOC flag).
 *
 * \par Expected results
 * - Complete callback 1x, half transfer callback 2x.
 * - Buffer = { 0x202, 0x201 } (3rd sample overwrote the 1st one).
 * - Conversion keeps running (ADSTART = 1).
 */
void Ut_Adc_Task_PollCircular_WrapsToBufferStart( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 2u );

    periphConfig.DataConfig.BufferMode = ADC_BUFFER_MODE_CIRCULAR;

    Ut_Adc_PeriphInit( &periphConfig );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    for( uint32_t idx = 0u; 3u > idx; idx++ )
    {
        UT_ADC_REG->DR  = 0x200u + idx;
        UT_ADC_REG->ISR = ADC_ISR_EOC;
        Adc_Task();
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 2u, utAdc_HalfCnt );
    TEST_ASSERT_EQUAL_HEX16( 0x202u, utAdc_Buffer[ 0u ] );
    TEST_ASSERT_EQUAL_HEX16( 0x201u, utAdc_Buffer[ 1u ] );
    TEST_ASSERT_EQUAL_HEX32( ADC_CR_ADSTART, UT_ADC_REG->CR & ADC_CR_ADSTART );
}


/**
 * \brief   Adc_Task() in polling mode reports overrun and injected sequence end.
 *
 * \details Initializes polling mode with injected complete callback, starts
 *          conversion and processes OVR flag, then JEOS flag.
 *
 * \par Expected results
 * - OVR: error callback with ADC_ERROR_OVERRUN, OVR flag cleared (written 1).
 * - JEOS: injected complete callback, JEOS and JEOC flags cleared.
 * - Data buffer is not written.
 */
void Ut_Adc_Task_PollOverrunAndInjected_Callbacks( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 4u );

    periphConfig.DataConfig.InjCompleteCallback = Ut_Adc_InjCallback;

    Ut_Adc_PeriphInit( &periphConfig );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    UT_ADC_REG->ISR = ADC_ISR_OVR;
    Adc_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_ERROR_OVERRUN, utAdc_LastError );
    TEST_ASSERT_EQUAL_HEX32( ADC_ISR_OVR, UT_ADC_REG->ISR );

    UT_ADC_REG->ISR = ADC_ISR_JEOS;
    Adc_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_InjCnt );
    TEST_ASSERT_EQUAL_HEX32( ADC_ISR_JEOS | ADC_ISR_JEOC, UT_ADC_REG->ISR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_Buffer[ 0u ] );
}


/**
 * \brief   ISR transfer mode - interrupts enabled by start, data stored by ISR.
 *
 * \details Initializes ISR mode with buffer of 2 samples, starts conversion and
 *          calls captured ISR twice with EOC flag (DR = 0xABC, 0xDEF).
 *
 * \par Expected results
 * - IER is 0 after initialization, EOC and OVR interrupts enabled by start.
 * - Buffer = { 0xABC, 0xDEF }, half and complete callback called once.
 * - EOC and OVR interrupts disabled after one-shot transfer.
 */
void Ut_Adc_Isr_Transfer_InterruptsEnabledDataStored( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, 2u );

    Ut_Adc_PeriphInit( &periphConfig );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->IER );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_IT_EOC | LL_ADC_IT_OVR, UT_ADC_REG->IER );

    UT_ADC_REG->DR = 0xABCu;
    Ut_Adc_Call_Isr( ADC_ISR_EOC );
    UT_ADC_REG->DR = 0xDEFu;
    Ut_Adc_Call_Isr( ADC_ISR_EOC );

    TEST_ASSERT_EQUAL_HEX16( 0xABCu, utAdc_Buffer[ 0u ] );
    TEST_ASSERT_EQUAL_HEX16( 0xDEFu, utAdc_Buffer[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_CompleteCnt );

    /* Transfer stopped - interrupts disabled */
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->IER & ( LL_ADC_IT_EOC | LL_ADC_IT_OVR ) );
}


/**
 * \brief   ISR handles only enabled interrupt sources.
 *
 * \details Initializes ISR mode with buffer, calls ISR with EOC, OVR and JEOS
 *          flags before conversion start, then starts conversion and calls ISR
 *          with OVR flag.
 *
 * \par Expected results
 * - Before start: no callback (disabled interrupts are ignored).
 * - After start: error callback with ADC_ERROR_OVERRUN, buffer not written.
 */
void Ut_Adc_Isr_OverrunAndDisabledSources_HandledByEnableMask( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, 2u );

    Ut_Adc_PeriphInit( &periphConfig );

    /* Transfer not started - pending flags with disabled interrupts are ignored */
    Ut_Adc_Call_Isr( ADC_ISR_EOC | ADC_ISR_OVR | ADC_ISR_JEOS );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_ErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_InjCnt );

    UT_ADC_REG->ISR = ADC_ISR_ADRDY;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    Ut_Adc_Call_Isr( ADC_ISR_OVR );

    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_ERROR_OVERRUN, utAdc_LastError );
    TEST_ASSERT_EQUAL_HEX16( 0u, utAdc_Buffer[ 0u ] );
}


/**
 * \brief   ADC1 and ADC2 share the ADC1_2 line - the line is disabled in NVIC only when the last
 *          peripheral releases it.
 *
 * \details ADC1 and ADC2 in ISR mode, ADC2 end of conversion processed by the line handler,
 *          ADC1 deinitialized, then ADC2.
 *
 * \par Expected results
 * - Handler registered on NVIC_PERIPH_IRQ_ADC1_2, ADC2 result stored by the handler.
 * - Deinit of ADC1 (ADC2 still in ISR mode): no Nvic_Set_PeriphIrq_Inactive call.
 * - Deinit of ADC2: Nvic_Set_PeriphIrq_Inactive called once.
 */
void Ut_Adc_Isr_SharedInterrupt_DisabledByLastUser( void )
{
    adc_PeriphConfig_t config1 = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, UT_ADC_BUF_SIZE );
    adc_PeriphConfig_t config2 = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, 2u );

    config2.PeriphId          = ADC_PERIPH_2;
    config2.RegChannels[ 0u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_1, ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_12_5_CYCLES };

    Ut_Adc_PeriphInit( &config1 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( &config2 ) );
    TEST_ASSERT_EQUAL( UT_ADC_NVIC, utAdc_IsrIrqId );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( ADC_PERIPH_2 ) );
    ADC2->DR  = 0x123u;
    ADC2->ISR = ADC_ISR_EOC;
    utAdc_Isr();
    TEST_ASSERT_EQUAL_HEX16( 0x123u, utAdc_Buffer[ 0u ] );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( ADC_PERIPH_1 ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_NvicInactiveCnt );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( ADC_PERIPH_2 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_NvicInactiveCnt );
}


/**
 * \brief   ADC3 / ADC4 / ADC5 use their own interrupt line.
 *
 * \details ADC3 in ISR mode (MCUs with ADC3), pin channel 1 (PB1), end of conversion.
 *
 * \par Expected results
 * - Handler registered on NVIC_PERIPH_IRQ_ADC3, ADC3 result stored, line disabled by deinit.
 */
void Ut_Adc_Isr_Adc3Line_OwnInterrupt( void )
{
#if defined(ADC3)
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, 2u );

    config.PeriphId          = ADC_PERIPH_3;
    config.RegChannels[ 0u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_1, ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_12_5_CYCLES };

    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_ADC3, utAdc_IsrIrqId );
    TEST_ASSERT_EQUAL( GPIO_PORT_B, utAdc_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_1, utAdc_GpioConfig.PinId );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( ADC_PERIPH_3 ) );
    ADC3->DR  = 0x456u;
    ADC3->ISR = ADC_ISR_EOC;
    utAdc_Isr();
    TEST_ASSERT_EQUAL_HEX16( 0x456u, utAdc_Buffer[ 0u ] );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( ADC_PERIPH_3 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_NvicInactiveCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without ADC3" );
#endif /* ADC3 */
}


/**
 * \brief   Interrupt service routine of every ADC line ends with DSB.
 *
 * \details ADC1 (line ADC1_2) and ADC3 / ADC4 / ADC5 (own lines, MCUs with the peripheral) in ISR
 *          mode one by one, captured ISR called once without flags, peripheral deinitialized.
 *
 * \note    Device errata bug AB#1104 (ES0430 / ES0431 / ES0523 2.1.3 "Store immediate
 *          overlapping exception return operation might vector to incorrect interrupt", Arm ID
 *          838869): a buffered store with immediate offset still pending at the exception return
 *          may vector to an incorrect interrupt. Workaround - DSB before the exception return of
 *          every handler.
 *
 * \par Expected results
 * - Every ISR executes exactly one DSB.
 */
void Ut_Adc_Isr_AllLines_EndWithDsb( void )
{
    const struct
    {
        adc_PeriphId_t PeriphId;
        ADC_TypeDef *  PeriphReg;
    }   isrLut[] =
    {
        { ADC_PERIPH_1, ADC1 },
#if defined(ADC3)
        { ADC_PERIPH_3, ADC3 },
#endif /* ADC3 */
#if defined(ADC4)
        { ADC_PERIPH_4, ADC4 },
#endif /* ADC4 */
#if defined(ADC5)
        { ADC_PERIPH_5, ADC5 },
#endif /* ADC5 */
    };

    for( uint32_t idx = 0u; ( sizeof( isrLut ) / sizeof( isrLut[ 0u ] ) ) > idx; idx++ )
    {
        adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_ISR, 2u );

        config.PeriphId          = isrLut[ idx ].PeriphId;
        config.RegChannels[ 0u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_1, ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_12_5_CYCLES };

        utAdc_Isr = NULL;
        Ut_Adc_PeriphInit( &config );
        TEST_ASSERT_NOT_NULL( utAdc_Isr );

        isrLut[ idx ].PeriphReg->ISR = 0u;

        const uint32_t dsbCnt = CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB );

        utAdc_Isr();

        TEST_ASSERT_EQUAL_UINT32( dsbCnt + 1u, CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB ) );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( isrLut[ idx ].PeriphId ) );
    }
}


/* ============================== DMA TRANSFER ============================== */

/**
 * \brief   DMA mode initialization configures ADC DMA requests and the DMA channel.
 *
 * \details ADC1, DMA2 channel 4, circular buffer of 8 samples.
 *
 * \par Expected results
 * - Dma_Init(): DMA2 channel 4, DMAMUX request ADC1, peripheral to memory, circular, 16 bit,
 *   count 8, peripheral address of ADC1 DR, memory address of the buffer.
 * - ADC CFGR DMAEN = 1, DMACFG = 1 (circular), overrun mode data preserved.
 */
void Ut_Adc_Dma_Init_ChannelConfigured( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    config.DataConfig.BufferMode   = ADC_BUFFER_MODE_CIRCULAR;
    config.DataConfig.DmaPeriphId  = ADC_DMA_PERIPH_2;
    config.DataConfig.DmaChannelId = ADC_DMA_CHANNEL_4;
    Ut_Adc_PeriphInit( &config );

    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_DmaInitCnt );
    TEST_ASSERT_EQUAL( DMA_PERIPH_2, utAdc_DmaConfig.DmaPeriphId );
    TEST_ASSERT_EQUAL( DMA_CHANNEL_4, utAdc_DmaConfig.DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_ADC1, utAdc_DmaConfig.PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_PERIPH_TO_MEMORY, utAdc_DmaConfig.Direction );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_CIRCULAR, utAdc_DmaConfig.TransferMode );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, utAdc_DmaConfig.PeriphTransferSize );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_16BIT, utAdc_DmaConfig.MemoryTransferSize );
    TEST_ASSERT_EQUAL_UINT32( UT_ADC_BUF_SIZE, utAdc_DmaConfig.DataCount );
    TEST_ASSERT_EQUAL( (dma_PeriphAddr_t)(uintptr_t)&UT_ADC_REG->DR, utAdc_DmaConfig.PeriphAddress );
    TEST_ASSERT_EQUAL( (dma_MemoryAddr_t)(uintptr_t)utAdc_Buffer, utAdc_DmaConfig.MemoryAddress );
    TEST_ASSERT_BITS_HIGH( ADC_CFGR_DMAEN | ADC_CFGR_DMACFG, UT_ADC_REG->CFGR );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_OVR_DATA_PRESERVED, LL_ADC_REG_GetOverrun( UT_ADC_REG ) );
}


/**
 * \brief   DMA mode rejects not existing DMA peripheral / channel and invalid priority.
 *
 * \par Expected results
 * - ADC_DMA_PERIPH_CNT, ADC_DMA_CHANNEL_CNT: ADC_REQUEST_ERROR, Dma_Init() not called.
 */
void Ut_Adc_Dma_Init_InvalidChannel_ReturnsError( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    Ut_Adc_Ignore_PeriphMocks();

    config.DataConfig.DmaPeriphId = ADC_DMA_PERIPH_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    config.DataConfig.DmaPeriphId  = ADC_DMA_PERIPH_1;
    config.DataConfig.DmaChannelId = ADC_DMA_CHANNEL_CNT;
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
 * - Transfer complete: complete callback 1x, regular conversion stopped (ADSTART = 0),
 *   overrun interrupt disabled.
 */
void Ut_Adc_Dma_Callbacks_EventsReported( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    config.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_BITS_HIGH( LL_ADC_IT_OVR, UT_ADC_REG->IER );

    utAdc_DmaConfig.HalfTransferCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_HalfCnt );

    utAdc_DmaConfig.TransferErrorCallback();
    TEST_ASSERT_EQUAL( ADC_ERROR_DMA_TRANSFER, utAdc_LastError );

    utAdc_DmaConfig.TransferCompleteCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_CompleteCnt );
    TEST_ASSERT_BITS_LOW( ADC_CR_ADSTART, UT_ADC_REG->CR );
    TEST_ASSERT_BITS_LOW( LL_ADC_IT_OVR, UT_ADC_REG->IER );
}


/**
 * \brief   End of a one shot DMA transfer makes the next start execute the hardware dummy
 *          conversion.
 *
 * \details One shot buffer, single conversion mode: start, sequence finished by HW (ADSTART
 *          cleared), DMA transfer complete, start again.
 *
 * \note    Device errata ES0430 2.7.10 / ES0431 2.5.10 / ES0523 2.6.8 - the first conversion after
 *          the end of a one shot DMA transfer may convert channel 0 (workaround: ADDIS + ADEN).
 *
 * \note    Emulated registers are plain memory - the LL write of ADSTART clears ADEN ("rs" bit, no
 *          effect on HW) and the write 1 to clear of OVR clears ADRDY. Both are set again by the
 *          test after the start.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no stop request needed (ADSTART already 0).
 * - Next Adc_Set_RegStart(): ADC disabled once (ADDIS) and enabled again, conversion started.
 */
void Ut_Adc_Dma_OneShotComplete_DummyConversionOnNextStart( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    UT_ADC_REG->CR  |= ADC_CR_ADEN;
    UT_ADC_REG->ISR  = ADC_ISR_ADRDY;

    /* Sequence finished by HW */
    UT_ADC_REG->CR &= ~ADC_CR_ADSTART;
    utAdc_DmaConfig.TransferCompleteCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utAdc_ModelDisableCnt[ UT_ADC_PERIPH ] );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_ModelDisableCnt[ UT_ADC_PERIPH ] );
    TEST_ASSERT_BITS_HIGH( ADC_CR_ADSTART, UT_ADC_REG->CR );
}


/**
 * \brief   Overrun in DMA mode is reported by the ADC interrupt, the transfer keeps running.
 *
 * \par Expected results
 * - Error callback ADC_ERROR_OVERRUN, OVR flag cleared (written), DMA requests kept (DMAEN = 1).
 */
void Ut_Adc_Isr_DmaOverrun_ErrorReported( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    config.RegTriggerMode = ADC_REG_TRIGGER_MODE_CONTINUOUS;
    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );

    Ut_Adc_Call_Isr( ADC_ISR_OVR );

    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_ERROR_OVERRUN, utAdc_LastError );
    TEST_ASSERT_BITS_HIGH( ADC_CFGR_DMAEN, UT_ADC_REG->CFGR );
}


/**
 * \brief   Adc_Deinit() releases the DMA channel of DMA mode.
 *
 * \details ADC1 in DMA mode (DMA1 channel 6), Adc_Deinit() twice.
 *
 * \par Expected results
 * - ADC_REQUEST_OK, DMA channel interrupt disabled (1x), ADC DMA requests disabled (DMAEN,
 *   DMACFG = 0).
 * - Second Adc_Deinit() does not touch the DMA channel again.
 */
void Ut_Adc_Dma_Deinit_ChannelReleased( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    Ut_Adc_PeriphInit( &config );
    TEST_ASSERT_BITS_HIGH( ADC_CFGR_DMAEN, UT_ADC_REG->CFGR );

    Dma_Set_InterruptInactive_StubWithCallback( Ut_Adc_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_DmaIrqOffCnt );
    TEST_ASSERT_BITS_LOW( ADC_CFGR_DMAEN | ADC_CFGR_DMACFG, UT_ADC_REG->CFGR );

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
    Ut_Adc_Set_ModelActive();
    Dma_Init_StubWithCallback( NULL );
    Dma_Init_IgnoreAndReturn( DMA_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &config ) );

    Ut_Adc_Reset_Mocks();
    Ut_Adc_Ignore_PeriphMocks();
    (void)Adc_Deinit( UT_ADC_PERIPH );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( &config ) );

    /* First value of the ignore queue is returned by the next call */
    Ut_Adc_Reset_Mocks();
    Dma_Set_TransferActive_IgnoreAndReturn( DMA_REQUEST_ERROR );
    Ut_Adc_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_RegStart( UT_ADC_PERIPH ) );
}


/**
 * \brief   DMA requests and callbacks of all other ADC peripherals.
 *
 * \details ADC2 .. ADC5 present on the MCU in DMA mode one after another (pin channel 1 of
 *          every peripheral), callbacks captured from Dma_Init() are called.
 *
 * \par Expected results
 * - Dma_Init() with DMAMUX request of the peripheral (DMA_REQ_ADC2 .. DMA_REQ_ADC5).
 * - Half / complete / error callback of the configuration called for every peripheral.
 */
void Ut_Adc_Dma_OtherPeripherals_RequestAndCallbacks( void )
{
    adc_PeriphConfig_t config = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );
    const struct
    {
        adc_PeriphId_t    PeriphId;
        dma_PeriphReqId_t RequestId;
    }   periphLut[ ] =
    {
        { ADC_PERIPH_2, DMA_REQ_ADC2 },
#if defined(ADC3)
        { ADC_PERIPH_3, DMA_REQ_ADC3 },
#endif /* ADC3 */
#if defined(ADC4)
        { ADC_PERIPH_4, DMA_REQ_ADC4 },
#endif /* ADC4 */
#if defined(ADC5)
        { ADC_PERIPH_5, DMA_REQ_ADC5 },
#endif /* ADC5 */
    };

    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ModelActive();

    for( uint32_t idx = 0u; ( sizeof( periphLut ) / sizeof( periphLut[ 0u ] ) ) > idx; idx++ )
    {
        config.PeriphId          = periphLut[ idx ].PeriphId;
        config.RegChannels[ 0u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_1, ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_12_5_CYCLES };

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( &config ) );
        TEST_ASSERT_EQUAL( periphLut[ idx ].RequestId, utAdc_DmaConfig.PeripheralReqId );

        utAdc_DmaConfig.HalfTransferCallback();
        utAdc_DmaConfig.TransferErrorCallback();
        utAdc_DmaConfig.TransferCompleteCallback();

        TEST_ASSERT_EQUAL_UINT32( idx + 1u, utAdc_HalfCnt );
        TEST_ASSERT_EQUAL_UINT32( idx + 1u, utAdc_CompleteCnt );
        TEST_ASSERT_EQUAL( ADC_ERROR_DMA_TRANSFER, utAdc_LastError );

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( periphLut[ idx ].PeriphId ) );
    }
}


/**
 * \brief   Adc_Set_DataConfig() switches data transfer mode.
 *
 * \details Initializes polling mode, then changes data configuration to ISR mode.
 *
 * \par Expected results
 * - ADC_REQUEST_OK, ISR is registered in NVIC.
 * - Overrun mode is "data overwritten".
 * - Adc_Get_DataConfig() reads back ISR transfer mode.
 */
void Ut_Adc_Set_DataConfig_ChangeMode_NewModeInitialized( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 4u );
    adc_DataConfig_t   dataConfig   = periphConfig.DataConfig;
    adc_DataConfig_t   readConfig;

    Ut_Adc_PeriphInit( &periphConfig );

    dataConfig.TransferMode = ADC_TRANSFER_MODE_ISR;
    Ut_Adc_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_DataConfig( UT_ADC_PERIPH, &dataConfig ) );

    TEST_ASSERT_NOT_NULL( utAdc_Isr );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_OVR_DATA_OVERWRITTEN, LL_ADC_REG_GetOverrun( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_DataConfig( UT_ADC_PERIPH, &readConfig ) );
    TEST_ASSERT_EQUAL( ADC_TRANSFER_MODE_ISR, readConfig.TransferMode );
}


/**
 * \brief   Adc_Set_DataConfig() rejects invalid parameters and running transfer.
 *
 * \details Calls Adc_Set_DataConfig() with NULL configuration, invalid peripheral,
 *          invalid buffer mode, and with valid configuration during running transfer.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 */
void Ut_Adc_Set_DataConfig_RunningTransferOrInvalid_ReturnsError( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 4u );
    adc_DataConfig_t   dataConfig   = periphConfig.DataConfig;

    Ut_Adc_PeriphInit( &periphConfig );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_DataConfig( UT_ADC_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_DataConfig( ADC_PERIPH_CNT, &dataConfig ) );

    dataConfig.BufferMode = ADC_BUFFER_MODE_CNT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_DataConfig( UT_ADC_PERIPH, &dataConfig ) );

    /* Transfer running */
    dataConfig = periphConfig.DataConfig;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_DataConfig( UT_ADC_PERIPH, &dataConfig ) );
}


/**
 * \brief   Adc_Get_Flag() / Adc_Clear_Flag() handle all module flags.
 *
 * \details For every flag (EOC, EOS, OVR, JEOC, JEOS, AWD1-3): presets only the
 *          flag, then all other flags, then clears the flag from zeroed ISR.
 *          Finally invalid flag and NULL pointer are used.
 *
 * \par Expected results
 * - Flag is ACTIVE when its ISR bit is set and INACTIVE otherwise.
 * - Clear writes 1 to the flag bit only (write-1-to-clear).
 * - Invalid flag and NULL pointer: ADC_REQUEST_ERROR.
 */
void Ut_Adc_Get_Flag_AllFlags_ReportedAndCleared( void )
{
    const uint32_t  llFlags[] = { LL_ADC_FLAG_EOC, LL_ADC_FLAG_EOS, LL_ADC_FLAG_OVR, LL_ADC_FLAG_JEOC,
                                  LL_ADC_FLAG_JEOS, LL_ADC_FLAG_AWD1, LL_ADC_FLAG_AWD2, LL_ADC_FLAG_AWD3 };
    adc_FlagState_t flagState = ADC_FLAG_INACTIVE;

    for( adc_FlagId_t flagId = ADC_FLAG_REG_EOC; ADC_FLAG_CNT > flagId; flagId++ )
    {
        UT_ADC_REG->ISR = llFlags[ flagId ];
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( UT_ADC_PERIPH, flagId, &flagState ) );
        TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, flagState );

        UT_ADC_REG->ISR = ~llFlags[ flagId ];
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( UT_ADC_PERIPH, flagId, &flagState ) );
        TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, flagState );

        /* Write 1 to clear */
        UT_ADC_REG->ISR = 0u;
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( UT_ADC_PERIPH, flagId ) );
        TEST_ASSERT_EQUAL_HEX32( llFlags[ flagId ], UT_ADC_REG->ISR );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_Flag( UT_ADC_PERIPH, ADC_FLAG_CNT, &flagState ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_Flag( UT_ADC_PERIPH, ADC_FLAG_REG_EOC, NULL ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Clear_Flag( UT_ADC_PERIPH, ADC_FLAG_CNT ) );
}


/**
 * \brief   Adc_AwdInit() configures AWD1 monitoring all regular channels.
 *
 * \details Initializes AWD1, mode all regular channels, thresholds 100 - 3000,
 *          filter 4.
 *
 * \par Expected results
 * - AWD1 monitors all regular channels.
 * - Thresholds read back 100 / 3000, filter reads back 4.
 */
void Ut_Adc_AwdInit_Awd1Regular_ChannelsThresholdsAndFilter( void )
{
    adc_AwdConfig_t    awdConfig = { .AwdId = ADC_AWD_1, .AwdMode = ADC_AWD_MODE_ALL_REGULAR,
                                     .AwdLowThreshold = 100u, .AwdHighThreshold = 3000u, .AwdFilter = ADC_AWD_FILTER_4 };
    adc_AwdThreshold_t low       = 0u;
    adc_AwdThreshold_t high      = 0u;
    adc_AwdFilter_t    filter    = ADC_AWD_FILTER_CNT;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_AWD_ALL_CHANNELS_REG, LL_ADC_GetAnalogWDMonitChannels( UT_ADC_REG, LL_ADC_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 100u, low );
    TEST_ASSERT_EQUAL_UINT16( 3000u, high );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdFilter( UT_ADC_PERIPH, ADC_AWD_1, &filter ) );
    TEST_ASSERT_EQUAL( ADC_AWD_FILTER_4, filter );
}


/**
 * \brief   Adc_AwdInit() configures AWD2 monitoring all channels.
 *
 * \details Initializes AWD2, mode all channels, thresholds 160 - 3200 (12 bit
 *          resolution), no filter.
 * \note    Bug AB#530: AWD2 / AWD3 compare 8 MSB of the result - thresholds were
 *          written unchanged (bits 8 - 11 to reserved bits).
 *
 * \par Expected results
 * - AWD2 monitors all regular and injected channels.
 * - TR2 holds 8 bit thresholds 10 / 200, thresholds read back 160 / 3200.
 */
void Ut_Adc_AwdInit_Awd2AllChannels_ThresholdsSet( void )
{
    adc_AwdConfig_t    awdConfig = { .AwdId = ADC_AWD_2, .AwdMode = ADC_AWD_MODE_ALL,
                                     .AwdLowThreshold = 160u, .AwdHighThreshold = 3200u, .AwdFilter = ADC_AWD_FILTER_NONE };
    adc_AwdThreshold_t low       = 0u;
    adc_AwdThreshold_t high      = 0u;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_AWD_ALL_CHANNELS_REG_INJ, LL_ADC_GetAnalogWDMonitChannels( UT_ADC_REG, LL_ADC_AWD2 ) );
    TEST_ASSERT_EQUAL_HEX32( ( 200u << ADC_TR2_HT2_Pos ) | ( 10u << ADC_TR2_LT2_Pos ), UT_ADC_REG->TR2 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_2, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 160u, low );
    TEST_ASSERT_EQUAL_UINT16( 3200u, high );
}


/**
 * \brief   Analog watchdog thresholds are converted with the configured resolution.
 *
 * \details 8 bit resolution (CFGR.RES preset): AWD1 thresholds 20 - 250, AWD3
 *          thresholds 20 - 250. 12 bit resolution: AWD3 thresholds 100 - 4000,
 *          threshold above the maximum of 8 bit resolution.
 * \note    Bug AB#530: thresholds were not converted to the register format of the
 *          watch-dog (AWD1 12 bit, AWD2 / AWD3 8 bit).
 *
 * \par Expected results
 * - 8 bit: TR1 holds thresholds left aligned to 12 bits (320 / 4000), TR3 holds 20 / 250,
 *   both read back 20 / 250. Threshold 256: ADC_REQUEST_ERROR.
 * - 12 bit: AWD3 thresholds read back 96 / 4000 (4 LSB ignored by HW).
 */
void Ut_Adc_Set_AwdThresholds_Resolution_ConvertedToRegisterFormat( void )
{
    adc_AwdThreshold_t low  = 0u;
    adc_AwdThreshold_t high = 0u;

    LL_ADC_SetResolution( UT_ADC_REG, LL_ADC_RESOLUTION_8B );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, 20u, 250u ) );
    TEST_ASSERT_EQUAL_HEX32( ( 4000u << ADC_TR1_HT1_Pos ) | ( 320u << ADC_TR1_LT1_Pos ), UT_ADC_REG->TR1 & ( ADC_TR1_HT1 | ADC_TR1_LT1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 20u, low );
    TEST_ASSERT_EQUAL_UINT16( 250u, high );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_3, 20u, 250u ) );
    TEST_ASSERT_EQUAL_HEX32( ( 250u << ADC_TR3_HT3_Pos ) | ( 20u << ADC_TR3_LT3_Pos ), UT_ADC_REG->TR3 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_3, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 20u, low );
    TEST_ASSERT_EQUAL_UINT16( 250u, high );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, 0u, 256u ) );

    LL_ADC_SetResolution( UT_ADC_REG, LL_ADC_RESOLUTION_12B );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_3, 100u, 4000u ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_3, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 96u, low );
    TEST_ASSERT_EQUAL_UINT16( 4000u, high );
}


/**
 * \brief   Analog watchdog API rejects modes not supported by HW.
 *
 * \details Calls:
 * - AWD2 with "all regular" mode (AWD2/3 do not distinguish groups),
 * - AWD1 with single channel mode (no channel selector),
 * - invalid AWD and NULL configuration,
 * - AWD2 filter 2 / no filter and AWD2 filter read,
 * - AWD1 threshold and filter change during conversion.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR in all cases except AWD2 "no filter", which returns
 *   ADC_REQUEST_OK.
 */
void Ut_Adc_AwdInit_UnsupportedModes_ReturnsError( void )
{
    adc_AwdConfig_t awdConfig = { .AwdId = ADC_AWD_2, .AwdMode = ADC_AWD_MODE_ALL_REGULAR,
                                  .AwdLowThreshold = 0u, .AwdHighThreshold = 4095u, .AwdFilter = ADC_AWD_FILTER_NONE };
    adc_AwdFilter_t filter    = ADC_AWD_FILTER_CNT;

    /* AWD2 / AWD3 do not distinguish groups */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );

    /* Single channel modes have no channel selector */
    awdConfig.AwdId   = ADC_AWD_1;
    awdConfig.AwdMode = ADC_AWD_MODE_SINGLE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );

    awdConfig.AwdId = ADC_AWD_CNT;
    awdConfig.AwdMode = ADC_AWD_MODE_ALL;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_AwdInit( UT_ADC_PERIPH, NULL ) );

    /* Filtering only on AWD1 */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_AwdFilter( UT_ADC_PERIPH, ADC_AWD_2, ADC_AWD_FILTER_2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_AwdFilter( UT_ADC_PERIPH, ADC_AWD_2, ADC_AWD_FILTER_NONE ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_AwdFilter( UT_ADC_PERIPH, ADC_AWD_2, &filter ) );

    /* Conversion ongoing */
    UT_ADC_REG->CR = ADC_CR_ADSTART;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, 1u, 2u ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_AwdFilter( UT_ADC_PERIPH, ADC_AWD_1, ADC_AWD_FILTER_2 ) );
}


/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief HW model of all ADC peripherals - runs in background thread.
 *
 * Calibration ends immediately, disable request clears ADEN (counted), stop requests clear
 * the conversion start bits. JEOS of ADC1 is set on request of the test, the JEOS state
 * of ADC1 at the injected stop request is recorded.
 */
static void Ut_Adc_HwModel( void )
{
    for( uint32_t regIdx = 0u; UT_ADC_MODEL_CNT > regIdx; regIdx++ )
    {
        ADC_TypeDef * const adcReg = utAdc_ModelRegs[ regIdx ];
        const uint32_t      cr     = adcReg->CR;

        if( 0u != ( cr & ADC_CR_ADCAL ) )
        {
            (void)__atomic_and_fetch( &adcReg->CR, ~ADC_CR_ADCAL, __ATOMIC_SEQ_CST );
        }
        else
        {
            /* Calibration not requested */
        }

        if( 0u != ( cr & ADC_CR_ADDIS ) )
        {
            /* Counted before the disable is visible to the module */
            utAdc_ModelDisableCnt[ regIdx ]++;
            (void)__atomic_and_fetch( &adcReg->CR, ~( ADC_CR_ADDIS | ADC_CR_ADEN ), __ATOMIC_SEQ_CST );
        }
        else
        {
            /* Disable not requested */
        }

        if( 0u != ( cr & ADC_CR_ADSTP ) )
        {
            (void)__atomic_and_fetch( &adcReg->CR, ~( ADC_CR_ADSTP | ADC_CR_ADSTART ), __ATOMIC_SEQ_CST );
        }
        else
        {
            /* Regular stop not requested */
        }

        if( ADC1 == adcReg )
        {
            if( 0u != utAdc_ModelJeosRequest )
            {
                (void)__atomic_or_fetch( &adcReg->ISR, ADC_ISR_JEOS, __ATOMIC_SEQ_CST );
            }
            else
            {
                /* End of injected sequence not requested */
            }

            if( 0u != ( cr & ADC_CR_JADSTP ) )
            {
                utAdc_ModelJeosAtStop = adcReg->ISR & ADC_ISR_JEOS;
            }
            else
            {
                /* Injected stop not requested */
            }
        }
        else
        {
            /* JEOS emulated for ADC1 only */
        }

        if( 0u != ( cr & ADC_CR_JADSTP ) )
        {
            (void)__atomic_and_fetch( &adcReg->CR, ~( ADC_CR_JADSTP | ADC_CR_JADSTART ), __ATOMIC_SEQ_CST );
        }
        else
        {
            /* Injected stop not requested */
        }
    }
}


/**
 * \brief Starts the ADC HW model.
 */
static void Ut_Adc_Set_ModelActive( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( Ut_Adc_HwModel ) );
}


/**
 * \brief RCC clock stub - HCLK (ADC HCLK sources) utAdc_HclkHz, SYSCLK 170 MHz, PLL P 500 kHz.
 */
static rcc_RequestState_t Ut_Adc_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)callCnt;

    switch( periphId )
    {
        case RCC_PERIPH_ADC12_HCLK:
#if defined(ADC345_COMMON)
        case RCC_PERIPH_ADC345_HCLK:
#endif /* ADC345_COMMON */
            *periphClk = utAdc_HclkHz;
            break;

        case RCC_PERIPH_ADC12_SYSCLK:
#if defined(ADC345_COMMON)
        case RCC_PERIPH_ADC345_SYSCLK:
#endif /* ADC345_COMMON */
            *periphClk = UT_ADC_SYSCLK_HZ;
            break;

        case RCC_PERIPH_ADC12_PLLP:
#if defined(ADC345_COMMON)
        case RCC_PERIPH_ADC345_PLLP:
#endif /* ADC345_COMMON */
            *periphClk = UT_ADC_PLLP_HZ;
            break;

        default:
            *periphClk = 0u;
            break;
    }

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
 * \brief NVIC handler registration stub - stores ISR of the ADC and its line.
 */
static nvic_RequestState_t Ut_Adc_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( irqHandler );

    utAdc_Isr      = irqHandler;
    utAdc_IsrIrqId = irqId;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief NVIC line deactivation stub - counts deactivations.
 */
static nvic_RequestState_t Ut_Adc_NvicSetInactiveStub( nvic_PeriphIrqList_t irqId, int callCnt )
{
    (void)irqId;
    (void)callCnt;

    utAdc_NvicInactiveCnt++;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief DMA default configuration stub - zeroed configuration.
 */
static dma_RequestState_t Ut_Adc_DmaDefaultStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    (void)callCnt;

    (void)memset( dmaConfig, 0, sizeof( dma_ConfigStruct_t ) );

    return ( DMA_REQUEST_OK );
}


/**
 * \brief DMA initialization stub - stores the channel configuration (incl. callbacks).
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
 * \brief Dma_Set_InterruptInactive() stub counting the calls (DMA channel released).
 */
static dma_RequestState_t Ut_Adc_DmaIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utAdc_DmaIrqOffCnt++;

    return ( DMA_REQUEST_OK );
}


/**
 * \brief Ignores all calls of RCC / NVIC / GPIO / DMA functions used by the module.
 */
static void Ut_Adc_Ignore_PeriphMocks( void )
{
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Gpio_Init_StubWithCallback( Ut_Adc_GpioInitStub );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Adc_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_StubWithCallback( Ut_Adc_NvicSetInactiveStub );
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
 * \brief Releases static module context of the previous test (clock source, channel
 *        inputs, data handling of all peripherals) and re-initializes the mocks. Registers
 *        must be zeroed.
 */
static void Ut_Adc_Release( void )
{
    utAdc_HclkHz = UT_ADC_HCLK_HZ;

    Ut_Adc_Ignore_PeriphMocks();

    for( adc_PeriphId_t periphIdx = ADC_PERIPH_1; ADC_PERIPH_CNT > periphIdx; periphIdx++ )
    {
        (void)Adc_Deinit( periphIdx );
    }

    (void)Adc_Set_ClockSource( ADC_CLK_SRC_HCLK );

    Ut_Adc_Reset_Mocks();
}


/**
 * \brief Returns ADC1 configuration: one regular VREF channel, software trigger.
 *
 * \param xferMode   [in]: Data transfer mode
 * \param bufferSize [in]: Count of samples of data buffer (0 - no buffer)
 */
static adc_PeriphConfig_t Ut_Adc_Get_PeriphConfig( adc_TransferMode_t xferMode, adc_BufferSize_t bufferSize )
{
    adc_PeriphConfig_t periphConfig = { 0 };

    periphConfig.PeriphId        = UT_ADC_PERIPH;
    periphConfig.Resolution      = ADC_RESOLUTION_12BIT;
    periphConfig.RegTriggerMode  = ADC_REG_TRIGGER_MODE_SINGLE;
    periphConfig.RegTriggerEdge  = ADC_TRIGGER_EDGE_RISING;
    periphConfig.RegTriggerId    = ADC_REG_TRIGGER_SOFTWARE;
    periphConfig.RegChannelsCnt  = 1u;
    periphConfig.RegChannels[ 0u ].ChannelId       = UT_ADC_VREF_CHANNEL;
    periphConfig.RegChannels[ 0u ].ChannelInput    = ADC_CHANNEL_INPUT_VREF;
    periphConfig.RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_640_5_CYCLES;
    periphConfig.InjTriggerMode  = ADC_INJ_TRIGGER_MODE_CONTINUOUS;
    periphConfig.InjTriggerId    = ADC_INJ_TRIGGER_SOFTWARE;
    periphConfig.InjTriggerEdge  = ADC_TRIGGER_EDGE_RISING;
    periphConfig.InjChannelsCnt  = 0u;

    periphConfig.DataConfig.TransferMode             = xferMode;
    periphConfig.DataConfig.DataBuffer               = ( 0u < bufferSize ) ? utAdc_Buffer : NULL;
    periphConfig.DataConfig.BufferSize               = bufferSize;
    periphConfig.DataConfig.BufferMode               = ADC_BUFFER_MODE_ONE_SHOT;
    periphConfig.DataConfig.DmaPeriphId              = ADC_DMA_PERIPH_1;
    periphConfig.DataConfig.DmaChannelId             = ADC_DMA_CHANNEL_6;
    periphConfig.DataConfig.DmaPriority              = ADC_DMA_PRIORITY_LOW;
    periphConfig.DataConfig.IrqPriority              = UT_ADC_PRIO;
    periphConfig.DataConfig.HalfTransferCallback     = Ut_Adc_HalfCallback;
    periphConfig.DataConfig.TransferCompleteCallback = Ut_Adc_CompleteCallback;
    periphConfig.DataConfig.ErrorCallback            = Ut_Adc_ErrorCallback;
    periphConfig.DataConfig.InjCompleteCallback      = NULL;

    return ( periphConfig );
}


/**
 * \brief Initializes ADC1 with HW model and ignored RCC / GPIO / NVIC calls.
 *
 * \param periphConfig [in]: Peripheral configuration
 */
static void Ut_Adc_PeriphInit( adc_PeriphConfig_t * const periphConfig )
{
    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ModelActive();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( periphConfig ) );
}


/**
 * \brief Presets enabled and ready ADC1 (regulator on).
 */
static void Ut_Adc_Set_Enabled( void )
{
    UT_ADC_REG->CR  = ADC_CR_ADVREGEN | ADC_CR_ADEN;
    UT_ADC_REG->ISR = ADC_ISR_ADRDY;
}


/**
 * \brief Calls the captured ADC ISR with given ISR register flags.
 *
 * \param isrFlags [in]: Value of the ISR register
 */
static void Ut_Adc_Call_Isr( uint32_t isrFlags )
{
    TEST_ASSERT_NOT_NULL_MESSAGE( utAdc_Isr, "ADC ISR not registered" );

    UT_ADC_REG->ISR = isrFlags;
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
