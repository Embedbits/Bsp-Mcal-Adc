/**
 * \author Mr.Nobody
 * \file Test_Adc.c
 * \ingroup Adc
 * \brief Unit tests of Analog-to-Digital Converter (ADC) module.
 *
 * Adc.c, Adc_Isr.c, Adc_Poll.c and Adc_Dma.c are compiled unchanged with real
 * LL drivers. ADC registers are emulated by RegMem, RCC, NVIC, GPIO and GPDMA
 * modules are mocked by CMock. ADC ISR registered in NVIC is captured by stub
 * and called directly to test interrupt data handling.
 *
 * Sequences where the ADC reacts on a CR write (regulator ready LDORDY,
 * calibration end, disable, stop of conversion) are emulated by HW model running
 * in background thread (Ut_Adc_HwModel). Tests without the model check register
 * values only.
 *
 * \note Emulated registers are plain memory - ISR flags are not cleared by
 *       data register reads, tests preset the flags handled by the module
 *       before each step. ADRDY is set by the module itself (write of ADRDY
 *       to ISR before enable).
 *
 * \note Clock source, channel inputs and data handling are kept in static
 *       module context - setUp() releases them (Adc_Deinit and clock source
 *       HCLK with ignored mocks) and re-initializes the mocks.
 *
 * \note STM32U5 ADC1 / ADC2 extended calibration depends on DBGMCU IDCODE. The
 *       emulated register is zero (extended calibration not available) unless
 *       the test presets it.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "UtCommon.h"                       /* Common test helpers            */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Adc_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockGpdma_Port.h"                 /* GPDMA module mock              */
#include "Stm32_adc.h"                      /* ADC registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 Ut_Adc_HwModel              ( void );
static void                 Ut_Adc_Set_ModelActive      ( void );
static rcc_RequestState_t   Ut_Adc_RccGetClkStub        ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static gpio_RequestState_t  Ut_Adc_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static nvic_RequestState_t  Ut_Adc_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static gpdma_RequestState_t Ut_Adc_GpdmaInitStub        ( gpdma_ConfigStruct_t * const configStruct, int callCnt );
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
#define UT_ADC_NVIC                         ( NVIC_PERIPH_IRQ_ADC1 )

/** Clock frequencies returned by RCC mock [Hz] (PLL2R is off) */
#define UT_ADC_HCLK_HZ                      ( 48000000u )
#define UT_ADC_SYSCLK_HZ                    ( 160000000u )
#define UT_ADC_HSI_HZ                       ( 16000000u )
#define UT_ADC_MSIK_HZ                      ( 4000000u )
#define UT_ADC_HSE_HZ                       ( 8000000u )

/** Internal channels of ADC1 */
#define UT_ADC_VREF_CHANNEL                 ( ADC_CHANNEL_0 )
#define UT_ADC_TEMP_CHANNEL                 ( ADC_CHANNEL_19 )

/** DBGMCU IDCODE device identifications (RM0456) */
#define UT_ADC_DEV_ID_U535_U545             ( 0x455uL )
#define UT_ADC_DEV_ID_U5F_U5G               ( 0x476uL )
#define UT_ADC_DEV_ID_U59X_U5AX             ( 0x481uL )
#define UT_ADC_DEV_ID_U575_U585             ( 0x482uL )

/** DBGMCU IDCODE revisions - extended calibration of STM32U575 / U585 / U59x / U5Ax from revision 0x3000 */
#define UT_ADC_REV_ID_1000                  ( 0x1000uL << DBGMCU_IDCODE_REV_ID_Pos )
#define UT_ADC_REV_ID_2001                  ( 0x2001uL << DBGMCU_IDCODE_REV_ID_Pos )
#define UT_ADC_REV_ID_3000                  ( 0x3000uL << DBGMCU_IDCODE_REV_ID_Pos )

/** Extended calibration: CR.CALINDEX and CALFACT2 written by the module (RM0456, ST HAL) */
#define UT_ADC_EXT_CALIB_CALINDEX           ( 0x9uL << ADC_CR_CALINDEX_Pos )
#define UT_ADC_EXT_CALIB_CALFACT2           ( 0x03021100uL )
#define UT_ADC_EXT_CALIB_CALFACT2_MASK      ( 0xFFFFFF00uL )

/** Count of devices of the extended calibration test */
#define UT_ADC_EXT_CALIB_DEV_CNT            ( 5u )

/** Interrupt priority of test configurations */
#define UT_ADC_PRIO                         ( 7u )

/** Size of the test data buffer */
#define UT_ADC_BUF_SIZE                     ( 8u )

/** ADC CR bits written by the module and handled by HW model */
#define UT_ADC_CR_HW_BITS                   ( ADC_CR_ADCAL | ADC_CR_ADDIS | ADC_CR_ADSTP | ADC_CR_JADSTP )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC for the ADC */
static nvic_IsrCallback_t       utAdc_Isr;

/** GPIO configuration of the last Gpio_Init call and count of calls */
static gpio_Config_t            utAdc_GpioConfig;
static uint32_t                 utAdc_GpioInitCnt;

/** GPDMA configuration of the last Gpdma_Init call (transfer configuration copied) and count of calls */
static gpdma_ConfigStruct_t     utAdc_GpdmaConfig;
static gpdma_TransferConfig_t   utAdc_GpdmaXferConfig;
static uint32_t                 utAdc_GpdmaInitCnt;

/** Conversion data buffer */
static adc_Data_t               utAdc_Buffer[ UT_ADC_BUF_SIZE ];

/** Counts of callback calls */
static uint32_t                 utAdc_HalfCnt;
static uint32_t                 utAdc_CompleteCnt;
static uint32_t                 utAdc_InjCnt;
static uint32_t                 utAdc_ErrorCnt;

/** Parameter of the last error callback */
static adc_ErrorId_t            utAdc_LastError;

/** Voltage regulator state (ADVREGEN) seen by the HW model in the previous step */
static uint32_t                 utAdc_ModelRegulatorOn;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    /* Stops HW model of previous test and clears registers */
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    Ut_Adc_Release();

    utAdc_Isr          = NULL;
    utAdc_GpioInitCnt  = 0u;
    utAdc_GpdmaInitCnt = 0u;
    utAdc_HalfCnt     = 0u;
    utAdc_CompleteCnt = 0u;
    utAdc_InjCnt      = 0u;
    utAdc_ErrorCnt    = 0u;
    utAdc_LastError   = ADC_ERROR_CNT;

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
 * \details Calls Adc_Init() with (RCC clock stub SYSCLK 160 MHz, MSIK 4 MHz, PLL2R off):
 * - SYSCLK / 2 = 80 MHz, above maximal ADC clock frequency 55 MHz,
 * - MSIK / 32 = 125 kHz, below minimal ADC clock frequency 140 kHz,
 * - PLL2R (0 Hz - clock not available),
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

    /* SYSCLK / 2 = 80 MHz is above maximal ADC clock */
    config.ClockSource  = ADC_CLK_SRC_SYSCLK;
    config.ClockDivider = ADC_CLK_DIV_2;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* MSIK / 32 = 125 kHz is below minimal ADC clock */
    config.ClockSource  = ADC_CLK_SRC_MSIK;
    config.ClockDivider = ADC_CLK_DIV_32;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* PLL2R is not running */
    config.ClockSource  = ADC_CLK_SRC_PLL2R;
    config.ClockDivider = ADC_CLK_DIV_1;
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
 * - ADC_REQUEST_OK is returned, RCC_PERIPH_ADC_HCLK is activated in RCC.
 * - Common clock prescaler PRESC = / 2 (STM32U5 ADC1 / ADC2 have no synchronous
 *   clock mode, HCLK is an input of the kernel clock multiplexer).
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
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC_HCLK, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_ASYNC_DIV2, LL_ADC_GetCommonClock( UT_ADC_COMMON ) );
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
 * - Ignored on MCUs with one ADC (STM32H503 - test function exists always, the
 *   runner collects test functions without preprocessor).
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
 * \details Calls Adc_Init() with clock HCLK / 1 (48 MHz) and ADC1 slot configured
 *          for one regular VREF channel in polling mode. HW model handles regulator
 *          ready flag and calibration.
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

/* ============================== CLOCK ===================================== */

/**
 * \brief   Adc_Set_ClockSource() activates the source in RCC and stores it.
 *
 * \details
 * 1. Sets HSI clock source with successful RCC activation.
 * 2. Sets MSIK clock source with failing RCC activation.
 * 3. Calls Adc_Get_ClockSource() with NULL pointer.
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK, RCC_PERIPH_ADC_HSI activated, read back source is HSI.
 * 2. ADC_REQUEST_ERROR, read back source stays HSI.
 * 3. ADC_REQUEST_ERROR.
 */
void Ut_Adc_Set_ClockSource_RccActivatedAndShadowUpdated( void )
{
    adc_ClkSrc_t clkSrc = ADC_CLK_SRC_CNT;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC_HSI, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockSource( ADC_CLK_SRC_HSI ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( ADC_CLK_SRC_HSI, clkSrc );

    /* RCC error - source is not changed */
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_ADC_MSIK, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockSource( ADC_CLK_SRC_MSIK ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockSource( &clkSrc ) );
    TEST_ASSERT_EQUAL( ADC_CLK_SRC_HSI, clkSrc );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_ClockSource( NULL ) );
}


/**
 * \brief   Adc_Set_ClockSource() rejects invalid source and enabled ADC.
 *
 * \details
 * 1. Sets invalid clock source ADC_CLK_SRC_CNT.
 * 2. Presets ADEN (ADC enabled) and sets HSI clock source.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in both cases.
 * - RCC is not called (no expectation on RCC mock).
 */
void Ut_Adc_Set_ClockSource_InvalidOrEnabled_ReturnsErrorWithoutRcc( void )
{
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockSource( ADC_CLK_SRC_CNT ) );

    UT_ADC_REG->CR = ADC_CR_ADEN;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockSource( ADC_CLK_SRC_HSI ) );
}


/**
 * \brief   Adc_Set_ClockDivider() with HCLK source sets the common prescaler.
 *
 * \details Sets dividers 1, 2, 4 and 8 (clock source HCLK 48 MHz from setUp), then
 *          invalid divider and NULL read pointer.
 *
 * \par Expected results
 * - Dividers 1, 2, 4, 8: ADC_REQUEST_OK, CCR PRESC = / 1, / 2, / 4, / 8 (STM32U5
 *   ADC1 / ADC2 divide every kernel clock incl. HCLK by the prescaler), read back
 *   divider equals the set one.
 * - ADC_CLK_DIV_CNT and Adc_Get_ClockDivider( NULL ): ADC_REQUEST_ERROR.
 */
void Ut_Adc_Set_ClockDivider_Hclk_PrescalerAndReadBack( void )
{
    const adc_ClkDiv_t divs[]   = { ADC_CLK_DIV_1, ADC_CLK_DIV_2, ADC_CLK_DIV_4, ADC_CLK_DIV_8 };
    const uint32_t     llDivs[] = { LL_ADC_CLOCK_ASYNC_DIV1, LL_ADC_CLOCK_ASYNC_DIV2, LL_ADC_CLOCK_ASYNC_DIV4, LL_ADC_CLOCK_ASYNC_DIV8 };
    adc_ClkDiv_t       clkDiv   = ADC_CLK_DIV_CNT;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    for( uint32_t idx = 0u; ( sizeof( divs ) / sizeof( divs[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockDivider( divs[ idx ] ) );
        TEST_ASSERT_EQUAL_HEX32( llDivs[ idx ], LL_ADC_GetCommonClock( UT_ADC_COMMON ) );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockDivider( &clkDiv ) );
        TEST_ASSERT_EQUAL( divs[ idx ], clkDiv );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_ClockDivider( NULL ) );
}


/**
 * \brief   Adc_Set_ClockDivider() with other kernel clock sources sets PRESC and
 *          checks ADC clock frequency limits.
 *
 * \details Selects HSI16 (16 MHz) clock source, then:
 * 1. sets divider 16 (1 MHz),
 * 2. sets divider 256 (62.5 kHz, below minimal ADC clock 140 kHz).
 * Selects SYSCLK (160 MHz) clock source, then:
 * 3. sets divider 2 (80 MHz, above maximal ADC clock 55 MHz),
 * 4. sets divider 4 (40 MHz).
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK, CCR = kernel clock / 16, read back divider 16.
 * 2. ADC_REQUEST_ERROR, CCR keeps kernel clock / 16.
 * 3. ADC_REQUEST_ERROR, CCR keeps kernel clock / 16.
 * 4. ADC_REQUEST_OK, CCR = kernel clock / 4.
 */
void Ut_Adc_Set_ClockDivider_AsyncSource_PrescalerAndFrequencyLimits( void )
{
    adc_ClkDiv_t clkDiv = ADC_CLK_DIV_CNT;

    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockSource( ADC_CLK_SRC_HSI ) );

    /* HSI16 16 MHz / 16 = 1 MHz */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockDivider( ADC_CLK_DIV_16 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_GetCommonClock( UT_ADC_COMMON ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_ClockDivider( &clkDiv ) );
    TEST_ASSERT_EQUAL( ADC_CLK_DIV_16, clkDiv );

    /* HSI16 16 MHz / 256 = 62.5 kHz is below minimal ADC clock - divider is not changed */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_256 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_GetCommonClock( UT_ADC_COMMON ) );

    /* SYSCLK 160 MHz / 2 = 80 MHz is above maximal ADC clock - divider is not changed */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockSource( ADC_CLK_SRC_SYSCLK ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_ClockDivider( ADC_CLK_DIV_2 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_GetCommonClock( UT_ADC_COMMON ) );

    /* SYSCLK 160 MHz / 4 = 40 MHz */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_ClockDivider( ADC_CLK_DIV_4 ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_CLOCK_ASYNC_DIV4, LL_ADC_GetCommonClock( UT_ADC_COMMON ) );
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

/* ======================== PERIPHERAL INITIALIZATION ======================= */

/**
 * \brief   Adc_PeriphInit() configures all registers of a polling VREF conversion.
 *
 * \details Initializes ADC1 by test configuration: one regular VREF channel
 *          (channel 0, 814 cycles), software trigger, single mode, 12-bit,
 *          polling without buffer. HW model handles regulator ready flag and
 *          calibration.
 *
 * \par Expected results
 * - ADC is powered up and enabled (ADVREGEN = ADEN = 1), no pending
 *   calibration / disable / stop request.
 * - Regular sequence length 1, rank 1 = channel 0, software trigger, single
 *   conversion mode, 12-bit resolution, overrun mode "data overwritten".
 * - VREFINT internal path enabled, channel 0 sampling time 814 cycles.
 * - Channel 0 is preselected (PCSEL - STM32U5 converts preselected channels only).
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

    /* Internal VREF path, channel preselection and sampling time */
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_PATH_INTERNAL_VREFINT, LL_ADC_GetCommonPathInternalCh( UT_ADC_COMMON ) & LL_ADC_PATH_INTERNAL_VREFINT );
    TEST_ASSERT_EQUAL_HEX32( 1uL << UT_ADC_VREF_CHANNEL, UT_ADC_REG->PCSEL );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_SAMPLINGTIME_814CYCLES,
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
 * - VREF channel with 5 cycles sampling time (below required 4 us at 48 MHz),
 * - VREF input on channel 5 (VREF is connected to channel 0 only),
 * - pin input on channel 19 (channel without pin),
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

    /* VREF needs at least 4 us sampling time (5 cycles at 48 MHz = 104 ns) */
    periphConfig.RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_5_CYCLES;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* VREF is not connected to channel 5 */
    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.RegChannels[ 0u ].ChannelId = ADC_CHANNEL_5;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* Channel 19 has no pin */
    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.RegChannels[ 0u ].ChannelId    = UT_ADC_TEMP_CHANNEL;
    periphConfig.RegChannels[ 0u ].ChannelInput = ADC_CHANNEL_INPUT_PIN_SINGLE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    /* Same channel twice with different sampling time */
    periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );
    periphConfig.RegChannelsCnt  = 2u;
    periphConfig.RegChannels[ 1u ] = periphConfig.RegChannels[ 0u ];
    periphConfig.RegChannels[ 1u ].ChannelSampling = ADC_CHANNEL_SAMPLING_391_CYCLES;
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
 * - DMA mode with invalid DMA channel and with buffer size UINT16_MAX.
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
    periphConfig.DataConfig.BufferSize = UINT16_MAX;
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
 * \details Initializes ADC1 with regular sequence: channel 3 differential and
 *          channel 5 single-ended (12 cycles). GPIO initialization is captured by stub.
 *
 * \par Expected results
 * - Gpio_Init() is called 3 times (PC2, PC3 for channel 3 - negative input is the
 *   pin of channel 4, PA0 for channel 5), the last configuration is PA0, analog
 *   mode, no pull.
 * - Channel 3 is differential, channel 5 single-ended.
 * - Channels 3 and 5 are preselected (PCSEL).
 * - Regular sequence length 2, rank 1 = channel 3, rank 2 = channel 5.
 */
void Ut_Adc_PeriphInit_PinChannels_GpioAnalogAndDifferential( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    periphConfig.RegChannelsCnt = 2u;
    periphConfig.RegChannels[ 0u ].ChannelId       = ADC_CHANNEL_3;
    periphConfig.RegChannels[ 0u ].ChannelInput    = ADC_CHANNEL_INPUT_PIN_DIFF;
    periphConfig.RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_12_CYCLES;
    periphConfig.RegChannels[ 1u ].ChannelId       = ADC_CHANNEL_5;
    periphConfig.RegChannels[ 1u ].ChannelInput    = ADC_CHANNEL_INPUT_PIN_SINGLE;
    periphConfig.RegChannels[ 1u ].ChannelSampling = ADC_CHANNEL_SAMPLING_12_CYCLES;

    Ut_Adc_PeriphInit( &periphConfig );

    /* Channel 3 differential: PC2 (+) and PC3 (-), channel 5: PA0 */
    TEST_ASSERT_EQUAL_UINT32( 3u, utAdc_GpioInitCnt );
    TEST_ASSERT_EQUAL( GPIO_PORT_A,          utAdc_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_0,        utAdc_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ANALOG, utAdc_GpioConfig.PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_NONE,   utAdc_GpioConfig.PinPull );

    TEST_ASSERT_NOT_EQUAL( 0u, LL_ADC_GetChannelSingleDiff( UT_ADC_REG, LL_ADC_CHANNEL_3 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, LL_ADC_GetChannelSingleDiff( UT_ADC_REG, LL_ADC_CHANNEL_5 ) );
    TEST_ASSERT_EQUAL_HEX32( ( 1uL << 3u ) | ( 1uL << 5u ), UT_ADC_REG->PCSEL );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS, LL_ADC_REG_GetSequencerLength( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_UINT32( 3u, __LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_REG_GetSequencerRanks( UT_ADC_REG, LL_ADC_REG_RANK_1 ) ) );
    TEST_ASSERT_EQUAL_UINT32( 5u, __LL_ADC_CHANNEL_TO_DECIMAL_NB( LL_ADC_REG_GetSequencerRanks( UT_ADC_REG, LL_ADC_REG_RANK_2 ) ) );
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
 * \details Initializes ADC1 with injected group only: VREF (channel 0) and
 *          temperature sensor (channel 19), software trigger, continuous mode.
 *
 * \par Expected results
 * - Injected sequence length 2, rank 1 = channel 0, rank 2 = channel 19.
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
    periphConfig.InjChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_814_CYCLES;
    periphConfig.InjChannels[ 1u ].ChannelId       = UT_ADC_TEMP_CHANNEL;
    periphConfig.InjChannels[ 1u ].ChannelInput    = ADC_CHANNEL_INPUT_TEMP;
    periphConfig.InjChannels[ 1u ].ChannelSampling = ADC_CHANNEL_SAMPLING_814_CYCLES;

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
 * \brief   Adc_PeriphInit() reports timeout of the voltage regulator start-up.
 *
 * \details Initializes ADC1 by valid configuration without HW model - the
 *          regulator ready flag LDORDY is never set by HW.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned (timeout).
 * - Calibration is not started and ADC is not enabled (ADCAL = ADEN = 0).
 */
void Ut_Adc_PeriphInit_RegulatorNotReady_ReturnsError( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    Ut_Adc_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_PeriphInit( &periphConfig ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CR & ( ADC_CR_ADCAL | ADC_CR_ADEN ) );
}


/**
 * \brief   Adc_PeriphInit() in DMA transfer mode configures data management and
 *          GPDMA channel.
 *
 * \details Initializes ADC1 in DMA mode with circular buffer of 8 samples (GPDMA1
 *          channel 7). GPDMA initialization is captured by stub, then ADC1 is
 *          deinitialized.
 *
 * \par Expected results
 * - ADC_REQUEST_OK is returned.
 * - CFGR1 DMNGT = DMA circular mode (STM32U5 ADC1 / ADC2 data management field).
 * - Gpdma_Init() is called once: GPDMA1 channel 7, peripheral to memory, ADC1
 *   request, block size 16 bytes. Channel interrupt is enabled.
 * - Adc_Deinit(): ADC_REQUEST_OK, DMNGT = no DMA transfer.
 */
void Ut_Adc_PeriphInit_DmaCircular_DataManagementAndGpdmaConfigured( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_DMA, UT_ADC_BUF_SIZE );

    periphConfig.DataConfig.BufferMode = ADC_BUFFER_MODE_CIRCULAR;

    Ut_Adc_Ignore_PeriphMocks();
    Ut_Adc_Set_ModelActive();
    Gpdma_Get_DefaultConfig_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Init_StubWithCallback( Ut_Adc_GpdmaInitStub );
    Gpdma_Set_InterruptActive_ExpectAndReturn( GPDMA_PERIPH_1, GPDMA_CHANNEL_7, GPDMA_REQUEST_OK );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_PeriphInit( &periphConfig ) );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_DMA_TRANSFER_UNLIMITED, LL_ADC_REG_GetDataTransferMode( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utAdc_GpdmaInitCnt );
    TEST_ASSERT_EQUAL( GPDMA_PERIPH_1, utAdc_GpdmaConfig.PeriphId );
    TEST_ASSERT_EQUAL( GPDMA_CHANNEL_7, utAdc_GpdmaConfig.ChannelId );
    TEST_ASSERT_EQUAL( GPDMA_DIR_PERIPH_TO_MEMORY, utAdc_GpdmaXferConfig.Direction );
    TEST_ASSERT_EQUAL( GPDMA_REQ_ADC1, utAdc_GpdmaXferConfig.RequestSource );
    TEST_ASSERT_EQUAL_UINT32( UT_ADC_BUF_SIZE * sizeof( adc_Data_t ), utAdc_GpdmaXferConfig.BlockSize );

    /* DMA channel is released by deinitialization */
    Gpdma_Set_ChannelInactive_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_InterruptInactive_IgnoreAndReturn( GPDMA_REQUEST_OK );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Deinit( UT_ADC_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_REG_DMA_TRANSFER_NONE, LL_ADC_REG_GetDataTransferMode( UT_ADC_REG ) );
}

/* =========================== DEINITIALIZATION ============================= */

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

/* =========================== PERIPHERAL STATE ============================= */

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
 * \brief   Adc_Set_PeriphActive() runs the extended calibration on devices which
 *          support it.
 *
 * \details For every device identification (DBGMCU IDCODE preset): presets enabled
 *          voltage regulator, cleared calibration factors and activates ADC1 with
 *          HW model, then disables ADC1. Devices:
 * - STM32U535 / U545 (0x455) revision 0x1000,
 * - STM32U5Fx / U5Gx (0x476) revision 0x1000,
 * - STM32U575 / U585 (0x482) revision 0x3000,
 * - STM32U59x / U5Ax (0x481) revision 0x2001,
 * - unknown device (IDCODE 0).
 *
 * \par Expected results
 * - ADC_REQUEST_OK, ADC enabled (ADEN = 1, ADCAL = ADDIS = 0) for every device.
 * - Extended calibration (first three devices): CR.CALINDEX = 9, CALFACT2 upper
 *   bits = 0x03021100, CALFACT LATCH_COEF = 1.
 * - Other devices: CALINDEX = 0, CALFACT2 = 0, LATCH_COEF = 0.
 * - Adc_Set_PeriphInactive() returns ADC_REQUEST_OK.
 */
void Ut_Adc_Set_PeriphActive_DeviceId_ExtendedCalibrationSelected( void )
{
    const uint32_t idCodes[ UT_ADC_EXT_CALIB_DEV_CNT ]   = { UT_ADC_REV_ID_1000 | UT_ADC_DEV_ID_U535_U545,
                                                             UT_ADC_REV_ID_1000 | UT_ADC_DEV_ID_U5F_U5G,
                                                             UT_ADC_REV_ID_3000 | UT_ADC_DEV_ID_U575_U585,
                                                             UT_ADC_REV_ID_2001 | UT_ADC_DEV_ID_U59X_U5AX,
                                                             0u };
    const uint32_t calIndex[ UT_ADC_EXT_CALIB_DEV_CNT ]  = { UT_ADC_EXT_CALIB_CALINDEX, UT_ADC_EXT_CALIB_CALINDEX, UT_ADC_EXT_CALIB_CALINDEX, 0u, 0u };
    const uint32_t calFact2[ UT_ADC_EXT_CALIB_DEV_CNT ]  = { UT_ADC_EXT_CALIB_CALFACT2, UT_ADC_EXT_CALIB_CALFACT2, UT_ADC_EXT_CALIB_CALFACT2, 0u, 0u };
    const uint32_t latchCoef[ UT_ADC_EXT_CALIB_DEV_CNT ] = { ADC_CALFACT_LATCH_COEF, ADC_CALFACT_LATCH_COEF, ADC_CALFACT_LATCH_COEF, 0u, 0u };

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Ut_Adc_Set_ModelActive();

    for( uint32_t idx = 0u; UT_ADC_EXT_CALIB_DEV_CNT > idx; idx++ )
    {
        DBGMCU->IDCODE       = idCodes[ idx ];
        UT_ADC_REG->CALFACT  = 0u;
        UT_ADC_REG->CALFACT2 = 0u;
        UT_ADC_REG->CR       = ADC_CR_ADVREGEN;

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_PeriphActive( UT_ADC_PERIPH ) );

        TEST_ASSERT_EQUAL_HEX32( ADC_CR_ADEN, UT_ADC_REG->CR & ( ADC_CR_ADEN | ADC_CR_ADCAL | ADC_CR_ADDIS ) );
        TEST_ASSERT_EQUAL_HEX32( calIndex[ idx ], UT_ADC_REG->CR & ADC_CR_CALINDEX );
        TEST_ASSERT_EQUAL_HEX32( calFact2[ idx ], UT_ADC_REG->CALFACT2 & UT_ADC_EXT_CALIB_CALFACT2_MASK );
        TEST_ASSERT_EQUAL_HEX32( latchCoef[ idx ], UT_ADC_REG->CALFACT & ADC_CALFACT_LATCH_COEF );

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_PeriphInactive( UT_ADC_PERIPH ) );
    }
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

/* ======================== RESOLUTION / SAMPLING =========================== */

/**
 * \brief   Adc_Set_Resolution() / Adc_Get_Resolution() handle all resolutions.
 *
 * \details Sets every resolution of ADC1 / ADC2 (12, 10, 8, 14 bit), then 6 bit
 *          resolution (ADC4 only), invalid resolution, NULL read pointer and change
 *          of resolution during conversion.
 *
 * \par Expected results
 * - Every valid resolution: ADC_REQUEST_OK, CFGR1 RES matches, read back equals.
 * - 6 bit resolution: ADC_REQUEST_ERROR, resolution stays 14-bit.
 * - Invalid value and NULL pointer: ADC_REQUEST_ERROR.
 * - During conversion (ADSTART = 1): ADC_REQUEST_ERROR, resolution stays 14-bit.
 */
void Ut_Adc_Set_Resolution_AllOptions_RegisterAndReadBack( void )
{
    const adc_Resolution_t resolutions[] = { ADC_RESOLUTION_12BIT, ADC_RESOLUTION_10BIT, ADC_RESOLUTION_8BIT, ADC_RESOLUTION_14BIT };
    const uint32_t         llRes[]       = { LL_ADC_RESOLUTION_12B, LL_ADC_RESOLUTION_10B, LL_ADC_RESOLUTION_8B, LL_ADC_RESOLUTION_14B };
    adc_Resolution_t       res           = ADC_RESOLUTION_CNT;

    for( uint32_t idx = 0u; ( sizeof( resolutions ) / sizeof( resolutions[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_Resolution( UT_ADC_PERIPH, resolutions[ idx ] ) );
        TEST_ASSERT_EQUAL_HEX32( llRes[ idx ], LL_ADC_GetResolution( UT_ADC_REG ) );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Resolution( UT_ADC_PERIPH, &res ) );
        TEST_ASSERT_EQUAL( resolutions[ idx ], res );
    }

    /* Resolution out of the list is refused without a register write */
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_Resolution( UT_ADC_PERIPH, ADC_RESOLUTION_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_RESOLUTION_14B, LL_ADC_GetResolution( UT_ADC_REG ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_Resolution( UT_ADC_PERIPH, NULL ) );

    /* Conversion ongoing */
    UT_ADC_REG->CR = ADC_CR_ADSTART;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_Resolution( UT_ADC_PERIPH, ADC_RESOLUTION_12BIT ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_RESOLUTION_14B, LL_ADC_GetResolution( UT_ADC_REG ) );
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

    for( adc_ChannelSampling_t idx = ADC_CHANNEL_SAMPLING_5_CYCLES; ADC_CHANNEL_SAMPLING_CNT > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_12, idx ) );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_12, &sampling ) );
        TEST_ASSERT_EQUAL( idx, sampling );
    }

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_CNT, ADC_CHANNEL_SAMPLING_5_CYCLES ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_12, ADC_CHANNEL_SAMPLING_CNT ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Get_SamplingTime( UT_ADC_PERIPH, ADC_CHANNEL_12, NULL ) );
}


/**
 * \brief   Adc_Set_SamplingTime() rejects too short sampling of internal channel.
 *
 * \details Initializes VREF channel, disables ADC and sets 68 cycles sampling
 *          time of VREF channel (1.4 us at 48 MHz, shorter than required 4 us).
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned.
 * - Sampling time of the channel stays 814 cycles.
 */
void Ut_Adc_Set_SamplingTime_InternalChannelTooShort_ReturnsError( void )
{
    adc_PeriphConfig_t periphConfig = Ut_Adc_Get_PeriphConfig( ADC_TRANSFER_MODE_POLL, 0u );

    Ut_Adc_PeriphInit( &periphConfig );
    UT_ADC_REG->CR &= ~ADC_CR_ADEN;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_SamplingTime( UT_ADC_PERIPH, UT_ADC_VREF_CHANNEL, ADC_CHANNEL_SAMPLING_68_CYCLES ) );
    TEST_ASSERT_EQUAL_HEX32( LL_ADC_SAMPLINGTIME_814CYCLES,
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
    adc_ChannelConfig_t channelConfig = { .ChannelId = ADC_CHANNEL_3, .ChannelInput = ADC_CHANNEL_INPUT_PIN_SINGLE, .ChannelSampling = ADC_CHANNEL_SAMPLING_12_CYCLES };

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );

    UT_ADC_REG->CR = ADC_CR_ADEN;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_ChannelInit( UT_ADC_PERIPH, &channelConfig ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_ChannelInit( UT_ADC_PERIPH, NULL ) );
}

/* =============================== TRIGGER ================================== */

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

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_ADC_REG->CFGR1 );
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

/* ========================= CONVERSION START / STOP ======================== */

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

/* ---------------------------- Polling mode -------------------------------- */

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
        else
        {
            /* No action required */
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

/* ---------------------------- Interrupt mode ------------------------------ */

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

/* ============================ DATA HANDLING =============================== */

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

/* ================================ FLAGS =================================== */

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

/* =========================== ANALOG WATCH-DOG ============================= */

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
 *          resolution preset in CFGR1), no filter.
 * \note    STM32U5 ADC1 / ADC2: thresholds of all watch-dogs are compared with
 *          the 14 bit result - 12 bit thresholds are written shifted by 2 bits.
 *
 * \par Expected results
 * - AWD2 monitors all regular and injected channels.
 * - LTR2 / HTR2 hold 14 bit thresholds 640 / 12800, thresholds read back 160 / 3200.
 */
void Ut_Adc_AwdInit_Awd2AllChannels_ThresholdsSet( void )
{
    adc_AwdConfig_t    awdConfig = { .AwdId = ADC_AWD_2, .AwdMode = ADC_AWD_MODE_ALL,
                                     .AwdLowThreshold = 160u, .AwdHighThreshold = 3200u, .AwdFilter = ADC_AWD_FILTER_NONE };
    adc_AwdThreshold_t low       = 0u;
    adc_AwdThreshold_t high      = 0u;

    LL_ADC_SetResolution( UT_ADC_REG, LL_ADC_RESOLUTION_12B );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( UT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL_HEX32( LL_ADC_AWD_ALL_CHANNELS_REG_INJ, LL_ADC_GetAnalogWDMonitChannels( UT_ADC_REG, LL_ADC_AWD2 ) );
    TEST_ASSERT_EQUAL_HEX32( 640u, UT_ADC_REG->LTR2 );
    TEST_ASSERT_EQUAL_HEX32( 12800u, UT_ADC_REG->HTR2 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_2, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 160u, low );
    TEST_ASSERT_EQUAL_UINT16( 3200u, high );
}


/**
 * \brief   Analog watchdog thresholds are converted with the configured resolution.
 *
 * \details 8 bit resolution (CFGR1.RES preset): AWD1 thresholds 20 - 250, AWD3
 *          thresholds 20 - 250, threshold above the maximum of 8 bit resolution.
 *          12 bit resolution: AWD3 thresholds 100 - 4000. 14 bit resolution: AWD1
 *          thresholds 0 - 16383 and threshold above the maximum.
 * \note    STM32U5 ADC1 / ADC2: thresholds of all watch-dogs are compared with the
 *          14 bit result (left aligned to 14 bits for lower resolution).
 *
 * \par Expected results
 * - 8 bit: LTR1 / HTR1 and LTR3 / HTR3 hold thresholds left aligned to 14 bits
 *   (1280 / 16000), both read back 20 / 250. Threshold 256: ADC_REQUEST_ERROR.
 * - 12 bit: AWD3 thresholds read back 100 / 4000 (no LSB is lost).
 * - 14 bit: AWD1 thresholds 0 / 16383 read back, threshold 16384: ADC_REQUEST_ERROR.
 */
void Ut_Adc_Set_AwdThresholds_Resolution_ConvertedToRegisterFormat( void )
{
    adc_AwdThreshold_t low  = 0u;
    adc_AwdThreshold_t high = 0u;

    LL_ADC_SetResolution( UT_ADC_REG, LL_ADC_RESOLUTION_8B );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, 20u, 250u ) );
    TEST_ASSERT_EQUAL_HEX32( 1280u, UT_ADC_REG->LTR1 & ADC_LTR_LT );
    TEST_ASSERT_EQUAL_HEX32( 16000u, UT_ADC_REG->HTR1 & ADC_HTR_HT );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 20u, low );
    TEST_ASSERT_EQUAL_UINT16( 250u, high );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_3, 20u, 250u ) );
    TEST_ASSERT_EQUAL_HEX32( 1280u, UT_ADC_REG->LTR3 );
    TEST_ASSERT_EQUAL_HEX32( 16000u, UT_ADC_REG->HTR3 );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_3, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 20u, low );
    TEST_ASSERT_EQUAL_UINT16( 250u, high );

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, 0u, 256u ) );

    LL_ADC_SetResolution( UT_ADC_REG, LL_ADC_RESOLUTION_12B );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_3, 100u, 4000u ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_3, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 100u, low );
    TEST_ASSERT_EQUAL_UINT16( 4000u, high );

    LL_ADC_SetResolution( UT_ADC_REG, LL_ADC_RESOLUTION_14B );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, 0u, 16383u ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, &low, &high ) );
    TEST_ASSERT_EQUAL_UINT16( 0u, low );
    TEST_ASSERT_EQUAL_UINT16( 16383u, high );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Set_AwdThresholds( UT_ADC_PERIPH, ADC_AWD_1, 0u, 16384u ) );
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
 * \brief HW model of ADC1 - runs in background thread.
 *
 * Enabled voltage regulator signals ready flag LDORDY (once per enable - the flag
 * is not written later, while the test presets ISR), calibration ends immediately,
 * disable request clears ADEN, stop requests clear the conversion start bits.
 */
static void Ut_Adc_HwModel( void )
{
    const uint32_t cr          = UT_ADC_REG->CR;
    const uint32_t regulatorOn = cr & ADC_CR_ADVREGEN;

    if( ( 0u != regulatorOn            ) &&
        ( 0u == utAdc_ModelRegulatorOn )    )
    {
        (void)__atomic_or_fetch( &UT_ADC_REG->ISR, ADC_ISR_LDORDY, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Regulator state not changed or regulator off - ready flag is not changed */
    }

    utAdc_ModelRegulatorOn = regulatorOn;

    if( 0u != ( cr & ADC_CR_ADCAL ) )
    {
        (void)__atomic_and_fetch( &UT_ADC_REG->CR, ~ADC_CR_ADCAL, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Calibration not requested */
    }

    if( 0u != ( cr & ADC_CR_ADDIS ) )
    {
        (void)__atomic_and_fetch( &UT_ADC_REG->CR, ~( ADC_CR_ADDIS | ADC_CR_ADEN ), __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Disable not requested */
    }

    if( 0u != ( cr & ADC_CR_ADSTP ) )
    {
        (void)__atomic_and_fetch( &UT_ADC_REG->CR, ~( ADC_CR_ADSTP | ADC_CR_ADSTART ), __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Regular stop not requested */
    }

    if( 0u != ( cr & ADC_CR_JADSTP ) )
    {
        (void)__atomic_and_fetch( &UT_ADC_REG->CR, ~( ADC_CR_JADSTP | ADC_CR_JADSTART ), __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Injected stop not requested */
    }
}


/**
 * \brief Starts the ADC HW model.
 */
static void Ut_Adc_Set_ModelActive( void )
{
    utAdc_ModelRegulatorOn = 0u;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( Ut_Adc_HwModel ) );
}


/**
 * \brief RCC clock stub - HCLK 48 MHz, SYSCLK 160 MHz, HSI16 16 MHz, MSIK 4 MHz,
 *        HSE 8 MHz, PLL2R off.
 */
static rcc_RequestState_t Ut_Adc_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)callCnt;

    if( RCC_PERIPH_ADC_HCLK == periphId )
    {
        *periphClk = UT_ADC_HCLK_HZ;
    }
    else if( RCC_PERIPH_ADC_SYSCLK == periphId )
    {
        *periphClk = UT_ADC_SYSCLK_HZ;
    }
    else if( RCC_PERIPH_ADC_HSI == periphId )
    {
        *periphClk = UT_ADC_HSI_HZ;
    }
    else if( RCC_PERIPH_ADC_MSIK == periphId )
    {
        *periphClk = UT_ADC_MSIK_HZ;
    }
    else if( RCC_PERIPH_ADC_HSE == periphId )
    {
        *periphClk = UT_ADC_HSE_HZ;
    }
    else
    {
        *periphClk = 0u;
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
 * \brief NVIC handler registration stub - stores ISR of the ADC.
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
 * \brief GPDMA initialization stub - stores the channel and transfer configuration.
 */
static gpdma_RequestState_t Ut_Adc_GpdmaInitStub( gpdma_ConfigStruct_t * const configStruct, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( configStruct );
    TEST_ASSERT_NOT_NULL( configStruct->TransferConfig );

    utAdc_GpdmaConfig     = *configStruct;
    utAdc_GpdmaXferConfig = *configStruct->TransferConfig;
    utAdc_GpdmaInitCnt++;

    return ( GPDMA_REQUEST_OK );
}


/**
 * \brief Ignores all calls of RCC / NVIC / GPIO functions used by the module.
 */
static void Ut_Adc_Ignore_PeriphMocks( void )
{
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Adc_RccGetClkStub );
    Gpio_Init_StubWithCallback( Ut_Adc_GpioInitStub );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Adc_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
}


/**
 * \brief Removes all expectations and ignores of the mocks.
 */
static void Ut_Adc_Reset_Mocks( void )
{
    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockGpdma_Port_Destroy();
    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockGpdma_Port_Init();
}


/**
 * \brief Releases static module context of the previous test (clock source, channel
 *        inputs, data handling) and re-initializes the mocks. Registers must be zeroed.
 */
static void Ut_Adc_Release( void )
{
    Ut_Adc_Ignore_PeriphMocks();

    (void)Adc_Deinit( UT_ADC_PERIPH );
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
    periphConfig.RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_814_CYCLES;
    periphConfig.InjTriggerMode  = ADC_INJ_TRIGGER_MODE_CONTINUOUS;
    periphConfig.InjTriggerId    = ADC_INJ_TRIGGER_SOFTWARE;
    periphConfig.InjTriggerEdge  = ADC_TRIGGER_EDGE_RISING;
    periphConfig.InjChannelsCnt  = 0u;

    if( 0u < bufferSize )
    {
        periphConfig.DataConfig.DataBuffer = utAdc_Buffer;
    }
    else
    {
        periphConfig.DataConfig.DataBuffer = NULL;
    }

    periphConfig.DataConfig.TransferMode             = xferMode;
    periphConfig.DataConfig.BufferSize               = bufferSize;
    periphConfig.DataConfig.BufferMode               = ADC_BUFFER_MODE_ONE_SHOT;
    periphConfig.DataConfig.DmaPeriphId              = ADC_DMA_PERIPH_1;
    periphConfig.DataConfig.DmaChannelId             = ADC_DMA_CHANNEL_7;
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
