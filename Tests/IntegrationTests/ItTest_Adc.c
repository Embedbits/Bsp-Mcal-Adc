/**
 * \author Mr.Nobody
 * \file ItTest_Adc.c
 * \ingroup Adc
 * \brief Integration tests of Analog-to-digital converter (ADC) module on target (STM32H7 16-bit ADC).
 *
 * Adc module runs on the MCU together with real RCC, NVIC, DAC and DMA modules
 * and hardware. Tests verify behavior which cannot be verified by unit tests
 * (emulated registers): calibration, conversion of known signals, resolution,
 * regular sequence, injected group, data transfer in polling, interrupt and DMA
 * mode, analog watchdog and clock configuration.
 *
 * No external wiring and no board specific configuration is used. The signals
 * converted by the tests are the two outputs of DAC1 connected internally to
 * ADC2 (DAC output mode "on-chip peripherals only", pins are not driven):
 * - signal A - DAC1 output 1 (ADC2 channel 16) at 1/4 of the reference voltage
 * - signal B - DAC1 output 2 (ADC2 channel 17) at 3/4 of the reference voltage
 * The DAC and the ADC share the reference voltage, so the expected conversion
 * results do not depend on the analog supply of the board. Tests run on every
 * STM32H7 line (the internal channels of the DAC exist on ADC2 of all lines).
 * STM32H7A3 / H7B0 / H7B3 additionally convert VREFINT and the temperature sensor
 * of ADC2 (the internal channels of the other lines are on the 12-bit ADC3, which
 * is not handled by the module, or on ADC3 of STM32H74x / H75x).
 *
 * Default clock of StartUp: HSI 64 MHz, HCLK = 64 MHz, ADC kernel clock HCLK / 4
 * = 16 MHz (conversion clock 8 MHz).
 *
 *
 * Divider tests: two resistor dividers from +3V3 (= VDDA of the board) to GND on external channels of
 * ADC1 - PA0 (channel 16): 10 kOhm to +3V3 / 4.7 kOhm to GND, PC0 (channel 10): 4.7 kOhm to +3V3 / 10 kOhm to
 * GND. The ratio of the conversion result to the full scale equals the divider ratio (independent of VDDA).
 * Fixture of the board (EmBi_Platform IntegrationTesting, "Board fixtures"): feature ADC_DIVIDERS - the
 * divider tests are ignored on a board with a fixture without it (a board without a fixture is expected to
 * have the dividers).
 * NUCLEO-H723ZG / NUCLEO-H755ZI-Q: the Ethernet PHY of the board (LAN8742A, RMII) drives its 50 MHz reference
 * clock on PA1, the pin next to PA0 - while it runs, the PA0 conversion spreads by about +-60 LSB (+-1 LSB with
 * the PHY in power-down). The divider tests put the PHY into power-down over MDIO (Eth module) before the
 * conversion (AB#2248); the PHY is reset by NRST with the next test case.
 *
 * Used resources: ADC2, DAC1 (both channels), DMA2 stream 0 (DMA mode); dividers: ADC1, ETH (MDIO, boards with
 * the Ethernet PHY).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Adc_Port.h"                       /* Module under test              */
#include "Dac_Port.h"                       /* Source of the converted signals */
#include "Eth_Port.h"                       /* Power-down of the Ethernet PHY */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void         ItAdc_Get_Config         ( adc_Config_t * const config, adc_TransferMode_t transferMode,
                                              adc_BufferSize_t bufferSize, adc_BufferMode_t bufferMode,
                                              adc_RegTriggerMode_t triggerMode );
static void         ItAdc_Add_RegChannel    ( adc_Config_t * const config, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput );
static void         ItAdc_Add_InjChannel    ( adc_Config_t * const config, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput );
static void         ItAdc_Init_Dac          ( void );
static void         ItAdc_Wait              ( volatile const uint32_t * const counter, uint32_t expectedCnt );
static void         ItAdc_Wait_Sample       ( uint32_t sampleIdx );
static void         ItAdc_Poll              ( uint32_t loopCnt );
static void         ItAdc_Wait_DacSettled   ( void );

static void         ItAdc_HalfCallback      ( void );
static void         ItAdc_CompleteCallback  ( void );
static void         ItAdc_InjCallback       ( void );
static void         ItAdc_ErrorCallback     ( adc_ErrorId_t errorId );
static void         ItAdc_RequireDividers  ( void );
static void         ItAdc_Set_EthPhyPowerDown( void );
#if defined(IT_BOARD_STM32H723xG)  || \
    defined(IT_BOARD_STM32H755xI)
/* Boards with the Ethernet PHY (IT_ADC_ETH_PHY of the board block) */
static void         ItAdc_EthTxCompleteCallback( eth_FrameTag_t tag,
                                                 eth_RequestState_t status,
                                                 const eth_PtpTime_t * const timestamp );
static void         ItAdc_EthRxCallback    ( void );
#endif /* NUCLEO-H723ZG / NUCLEO-H755ZI-Q */
static void         ItAdc_Check_Divider    ( adc_Data_t raw12,
                                             uint32_t topOhm,
                                             uint32_t bottomOhm,
                                             const char * const message );
static void         ItAdc_Add_PeriphRegChannel( adc_Config_t * const config,
                                                adc_PeriphId_t periphId,
                                                adc_ChannelId_t channelId,
                                                adc_ChannelInput_t channelInput );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection) */
#if defined(IT_BOARD_STM32H723xG)  || \
    defined(IT_BOARD_STM32H755xI)  || \
    defined(IT_BOARD_STM32H7A3xI)  || \
    defined(IT_BOARD_STM32H7A3xIQ)

    /** NUCLEO-H723ZG / NUCLEO-H755ZI-Q / NUCLEO-H7A3ZI-Q: dividers on PA0 (ADC1_INP16, channel 16) and
     *  PC0 (ADC1_INP10, channel 10) */
    #define IT_ADC_DIV_AVAILABLE            ( 1u )
    #define IT_ADC_CH_PA0                   ( ADC_CHANNEL_16 )

    #if defined(IT_BOARD_STM32H723xG)  || \
        defined(IT_BOARD_STM32H755xI)

        /** NUCLEO-H723ZG / NUCLEO-H755ZI-Q: Ethernet PHY on the board, its reference clock on PA1 disturbs PA0 */
        #define IT_ADC_ETH_PHY              ( 1u )
    #else

        /** NUCLEO-H7A3ZI-Q: no Ethernet PHY */
        #define IT_ADC_ETH_PHY              ( 0u )
    #endif

#else
    #error "Board of Adc integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** ADC peripheral of the dividers (PA0 and PC0 are inputs of ADC1) */
#define IT_ADC_DIV_PERIPH                   ( ADC_PERIPH_1 )

#if ( 1u == IT_ADC_ETH_PHY )
/** Ethernet PHY of the board: MDIO address, basic control register (BMCR) and its power-down bit (IEEE 802.3
 *  clause 22 register set of LAN8742A) */
#define IT_ADC_ETH_PHY_ADDR                 ( 0u )
#define IT_ADC_ETH_PHY_BMCR                 ( 0u )
#define IT_ADC_ETH_PHY_BMCR_POWER_DOWN      ( 0x0800u )

/** Descriptors and receive buffer of the Ethernet initialization (MDIO access only, no frame is transferred) */
#define IT_ADC_ETH_DESC_CNT                 ( 4u )
#define IT_ADC_ETH_RX_BUFFER_SIZE           ( 256u )

/** Memory of the Ethernet initialization */
static eth_Descriptor_t       itAdc_EthTxDescriptors[ IT_ADC_ETH_DESC_CNT ];
static eth_Descriptor_t       itAdc_EthRxDescriptors[ IT_ADC_ETH_DESC_CNT ];
static eth_BufferByte_t       itAdc_EthRxBufferPool[ IT_ADC_ETH_DESC_CNT * IT_ADC_ETH_RX_BUFFER_SIZE ];
static eth_DescriptorMemory_t itAdc_EthMemory;
#endif /* IT_ADC_ETH_PHY */


/** ADC peripheral under test (ADC2 - DAC1 outputs are connected to its internal channels) */
#define IT_ADC_PERIPH                       ( ADC_PERIPH_2 )

/** Channels and inputs of the signals A / B (DAC1 output 1 / 2) */
#define IT_ADC_CH_A                         ( ADC_CHANNEL_16 )
#define IT_ADC_IN_A                         ( ADC_CHANNEL_INPUT_DAC1 )
#define IT_ADC_CH_B                         ( ADC_CHANNEL_17 )
#define IT_ADC_IN_B                         ( ADC_CHANNEL_INPUT_DAC2 )

/** DAC output values (12-bit) of the signals: A = 1/4, B = 3/4 of the reference voltage */
#define IT_ADC_DAC_VALUE_A                  ( 1024u )
#define IT_ADC_DAC_VALUE_B                  ( 3072u )

/** Maximal difference between DAC value and 12-bit conversion result [LSB] (offset / gain of unbuffered DAC output) */
#define IT_ADC_TOL_LSB                      ( 160u )

/** Sampling time for the unbuffered DAC output and internal channels (387.5 cycles at 8 MHz = 48 us) */
#define IT_ADC_SAMPLING                     ( ADC_CHANNEL_SAMPLING_387_5_CYCLES )

/** Kernel clock: HCLK / 4 (HCLK 64 MHz, conversion clock = kernel clock / 2 within 0.12 - 50 MHz) */
#define IT_ADC_CLK_SRC                      ( ADC_CLK_SRC_HCLK )
#define IT_ADC_CLK_DIV                      ( ADC_CLK_DIV_4 )

/** Maximal difference of repeated conversions of the same signal [LSB, 12-bit] */
#define IT_ADC_NOISE_LSB                    ( 40u )

/** Data buffer size */
#define IT_ADC_BUF_SIZE                     ( 8u )

/** Maximal count of wait loop iterations (Adc_Task called) */
#define IT_ADC_WAIT_LOOPS                   ( 1000000u )

/** Count of Adc_Task calls after a discontinuous subgroup - takes far longer than several conversions */
#define IT_ADC_IDLE_LOOPS                   ( 20000u )

/** Value of a data buffer item which was not written by the module */
#define IT_ADC_BUF_EMPTY                    ( 0xFFFFu )

/** Upper threshold of the analog watch-dog window between the signals A and B [LSB, 12-bit] */
#define IT_ADC_AWD_HIGH_LSB                 ( ( IT_ADC_DAC_VALUE_A + IT_ADC_DAC_VALUE_B ) / 2u )

/** Count of wait loop iterations of the DAC output settling (unbuffered output, internal load) */
#define IT_ADC_DAC_SETTLE_LOOPS             ( 20000u )

/** Plausible VREFINT voltage range at the supply of the boards (VREFINT 1.216 V typical, VDDA 3.0 - 3.6 V) as 12-bit result */
#define IT_ADC_VREF_MIN_LSB                 ( 1300u )
#define IT_ADC_VREF_MAX_LSB                 ( 1700u )

/** Plausible temperature sensor result at room temperature (V30 = 0.62 V typical, 2 mV / C; VDDA 3.0 - 3.6 V) as 12-bit result */
#define IT_ADC_TEMP_MIN_LSB                 ( 500u )
#define IT_ADC_TEMP_MAX_LSB                 ( 1100u )

/** External channels of the dividers: PA0 and PC0 (INP10) */
#define IT_ADC_CH_DIV_PA0                   ( IT_ADC_CH_PA0 )
#define IT_ADC_CH_DIV_PC0                   ( ADC_CHANNEL_10 )

/** Resistors of the dividers [Ohm]: PA0 10k to +3V3 / 4.7k to GND, PC0 4.7k to +3V3 / 10k to GND */
#define IT_ADC_DIV_R_10K                    ( 10000u )
#define IT_ADC_DIV_R_4K7                    ( 4700u )

/** Tolerance of the divider ratio [% of the expected result] (resistors 1 %, ADC offset / gain) */
#define IT_ADC_DIV_TOL_PCT                  ( 3u )

/** Samples of the divider sequences: PA0, PC0 */
#define IT_ADC_DIV_SAMPLES                  ( 2u )

/** Full scale of a 12-bit result [LSB] and count of percent in one unit */
#define IT_ADC_FULL_SCALE_12BIT             ( 4095u )
#define IT_ADC_PERCENT                      ( 100u )

/* ============================== MACROS ==================================== */

/** 12-bit value converted to the resolution of 16 bits */
#define IT_ADC_TO_16BIT( value12 )          ( (value12) << 4u )

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

    ItAdc_Init_Dac();
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
 * \details Initializes the module with clock HCLK / 4 and one regular channel
 *          of ADC2 (polling mode), then reads the clock configuration back.
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
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
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
 * - DAC1 output 1 as input of channel 15 (channel without the internal connection),
 * - 6-bit resolution (not supported by the 16-bit ADC),
 * - configuration slot with identification of another peripheral.
 *
 * \par Expected results
 * - ADC_REQUEST_ERROR is returned in all cases.
 */
void It_Adc_Init_InvalidConfig_ReturnsError( void )
{
    adc_Config_t config;

    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( ADC_NULL_PTR ) );

    /* Internal signal on channel without the connection */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, ADC_CHANNEL_15, IT_ADC_IN_A );
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* Resolution of the 12-bit ADC3 of STM32H72x / H73x */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    config.PeriphConfig[ IT_ADC_PERIPH ].Resolution = ADC_RESOLUTION_6BIT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );

    /* Slot of ADC2 describes ADC1 */
    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    config.PeriphConfig[ IT_ADC_PERIPH ].PeriphId = ADC_PERIPH_1;
    TEST_ASSERT_EQUAL( ADC_REQUEST_ERROR, Adc_Init( &config ) );
}


/**
 * \brief   Adc_Deinit() on target allows new initialization of the peripheral.
 *
 * \details
 * 1. Initializes ADC2 (signal A, polling mode).
 * 2. Initializes it again while enabled.
 * 3. Deinitializes ADC2 and initializes it again.
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
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
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
 * \brief   Polling conversion of signal A matches the DAC output.
 *
 * \details Initializes ADC2 with signal A in polling mode (12-bit, one-shot
 *          buffer of 1 sample), starts conversion and polls Adc_Task() until
 *          transfer complete.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - Result is 1024 LSB +- 160 LSB.
 */
void It_Adc_Set_RegStart_SignalAPoll_ResultMatchesDacOutput( void )
{
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, itAdc_Buffer[ 0u ] );
}


/**
 * \brief   Conversion result follows the change of the DAC output.
 *
 * \details Converts signal A, writes the DAC1 output 1 value of signal B, waits
 *          for the DAC output to settle and converts again.
 *
 * \par Expected results
 * - 1st result is 1024 LSB +- 160 LSB, 2nd result is 3072 LSB +- 160 LSB.
 */
void It_Adc_Set_RegStart_DacOutputChanged_ResultFollows( void )
{
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, itAdc_Buffer[ 0u ] );

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Set_Data( DAC_PERIPH_1, DAC_CHANNEL_1, IT_ADC_DAC_VALUE_B ) );
    ItAdc_Wait_DacSettled();

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 2u );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_B, itAdc_Buffer[ 0u ] );
}


/**
 * \brief   Manual polling: software conversion started repeatedly, result read by Adc_Get_RegData().
 *
 * \details Polling mode without buffer, 3x: start, wait for EOC flag, read data.
 *
 * \par Expected results
 * - EOC flag active after every start, data are 1024 LSB +- 160 LSB.
 * - EOC flag cleared by reading of the data.
 */
void It_Adc_Get_RegData_ManualPolling_RepeatedSoftwareStart( void )
{
    adc_Config_t    config;
    adc_FlagState_t eocFlag = ADC_FLAG_INACTIVE;
    adc_Data_t      data    = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 0u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    config.PeriphConfig[ IT_ADC_PERIPH ].DataConfig.DataBuffer = ADC_NULL_PTR;
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    for( uint32_t convIdx = 0u; 3u > convIdx; convIdx++ )
    {
        eocFlag = ADC_FLAG_INACTIVE;

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );

        for( uint32_t loopIdx = 0u;
             ( IT_ADC_WAIT_LOOPS > loopIdx ) &&
             ( ADC_FLAG_INACTIVE == eocFlag );
             loopIdx++ )
        {
            TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOC, &eocFlag ) );
        }

        TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, eocFlag );
        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_RegData( IT_ADC_PERIPH, &data ) );
        TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, data );

        TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOC, &eocFlag ) );
        TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, eocFlag );
    }
}


/**
 * \brief   8-bit resolution on target scales the conversion result.
 *
 * \details Initializes ADC2 with signal A and 8-bit resolution, reads the
 *          resolution back and converts in polling mode.
 *
 * \par Expected results
 * - Resolution reads back 8-bit.
 * - Result is below 256, result << 4 is 1024 LSB +- 320 LSB.
 */
void It_Adc_Set_Resolution_8Bit_ResultScaled( void )
{
    adc_Config_t     config;
    adc_Resolution_t resolution = ADC_RESOLUTION_12BIT;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    config.PeriphConfig[ IT_ADC_PERIPH ].Resolution = ADC_RESOLUTION_8BIT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Resolution( IT_ADC_PERIPH, &resolution ) );
    TEST_ASSERT_EQUAL( ADC_RESOLUTION_8BIT, resolution );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_LESS_THAN_UINT16( 256u, itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT16_WITHIN( 2u * IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, (adc_Data_t)( itAdc_Buffer[ 0u ] << 4u ) );
}


/**
 * \brief   16-bit resolution on target scales the conversion result.
 *
 * \details Initializes ADC2 with signal B and 16-bit resolution, reads the
 *          resolution back and converts in polling mode.
 *
 * \par Expected results
 * - Resolution reads back 16-bit.
 * - Result is 49152 LSB +- 2560 LSB (3/4 of 16-bit range, tolerance of 160 LSB at 12 bits).
 */
void It_Adc_Set_Resolution_16Bit_ResultScaled( void )
{
    adc_Config_t     config;
    adc_Resolution_t resolution = ADC_RESOLUTION_12BIT;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_B, IT_ADC_IN_B );
    config.PeriphConfig[ IT_ADC_PERIPH ].Resolution = ADC_RESOLUTION_16BIT;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Resolution( IT_ADC_PERIPH, &resolution ) );
    TEST_ASSERT_EQUAL( ADC_RESOLUTION_16BIT, resolution );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TO_16BIT( IT_ADC_TOL_LSB ), IT_ADC_TO_16BIT( IT_ADC_DAC_VALUE_B ), itAdc_Buffer[ 0u ] );
}


/**
 * \brief   Regular sequence of both signals in ISR mode.
 *
 * \details Initializes ADC2 in ISR mode with buffer of 2 samples and regular
 *          sequence signal A, signal B, starts conversion and waits for
 *          transfer complete.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - 1st sample is 1024 LSB +- 160 LSB, 2nd sample is 3072 LSB +- 160 LSB.
 */
void It_Adc_Set_RegStart_SequenceAbIsr_ValuesInOrder( void )
{
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_B, IT_ADC_IN_B );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_B, itAdc_Buffer[ 1u ] );
}


/**
 * \brief   Continuous conversion with circular DMA buffer.
 *
 * \details Initializes ADC2 in DMA mode (DMA2 stream 0) with circular buffer of
 *          8 samples and continuous conversion of signal A, starts it, waits for
 *          3 buffer completions and stops the conversion.
 *
 * \par Expected results
 * - Complete and half transfer callbacks called at least 3x, no error callback.
 * - All samples differ from the 1st one by at most 40 LSB (noise).
 * - 1st sample is 1024 LSB +- 160 LSB.
 * - After stop no further transfer complete is reported.
 */
void It_Adc_Set_RegStart_ContinuousDmaCircular_HalfAndCompleteCallbacks( void )
{
    adc_Config_t config;
    uint32_t     completeCnt = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_DMA, IT_ADC_BUF_SIZE, ADC_BUFFER_MODE_CIRCULAR, ADC_REG_TRIGGER_MODE_CONTINUOUS );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
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

    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, itAdc_Buffer[ 0u ] );

    /* Stopped - no more transfers (wait of a few buffer periods) */
    completeCnt = itAdc_CompleteCnt;
    ItAdc_Wait( &itAdc_CompleteCnt, completeCnt + 1u );
    TEST_ASSERT_EQUAL_UINT32( completeCnt, itAdc_CompleteCnt );
}


/**
 * \brief   One shot DMA buffer can be filled again after restart.
 *
 * \details DMA mode, one shot buffer of 8 samples, continuous conversion of
 *          signal A. Starts the conversion, waits for transfer complete
 *          (conversion stopped by the module), restarts it and waits again.
 *
 * \par Expected results
 * - Transfer complete callback 1x after each start, no error callback (no overrun after
 *   the automatic stop).
 * - The last sample is 1024 LSB +- 160 LSB.
 */
void It_Adc_Set_RegStart_DmaOneShotRestart_BufferFilledTwice( void )
{
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_DMA, IT_ADC_BUF_SIZE, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_CONTINUOUS );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );

    itAdc_Buffer[ IT_ADC_BUF_SIZE - 1u ] = 0xFFFFu;

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 2u );
    TEST_ASSERT_EQUAL_UINT32( 2u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );

    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, itAdc_Buffer[ IT_ADC_BUF_SIZE - 1u ] );
}


/**
 * \brief   Injected conversion of both signals on target.
 *
 * \details Initializes ADC2 with injected sequence signal A, signal B (ISR mode),
 *          starts the injected group by software and waits for injected complete
 *          callback.
 *
 * \par Expected results
 * - Injected complete callback 1x.
 * - Rank 1 gives 1024 LSB +- 160 LSB, rank 2 gives 3072 LSB +- 160 LSB.
 */
void It_Adc_Set_InjStart_SequenceAbInjected_InjCallbackAndData( void )
{
    adc_Config_t config;
    adc_Data_t   injData = 0u;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_InjChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    ItAdc_Add_InjChannel( &config, IT_ADC_CH_B, IT_ADC_IN_B );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_InjStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_InjCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_InjCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_InjData( IT_ADC_PERIPH, ADC_INJ_SEQUENCE_1, &injData ) );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, injData );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_InjData( IT_ADC_PERIPH, ADC_INJ_SEQUENCE_2, &injData ) );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_B, injData );
}

/*----------------------------- Analog watchdog ------------------------------*/

/**
 * \brief   Analog watchdog detects signal A above the window.
 *
 * \details Initializes ADC2 with signal A and AWD1 on all regular channels with
 *          window 0 - 512 LSB (signal A ~1024 LSB), clears AWD1 flag and converts.
 *
 * \par Expected results
 * - Thresholds read back 0 / 512.
 * - AWD1 flag is ACTIVE after conversion.
 */
void It_Adc_AwdInit_SignalAboveHighThreshold_AwdFlagActive( void )
{
    adc_Config_t       config;
    adc_AwdConfig_t    awdConfig;
    adc_FlagState_t    awdFlag  = ADC_FLAG_INACTIVE;
    adc_AwdThreshold_t lowThr   = 0xFFFFu;
    adc_AwdThreshold_t highThr  = 0xFFFFu;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    awdConfig.AwdId            = ADC_AWD_1;
    awdConfig.AwdMode          = ADC_AWD_MODE_ALL_REGULAR;
    awdConfig.AwdLowThreshold  = 0u;
    awdConfig.AwdHighThreshold = 512u;
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_AwdThresholds( IT_ADC_PERIPH, ADC_AWD_1, &lowThr, &highThr ) );
    TEST_ASSERT_EQUAL_UINT16( 0u, lowThr );
    TEST_ASSERT_EQUAL_UINT16( 512u, highThr );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, awdFlag );
}


/**
 * \brief   Analog watchdog does not trigger for signal A inside the window.
 *
 * \details Initializes ADC2 with signal A and AWD1 on all regular channels with
 *          window 512 - 2000 LSB, clears AWD1 flag and converts.
 *
 * \par Expected results
 * - AWD1 flag is INACTIVE after conversion.
 */
void It_Adc_AwdInit_SignalInsideWindow_AwdFlagInactive( void )
{
    adc_Config_t    config;
    adc_AwdConfig_t awdConfig;
    adc_FlagState_t awdFlag = ADC_FLAG_ACTIVE;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 1u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    awdConfig.AwdId            = ADC_AWD_1;
    awdConfig.AwdMode          = ADC_AWD_MODE_ALL_REGULAR;
    awdConfig.AwdLowThreshold  = 512u;
    awdConfig.AwdHighThreshold = 2000u;
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, awdFlag );
}

/*------------------------------ Internal channels ---------------------------*/

/**
 * \brief   VREFINT and temperature sensor of ADC2 give plausible results.
 *
 * \details STM32H7A3 / H7B0 / H7B3 (ignored on the other lines): initializes ADC2 in ISR mode with regular
 *          sequence VREFINT, temperature sensor, starts conversion and waits for
 *          transfer complete.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - VREFINT result 1300 - 1700 LSB (1.216 V at VDDA 3.0 - 3.6 V).
 * - Temperature sensor result 500 - 1100 LSB (room temperature).
 */
void It_Adc_Set_RegStart_VrefTempSequenceIsr_ValuesPlausible( void )
{
#if defined(ADC_VER_V5_3)
    adc_Config_t config;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_ISR, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, ADC_CHANNEL_19, ADC_CHANNEL_INPUT_VREF );
    ItAdc_Add_RegChannel( &config, ADC_CHANNEL_18, ADC_CHANNEL_INPUT_TEMP );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_UINT16_WITHIN( ( IT_ADC_VREF_MAX_LSB - IT_ADC_VREF_MIN_LSB ) / 2u, ( IT_ADC_VREF_MAX_LSB + IT_ADC_VREF_MIN_LSB ) / 2u, itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT16_WITHIN( ( IT_ADC_TEMP_MAX_LSB - IT_ADC_TEMP_MIN_LSB ) / 2u, ( IT_ADC_TEMP_MAX_LSB + IT_ADC_TEMP_MIN_LSB ) / 2u, itAdc_Buffer[ 1u ] );
#else
    TEST_IGNORE_MESSAGE( "VREFINT and temperature sensor are not on ADC1 / ADC2 of this line" );
#endif /* ADC_VER_V5_3 */
}


/**
 * \brief   Regular discontinuous mode converts 1 rank per software start.
 *
 * \details Initializes ADC2 in polling mode with one-shot buffer of 2 samples and regular
 *          sequence signal A, signal B converted 1 rank per trigger
 *          (ADC_REG_TRIGGER_MODE_DISCONT_1RANK, software trigger), clears the end of
 *          sequence flag and:
 *          1. starts the regular group, waits for the 1st sample and calls Adc_Task() for
 *             IT_ADC_IDLE_LOOPS more iterations (far longer than one conversion),
 *          2. starts the regular group again and waits for transfer complete.
 *          1 rank per trigger is used, because RM0433 Rev 9 25.4.16 states that ADSTART is
 *          cleared at the end of conversion (EOC) in discontinuous mode with software trigger.
 * \note    Task AB#1824: the regular discontinuous mode was not available. Without DISCEN the
 *          1st start converts the whole sequence (2nd sample written, EOS set).
 *
 * \par Expected results
 * - Conversion mode reads back ADC_REG_TRIGGER_MODE_DISCONT_1RANK.
 * 1. 1 sample is stored, the 2nd buffer item is not written, no transfer complete, end of
 *    sequence flag INACTIVE.
 * 2. The start is accepted (ADSTART was cleared by HW after the 1st subgroup): transfer complete 1x,
 *    end of sequence flag ACTIVE, no error.
 * - 1st sample is 1024 LSB +- 160 LSB (signal A), 2nd sample is 3072 LSB +- 160 LSB (signal B).
 */
void It_Adc_Set_RegStart_DiscontinuousOneRank_EachStartConvertsSubgroup( void )
{
    adc_Config_t         config;
    adc_RegTriggerMode_t triggerMode = ADC_REG_TRIGGER_MODE_CNT;
    adc_FlagState_t      eosFlag     = ADC_FLAG_ACTIVE;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_DISCONT_1RANK );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_B, IT_ADC_IN_B );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_TriggerMode( IT_ADC_PERIPH, &triggerMode ) );
    TEST_ASSERT_EQUAL( ADC_REG_TRIGGER_MODE_DISCONT_1RANK, triggerMode );

    /* 1st subgroup: rank 1 (signal A) */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOS ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait_Sample( 0u );
    ItAdc_Poll( IT_ADC_IDLE_LOOPS );

    TEST_ASSERT_NOT_EQUAL( IT_ADC_BUF_EMPTY, itAdc_Buffer[ 0u ] );
    TEST_ASSERT_EQUAL_HEX16( IT_ADC_BUF_EMPTY, itAdc_Buffer[ 1u ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOS, &eosFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, eosFlag );

    /* 2nd subgroup: rank 2 (signal B) */
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_REG_EOS, &eosFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_ACTIVE, eosFlag );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_B, itAdc_Buffer[ 1u ] );
}


/**
 * \brief   Analog watchdog 1 monitors only the selected channel.
 *
 * \details Initializes ADC2 with regular sequence signal A, signal B in polling mode (buffer
 *          of 2 samples) and the window 0 - IT_ADC_AWD_HIGH_LSB (between the signals), then:
 *          1. AWD1 on the single regular channel of signal A, converts the sequence,
 *          2. AWD1 on the single regular channel of signal B, converts the sequence again.
 * \note    Task AB#1825: the single channel modes were rejected (no channel in adc_AwdConfig_t).
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK, AWD1 flag INACTIVE (signal B is outside the window, but not monitored).
 * 2. ADC_REQUEST_OK, AWD1 flag ACTIVE.
 * - Transfer complete 2x, no error, samples 1024 / 3072 LSB +- 160 LSB.
 */
void It_Adc_AwdInit_Awd1SingleChannel_OnlySelectedChannelMonitored( void )
{
    adc_Config_t    config;
    adc_AwdConfig_t awdConfig;
    adc_FlagState_t awdFlag = ADC_FLAG_ACTIVE;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_B, IT_ADC_IN_B );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* 1. Signal A only - inside the window */
    awdConfig.AwdId            = ADC_AWD_1;
    awdConfig.AwdMode          = ADC_AWD_MODE_SINGLE_REGULAR;
    awdConfig.AwdChannelId     = IT_ADC_CH_A;
    awdConfig.AwdChannelMask   = ADC_AWD_CHANNEL_MASK_NONE;
    awdConfig.AwdLowThreshold  = 0u;
    awdConfig.AwdHighThreshold = IT_ADC_AWD_HIGH_LSB;
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_A, itAdc_Buffer[ 0u ] );
    TEST_ASSERT_UINT16_WITHIN( IT_ADC_TOL_LSB, IT_ADC_DAC_VALUE_B, itAdc_Buffer[ 1u ] );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD1, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, awdFlag );

    /* 2. Signal B only - above the window */
    awdConfig.AwdChannelId = IT_ADC_CH_B;
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
 * \details Initializes ADC2 with regular sequence signal A, signal B in polling mode (buffer
 *          of 2 samples) and the window 0 - IT_ADC_AWD_HIGH_LSB (between the signals), then:
 *          1. AWD2 in single mode with channel set {16} (signal A), converts the sequence,
 *          2. AWD2 with channel set {16, 17} (signals A and B), converts the sequence again.
 * \note    Task AB#1825: the channel set of AWD2 / AWD3 was not configurable.
 *
 * \par Expected results
 * 1. ADC_REQUEST_OK, AWD2 flag INACTIVE (signal B is outside the window, but not monitored).
 * 2. ADC_REQUEST_OK, AWD2 flag ACTIVE.
 * - Transfer complete 2x, no error.
 */
void It_Adc_AwdInit_Awd2ChannelMask_OnlySelectedChannelsMonitored( void )
{
    adc_Config_t    config;
    adc_AwdConfig_t awdConfig;
    adc_FlagState_t awdFlag = ADC_FLAG_ACTIVE;

    ItAdc_Get_Config( &config, ADC_TRANSFER_MODE_POLL, 2u, ADC_BUFFER_MODE_ONE_SHOT, ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_A, IT_ADC_IN_A );
    ItAdc_Add_RegChannel( &config, IT_ADC_CH_B, IT_ADC_IN_B );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    /* 1. Signal A only - inside the window */
    awdConfig.AwdId            = ADC_AWD_2;
    awdConfig.AwdMode          = ADC_AWD_MODE_SINGLE;
    awdConfig.AwdChannelId     = IT_ADC_CH_A;
    awdConfig.AwdChannelMask   = ADC_AWD_CHANNEL_MASK_16;
    awdConfig.AwdLowThreshold  = 0u;
    awdConfig.AwdHighThreshold = IT_ADC_AWD_HIGH_LSB;
    awdConfig.AwdFilter        = ADC_AWD_FILTER_NONE;
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_AwdInit( IT_ADC_PERIPH, &awdConfig ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Clear_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2 ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Get_Flag( IT_ADC_PERIPH, ADC_FLAG_AWD2, &awdFlag ) );
    TEST_ASSERT_EQUAL( ADC_FLAG_INACTIVE, awdFlag );

    /* 2. Signals A and B - signal B above the window */
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
 * \details Polling mode, one-shot buffer of 2 samples, regular sequence channel 0 (PA0, INP0
 *          through ADC_OR.OP0) and channel 10 (PC0).
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - PA0: 4095 x 4.7k / 14.7k = 1309 LSB +- 3 %, PC0: 4095 x 10k / 14.7k = 2785 LSB +- 3 %.
 */
void It_Adc_Dividers_PollSequence_RatiosMatchResistors( void )
{
    adc_Config_t config;

    ItAdc_RequireDividers();
    ItAdc_Set_EthPhyPowerDown();

    ItAdc_Get_Config( &config,
                      ADC_TRANSFER_MODE_POLL,
                      IT_ADC_DIV_SAMPLES,
                      ADC_BUFFER_MODE_ONE_SHOT,
                      ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_PeriphRegChannel( &config, IT_ADC_DIV_PERIPH, IT_ADC_CH_DIV_PA0, ADC_CHANNEL_INPUT_PIN_SINGLE );
    ItAdc_Add_PeriphRegChannel( &config, IT_ADC_DIV_PERIPH, IT_ADC_CH_DIV_PC0, ADC_CHANNEL_INPUT_PIN_SINGLE );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );

    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_DIV_PERIPH ) );
    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    ItAdc_Check_Divider( itAdc_Buffer[ 0u ], IT_ADC_DIV_R_10K, IT_ADC_DIV_R_4K7, "PA0 divider (channel 0)" );
    ItAdc_Check_Divider( itAdc_Buffer[ 1u ], IT_ADC_DIV_R_4K7, IT_ADC_DIV_R_10K, "PC0 divider (channel 10)" );
}


/**
 * \brief   Divider voltages converted in DMA mode.
 *
 * \details DMA mode (DMA2 stream 0), one-shot buffer of 2 samples, regular sequence of ADC1
 *          channel PA0 and channel 10 (PC0). VREFINT is not part of the sequence (it is an
 *          input of another ADC on the classic lines), the ratio is independent of VDDA.
 *
 * \par Expected results
 * - Transfer complete callback 1x, no error callback.
 * - Divider ratios as in polling mode (+- 3 %).
 */
void It_Adc_Dividers_DmaSequence_RatiosMatchResistors( void )
{
    adc_Config_t config;

    ItAdc_RequireDividers();
    ItAdc_Set_EthPhyPowerDown();

    ItAdc_Get_Config( &config,
                      ADC_TRANSFER_MODE_DMA,
                      IT_ADC_DIV_SAMPLES,
                      ADC_BUFFER_MODE_ONE_SHOT,
                      ADC_REG_TRIGGER_MODE_SINGLE );
    ItAdc_Add_PeriphRegChannel( &config, IT_ADC_DIV_PERIPH, IT_ADC_CH_DIV_PA0, ADC_CHANNEL_INPUT_PIN_SINGLE );
    ItAdc_Add_PeriphRegChannel( &config, IT_ADC_DIV_PERIPH, IT_ADC_CH_DIV_PC0, ADC_CHANNEL_INPUT_PIN_SINGLE );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Init( &config ) );
    TEST_ASSERT_EQUAL( ADC_REQUEST_OK, Adc_Set_RegStart( IT_ADC_DIV_PERIPH ) );

    ItAdc_Wait( &itAdc_CompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itAdc_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itAdc_ErrorCnt );
    ItAdc_Check_Divider( itAdc_Buffer[ 0u ], IT_ADC_DIV_R_10K, IT_ADC_DIV_R_4K7, "PA0 divider" );
    ItAdc_Check_Divider( itAdc_Buffer[ 1u ], IT_ADC_DIV_R_4K7, IT_ADC_DIV_R_10K, "PC0 divider (channel 10)" );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Fills module configuration: ADC2 used (software trigger, 12-bit), other
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
        periphCfg->InjTriggerMode = ADC_INJ_TRIGGER_MODE_CONTINUOUS;   /* One trigger converts the whole sequence */
        periphCfg->InjTriggerId   = ADC_INJ_TRIGGER_SOFTWARE;
        periphCfg->InjTriggerEdge = ADC_TRIGGER_EDGE_RISING;
        periphCfg->InjChannelsCnt = 0u;

        periphCfg->DataConfig.TransferMode             = transferMode;
        periphCfg->DataConfig.DataBuffer               = itAdc_Buffer;
        periphCfg->DataConfig.BufferSize               = bufferSize;
        periphCfg->DataConfig.BufferMode               = bufferMode;
        periphCfg->DataConfig.DmaPeriphId              = ADC_DMA_PERIPH_2;
        periphCfg->DataConfig.DmaChannelId             = ADC_DMA_CHANNEL_0;
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
 * \brief Appends channel to the regular sequence of the ADC under test.
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
 * \brief Adds a channel to the regular sequence of a peripheral.
 *
 * \param config       [out]: Module configuration
 * \param periphId      [in]: ADC peripheral
 * \param channelId     [in]: Channel
 * \param channelInput  [in]: Channel input
 */
static void ItAdc_Add_PeriphRegChannel( adc_Config_t * const config,
                                        adc_PeriphId_t periphId,
                                        adc_ChannelId_t channelId,
                                        adc_ChannelInput_t channelInput )
{
    adc_PeriphConfig_t * const periphCfg = &config->PeriphConfig[ periphId ];

    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].ChannelId       = channelId;
    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].ChannelInput    = channelInput;
    periphCfg->RegChannels[ periphCfg->RegChannelsCnt ].ChannelSampling = IT_ADC_SAMPLING;
    periphCfg->RegChannelsCnt++;
}


/**
 * \brief Appends channel to the injected sequence of the ADC under test.
 *
 * \param config       [in,out]: Module configuration
 * \param channelId        [in]: Channel
 * \param channelInput     [in]: Channel input
 */
static void ItAdc_Add_InjChannel( adc_Config_t * const config, adc_ChannelId_t channelId, adc_ChannelInput_t channelInput )
{
    adc_PeriphConfig_t * const periphCfg = &config->PeriphConfig[ IT_ADC_PERIPH ];

    periphCfg->InjChannels[ periphCfg->InjChannelsCnt ].ChannelId       = channelId;
    periphCfg->InjChannels[ periphCfg->InjChannelsCnt ].ChannelInput    = channelInput;
    periphCfg->InjChannels[ periphCfg->InjChannelsCnt ].ChannelSampling = IT_ADC_SAMPLING;
    periphCfg->InjChannelsCnt++;
}


/**
 * \brief Initializes both outputs of DAC1 as internal signals A / B of the tests.
 *
 * Output mode "on-chip peripherals only" (unbuffered, pins are not driven), software
 * written data, 12-bit right aligned values of \ref IT_ADC_DAC_VALUE_A / \ref IT_ADC_DAC_VALUE_B.
 */
static void ItAdc_Init_Dac( void )
{
    dac_Config_t dacConfig;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Get_DefaultConfig( &dacConfig ) );

    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_1 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_1 ].OutputMode  = DAC_OUTPUT_INTERNAL;
    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_1 ].InitValue   = IT_ADC_DAC_VALUE_A;

    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_2 ].ChannelUsed = DAC_FUNCTION_ACTIVE;
    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_2 ].OutputMode  = DAC_OUTPUT_INTERNAL;
    dacConfig.PeriphConfig[ DAC_PERIPH_1 ].Channels[ DAC_CHANNEL_2 ].InitValue   = IT_ADC_DAC_VALUE_B;

    TEST_ASSERT_EQUAL( DAC_REQUEST_OK, Dac_Init( &dacConfig ) );

    ItAdc_Wait_DacSettled();
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
 * \brief Waits until the DAC output settles after the initialization / data write.
 */
static void ItAdc_Wait_DacSettled( void )
{
    for( volatile uint32_t loopIdx = 0u; IT_ADC_DAC_SETTLE_LOOPS > loopIdx; loopIdx++ )
    {
        /* Waiting for the DAC output */
    }
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
 * \brief   Puts the Ethernet PHY of the board into power-down before a divider conversion (AB#2248).
 *
 * \details NUCLEO-H723ZG / NUCLEO-H755ZI-Q: the PHY drives its 50 MHz RMII reference clock on PA1, next to the
 *          divider input PA0, and the PA0 conversion spreads by about +-60 LSB while it runs. The Eth module
 *          initializes the MAC and the MDIO bus (pins of the Eth integration tests), writes the power-down bit of
 *          BMCR and is deinitialized; the PHY stays in power-down until NRST resets it with the next test case.
 *          A board without the PHY needs no action.
 */
static void ItAdc_Set_EthPhyPowerDown( void )
{
#if ( 1u == IT_ADC_ETH_PHY )
    eth_Config_t config;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Get_DefaultConfig( &config ) );

    itAdc_EthMemory.TxDescriptors = itAdc_EthTxDescriptors;
    itAdc_EthMemory.RxDescriptors = itAdc_EthRxDescriptors;
    itAdc_EthMemory.RxBufferPool  = itAdc_EthRxBufferPool;

    config.Memory          = &itAdc_EthMemory;
    config.TxDescriptorCnt = IT_ADC_ETH_DESC_CNT;
    config.RxDescriptorCnt = IT_ADC_ETH_DESC_CNT;
    config.RxBufferSize    = IT_ADC_ETH_RX_BUFFER_SIZE;
    config.Pins.RefClkPin  = ETH_RMII_REF_CLK_PIN_PA1;
    config.Pins.CrsDvPin   = ETH_RMII_CRS_DV_PIN_PA7;
    config.Pins.Rxd0Pin    = ETH_RMII_RXD0_PIN_PC4;
    config.Pins.Rxd1Pin    = ETH_RMII_RXD1_PIN_PC5;
    config.Pins.Txd0Pin    = ETH_RMII_TXD0_PIN_PB12;
    config.Pins.Txd1Pin    = ETH_RMII_TXD1_PIN_PB13;
    config.Pins.TxEnPin    = ETH_RMII_TX_EN_PIN_PB11;
    config.Pins.MdcPin     = ETH_MDC_PIN_PC1;
    config.Pins.MdioPin    = ETH_MDIO_PIN_PA2;

    /* Mandatory callbacks of the Eth module, no frame is transferred */
    config.TxCompleteCallback = ItAdc_EthTxCompleteCallback;
    config.RxCallback         = ItAdc_EthRxCallback;

    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Init( &config ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Set_PhyRegister( IT_ADC_ETH_PHY_ADDR,
                                                            IT_ADC_ETH_PHY_BMCR,
                                                            IT_ADC_ETH_PHY_BMCR_POWER_DOWN ) );
    TEST_ASSERT_EQUAL( ETH_REQUEST_OK, Eth_Deinit() );
#else
    /* Board without an Ethernet PHY */
    return;
#endif /* IT_ADC_ETH_PHY */
}


#if ( 1u == IT_ADC_ETH_PHY )
/**
 * \brief Transmit complete callback of the Ethernet initialization (no frame is transmitted).
 *
 * \param tag       [in]: Application tag of the frame
 * \param status    [in]: Result of the transmission
 * \param timestamp [in]: Transmit timestamp
 */
static void ItAdc_EthTxCompleteCallback( eth_FrameTag_t tag,
                                         eth_RequestState_t status,
                                         const eth_PtpTime_t * const timestamp )
{
    (void)tag;
    (void)status;
    (void)timestamp;
}


/** \brief Receive callback of the Ethernet initialization (no frame is expected, the PHY is put into power-down) */
static void ItAdc_EthRxCallback( void )
{
    return;
}
#endif /* IT_ADC_ETH_PHY */


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
