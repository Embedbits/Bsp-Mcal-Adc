#===================== Integration tests configuration ========================#
# Integration tests of Adc module on target (Nucleo board). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Adc.c and
# all libraries of the firmware are listed here.

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_Adc.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(MCU_FAMILY_ID STREQUAL "STM32H7RSxx")

    # STM32H7R / STM32H7S: 12-bit ADC of STM32H5, GPDMA1 data transfer.
    # The internal flash has 64 KB. The firmware with the ADC, GPDMA and RCC modules needs 77 KB
    # when built without optimization (-O0 of the integration test build) and 41 KB with -Os, so
    # the test is created only if the integration test build is optimized.
    if(CMAKE_C_FLAGS_INTEGRATIONTEST MATCHES "-O0")
        message(WARNING "Integration test IT_Adc of STM32H7R / STM32H7S is not created: the firmware does not fit into 64 KB flash at -O0 (${CMAKE_C_FLAGS_INTEGRATIONTEST}) - configure with -DINTEGRATION_TEST_OPTIMIZATION=-Os")
    else()
        IntegrationTesting_Add_Test(
            NAME            Adc
            TEST_SOURCE     ItTest_Adc_H7RS.c
            SOURCES         ItTarget_Adc.c      # Target interface of the test set
            LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                            Nvic_Lib            # Fault handlers, system reset (ItTarget)
                            Adc_Lib             # Module under test
                            Gpdma_Lib           # DMA channel of the ADC
            TIMEOUT         60
        )
    endif()

else()

    # Classic STM32H7: 16-bit ADC, signals from the internal outputs of DAC1
    IntegrationTesting_Add_Test(
        NAME            Adc
        TEST_SOURCE     ItTest_Adc.c
        SOURCES         ItTarget_Adc.c      # Target interface of the test set
        LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                        Nvic_Lib            # Fault handlers, system reset (ItTarget)
                        Adc_Lib             # Module under test
                        Dma_Lib             # DMA stream of the ADC
                        Dac_Lib             # Source of the converted signals
        TIMEOUT         60
    )

endif()
