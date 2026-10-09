#===================== Integration tests configuration ========================#
# Integration tests of Adc module on target (NUCLEO-G474RE). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# Internal channels (VREFINT, temperature sensor) are converted, DAC1_OUT1 (PA4)
# is measured by ADC2 channel 17 (same pin) through Dac module. No wiring is needed.
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

IntegrationTesting_Add_Test(
    NAME            Adc
    TEST_SOURCE     ItTest_Adc.c
    SOURCES         ItTarget_Adc.c      # Target interface of the test set
    LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                    Nvic_Lib            # Fault handlers, system reset (ItTarget)
                    Adc_Lib             # Module under test
                    Dac_Lib             # DAC1 output measured by ADC2
    TIMEOUT         60
)
