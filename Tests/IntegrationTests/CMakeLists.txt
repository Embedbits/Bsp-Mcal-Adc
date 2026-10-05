#===================== Integration tests configuration ========================#
# Integration tests of Adc module on target (Nucleo board). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).

IntegrationTesting_Add_Test(
    NAME            Adc
    TEST_SOURCE     ItTest_Adc.c
    LINK_LIBS       Adc_Lib
    TIMEOUT         60
)
