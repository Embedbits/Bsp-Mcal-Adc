#======================== Unit tests configuration ============================#
# Unit tests of Adc module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(ADC_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${ADC_DIR}/.." ABSOLUTE)

# STM32H7R / STM32H7S: 12-bit ADC of STM32H5 (tests of the STM32H5 module with the STM32H7R / H7S device
# data, GPDMA), classic STM32H7 lines: 16-bit ADC (DMA streams)
if(MCU_FAMILY_ID STREQUAL "STM32H7RSxx")
    set(ADC_UT_SOURCE   Test_Adc_H7RS.c)
    set(ADC_UT_DMA_DIR  ${MCAL_DIR}/Gpdma)
    set(ADC_UT_DMA_MOCK ${MCAL_DIR}/Gpdma/Gpdma_Port.h)
else()
    set(ADC_UT_SOURCE   Test_Adc.c)
    set(ADC_UT_DMA_DIR  ${MCAL_DIR}/Dma)
    set(ADC_UT_DMA_MOCK ${MCAL_DIR}/Dma/Dma_Port.h)
endif()

# Tests use HW model thread (calibration / disable / stop) - executed serially
UnitTesting_Add_Test(
    NAME            Adc
    TEST_SOURCE     ${ADC_UT_SOURCE}
    UUT_SOURCES     ${ADC_DIR}/Adc.c
                    ${ADC_DIR}/Adc_Dma.c
                    ${ADC_DIR}/Adc_Isr.c
                    ${ADC_DIR}/Adc_Poll.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
                    ${ADC_UT_DMA_MOCK}
    INCLUDE_DIRS    ${ADC_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpio
                    ${ADC_UT_DMA_DIR}
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
