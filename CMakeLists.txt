# Template version: 0.0.1
# Sets the version of CMakeLists.txt file.
# Never, under any circumstances, change this line. It is used by
# the module generation script to identify the template version.

#======================== Library configuration ===============================#
# Publicly included directories to provide headers include
set( Adc_PublicIncludeDirs
    
)


# Privately included directories to provide headers include
set( Adc_PrivateIncludeDirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)


# Source files list for current library generation
set( Adc_SourceFiles
    Adc.c
    Adc_Dma.c
    Adc_Isr.c
    Adc_Poll.c
)


# List of public header files provided by the current library
set( Adc_PublicHeaders 
    Adc_Types.h
    Adc_Port.h
)


# List of dependent libraries that are publicly used by the current library
set( Adc_PublicDependentLibs
    Dma_Lib                                 # DMA streams (classic STM32H7 lines)
    Gpdma_Lib                               # GPDMA1 / HPDMA1 (STM32H7R / STM32H7S)
)


# List of dependent libraries that are privately used by the current library
set( Adc_PrivateDependentLibs
    Rcc_Lib
    Gpio_Lib
    Nvic_Lib
)


#========================== Library generation ================================#
# Create library
add_library(Adc_Lib STATIC ${Adc_SourceFiles})


# Include necessary libraries to the target
target_link_libraries( Adc_Lib
    PUBLIC 
        ${Adc_PublicDependentLibs} 
    PRIVATE 
        ${Adc_PrivateDependentLibs}
)


# Set public headers directory path in build folder
set(PUBLIC_HEADERS_DIR ${CMAKE_CURRENT_BINARY_DIR}/PublicHeaders)

# Generates the public headers directory in build folder
file(MAKE_DIRECTORY ${PUBLIC_HEADERS_DIR})

# Process all public headers
foreach(PublicHeader IN LISTS Adc_PublicHeaders)
    
    # Copy (or configure) only the public header there
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/${PublicHeader} 
        ${PUBLIC_HEADERS_DIR}/${PublicHeader}
        COPYONLY
    )
    
endforeach()


# Set include directories for the created target
target_include_directories( Adc_Lib
    PUBLIC
        $<BUILD_INTERFACE:${PUBLIC_HEADERS_DIR}>
        $<INSTALL_INTERFACE:include>
        ${Adc_PublicIncludeDirs}
    PRIVATE
        ${Adc_PrivateIncludeDirs}
)


# Print status message
message(STATUS "Adc_Lib library created")

#========================= Documentation configuration ========================#
if(DOXYGEN_AVAILABLE STREQUAL "ON")
    Doxygen_AddPath("${CMAKE_CURRENT_SOURCE_DIR}/")
endif()

#========================= Unit tests configuration ===========================#
if(UNIT_TESTING_AVAILABLE STREQUAL "ON")
    UnitTesting_AddPath("${CMAKE_CURRENT_SOURCE_DIR}/Tests/UnitTests")
endif()

#===================== Integration tests configuration ========================#
if(INTEGRATION_TESTING_AVAILABLE STREQUAL "ON")
    IntegrationTesting_AddPath("${CMAKE_CURRENT_SOURCE_DIR}/Tests/IntegrationTests")
endif()
