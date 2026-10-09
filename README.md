# ADC MCAL Module

The **ADC (Analog-to-Digital Converter) MCAL module** provides an abstraction layer for managing analog-to-digital conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of ADC peripheral
- Task handler for periodic servicing
- Support for default configuration retrieval
- Standardized request state return values
- Versioning support for module management

---

## Public API

### Module Information

- `adc_ModuleVersion_t Adc_Get_ModuleVersion ( void );`  
  Returns the current version of the ADC module.

---

### Initialization

- `adc_RequestState_t Adc_Init ( adc_PeriphConfig_t * const adcConfig );`  
  Initializes the ADC peripheral with the provided configuration.

- `adc_RequestState_t Adc_Deinit ( adc_PeriphConfig_t * const adcConfig );`  
  Deinitializes the ADC peripheral and resets the configuration.

- `void Adc_Task ( void );`  
  Handles ADC-related periodic tasks (if required by the implementation).

- `adc_RequestState_t Adc_Get_DefaultConfig ( adc_PeriphConfig_t * const adcConfig );`  
  Retrieves a default configuration structure for ADC initialization.

---

## Usage Notes

- The ADC module is hardware dependent and must be configured per STM32 family branch.  
- The **default configuration API** helps to ensure safe initialization.  
- The `Task` function should be periodically called if asynchronous handling or background processing is implemented.  
- DMA mode: `adc_DataConfig_t::Dma` selects the DMA stream of the ADC request from the list `adc_Dma_t` - one item per ADC
  peripheral, DMA peripheral and stream, named `ADC_DMA_ADCx_DMAy_STREAMz` (ADC1: DMA2 stream 0 or 4, ADC2: DMA2
  stream 2 or 3, ADC3: DMA2 stream 0 or 1); the channel selection of the stream is part of the item. An item of
  another ADC peripheral and `ADC_DMA_UNUSED` are refused in the DMA mode.  
- Channel pins: the table of the channel pins is generated from the STM32CubeMX database per device line (signals ADCx_INy,
  union of the packages of the devices of the line) - a channel has a pin only on the device lines that have it (e.g. the
  channels 10 - 15 on PC0 - PC5 are not on STM32F410Cx / STM32F410Tx / STM32F412Cx); a channel without a pin is refused for the pin input.  

---

# 🛠 CMake Integration

```cmake
target_link_libraries(User_Lib PRIVATE Adc_Lib)
```

---

## License

This project is licensed under the **Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** — [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.

---

## 🌐 Useful Links

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Azure DevOps](https://azure.microsoft.com/en-us/services/devops/)
- [Embedbits Github](https://github.com/Embedbits)
- [CC BY-NC 4.0 License](https://creativecommons.org/licenses/by-nc/4.0/)
