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

## Regular group conversion modes

`adc_PeriphConfig_t.RegTriggerMode` and `Adc_Set_TriggerMode()` / `Adc_Get_TriggerMode()` select the conversion mode of the regular group (RM0385 Rev 9 and RM0410 Rev 5 15.3.11 and 15.13.2, RM0431 Rev 4 14.3.11 and 14.13.2; ADC_CR1 bits DISCEN, DISCNUM[2:0], ADC_CR2 bit CONT):

| `adc_RegTriggerMode_t` | Registers | Behavior |
|---|---|---|
| `ADC_REG_TRIGGER_MODE_SINGLE` | DISCEN = 0, CONT = 0 | every trigger converts the whole sequence |
| `ADC_REG_TRIGGER_MODE_CONTINUOUS` | DISCEN = 0, CONT = 1 | after the first trigger the sequence is converted repeatedly |
| `ADC_REG_TRIGGER_MODE_DISCONT_1RANK` - `ADC_REG_TRIGGER_MODE_DISCONT_8RANKS` | DISCEN = 1, DISCNUM = n - 1, CONT = 0 | every trigger converts the next n ranks (subgroup) of the sequence |

In discontinuous mode the last subgroup of the sequence can have less than n ranks (no rollover) and the next trigger starts with rank 1 again. With software trigger every `Adc_Set_RegStart()` converts the next subgroup, the data transfer continues at the current buffer position. STM32F7 has no end of sequence flag - the end of the sequence is reported by the data transfer (transfer complete callback).

- DISCEN / DISCNUM are written by `Adc_Set_TriggerMode()`, CONT is written by `Adc_Set_RegStart()` and cleared by `Adc_Set_RegStop()`; CONT and DISCEN are never set at the same time.
- Discontinuous mode can not be combined with auto-injected mode (`ADC_INJ_TRIGGER_AUTO`, JAUTO) and must not be set for the regular and the injected group at the same time (`ADC_INJ_TRIGGER_MODE_SINGLE`, JDISCEN): `Adc_PeriphInit()` rejects the configuration and `Adc_Set_TriggerMode()` rejects a discontinuous mode while JAUTO or JDISCEN is set.
- `Adc_PeriphInit()` clears the auto-injected mode and the discontinuous modes of a previous configuration before the regular group is configured (`Adc_Deinit()` does not reset ADC_CR1).

---

## Analog watch-dog channel selection

`Adc_AwdInit()` selects the channels monitored by analog watch-dog 1 by `adc_AwdConfig_t.AwdMode` (RM0385 Rev 9 15.3.8, RM0410 Rev 5 15.3.8, RM0431 Rev 4 14.3.8; ADC_CR1 bits AWDCH[4:0], AWDSGL, AWDEN, JAWDEN):

| `adc_AwdMode_t` | Monitored channels |
|---|---|
| `ADC_AWD_MODE_ALL` | all channels of both groups |
| `ADC_AWD_MODE_ALL_REGULAR` / `ADC_AWD_MODE_ALL_INJECTED` | all channels of the group |
| `ADC_AWD_MODE_SINGLE` | channel `AwdChannelId` (regular or injected) |
| `ADC_AWD_MODE_SINGLE_REGULAR` / `ADC_AWD_MODE_SINGLE_INJECTED` | channel `AwdChannelId` of the group |

- The monitored channel of the single channel modes must be converted in the monitored group of the last `Adc_PeriphInit()` (the watch-dog compares only converted values). After `Adc_Deinit()` the single channel modes are rejected until the next `Adc_PeriphInit()`.
- `AwdChannelId` (`adc_ChannelId_t`) is used only by the single channel modes. The unused enumeration of channel bit masks `adc_AwdChannelId_t` was removed (analog watch-dog 1 monitors one channel).

---

## Usage Notes

- The ADC module is hardware dependent and must be configured per STM32 family branch.  
- The **default configuration API** helps to ensure safe initialization.  
- The `Task` function should be periodically called if asynchronous handling or background processing is implemented.  
- DMA mode: `adc_DataConfig_t::Dma` selects the DMA stream of the ADC request from the list `adc_Dma_t` - one item per ADC
  peripheral, DMA peripheral and stream, named `ADC_DMA_ADCx_DMAy_STREAMz` (ADC1: DMA2 stream 0 or 4, ADC2: DMA2
  stream 2 or 3, ADC3: DMA2 stream 0 or 1); the channel selection of the stream is part of the item. An item of
  another ADC peripheral and `ADC_DMA_UNUSED` are refused in the DMA mode.  

### STM32F7 device errata (ES0334)

- **Internal noise impacting the ADC accuracy** - not handled by the module. The accuracy options of
  the errata sheet (ADCDC1 in PWR_CR1 or ADCxDC2 in SYSCFG_PMC, only one of them, see AN4073) depend
  on the application (ADC clock, conversion timing of the ADC instances) and are not set.
- **ADC sequencer modification during conversion** - not affected: the regular and injected
  sequencers are written by `Adc_Init()` only, before any conversion is started.

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
