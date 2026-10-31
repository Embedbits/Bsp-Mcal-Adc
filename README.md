# ADC MCAL Module

The **ADC (Analog-to-Digital Converter) MCAL module** provides an abstraction layer for managing analog-to-digital conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of ADC peripheral
- Task handler for periodic servicing
- Support for default configuration retrieval
- Channel inputs: pins, internal TEMP / VREF / VBAT / VDD_CORE, DAC1 / DAC2 and the output of OPAMP1 (`ADC_CHANNEL_INPUT_OPAMP1`: ADC3 channel 10 on STM32H5E4 / H5E5 / H5F4 / H5F5, the pad PA7 as ADC1 channel 7 on STM32H503; no ADC option bit, the lines without the amplifier have no such input)
- Standardized request state return values
- Versioning support for module management

---

## STM32H5 Specifics

### Regular group conversion modes

`adc_PeriphConfig_t.RegTriggerMode` and `Adc_Set_TriggerMode()` / `Adc_Get_TriggerMode()` select the conversion mode of the regular group (RM0481 Rev 5, RM0492 Rev 3, RM0517 Rev 1: ADC_CFGR bits CONT, DISCEN, DISCNUM[2:0]):

| `adc_RegTriggerMode_t` | ADC_CFGR | Behavior |
|---|---|---|
| `ADC_REG_TRIGGER_MODE_SINGLE` | CONT = 0, DISCEN = 0 | every trigger converts the whole sequence |
| `ADC_REG_TRIGGER_MODE_CONTINUOUS` | CONT = 1, DISCEN = 0 | after the first trigger the sequence is converted repeatedly |
| `ADC_REG_TRIGGER_MODE_DISCONT_1RANK` - `ADC_REG_TRIGGER_MODE_DISCONT_8RANKS` | CONT = 0, DISCEN = 1, DISCNUM = n - 1 | every trigger converts the next n ranks (subgroup) of the sequence |

In discontinuous mode the last subgroup of the sequence can have less than n ranks (no rollover), the end of sequence flag (`ADC_FLAG_REG_EOS`) is set after the last rank and the next trigger starts with rank 1 again. With software trigger every `Adc_Set_RegStart()` converts one subgroup (ADSTART is cleared by the ADC at the end of the subgroup), the data transfer continues at the current buffer position.

- CONT and DISCEN are never set at the same time (forbidden by the reference manual); the bit which is cleared is written first.
- Discontinuous mode can not be combined with auto-injected mode (`ADC_INJ_TRIGGER_AUTO`, JAUTO): `Adc_PeriphInit()` rejects the configuration and `Adc_Set_TriggerMode()` rejects a discontinuous mode while JAUTO is set.
- `Adc_PeriphInit()` clears the auto-injected mode of a previous configuration before the regular group is configured.

### Analog watch-dog channel selection

`Adc_AwdInit()` selects the monitored channels by `adc_AwdConfig_t.AwdMode` (RM0481 Rev 5 28.4.28, ADC_CFGR AWD1CH / AWD1SGL / AWD1EN / JAWD1EN, ADC_AWD2CR / ADC_AWD3CR):

| Watch-dog | `ADC_AWD_MODE_ALL` | `_ALL_REGULAR` / `_ALL_INJECTED` | `ADC_AWD_MODE_SINGLE` | `_SINGLE_REGULAR` / `_SINGLE_INJECTED` |
|---|---|---|---|---|
| `ADC_AWD_1` | all channels | all channels of the group | channel `AwdChannelId` (regular or injected) | channel `AwdChannelId` of the group |
| `ADC_AWD_2`, `ADC_AWD_3` | all channels | rejected | channels of `AwdChannelMask` (regular or injected) | rejected |

- `adc_AwdChannelMask_t` is a channel set (bit n = channel n) built from `ADC_AWD_CHANNEL_MASK_0` - `ADC_AWD_CHANNEL_MASK_19` combined by bitwise OR (`ADC_AWD_CHANNEL_MASK_ALL`, `ADC_AWD_CHANNEL_MASK_NONE`). It is a type definition with symbolic constants, not an enumeration (MISRA - an OR of enumeration constants is not a value of the enumeration).
- The monitored channels must be converted in the monitored group of the last `Adc_PeriphInit()` (the reference manual requires the channels selected by AWD1CH / AWDxCH also in the SQi / JSQi bits); an empty channel set (it disables AWD2 / AWD3) is rejected. After `Adc_Deinit()` the single channel modes are rejected until the next `Adc_PeriphInit()`.
- `AwdChannelId` and `AwdChannelMask` are used only by the modes above, the other field is ignored.
- Device errata "ADC_AWDy_OUT reset by non-guarded channels" (ES0561 Rev 4 2.3.3, ES0565 Rev 7 2.7.4, ES0621 Rev 2 2.6.4) concern only the ADC_AWDy_OUT signal routed to the timers, not the watch-dog flags.

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
