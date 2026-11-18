# ADC MCAL Module

The **ADC (Analog-to-Digital Converter) MCAL module** provides an abstraction layer for managing analog-to-digital conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of ADC peripherals: deep power down exit, internal regulator, self-calibration (single-ended and differential), enable
- Common clock configuration: synchronous AHB clock (HCLK / 1, 2, 4) or asynchronous kernel clock (SYSCLK / PLL P) with prescaler, ADC clock frequency checked against the device limits
- Regular sequence (1 - 16 ranks) and injected sequence (1 - 4 ranks), software and hardware triggers with edge selection, single / continuous mode, auto-injected mode
- Channel inputs: single-ended and differential pins (pins configured as analog by the module), temperature sensor, internal reference voltage, VBAT / 3, OPAMP outputs
- Sampling time per channel with minimal sampling time check of internal signals
- Resolution 12 / 10 / 8 / 6 bit
- Data transfer of regular results by DMA, interrupt or polling (`Adc_Task()`) - one shot / circular buffer, half transfer, transfer complete, injected complete and error callbacks
- Analog watchdogs 1 - 3 (thresholds in the configured resolution, AWD1 filtering)
- Every register write verified by read-back, HW state checked before configuration changes

---

## STM32G4 specifics

Public interface of the STM32H5 module (`Dev/STM32H5`). Differences:

| Feature                   | STM32G4 behavior                                                                                             |
|---------------------------|--------------------------------------------------------------------------------------------------------------|
| ADC peripherals           | ADC1 / ADC2 (group ADC12, all lines), ADC3 (G471 / G473 / G474 / G483 / G484 / G491 / G4A1), ADC4 / ADC5 (G473 / G474 / G483 / G484) |
| Clock source              | `ADC_CLK_SRC_HCLK` (synchronous), `ADC_CLK_SRC_SYSCLK`, `ADC_CLK_SRC_PLLP` - applied to all clock groups (ADC12SEL, ADC345SEL); a different source releases the active one in RCC first (STM32H5 bug AB#1043) |
| Clock limits              | ADC clock 0.14 - 60 MHz; HCLK source supports dividers 1 / 2 / 4 only                                       |
| Triggers                  | STM32G4 trigger set (TIM1/2/3/4/6/7/8/15/16, TIM20 and HRTIM on devices with the timer, EXTI line 2 / 3 / 11 / 15, LPTIM1); connection differs between ADC1 / ADC2 and ADC3 / ADC4 / ADC5 - a trigger not connected to the peripheral is refused |
| Channels                  | `ADC_CHANNEL_0` - `ADC_CHANNEL_18` (no channel 19); channel 0 has no pin                                     |
| Differential input        | channel i (positive) and pin of channel i + 1 (negative), channels 1 - 15, both pins must exist             |
| Internal inputs           | ADC1: TEMP 16, VBAT/3 17, VREF 18, OPAMP1 13; ADC2: OPAMP2 16, OPAMP3 18; ADC3: VBAT/3 17, VREF 18, OPAMP3 13; ADC4: VREF 18, OPAMP6 17; ADC5: TEMP 4, VBAT/3 17, VREF 18, OPAMP4 5, OPAMP5 3. OPAMP outputs are selected only - the operational amplifier (internal output) is configured by the application |
| DMA                       | DMA1 / DMA2 channel selected by `DmaPeriphId` / `DmaChannelId`, request routed by DMAMUX (`DMA_REQ_ADCx`)   |
| Interrupt lines           | ADC1 / ADC2 share `ADC1_2` (one line handler, line disabled when the last user is released), ADC3 / ADC4 / ADC5 own lines |
| Errors                    | ADC overrun and DMA transfer error, GPDMA specific errors never reported                                     |

### Device errata (ES0430 / ES0431 / ES0523)

Handled by the module:

| Erratum | Handling |
|---------|----------|
| ES0430 2.7.1 / 2.7.2, ES0431 2.5.1 / 2.5.2, ES0523 2.6.1 / 2.6.2 - injected context queue (JQDIS = 0, JQM = 0) | injected context queue disabled (JQDIS = 1) by `Adc_PeriphInit()` |
| ES0430 2.7.5, ES0431 2.5.5, ES0523 2.6.5 - injected data stored in a wrong JDRx on JADSTP | `Adc_Set_InjStop()` waits for the end of the injected sequence (JEOS, limited by timeout) when AHB clock > 10 x ADC clock |
| ES0430 2.7.10, ES0431 2.5.10, ES0523 2.6.8 - channel 0 converted after a software stop / one shot DMA transfer | the next `Adc_Set_RegStart()` / `Adc_Set_InjStart()` disables and enables the ADC (hardware dummy conversion) if the other group is not converting |

Application level (no workaround in the module):

- ES0430 2.7.3 / 2.7.6 (dual mode) - dual / interleaved modes are not used by the module.
- ES0430 2.7.4 - ADC_AWDy_OUT reset by non-guarded channels (watchdog outputs to timers are not used by the module).
- ES0430 2.7.7 / 2.7.8 (Rev. Z), 2.7.11 - accuracy of concurrent conversions of several ADC instances (resolution, channel switching, clock configuration) - use the same resolution, clock source and prescaler 1, or sequence the conversions.
- ES0430 2.7.9, ES0431 2.5.9, ES0523 2.6.7 - result of a conversion done more than 1 ms after calibration or the previous conversion may be wrong - convert twice and keep the second result.

---

## Supported Hardware

| MCU line                         | Peripherals               | Interrupt lines                    |
|----------------------------------|---------------------------|------------------------------------|
| STM32G431 / G441 / G414 / G411xB | ADC1, ADC2                | ADC1_2                             |
| STM32G471 / G491 / G4A1 / G411xC | ADC1, ADC2, ADC3          | ADC1_2, ADC3                       |
| STM32G473 / G474 / G483 / G484   | ADC1 - ADC5               | ADC1_2, ADC3, ADC4, ADC5           |

Channel to pin mapping is generated from the STM32CubeMX database (signals ADCx_INy, equal on all lines with the peripheral except the pins PD11 - PD14 of ADC3 channels 8 - 11 that STM32G411xC does not have - a channel without a pin is refused for the pin input), e.g. ADC1: IN1 PA0, IN2 PA1, IN3 PA2, IN4 PA3, IN5 PB14, IN6 - IN9 PC0 - PC3, IN10 PF0, IN11 PB12, IN12 PB1, IN14 PB11, IN15 PB0; ADC2: IN17 PA4 (DAC1_OUT1).

Tested on NUCLEO-G474RE (STM32G474RE).

---

## Module Structure

| File          | Description                                                                 |
|---------------|-----------------------------------------------------------------------------|
| `Adc_Port.h`  | Public API                                                                  |
| `Adc_Types.h` | Public types                                                                |
| `Adc.c`       | Clock, peripheral, channel, sequence, trigger and watchdog configuration, data transfer core |
| `Adc_Dma.c`   | DMA data transfer (DMA channel with DMAMUX request of the ADC)              |
| `Adc_Isr.c`   | ADC interrupt (EOC / OVR / JEOS, ADC1_2 line shared by ADC1 and ADC2)       |
| `Adc_Poll.c`  | Polling data transfer (`Adc_Task()`)                                        |
| `Adc.h`, `Adc_Dma.h`, `Adc_Isr.h`, `Adc_Poll.h` | Private interfaces (not exported by the CMake library) |

---

## Public API

### Module Management
- `adc_ModuleVersion_t Adc_Get_ModuleVersion ( void );`
- `adc_RequestState_t  Adc_Init              ( adc_Config_t * const adcConfig );`
- `adc_RequestState_t  Adc_Deinit            ( adc_PeriphId_t periphId );`
- `void                Adc_Task              ( void );`

### Clock Configuration
- `Adc_Set_ClockSource` / `Adc_Get_ClockSource`, `Adc_Set_ClockDivider` / `Adc_Get_ClockDivider`

### Peripheral, Trigger and Conversion Control
- `Adc_PeriphInit`, `Adc_Set_PeriphActive`, `Adc_Set_PeriphInactive`
- `Adc_Set_` / `Adc_Get_` `TriggerSrc`, `TriggerMode`, `TriggerEdge`
- `Adc_Set_RegStart`, `Adc_Set_RegStop`, `Adc_Set_InjStart`, `Adc_Set_InjStop`

### Channels
- `Adc_ChannelInit`, `Adc_Set_` / `Adc_Get_` `Resolution`, `SamplingTime`, `ChannelInput`

### Data
- `Adc_Set_DataConfig`, `Adc_Get_DataConfig`, `Adc_Get_RegData`, `Adc_Get_InjData`, `Adc_Get_Flag`, `Adc_Clear_Flag`

### Analog Watchdog
- `Adc_AwdInit`, `Adc_Set_` / `Adc_Get_` `AwdThresholds`, `AwdFilter`

---

## Regular group conversion modes

`adc_PeriphConfig_t.RegTriggerMode` and `Adc_Set_TriggerMode()` / `Adc_Get_TriggerMode()` select the conversion mode of the regular group (RM0440 Rev 9 21.4.20 and 21.7.4, ADC_CFGR bits CONT, DISCEN, DISCNUM[2:0]):

| `adc_RegTriggerMode_t` | ADC_CFGR | Behavior |
|---|---|---|
| `ADC_REG_TRIGGER_MODE_SINGLE` | CONT = 0, DISCEN = 0 | every trigger converts the whole sequence |
| `ADC_REG_TRIGGER_MODE_CONTINUOUS` | CONT = 1, DISCEN = 0 | after the first trigger the sequence is converted repeatedly |
| `ADC_REG_TRIGGER_MODE_DISCONT_1RANK` - `ADC_REG_TRIGGER_MODE_DISCONT_8RANKS` | CONT = 0, DISCEN = 1, DISCNUM = n - 1 | every trigger converts the next n ranks (subgroup) of the sequence |

In discontinuous mode the last subgroup of the sequence can have less than n ranks (no rollover), the end of sequence flag (`ADC_FLAG_REG_EOS`) is set after the last rank and the next trigger starts with rank 1 again. With software trigger every `Adc_Set_RegStart()` converts one subgroup (ADSTART is cleared by the ADC at the end of the subgroup), the data transfer continues at the current buffer position.

- CONT and DISCEN are never set at the same time (forbidden by the reference manual); the bit which is cleared is written first.
- Discontinuous mode can not be combined with auto-injected mode (`ADC_INJ_TRIGGER_AUTO`, JAUTO): `Adc_PeriphInit()` rejects the configuration and `Adc_Set_TriggerMode()` rejects a discontinuous mode while JAUTO is set.
- `Adc_PeriphInit()` clears the auto-injected mode of a previous configuration before the regular group is configured.

---

## Analog watch-dog channel selection

`Adc_AwdInit()` selects the monitored channels by `adc_AwdConfig_t.AwdMode` (RM0440 Rev 9 21.4.28, ADC_CFGR AWD1CH / AWD1SGL / AWD1EN / JAWD1EN, ADC_AWD2CR / ADC_AWD3CR):

| Watch-dog | `ADC_AWD_MODE_ALL` | `_ALL_REGULAR` / `_ALL_INJECTED` | `ADC_AWD_MODE_SINGLE` | `_SINGLE_REGULAR` / `_SINGLE_INJECTED` |
|---|---|---|---|---|
| `ADC_AWD_1` | all channels | all channels of the group | channel `AwdChannelId` (regular or injected) | channel `AwdChannelId` of the group |
| `ADC_AWD_2`, `ADC_AWD_3` | all channels | rejected | channels of `AwdChannelMask` (regular or injected) | rejected |

- `adc_AwdChannelMask_t` is a channel set (bit n = channel n) built from `ADC_AWD_CHANNEL_MASK_0` - `ADC_AWD_CHANNEL_MASK_18` combined by bitwise OR (`ADC_AWD_CHANNEL_MASK_ALL`, `ADC_AWD_CHANNEL_MASK_NONE`). It is a type definition with symbolic constants, not an enumeration (MISRA - an OR of enumeration constants is not a value of the enumeration).
- The monitored channels must be converted in the monitored group of the last `Adc_PeriphInit()` (the reference manual requires the channels selected by AWD1CH / AWDxCH also in the sequence registers); an empty channel set (it disables AWD2 / AWD3) is rejected. After `Adc_Deinit()` the single channel modes are rejected until the next `Adc_PeriphInit()`.
- `AwdChannelId` and `AwdChannelMask` are used only by the modes above, the other field is ignored.
- Device errata "ADC_AWDy_OUT reset by non-guarded channels" (ES0430 Rev 9 2.7.4, ES0431 Rev 9 2.5.4, ES0523 Rev 6 2.6.4) concern only the ADC_AWDy_OUT signal routed to the timers, not the watch-dog flags.

---

## Usage Notes

- `Adc_Init()` configures the common clock first - all ADC peripherals must be disabled. Peripherals without regular and injected channels are not touched.
- Channel inputs, calibration and clock can be changed only with the ADC disabled - call `Adc_Deinit()` before reconfiguration.
- Internal inputs require a minimal sampling time (temperature sensor 5 us, VREFINT 4 us, VBAT 12 us) - the module refuses shorter sampling at the active ADC clock.
- DMA and ISR modes need a buffer for regular results, polling mode without buffer allows manual reading by `Adc_Get_RegData()`.
- `Adc_Deinit()` keeps the RCC clock of the ADC group enabled (the group may be used by the other ADC of the group).

### Example

```c
adc_Config_t config = { 0 };

config.ClockSource  = ADC_CLK_SRC_HCLK;
config.ClockDivider = ADC_CLK_DIV_4;
config.PeriphConfig[ ADC_PERIPH_1 ].PeriphId       = ADC_PERIPH_1;
config.PeriphConfig[ ADC_PERIPH_1 ].RegTriggerId   = ADC_REG_TRIGGER_SOFTWARE;
config.PeriphConfig[ ADC_PERIPH_1 ].RegChannelsCnt = 1u;
config.PeriphConfig[ ADC_PERIPH_1 ].RegChannels[ 0u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_18, ADC_CHANNEL_INPUT_VREF, ADC_CHANNEL_SAMPLING_640_5_CYCLES };
config.PeriphConfig[ ADC_PERIPH_1 ].DataConfig.TransferMode = ADC_TRANSFER_MODE_POLL;

if( ADC_REQUEST_OK == Adc_Init( &config ) )
{
    (void)Adc_Set_RegStart( ADC_PERIPH_1 );
}
```

---

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_Adc.c` - registers of all ADC peripherals are emulated, calibration / disable / stop by a HW model thread.
  ```bash
  cmake --preset STM32G474xE_UnitTest && cmake --build --preset STM32G474xE_UnitTest
  ctest --test-dir Build_STM32G474xE_UnitTest -L Adc
  ```
- Integration tests (target, NUCLEO-G474RE / G431RB / G491RE): `Tests/IntegrationTests/ItTest_Adc.c` - internal channels (VREFINT, temperature sensor with factory calibration) of ADC1 / ADC5, DAC1_OUT1 (PA4) measured by ADC2 channel 17, no external wiring. The divider tests use the resistor dividers of the common board wiring on external channels of ADC1: PA0 (channel 1, 10k to +3V3 / 4.7k to GND) and PC0 (channel 6, solder bridge SB36, 4.7k to +3V3 / 10k to GND); the ratio of the result to the full scale (polling) and the voltages with VDDA of the VREFINT conversion (DMA) are checked. The dividers are the fixture feature `ADC_DIVIDERS` of the board: on a board with a fixture without it the 2 divider tests are ignored (`ItAdc_RequireDividers()`), a board without a fixture is expected to have them.

---

## 🛠 CMake Integration

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
