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
- Integration tests (target, NUCLEO-G474RE): `Tests/IntegrationTests/ItTest_Adc.c` - internal channels (VREFINT, temperature sensor with factory calibration) of ADC1 / ADC5, DAC1_OUT1 (PA4) measured by ADC2 channel 17, no external wiring.

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
