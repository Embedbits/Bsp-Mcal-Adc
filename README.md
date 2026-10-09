# ADC MCAL Module

The **ADC (Analog-to-Digital Converter) MCAL module** provides an abstraction layer for managing analog-to-digital conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of ADC peripherals: deep power down exit, internal regulator, self-calibration (single-ended and differential), enable
- Common clock configuration: synchronous AHB clock (HCLK / 1, 2, 4) or asynchronous kernel clock (SYSCLK / PLLSAI1 R / PLLSAI2 R) with prescaler, ADC clock frequency checked against the device limits
- Regular sequence (1 - 16 ranks) and injected sequence (1 - 4 ranks), software and hardware triggers with edge selection, single / continuous mode, auto-injected mode
- Channel inputs: single-ended and differential pins (pins configured as analog by the module), temperature sensor, internal reference voltage, VBAT / 3, DAC1 outputs
- Sampling time per channel with minimal sampling time check of internal signals
- Resolution 12 / 10 / 8 / 6 bit
- Data transfer of regular results by DMA, interrupt or polling (`Adc_Task()`) - one shot / circular buffer, half transfer, transfer complete, injected complete and error callbacks
- Analog watchdogs 1 - 3 (thresholds in the configured resolution)
- Every register write verified by read-back, HW state checked before configuration changes

---

## STM32L4 / STM32L4+ specifics

Public interface of the STM32H5 module (`Dev/STM32H5`), `Adc_Port.h` identical. Differences:

| Feature                   | STM32L4 / STM32L4+ behavior                                                                                  |
|---------------------------|--------------------------------------------------------------------------------------------------------------|
| ADC peripherals           | ADC1 (all lines), ADC2 (L41x / L42x, L4P5 / L4Q5), ADC2 + ADC3 (L47x / L48x / L49x / L4Ax); one common register block (ADC1_COMMON / ADC12_COMMON / ADC123_COMMON) |
| Clock source              | `ADC_CLK_SRC_HCLK` (synchronous), `ADC_CLK_SRC_SYSCLK`, `ADC_CLK_SRC_PLLSAI1R` (not L41x / L42x), `ADC_CLK_SRC_PLLSAI2R` (L47x / L48x / L49x / L4Ax only) - one ADC clock enable and ADCSEL multiplexer; `adc_ClkSrc_t` contains the sources of the device line only (guards `RCC_CR_PLLSAI1ON`, `ADC_TYPES_PLLSAI2R_SUPPORT`), a different source releases the active one in RCC first (STM32H5 bug AB#1043) |
| Clock limits              | ADC clock 0.14 - 80 MHz; HCLK source supports dividers 1 / 2 / 4 only; asynchronous dividers 6 / 10 / 12 are not part of the interface |
| Triggers                  | STM32L4 trigger set common for all ADC peripherals: regular TIM1 TRGO / TRGO2 / CH1 / CH2 / CH3, TIM2 TRGO / CH2, TIM3 TRGO / CH4, TIM4 TRGO / CH4, TIM6 TRGO, TIM8 TRGO / TRGO2, TIM15 TRGO, EXTI line 11; injected TIM1 TRGO / TRGO2 / CH4, TIM2 TRGO / CH1, TIM3 TRGO / CH1 / CH3 / CH4, TIM4 TRGO, TIM6 TRGO, TIM8 TRGO / TRGO2 / CH4, TIM15 TRGO, EXTI line 15. `adc_RegTriggerId_t` / `adc_InjTriggerId_t` contain the triggers of the device line only - the triggers of a timer missing on the device (TIM3 not on L41x / L43x, TIM4 / TIM8 only on L47x / L49x / L4+) are not in the lists (guards by the CMSIS instance macros) |
| Channels                  | `ADC_CHANNEL_0` - `ADC_CHANNEL_18` (channel numbers); channel 0 has no pin                                  |
| Channel selection         | list `adc_Channel_t` - one item per valid combination (ADC peripheral, channel number, input / pin) of the device line: `ADC_CH_ADC<n>_IN<channel>_P<port><pin>` (single-ended), `ADC_CH_ADC<n>_IN<channel>_DIFF_P<port><pin>_P<port><pin>` (differential), `ADC_CH_ADC<n>_TEMP` / `VREF` / `VBAT` / `DAC1` / `DAC2` (internal signals); the item shall belong to the peripheral of the configuration |
| Differential input        | channel i (positive) and pin of channel i + 1 (negative), channels 1 - 15, both pins must exist             |
| Internal inputs           | ADC1: VREF 0, TEMP 17, VBAT/3 18; ADC2: DAC1 OUT1 17, DAC1 OUT2 18; ADC3: TEMP 17, VBAT/3 18, DAC1 OUT1 14, DAC1 OUT2 15. Single ADC devices L43x / L44x / L45x / L46x: DAC1 OUT1 / OUT2 on ADC1 channel 17 / 18 shared with TEMP / VBAT - selecting the DAC output disables the temperature sensor / VBAT path (CCR TSEN / VBATEN). No DAC on L41x / L42x, no DAC1 OUT2 on L45x / L46x. L4R / L4S: DAC1 outputs not connected to the ADC (measured on STM32L4R5, STM32CubeMX database - the LL driver declares ADC1 channel 17 / 18), the DAC inputs are refused. The DAC channel itself (output connected to on-chip peripherals) is configured by the Dac module |
| Analog watchdog filter    | Not available - only `ADC_AWD_FILTER_NONE` is accepted                                                       |
| DMA                       | `Dma` selects the DMA channel from the list `adc_Dma_t` - one item per ADC peripheral, DMA peripheral and channel, named `ADC_DMA_ADCx_DMAy_CHANNELz` (e.g. `ADC_DMA_ADC1_DMA1_CHANNEL1`). STM32L4: fixed request mapping (DMA_CSELR, the request selection is part of the item) - ADC1 DMA1 channel 1 / DMA2 channel 3, ADC2 DMA1 channel 2 / DMA2 channel 4, ADC3 DMA1 channel 3 / DMA2 channel 5; the list contains these channels only. STM32L4+: any DMA1 / DMA2 channel through DMAMUX1, the list contains all of them. Items of another ADC peripheral and `ADC_DMA_UNUSED` are refused in the DMA mode |
| Interrupt lines           | ADC1 / ADC2 share `ADC1_2` (`ADC1` on single ADC devices, one line handler, line disabled when the last user is released), ADC3 own line |
| Errors                    | ADC overrun and DMA transfer error, GPDMA specific errors never reported                                     |

### Device errata

The workarounds of the STM32G4 module (same ADC IP) are kept - STM32L4 / STM32L4+ errata sheets are not reviewed yet:

| STM32G4 erratum | Handling |
|-----------------|----------|
| ES0430 2.7.1 / 2.7.2 - injected context queue (JQDIS = 0, JQM = 0) | injected context queue disabled (JQDIS = 1) by `Adc_PeriphInit()` |
| ES0430 2.7.5 - injected data stored in a wrong JDRx on JADSTP | `Adc_Set_InjStop()` waits for the end of the injected sequence (JEOS, limited by timeout) when AHB clock > 10 x ADC clock |
| ES0430 2.7.10 - channel 0 converted after a software stop / one shot DMA transfer | the next `Adc_Set_RegStart()` / `Adc_Set_InjStart()` disables and enables the ADC (hardware dummy conversion) if the other group is not converting |
| Cortex-M4 r0p1 erratum 838869 | every ADC interrupt handler ends with DSB |

---

## Supported Hardware

| MCU line                                     | Peripherals         | Interrupt lines | DAC1 outputs measurable on    |
|----------------------------------------------|---------------------|-----------------|-------------------------------|
| STM32L412 / L422                             | ADC1, ADC2          | ADC1_2          | - (no DAC)                    |
| STM32L431 / L432 / L433 / L442 / L443        | ADC1                | ADC1            | ADC1 17 / 18 (shared)         |
| STM32L451 / L452 / L462                      | ADC1                | ADC1            | ADC1 17 (shared, OUT1 only)   |
| STM32L471 / L475 / L476 / L485 / L486        | ADC1, ADC2, ADC3    | ADC1_2, ADC3    | ADC2 17 / 18, ADC3 14 / 15    |
| STM32L496 / L4A6                             | ADC1, ADC2, ADC3    | ADC1_2, ADC3    | ADC2 17 / 18, ADC3 14 / 15    |
| STM32L4P5 / L4Q5                             | ADC1, ADC2          | ADC1_2          | ADC2 17 / 18                  |
| STM32L4R5 / L4R7 / L4R9 / L4S5 / L4S7 / L4S9 | ADC1                | ADC1            | - (not connected to the ADC)  |

Channel to pin mapping is generated from the STM32CubeMX database per device line (signals ADCx_INy, union of the packages of the devices of the line, generator `gen/cubemx_pins/adc_pins_lines.py` of the STM32L4 port tools) - a channel has a pin only on the device lines that have it, a channel without a pin is refused for the single-ended and differential pin input. E.g. ADC1 / ADC2: IN1 - IN4 PC0 - PC3 (not on STM32L432 / L442), IN5 - IN12 PA0 - PA7, IN13 / IN14 PC4 / PC5 (not on STM32L432 / L442), IN15 / IN16 PB0 / PB1; ADC3 (L47x / L49x): IN1 - IN4 PC0 - PC3, IN6 - IN13 PF3 - PF10 (not on STM32L475 / L485).

---

## Module Structure

| File          | Description                                                                 |
|---------------|-----------------------------------------------------------------------------|
| `Adc_Port.h`  | Public API                                                                  |
| `Adc_Types.h` | Public types                                                                |
| `Adc.c`       | Clock, peripheral, channel, sequence, trigger and watchdog configuration, data transfer core |
| `Adc_Dma.c`   | DMA data transfer (DMA channel with the request of the ADC)                |
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
- `Adc_ChannelInit`, `Adc_Set_` / `Adc_Get_` `Resolution`, `SamplingTime`, `ChannelInput` (`Adc_Set_ChannelInput( periphId, channel )`
  takes an item of `adc_Channel_t`, `Adc_Get_ChannelInput( periphId, channelId, &channel )` returns the item selected for the channel number)

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
- `Adc_Deinit()` keeps the RCC clock of the ADC enabled (the clock is common for all ADC peripherals).

### Example

```c
adc_Config_t config = { 0 };

config.ClockSource  = ADC_CLK_SRC_HCLK;
config.ClockDivider = ADC_CLK_DIV_4;
config.PeriphConfig[ ADC_PERIPH_1 ].PeriphId       = ADC_PERIPH_1;
config.PeriphConfig[ ADC_PERIPH_1 ].RegTriggerId   = ADC_REG_TRIGGER_SOFTWARE;
config.PeriphConfig[ ADC_PERIPH_1 ].RegChannelsCnt = 1u;
config.PeriphConfig[ ADC_PERIPH_1 ].RegChannels[ 0u ] = (adc_ChannelConfig_t){ ADC_CH_ADC1_VREF, ADC_CHANNEL_SAMPLING_640_5_CYCLES };
config.PeriphConfig[ ADC_PERIPH_1 ].DataConfig.TransferMode = ADC_TRANSFER_MODE_POLL;

if( ADC_REQUEST_OK == Adc_Init( &config ) )
{
    (void)Adc_Set_RegStart( ADC_PERIPH_1 );
}
```

---

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_Adc.c` - registers of all ADC peripherals are emulated, calibration / disable / stop by a HW model thread. Device specific parts (ADC2 / ADC3, DAC, timers, PLLSAI sources) are guarded - run on several presets:
  ```bash
  cmake --preset STM32L476xG_UnitTest && cmake --build --preset STM32L476xG_UnitTest
  ctest --test-dir Build/STM32L476xG_UnitTest -L Adc
  ```
- Integration tests (target, boards named by the MCU - the same tests on all 13 STM32L4 / STM32L4+ Nucleo boards): `Tests/IntegrationTests/ItTest_Adc.c` - internal signals only (VREFINT with factory calibration, temperature sensor with TS_CAL1 / TS_CAL2, VBAT / 3 supplied from VDD), ADC3 on MCUs with ADC3, DAC1 outputs (Dac module, internal connection) measured by the ADC channels connected to them (STM32L4R / L4S: refusal of the DAC inputs checked, measurement ignored); no wiring and no pin is used.

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
