# ADC MCAL Module

The **ADC (Analog-to-Digital Converter) MCAL module** provides an abstraction layer for managing analog-to-digital conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of ADC peripherals: deep power down exit, internal regulator, BOOST from the conversion clock, self-calibration (offset + linearity of single-ended inputs, offset of differential inputs), enable
- Common clock configuration: synchronous AHB clock (HCLK / 1, 2, 4) or asynchronous kernel clock (PLL2P / PLL3R / PER_CK) with prescaler, conversion clock frequency checked against the device limits
- Regular sequence (1 - 16 ranks) and injected sequence (1 - 4 ranks), software and hardware triggers with edge selection, single / continuous mode, auto-injected mode
- Channel inputs: single-ended and differential pins (pins configured as analog by the module, channels preselected in PCSEL), temperature sensor, internal reference voltage, VBAT / 4, DAC1 outputs
- Sampling time per channel with minimal sampling time check of internal signals
- Resolution 16 / 14 / 12 / 10 / 8 bit
- Data transfer of regular results by DMA, interrupt or polling (`Adc_Task()`) - one shot / circular buffer, half transfer, transfer complete, injected complete and error callbacks
- Analog watchdogs 1 - 3 (thresholds in the configured resolution)
- Every register write verified by read-back, HW state checked before configuration changes

---

## STM32H7 specifics

Public interface of the STM32H5 module (`Dev/STM32H5`). Differences:

| Feature                   | STM32H7 behavior                                                                                             |
|---------------------------|--------------------------------------------------------------------------------------------------------------|
| ADC peripherals           | 16-bit ADC: ADC1 / ADC2 (group ADC12, all lines), ADC3 (group ADC3, STM32H74x / H75x). The 12-bit ADC3 of STM32H72x / H73x (other register map) is not handled |
| Clock source              | `ADC_CLK_SRC_HCLK` (synchronous), `ADC_CLK_SRC_PLL2P`, `ADC_CLK_SRC_PLL3R`, `ADC_CLK_SRC_PER` - applied to all clock groups (common ADCSEL); a different source releases the active one in RCC first (STM32H5 bug AB#1043) |
| Clock limits              | Conversion clock = ADC clock / 2 (ADC clock on STM32H74x / H75x revision Y); conversion clock 0.12 - 50 MHz (36 MHz on revision Y); HCLK source supports dividers 1 / 2 / 4 only |
| BOOST                     | Set from the conversion clock before calibration as ST HAL `ADC_ConfigureBoostMode()` (00 up to 6.25 MHz, 01 up to 12.5 MHz, 10 up to 25 MHz, 11 above; revision Y: BOOST_0 above 20 MHz). CR.BOOST is written directly - `LL_ADC_SetBoostMode()` applies the revision Y check also on STM32H7A3 / H7B0 / H7B3 |
| Revision                  | STM32H74x / H75x revision Y / V detected by DBGMCU IDCODE (the same check as ST LL); 8-bit resolution code 100 (Y) / 111 (V) handled by `LL_ADC_SetResolution()` |
| Resolution                | 16 / 14 / 12 / 10 / 8 bit; `ADC_RESOLUTION_6BIT` (12-bit ADC3 of STM32H72x / H73x only) is refused       |
| Triggers                  | STM32H7 trigger set, the same for all handled ADC peripherals (TIM1/2/3/4/6/8/15, EXTI line 11 / 15, LPTIM1/2/3, HRTIM on devices with HRTIM, TIM23 / TIM24 on STM32H72x / H73x) |
| Channels                  | `ADC_CHANNEL_0` - `ADC_CHANNEL_19`; inputs on the dual pads PA0_C / PA1_C / PC2_C / PC3_C (analog switch) are not handled |
| Differential input        | INPx / INNx pin pair of the channel (signals ADCx_INPy / ADCx_INNy), both pins must exist                 |
| Internal inputs           | STM32H74x / H75x ADC3: TEMP 18, VREF 19, VBAT/4 17; STM32H7A3 / H7B0 / H7B3 ADC2: TEMP 18, VREF 19, VBAT/4 14; ADC2 of all lines: DAC1 output 1 on 16 (`ADC_CHANNEL_INPUT_DAC1`), DAC1 output 2 on 17 (`ADC_CHANNEL_INPUT_DAC2`). ADC1 has no internal input; on STM32H72x / H73x the internal inputs are on the not handled ADC3; VDD_CORE is not available |
| Analog watchdogs          | Thresholds of all watchdogs left aligned to 16 bits (ST HAL); no event filtering - only `ADC_AWD_FILTER_NONE` is accepted |
| DMA                       | DMA1 / DMA2 stream selected by `DmaPeriphId` / `DmaChannelId`, request routed by DMAMUX1 (`DMA_REQ_ADCx`), data management CFGR DMNGT (one shot / circular) |
| Interrupt lines           | ADC1 / ADC2 share `ADC` (ADC1_2, one line handler, line disabled when the last user is released), ADC3 own line; no DSB at the end of the handlers (Cortex-M7) |
| Errors                    | ADC overrun and DMA transfer error, GPDMA specific errors never reported                                     |
| Regulator                 | Start-up by fixed delay `LL_ADC_DELAY_INTERNAL_REGUL_STAB_US` (ST HAL - LDORDY flag is not available on all revisions) |

### Device errata

STM32H7 errata sheets are reviewed by AB#844 - the module does not contain the STM32G4 errata workarounds (injected queue, dummy conversion, injected stop timing). Datasheet limits used by the module (conversion clock minimum, sampling time minimums of internal channels) are confirmed by the same task.

### STM32H7R / STM32H7S

STM32H7R3 / H7R7 / H7S3 / H7S7 (Ral family STM32H7RS, macro `STM32H7RS`) have the **12-bit ADC of STM32H5** (no PCSEL /
BOOST / linearity calibration / CALFACT2, DMAEN / DMACFG data management, resolution 12 / 10 / 8 / 6 bits, thresholds TR1 -
TR3). `Adc.c`, `Adc_Dma.c` and `Adc_Types.h` compile the STM32H5 implementation under `STM32H7RS` (whole file family switch)
with the STM32H7R / H7S device data:

| Feature            | STM32H7R / H7S                                                                                          |
|--------------------|---------------------------------------------------------------------------------------------------------|
| Peripherals        | ADC1, ADC2 (ADC12_COMMON), interrupt line ADC1_2 shared (`Adc_Isr.c` of the STM32H7 module)              |
| Clock              | `ADC_CLK_SRC_HCLK` (synchronous HCLK / 1, 2, 4) or ADCSEL kernel clock PLL2P / PLL3R / PER_CK divided by PRESC (`RCC_PERIPH_ADC12_x`) |
| Triggers           | EXTSEL / JEXTSEL lists of STM32H7R / H7S (TIM1 / 2 / 3 / 4 / 6 / 9 / 12 / 15, LPTIM1 - 3, EXTI 11 / 15) |
| Internal channels  | ADC1: TEMP (16), VREF (17); ADC2: VBAT (16), VDDCORE (17, switch ADC2_OR.OP0); no channel 0 GPIO switch  |
| Pins               | ADC1 / ADC2 INPx / INNx from ST open pin data (equal on all four lines), e.g. INP0 PA0, INP1 PA1, INP2 PF11 / PF13, INP18 PA4 / PA5 |
| DMA                | GPDMA1 / HPDMA1 channel (16 channels), GPDMA1 requests `GPDMA_REQ_ADC1 / _ADC2`                           |
| Datasheet limits   | conversion clock 1.5 - 75 MHz and sampling minimums of TEMP / VREF / VBAT of STM32H5 - to be confirmed with the STM32H7R / H7S datasheet (AB#844) |

---

## Supported Hardware

| MCU line                                       | Peripherals                         | Interrupt lines |
|------------------------------------------------|-------------------------------------|-----------------|
| STM32H742 / H743 / H753 / H745 / H755 / H747 / H757 / H750 | ADC1, ADC2, ADC3                    | ADC, ADC3       |
| STM32H723 / H725 / H730 / H733 / H735          | ADC1, ADC2 (12-bit ADC3 not handled) | ADC             |
| STM32H7A3 / H7B0 / H7B3                        | ADC1, ADC2                          | ADC             |
| STM32H7R3 / H7R7 / H7S3 / H7S7                 | ADC1, ADC2 (12-bit ADC of STM32H5)  | ADC1_2          |

Channel to pin mapping is generated from ST open pin data per device line (signals ADCx_INPy / ADCx_INNy), e.g. ADC1 / ADC2: INP3 PA6 (INN3 PA7), INP4 PC4, INP5 PB1, INP7 PA7, INP8 PC5, INP9 PB0, INP10 PC0, INP11 PC1, INP14 PA2, INP15 PA3, INP16 PA0, INP18 PA4, INP19 PA5; ADC1 INP2 PF11, ADC2 INP2 PF13; ADC3 (STM32H74x / H75x): INP2 PF9, INP3 PF7, INP4 PF5, INP5 PF3.

Integration tests are prepared for the STM32H7 Nucleo boards (run together with the hardware, AB#889).

---

## Module Structure

| File          | Description                                                                 |
|---------------|-----------------------------------------------------------------------------|
| `Adc_Port.h`  | Public API                                                                  |
| `Adc_Types.h` | Public types                                                                |
| `Adc.c`       | Clock, peripheral, channel, sequence, trigger and watchdog configuration, data transfer core |
| `Adc_Dma.c`   | DMA data transfer (DMA stream with DMAMUX1 request of the ADC, DMNGT)       |
| `Adc_Isr.c`   | ADC interrupt (EOC / OVR / JEOS, ADC line shared by ADC1 and ADC2)          |
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

`adc_PeriphConfig_t.RegTriggerMode` and `Adc_Set_TriggerMode()` / `Adc_Get_TriggerMode()` select the conversion mode of the regular group (ADC_CFGR bits CONT, DISCEN, DISCNUM[2:0]; STM32H7: RM0433 Rev 9 25.4.21 and 25.6.4, the same in RM0399 Rev 5 chapter 26, RM0455 Rev 13 chapter 27 and RM0468 Rev 3 chapter 28; STM32H7R / H7S: RM0477 Rev 10 27.4.20 and 27.7.4):

| `adc_RegTriggerMode_t` | ADC_CFGR | Behavior |
|---|---|---|
| `ADC_REG_TRIGGER_MODE_SINGLE` | CONT = 0, DISCEN = 0 | every trigger converts the whole sequence |
| `ADC_REG_TRIGGER_MODE_CONTINUOUS` | CONT = 1, DISCEN = 0 | after the first trigger the sequence is converted repeatedly |
| `ADC_REG_TRIGGER_MODE_DISCONT_1RANK` - `ADC_REG_TRIGGER_MODE_DISCONT_8RANKS` | CONT = 0, DISCEN = 1, DISCNUM = n - 1 | every trigger converts the next n ranks (subgroup) of the sequence |

In discontinuous mode the last subgroup of the sequence can have less than n ranks (no rollover), the end of sequence flag (`ADC_FLAG_REG_EOS`) is set after the last rank and the next trigger starts with rank 1 again. The data transfer continues at the current buffer position.

- Software trigger: on STM32H7R / H7S every `Adc_Set_RegStart()` converts one subgroup (ADSTART is cleared by the ADC at the end of the subgroup, RM0477 Rev 10 27.4.15). On STM32H7 (16-bit ADC) the reference manual states that ADSTART is cleared at the end of conversion (EOC = 1) in discontinuous mode with software trigger (RM0433 Rev 9 25.4.16); the next subgroup is started by `Adc_Set_RegStart()`, the integration test uses 1 rank per trigger.
- CONT and DISCEN are never set at the same time (forbidden by the reference manual); the bit which is cleared is written first.
- Discontinuous mode can not be combined with auto-injected mode (`ADC_INJ_TRIGGER_AUTO`, JAUTO): `Adc_PeriphInit()` rejects the configuration and `Adc_Set_TriggerMode()` rejects a discontinuous mode while JAUTO is set.
- `Adc_PeriphInit()` clears the auto-injected mode of a previous configuration before the regular group is configured.

---

## Analog watch-dog channel selection

`Adc_AwdInit()` selects the monitored channels by `adc_AwdConfig_t.AwdMode` (ADC_CFGR AWD1CH / AWD1SGL / AWD1EN / JAWD1EN, ADC_AWD2CR / ADC_AWD3CR; STM32H7: RM0433 Rev 9 25.4.30, 25.6.19 and 25.6.20, the same in RM0399 Rev 5 chapter 26, RM0455 Rev 13 chapter 27 and RM0468 Rev 3 chapter 28; STM32H7R / H7S: RM0477 Rev 10 27.4.29, 27.7.19 and 27.7.20):

| Watch-dog | `ADC_AWD_MODE_ALL` | `_ALL_REGULAR` / `_ALL_INJECTED` | `ADC_AWD_MODE_SINGLE` | `_SINGLE_REGULAR` / `_SINGLE_INJECTED` |
|---|---|---|---|---|
| `ADC_AWD_1` | all channels | all channels of the group | channel `AwdChannelId` (regular or injected) | channel `AwdChannelId` of the group |
| `ADC_AWD_2`, `ADC_AWD_3` | all channels | rejected | channels of `AwdChannelMask` (regular or injected) | rejected |

- `adc_AwdChannelMask_t` is a channel set (bit n = channel n) built from the constants `ADC_AWD_CHANNEL_MASK_n` combined by bitwise OR (`ADC_AWD_CHANNEL_MASK_ALL`, `ADC_AWD_CHANNEL_MASK_NONE`): channels 0 - 19 on STM32H7 (AWDxCH[19:0]), channels 0 - 18 on STM32H7R / H7S (AWDxCH[18:0], AWD1CH values above 18 are reserved - channel 19 of `adc_ChannelId_t` has no input there). It is a type definition with symbolic constants, not an enumeration (MISRA - an OR of enumeration constants is not a value of the enumeration).
- The monitored channels must be converted in the monitored group of the last `Adc_PeriphInit()` (the reference manuals require the channels selected by AWD1CH / AWDxCH also in the sequence registers); an empty channel set (it disables AWD2 / AWD3) is rejected. After `Adc_Deinit()` the single channel modes are rejected until the next `Adc_PeriphInit()`.
- `AwdChannelId` and `AwdChannelMask` are used only by the modes above, the other field is ignored.
- Device errata "ADC_AWDy_OUT reset by non-guarded channels" (ES0392 Rev 15, ES0445 Rev 6, ES0478 Rev 12, ES0491 Rev 9, ES0596 Rev 10) concern only the ADC_AWDy_OUT signal routed to the timers, not the watch-dog flags.

---

## Usage Notes

- `Adc_Init()` configures the common clock first - all ADC peripherals must be disabled. Peripherals without regular and injected channels are not touched.
- Channel inputs, calibration and clock can be changed only with the ADC disabled - call `Adc_Deinit()` before reconfiguration.
- Internal inputs require a minimal sampling time (temperature sensor 9 us, VREFINT 4.3 us, VBAT 9.8 us) - the module refuses shorter sampling at the active conversion clock.
- DMA and ISR modes need a buffer for regular results, polling mode without buffer allows manual reading by `Adc_Get_RegData()`.
- The DMA buffer must be placed in memory accessible by DMA1 / DMA2 (AXI SRAM / SRAM1 - SRAM3, not DTCM) and kept coherent with the data cache by the application.
- `Adc_Deinit()` keeps the RCC clock of the ADC group enabled (the group may be used by the other ADC of the group).

### Example

```c
adc_Config_t config = { 0 };

config.ClockSource  = ADC_CLK_SRC_HCLK;
config.ClockDivider = ADC_CLK_DIV_4;
config.PeriphConfig[ ADC_PERIPH_1 ].PeriphId       = ADC_PERIPH_1;
config.PeriphConfig[ ADC_PERIPH_1 ].Resolution     = ADC_RESOLUTION_16BIT;
config.PeriphConfig[ ADC_PERIPH_1 ].RegTriggerId   = ADC_REG_TRIGGER_SOFTWARE;
config.PeriphConfig[ ADC_PERIPH_1 ].RegChannelsCnt = 1u;
config.PeriphConfig[ ADC_PERIPH_1 ].RegChannels[ 0u ] = (adc_ChannelConfig_t){ ADC_CHANNEL_3, ADC_CHANNEL_INPUT_PIN_SINGLE, ADC_CHANNEL_SAMPLING_64_5_CYCLES };
config.PeriphConfig[ ADC_PERIPH_1 ].DataConfig.TransferMode = ADC_TRANSFER_MODE_POLL;

if( ADC_REQUEST_OK == Adc_Init( &config ) )
{
    (void)Adc_Set_RegStart( ADC_PERIPH_1 );
}
```

---

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_Adc.c` - registers of all handled ADC peripherals are emulated, calibration / disable / stop by a HW model thread (calibration modes recorded), revision Y by preset DBGMCU IDCODE.
  ```bash
  cmake --preset STM32H743xI_UnitTest -B Build/UT_STM32H743xI && cmake --build Build/UT_STM32H743xI
  ctest --test-dir Build/UT_STM32H743xI -L Adc
  ```
- STM32H7R / H7S: `Tests/UnitTests/Test_Adc_H7RS.c` (selected by `MCU_FAMILY_ID` STM32H7RSxx) - tests of the STM32H5 module
  with the STM32H7R / H7S device data (RCC ADC12 sources, ADC1_2 line, no channel 0 switch), GPDMA mock.
- Integration tests (target, STM32H7 Nucleo boards, NUCLEO-H7S3L8): AB#889.
  The divider tests use two resistor dividers of the common board wiring (fixture feature `ADC_DIVIDERS`): PA0 (10k to +3V3 / 4.7k to GND) and PC0 (4.7k / 10k) on ADC1 (NUCLEO-H7A3ZI-Q, NUCLEO-H723ZG, NUCLEO-H755ZI-Q channels 16 and 10; NUCLEO-H7S3L8 channels 0 and 10); the ratio of the result to the full scale is checked independent of VDDA. On a board with a fixture without the feature the divider tests are ignored; a board without a fixture is expected to have the dividers.

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
