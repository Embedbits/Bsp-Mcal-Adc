# ADC MCAL Module

The **ADC (Analog-to-Digital Converter) MCAL module** provides an abstraction layer for managing analog-to-digital conversion peripherals on STM32 microcontrollers.  
This module is part of the MCAL layer and ensures a unified API across different STM32 families.  
Different families are maintained in separate branches; users can switch to the appropriate branch for their MCU family.

---

## Features

- Initialization and deinitialization of the module (common clock + all used peripherals) and of a single peripheral
- Kernel clock source selection through the RCC module and common prescaler with frequency range check (0.14 - 55 MHz)
- Voltage regulator start-up (LDORDY), offset and linearity self-calibration incl. the extended calibration of the device revisions which require it
- Regular sequence (up to 16 ranks) and injected sequence (up to 4 ranks), software / hardware (timer, LPTIM, EXTI) triggers with edge selection, single / continuous conversion, automatic injected conversion
- Channel inputs: single-ended pin, differential pin pair, internal VREFINT / temperature sensor / VBAT (minimal sampling time checked), channel preselection (PCSEL)
- Resolution 14, 12, 10, 8 bit
- Data transfer modes: polling (`Adc_Task()`), interrupt, DMA through the GPDMA module (one shot / circular buffer) with half transfer, transfer complete, injected complete and error callbacks
- Analog watchdogs AWD1 - AWD3 (thresholds in the configured resolution, AWD1 filtering)
- Every register write verified by read-back, HW state checked before configuration changes

---

## Supported Hardware

| MCU family | Peripherals             | Channels | Resolution | Kernel clock  |
|------------|-------------------------|----------|------------|---------------|
| STM32U5    | ADC1, ADC2 (if present) | 0 - 19   | 14-bit     | 0.14 - 55 MHz |

- ADC2 is available on STM32U59x / U5Ax / U5Fx / U5Gx only. STM32U535 / U545 / U575 / U585 have ADC1 only.
- ADC4 (12-bit, low power ADC) is not handled by the module.
- Pins of ADC1 / ADC2 (IN1 - IN17): PC0, PC1, PC2, PC3, PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PC4, PC5, PB0, PB1, PB2.
  Differential channel `i` uses the pin of channel `i + 1` as the negative input (channels 1 - 16).
- Internal channels of ADC1: VREFINT channel 0, VBAT / 4 channel 18, temperature sensor channel 19.
- Kernel clock (`adc_ClkSrc_t`) is the multiplexer ADCDACSEL shared with DAC1 and ADC4: HCLK, SYSCLK, PLL2R, HSE, HSI16, MSIK.
  STM32U5 ADC1 / ADC2 have no synchronous clock mode - the selected clock is divided by the common prescaler (`adc_ClkDiv_t`).
- The analog supply VDDA is validated (PWR_SVMCR.ASV) by the RCC module when the ADC clock is activated.

---

## Module Structure

| File           | Description                                                                          |
|----------------|--------------------------------------------------------------------------------------|
| `Adc_Port.h`   | Public API                                                                           |
| `Adc_Types.h`  | Public types                                                                         |
| `Adc.c`        | Clock, peripheral and channel configuration, calibration, triggers, analog watchdogs |
| `Adc_Poll.c`   | Data transfer in polling mode (`Adc_Task()`)                                         |
| `Adc_Isr.c`    | Data transfer in interrupt mode (ADC1 and ADC2 share interrupt ADC1_2)               |
| `Adc_Dma.c`    | Data transfer in DMA mode (GPDMA channel per peripheral, DMNGT data management)      |
| `Adc.h`, `Adc_Poll.h`, `Adc_Isr.h`, `Adc_Dma.h` | Private interfaces (not exported by the CMake library) |

---

## Public API

### Module Management
- `adc_ModuleVersion_t Adc_Get_ModuleVersion ( void );`
- `adc_RequestState_t  Adc_Init              ( adc_Config_t * const adcConfig );`
- `adc_RequestState_t  Adc_Deinit            ( adc_PeriphId_t periphId );`
- `void                Adc_Task              ( void );`

### Clock Configuration
- `adc_RequestState_t  Adc_Set_ClockSource  ( adc_ClkSrc_t clkSource );`
- `adc_RequestState_t  Adc_Get_ClockSource  ( adc_ClkSrc_t * const clkSource );`
- `adc_RequestState_t  Adc_Set_ClockDivider ( adc_ClkDiv_t clkDiv );`
- `adc_RequestState_t  Adc_Get_ClockDivider ( adc_ClkDiv_t * const clkDiv );`

### Peripheral Configuration and State
- `adc_RequestState_t  Adc_PeriphInit         ( adc_PeriphConfig_t * const adcConfig );`
- `adc_RequestState_t  Adc_Set_PeriphActive   ( adc_PeriphId_t periphId );`
- `adc_RequestState_t  Adc_Set_PeriphInactive ( adc_PeriphId_t periphId );`
- `Adc_Set_` / `Adc_Get_` `TriggerSrc`, `TriggerMode`, `TriggerEdge`, `Resolution`

### Channel Configuration
- `adc_RequestState_t  Adc_ChannelInit ( adc_PeriphId_t periphId, adc_ChannelConfig_t * const channelConfig );`
- `Adc_Set_` / `Adc_Get_` `SamplingTime`, `ChannelInput`

### Conversion and Data
- `adc_RequestState_t  Adc_Set_RegStart   ( adc_PeriphId_t periphId );`
- `adc_RequestState_t  Adc_Set_RegStop    ( adc_PeriphId_t periphId );`
- `adc_RequestState_t  Adc_Set_InjStart   ( adc_PeriphId_t periphId );`
- `adc_RequestState_t  Adc_Set_InjStop    ( adc_PeriphId_t periphId );`
- `adc_RequestState_t  Adc_Set_DataConfig ( adc_PeriphId_t periphId, const adc_DataConfig_t * const dataConfig );`
- `adc_RequestState_t  Adc_Get_DataConfig ( adc_PeriphId_t periphId, adc_DataConfig_t * const dataConfig );`
- `adc_RequestState_t  Adc_Get_RegData    ( adc_PeriphId_t periphId, adc_Data_t * const data );`
- `adc_RequestState_t  Adc_Get_InjData    ( adc_PeriphId_t periphId, adc_InjSequenceId_t rankId, adc_Data_t * const data );`
- `adc_RequestState_t  Adc_Get_Flag       ( adc_PeriphId_t periphId, adc_FlagId_t flagId, adc_FlagState_t * const flagState );`
- `adc_RequestState_t  Adc_Clear_Flag     ( adc_PeriphId_t periphId, adc_FlagId_t flagId );`

### Analog Watchdog
- `adc_RequestState_t  Adc_AwdInit ( adc_PeriphId_t periphId, adc_AwdConfig_t * const awdConfig );`
- `Adc_Set_` / `Adc_Get_` `AwdThresholds`, `AwdFilter`

---

## Usage Notes

- Clock source and divider can be changed only while all ADC peripherals are disabled. The resulting kernel clock must be within 0.14 - 55 MHz (e.g. HCLK 160 MHz needs divider 4 or more).
- Channel input, sampling time and single-ended / differential mode are configured while the ADC is disabled (`Adc_PeriphInit()` / `Adc_ChannelInit()`), the channel is preselected (PCSEL) automatically.
- Internal channels need a minimal sampling time: VREFINT 4 us, temperature sensor 5 us, VBAT 12 us. Shorter sampling time is refused.
- 6-bit resolution and the internal inputs VDD_CORE / DAC1 / DAC2 exist on ADC4 only (ADC4 is not handled by the module), they are not in the lists `adc_Resolution_t` and `adc_ChannelInput_t`.
- Analog watchdog thresholds are given in the configured resolution; the module aligns them to the 14-bit comparison of the HW. AWD2 / AWD3 support only the "all channels" mode and no filtering.
- Calibration runs on every activation of the peripheral (`Adc_Set_PeriphActive()`). The extended calibration step is selected from the device identification (DBGMCU IDCODE) as in ST HAL.
- In polling mode `Adc_Task()` must be called periodically, it stores conversion results into the data buffer and calls the callbacks.

### Example

```c
adc_Config_t adcConfig = { 0 };
adc_PeriphConfig_t * const adc1 = &adcConfig.PeriphConfig[ ADC_PERIPH_1 ];

adcConfig.ClockSource  = ADC_CLK_SRC_HCLK;                  /* 160 MHz / 4 = 40 MHz */
adcConfig.ClockDivider = ADC_CLK_DIV_4;

adc1->PeriphId       = ADC_PERIPH_1;
adc1->Resolution     = ADC_RESOLUTION_12BIT;
adc1->RegTriggerId   = ADC_REG_TRIGGER_SOFTWARE;
adc1->RegTriggerMode = ADC_REG_TRIGGER_MODE_SINGLE;
adc1->RegChannelsCnt = 1u;
adc1->RegChannels[ 0u ].ChannelId       = ADC_CHANNEL_9;    /* PA4 */
adc1->RegChannels[ 0u ].ChannelInput    = ADC_CHANNEL_INPUT_PIN_SINGLE;
adc1->RegChannels[ 0u ].ChannelSampling = ADC_CHANNEL_SAMPLING_68_CYCLES;
adc1->DataConfig.TransferMode           = ADC_TRANSFER_MODE_POLL;

if( ADC_REQUEST_OK == Adc_Init( &adcConfig ) )
{
    (void)Adc_Set_RegStart( ADC_PERIPH_1 );
}
```

---

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_Adc.c`
  ```bash
  cmake --preset STM32U5A5xJ_UnitTest && cmake --build --preset STM32U5A5xJ_UnitTest
  ctest --test-dir Build_STM32U5A5xJ_UnitTest -L Adc
  ```
- Integration tests (target, NUCLEO board): `Tests/IntegrationTests/ItTest_Adc.c` - internal channels of ADC1 (VREFINT, temperature sensor) are converted, no external wiring is needed.

---

## 🛠 CMake Integration

```cmake
target_link_libraries(User_Lib PRIVATE Adc_Lib)
```

1. Include `Adc_Lib` in your CMake library.
2. Include `Adc_Port.h` in your project.
3. Configure the module as needed for your hardware.

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
