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

- ```c
  adc_ModuleVersion_t Adc_Get_ModuleVersion ( void );
  ```  
  Returns the current version of the ADC module.

---

### Initialization

  ```c
- adc_RequestState_t Adc_Init ( adc_PeriphConfig_t * const adcConfig );
  ```  
  Initializes the ADC peripheral with the provided configuration.

  ```c
- adc_RequestState_t Adc_Deinit ( adc_PeriphConfig_t * const adcConfig );
  ```  
  Deinitializes the ADC peripheral and resets the configuration.

  ```c
- void Adc_Task ( void );
  ```  
  Handles ADC-related periodic tasks (if required by the implementation).

  ```c
- adc_RequestState_t Adc_Get_DefaultConfig ( adc_PeriphConfig_t * const adcConfig );
  ```  
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
