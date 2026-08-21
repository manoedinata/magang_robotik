#pragma once

// GPIO untuk UART ke STM32
#if defined(ARDUINO_ARCH_ESP32)
#define STM32_RX 20 // GPIO20 -> STM32 TX
#define STM32_TX 21 // GPIO21 -> STM32 RX
#elif defined(ARDUINO_ARCH_ESP8266)
#define STM32_RX 3  // GPIO3 -> STM32 TX
#define STM32_TX 1  // GPIO1 -> STM32 RX
#endif
