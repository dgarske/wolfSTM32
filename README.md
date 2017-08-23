# WolfSSL example for System Workbench (Open STM32 Tools)

This example includes:

* wolfCrypt test
* wolfCrypt benchmark
* wolfMQTT client

These examples use the CubeMX Hal for STM32. If you'd like to use the older Standard Peripheral library undefine WOLFSSL_STM32_CUBEMX in user_settings.h. To enable STM32F2 support define WOLFSSL_STM32F2.

## Important Files

`/Inc/user_settings.h`: All the configurations for the wolfSSL library on the STM32F4.

## Tools

* [System Workbench (Open STM32 Tools)](http://www.st.com/en/development-tools/sw4stm32.html)

## Usage

1. Open a new workspace.
2. File / Import -> Existing Projects into Workspace. Next
3. Choose the wolfSTM32 folder. Finish

## Help

For assistance please email [support@wolfssl.com](mailto:support@wolfssl.com).