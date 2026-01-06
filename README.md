# Formula Student Steering Wheel / Driver Dashboard  
**STM32F103C8T6 + CAN + Nextion Display**

## Overview

This repo contains the firmware for a **Formula Student combustion vehicle steering wheel / driver dashboard**.  
The system is built around an **STM32F103C8T6 (Blue Pill)** microcontroller and communicates with a **PE3 8400 ECU** over **CAN bus**.

Real-time vehicle data is received via CAN and displayed on a **Nextion HMI display**, allowing the driver to monitor key engine and vehicle parameters during operation.

## Displayed Parameters

The dashboard currently supports displaying:

- Engine RPM  
- Wheel speed  
- Manifold Absolute Pressure (MAP)  
- Engine coolant temperature (in celsius)
- Battery voltage  (Volts)
- Gear position  
- Throttle position (%)  
- Brake position (%)  

## Code Structure

The firmware follows an **interrupt-driven CAN reception architecture**.

### `main.c`
This file is the main entry point of the firmware and is responsible for:

- System clock configuration  
- GPIO, CAN, UART, and NVIC initialization  
- Starting the CAN peripheral and enabling interrupts  
- Decoding received CAN frames  
- Updating the Nextion display  


## CAN Communication Logic

### CAN Initialization
- Uses **CAN1** peripheral on the STM32F103  
- Configured in **Normal Mode**  
- Bit timing set to match ECU CAN baud rate  
- Hardware CAN filters used to accept only required CAN IDs  

### CAN Receive Interrupt
- CAN messages are received using the **FIFO1 message pending interrupt**
- The ISR (`HAL_CAN_RxFifo1MsgPendingCallback`) performs minimal processing:
  - Reads one CAN frame
  - Copies data into a shared buffer
  - Sets a flag indicating new data availability

### Main Loop Processing
- The main loop continuously checks for new CAN data
- When data is available:
  - CAN ID is decoded
  - Raw sensor values are extracted and scaled
  - Values are sent to the Nextion display

## Nextion Display Interface
Communication with the Nextion display is handled via **UART (USART1)**.

### Common Display Functions
The following helper functions are used throughout the code:
- `NXT_SendNum(object, value)`  
- `NXT_SendFloat(object, value, decimals)`  
- `NXT_SendTXT(object, text)`  
- `NXT_SendCmd(object, command, value, extra, refresh)`  

## Hardware Requirements

- STM32F103C8T6 (Blue Pill)
- CAN transceiver (e.g. TJA1050, SN65HVD230)
- Nextion HMI display
- 120Ω termination resistors on CAN bus
- Twisted-pair CANH/CANL wiring
- PE3 8400 ECU (or compatible CAN broadcaster)

## How to Build and Run

### 1. Software Setup
- Install **STM32CubeIDE**
- Clone or download this repository
- Open the project in STM32CubeIDE

### 2. Hardware Connections

#### CAN
- PA11 → CAN_RX  
- PA12 → CAN_TX  

#### UART (Nextion)
- PA9  → TX  
- PA10 → RX  

Ensure the CAN bus is properly terminated (≈60Ω total).

### 3. Build & Flash
- Select the correct STM32F103C8 target
- Build the project
- Flash the firmware using ST-Link or equivalent

### 4. Power-Up Sequence
1. Power the Low Voltage System  
2. ECU begins transmitting CAN data  
3. Dashboard initializes and waits for CAN frames  
4. Display updates automatically as data is received  

## Notes for Debugging

- If values remain `0` or `---`:
  - Check CAN wiring and transceiver
  - Verify CAN baud rate matches ECU
  - Confirm CAN IDs using a CAN analyzer
- If the display freezes:
  - Ensure no `HAL_Delay()` calls exist inside interrupts
  - Verify UART baud rate matches the Nextion project
- Use PC13 LED blink patterns to identify fatal errors


## 👥 Authors & Team

Developed by the **Formula Student Electronics Team**  
Maintained for reliability, clarity, and scrutineering compliance.
