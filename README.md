 
![IMG_2317](https://github.com/user-attachments/assets/3612826b-7e8f-4b12-94fe-f2078112d438)
# Breathing cube  

This project is a custom-designed **Meditation Timer**, combining hardware and software to create a device meant to help users build and maintain the habit of meditation.  

## Hardware Highlights
- Designed and manufactured a **4-layer PCB** with a stack-up of Signal-GND-PWR-Signal with an **STM32 microcontroller**.
- Collaborated with a PCB manufacturer to ensure production quality and specifications.
- Designed and implemented a power management system with a **BMS** for USB and **battery power switching**, ensuring efficient charging and reliable 3.3V regulation via a **buck converter**.
- Designed a layout to support **I²C communication**, interfacing with peripherals such as gyroscopes, and led drivers.
- The LP5812 LED drivers enable smooth, energy-efficient animations by offloading the sequence execution, allowing the MCU to remain in low-power mode while the LED animation continues

## Firmware Highlights
 - I²C Communication → Interfaced with gyroscopes and LED drivers to support motion-based mode selection.
 - Button Inputs → Capacitive touch buttons detect user interaction for starting, stopping, or adjusting meditation sessions.
 - Real-Time Clock (RTC) for Timing: Used RTC to track session duration and manage accurate timing without keeping the MCU fully active.
 - State Management & Interrupt Handling: Efficient use of interrupts and timers to manage user inputs, LED animations, and BLE interactions without unnecessary CPU usage.
 - Energy-Efficient Operation: Implemented low-power sleep modes for the STM32WB55, ensuring minimal energy consumption while maintaining BLE connectivity.
 - Bluetooth Low Energy (BLE) Communication: Enabled wireless configuration and control via BLE, allowing users to customize meditation settings.
 - Used DMA and Interrupts to control WS2812B led's in previous prototypes of product

## Design Aesthetics
- Enclosed the electronics in a **6 cm³ polycarbonate box**, offering a sleek design with beautifully diffused lighting effects.

## Power Supply Circuitry  

The power supply begins with a **USB-C connector**, which delivers **5V, max(3A)**. This connects to a **Battery Management System (BMS)** that operates as follows:  

1. **USB Plugged In**:  
   - The BMS disables the switch connecting the battery to the buck converter, isolating the battery.  
   - At the same time, the BMS charges the battery.  

2. **USB Not Plugged In**:  
   - The BMS enables the switch, allowing the battery to power the circuit.  

The output from the switch is fed into a **buck converter**, which steps down the input voltage to **3.3V** to power the entire circuit.
![IMG_2303](https://github.com/user-attachments/assets/ed096eaa-f72a-443a-a6af-a6ebdccd8727)
![IMG_2304](https://github.com/user-attachments/assets/ef337fea-2e1e-4785-ae30-d657dc8a0cd5)


![image](https://github.com/user-attachments/assets/ca5db52f-fed1-4477-909a-20fd2df3d5e0)


![image](https://github.com/user-attachments/assets/8fb4866a-da87-413b-a81c-4ff51392bc81)



![image](https://github.com/user-attachments/assets/345f035e-eceb-47a0-839d-9cabec9d8e98)


