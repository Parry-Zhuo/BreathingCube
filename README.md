 # Breathing cube( Revision 1)  
The breathing cube is a BLE controlled STM32 device that controls LED's for users to provide breathing patterns to follow while creating an ambient, relaxing environment.

![IMG_2317](https://github.com/user-attachments/assets/3612826b-7e8f-4b12-94fe-f2078112d438)

## Hardware Highlights
- Designed and manufactured a **4-layer STM32-based PCB** with a Signal–GND–PWR–Signal stack-up, featuring **50 Ω controlled impedance traces for a 2.4 GHz RF antenna.**
- Designed a layout to support **I²C communication**, interfacing with peripherals such as gyroscopes, and led drivers.
- Designed and implemented a basic BMS for USB and battery power switching, ensuring efficient charging and reliable 3.3V regulation via a **buck converter**.
- Implemented an LP5812 LED driver which controls the LED's utilizing a mixture of **current control and PWM** to control the LED's; maximizing energy usage, and reducing eye-fatigue for prolonged use

## Power Supply Circuitry  
The power supply begins with a **USB-C connector**, which delivers **5V, max(3A)**. This connects to a **Battery Management System (BMS)** that operates as follows:  

1. **USB Plugged In**:  
   - The BMS disables the switch connecting the battery to the buck converter, isolating the battery.  
   - At the same time, the BMS charges the battery.  

2. **USB Not Plugged In**:  
   - The BMS enables the switch, allowing the battery to power the circuit.  

The output from the switch is fed into a **buck converter**, which steps down the input voltage to **3.3V** to power the entire circuit.

  
## Firmware Highlights
 - I²C Communication → Interfaced with gyroscopes and LED drivers to support motion-based mode selection.
 - Button Inputs → Capacitive touch buttons detect user interaction for starting, stopping, or adjusting meditation sessions.
 - Real-Time Clock (RTC) for Timing: Used RTC to track session duration and manage accurate timing without keeping the MCU fully active.
 - State Management & Interrupt Handling: Efficient use of interrupts and timers to manage user inputs, LED animations, and BLE interactions without unnecessary CPU usage.
 - Bluetooth Low Energy (BLE) Communication: Enabled wireless configuration and control via BLE, allowing users to customize meditation settings.
 - Used DMA and Interrupts to control WS2812B led's in previous prototypes of product

![IMG_2303](https://github.com/user-attachments/assets/ed096eaa-f72a-443a-a6af-a6ebdccd8727)
![IMG_2304](https://github.com/user-attachments/assets/ef337fea-2e1e-4785-ae30-d657dc8a0cd5)



![image](https://github.com/user-attachments/assets/ca5db52f-fed1-4477-909a-20fd2df3d5e0)


![image](https://github.com/user-attachments/assets/8fb4866a-da87-413b-a81c-4ff51392bc81)

![image](https://github.com/user-attachments/assets/345f035e-eceb-47a0-839d-9cabec9d8e98)

![image](https://github.com/user-attachments/assets/ba8afc21-cb9b-4e06-9099-e2f29564439e)

