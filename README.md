# Meditation Timer  

This project is a custom-designed **Meditation Timer**, combining hardware and software to create a device meant to help users build and maintain the habit of meditation.  

## Hardware Highlights
- Designed and manufactured a **4-layer PCB** with a stack-up of Signal-GND-PWR-Signal with an **STM32 microcontroller**.
- Collaborated with a PCB manufacturer to ensure production quality and specifications.
- Developed basic **power circuitry** capable of operating up to 40 hours on a 3.3V supply.
- Designed a layout to support **I²C communication**, interfacing with peripherals such as gyroscopes, and led drivers.

## Software Features
- Dual operational modes:
  - **Using Mode**: LED-based breathing animation for meditation guidance to create an ambient environment
  - **Configuration Mode**: Gyroscope-controlled mode with capacitive touch sensors to configure settings
    - **Bluetooth connectivity**: Capable of more indepth configuration through BLE, that connects with phone
    - Programmed addressable **RGBW LEDs** for dynamic visual feedback.

## Design Aesthetics
- Enclosed the electronics in a **6 cm³ plexiglass box**, offering a sleek design with beautifully diffused lighting effects.

 
![IMG_2317](https://github.com/user-attachments/assets/3612826b-7e8f-4b12-94fe-f2078112d438)

![IMG_2303](https://github.com/user-attachments/assets/ed096eaa-f72a-443a-a6af-a6ebdccd8727)
![IMG_2304](https://github.com/user-attachments/assets/ef337fea-2e1e-4785-ae30-d657dc8a0cd5)

## Power Supply Circuitry  

The power supply begins with a **USB-C connector**, which delivers **5V, max(3A)**. This connects to a **Battery Management System (BMS)** that operates as follows:  

1. **USB Plugged In**:  
   - The BMS disables the switch connecting the battery to the buck converter, isolating the battery.  
   - At the same time, the BMS charges the battery.  

2. **USB Not Plugged In**:  
   - The BMS enables the switch, allowing the battery to power the circuit.  

The output from the switch is fed into a **buck converter**, which steps down the input voltage to **3.3V** to power the entire circuit.

![image](https://github.com/user-attachments/assets/ca5db52f-fed1-4477-909a-20fd2df3d5e0)


![image](https://github.com/user-attachments/assets/8fb4866a-da87-413b-a81c-4ff51392bc81)


![image](https://github.com/user-attachments/assets/345f035e-eceb-47a0-839d-9cabec9d8e98)
The LP5812 LED drivers enable smooth, energy-efficient animations by offloading the sequence execution, allowing the MCU to remain in low-power mode.

