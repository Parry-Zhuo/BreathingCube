This is a project where I created a meditation timer. 

Manufactured 4 Layer PCB with standard signal-gnd-pwr-signal layers.
Communicated with manufacturer to get PCB made.

Starting with hardware, where I created a PCB containing an STM32 as well as basic power circuitry and layout of different buses to communicate with different devices via I2C.
![IMG_2317](https://github.com/user-attachments/assets/3612826b-7e8f-4b12-94fe-f2078112d438)

![IMG_2303](https://github.com/user-attachments/assets/ed096eaa-f72a-443a-a6af-a6ebdccd8727)
![IMG_2304](https://github.com/user-attachments/assets/ef337fea-2e1e-4785-ae30-d657dc8a0cd5)

POWER SUPPLY CIRCUITRY:

Starts off with USB-C, where it communicates 5V,3A.
This goes to the BMS(Battery Management system), 
if USB is plugged in, turns off switch(from battery to buck convertor) while also charging battery.

if USB is NOT plugged in, it turns on switch allowing battery to power circuit.

The output of this switch goes to a Buck-convertor, changing whatever input voltage to 3.3V which powers the entire circuit.
![image](https://github.com/user-attachments/assets/ca5db52f-fed1-4477-909a-20fd2df3d5e0)


![image](https://github.com/user-attachments/assets/8fb4866a-da87-413b-a81c-4ff51392bc81)



![image](https://github.com/user-attachments/assets/345f035e-eceb-47a0-839d-9cabec9d8e98)


