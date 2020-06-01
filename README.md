# [LETBS] PSoC Final Project
This repository contains the last assignment of the electronic technologies and biosensors laboratory.
This project is about the development of a program able to store both accelerometer and potentiometer/photoresistor data in an external EEPROM. The system allows to change the main settings (e.g. frequency, FSR and other parameters) thanks to a user-friendly menu displayed using the CoolTerm interface. Eventually, according to UART communication protocol, the data is sent to the Bridge Control Panel software in order to be plotted.

In particular we used the [CY8CKIT-059](https://www.cypress.com/documentation/development-kitsboards/cy8ckit-059-psoc-5lp-prototyping-kit-onboard-programmer-and) development kit.

![board](https://user-images.githubusercontent.com/62112335/83408085-50c9f200-a412-11ea-8e34-597d6ac5b9d5.jpeg)

## Utilities:

**Accelerometer**
- LIS3DH 3-Axis Accelerometer breakout board
- <a href="https://cdn-learn.adafruit.com/downloads/pdf/adafruit-lis3dh-triple-axis-accelerometer-breakout.pdf">Breakout board reference manual</a>
- <a href="https://www.st.com/resource/en/datasheet/lis3dh.pdf">LIS3DH datasheet</a>

**EEPROM**
- 25LC256 256Kb EEPROM
- <a href="http://ww1.microchip.com/downloads/en/DeviceDoc/20005715A.pdf">25LC256 datasheet</a>

**Fixed Voltage regulator**
- 5V fixed voltage regulator to be used in combination with 4 AA batteries
-  <a href="http://www.ti.com/lit/ds/symlink/lm340.pdf">LM7805 datasheet</a>

![regulator](https://user-images.githubusercontent.com/62112335/83408103-57586980-a412-11ea-8734-153e75dd9741.png)


## TopDesign
1. KIT
![KIT](https://user-images.githubusercontent.com/62112335/83408096-54f60f80-a412-11ea-8491-f088eac839c0.png)

2. SPI
![SPI](https://user-images.githubusercontent.com/62112335/83408119-5d4e4a80-a412-11ea-893e-629319bbb640.png)

3. Analog
![ANALOG](https://user-images.githubusercontent.com/62112335/83408041-3db72200-a412-11ea-94fc-b4e11261367e.png)

4. Reset
![RESET](https://user-images.githubusercontent.com/62112335/83408108-59bac380-a412-11ea-8cd5-01040fd04b2a.png)

5. Bridge Control Panel
![BCP](https://user-images.githubusercontent.com/62112335/83408073-4c053e00-a412-11ea-8f33-90f47490fff7.png)


## Authors:
- Alessio Ratti
- Mattia Quaranta
- Marco Sinatra

![banner](https://user-images.githubusercontent.com/62112335/83408030-355ee700-a412-11ea-89e3-8feba66bd518.png)

