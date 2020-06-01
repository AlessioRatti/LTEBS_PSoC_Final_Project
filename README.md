# [LETBS] PSoC Final Project
This repository contains the last assignment of the electronic technologies and biosensors laboratory.
This project is about the development of a program able to store both accelerometer and potentiometer/photoresistor data in an external EEPROM. The system allows to change the main settings (e.g. frequency, FSR and other parameters) thanks to a user-friendly menu displayed using the CoolTerm interface. Eventually, according to UART communication protocol, the data is sent to the Bridge Control Panel software in order to be plotted.

In particular we used the [CY8CKIT-059](https://www.cypress.com/documentation/development-kitsboards/cy8ckit-059-psoc-5lp-prototyping-kit-onboard-programmer-and) development kit.

<img src="https://raw.githubusercontent.com/AlessioRatti/Group_02_Final_Project/master/Imgs/board.jpeg?token=AOZ42ACAFXG7VGZKZJTSAX262TRQY"     width="281" height="500" title="Prototype" alt="Prototype">

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

<img src="https://raw.githubusercontent.com/AlessioRatti/Group_02_Final_Project/master/Imgs/regulator.png?token=AOZ42AHBZEVS7RYL7SIWF6S62TP4E" width="146" height="273" title="Regulator" alt="Regulator">


## TopDesign
1. KIT
<img src="https://raw.githubusercontent.com/AlessioRatti/Group_02_Final_Project/master/Imgs/KIT.png?token=AOZ42ACWBABOIVCKMPMQLGC62TP36" width="525" height="407" title="Kit" alt="Kit">

2. SPI
<img src="https://raw.githubusercontent.com/AlessioRatti/Group_02_Final_Project/master/Imgs/SPI.png?token=AOZ42AFADWAZN24XN5SCWFS62TP4A" width="525" height="407" title="SPI" alt="SPI">

3. Analog
<img src="https://raw.githubusercontent.com/AlessioRatti/Group_02_Final_Project/master/Imgs/ANALOG.png?token=AOZ42ADGI3RRUBXUJZE3MGK62TPI6" width="525" height="407" title="Analog" alt="Analog">

4. Reset
<img src="https://raw.githubusercontent.com/AlessioRatti/Group_02_Final_Project/master/Imgs/RESET.png?token=AOZ42AAZY2QA7LQIZJWX53262TP4A" width="525" height="407" title="Reset" alt="Reset">

5. Bridge Control Panel
<img src="https://raw.githubusercontent.com/AlessioRatti/Group_02_Final_Project/master/Imgs/BCP.png?token=AOZ42AB7A6YALNZ6DDJAUR262TP34" width="525" height="407" title="Bridge Control Panel" alt="Bridge Control Panel">

## Authors:
- Alessio Ratti
- Mattia Quaranta
- Marco Sinatra

<img src="https://raw.githubusercontent.com/AlessioRatti/Group_02_Final_Project/master/Imgs/banner.png?token=AOZ42ADXN2MHWP3F3JP7MTC62TP4C" title="Banner" alt="Banner">
