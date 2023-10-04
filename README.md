# FXLS89xx_Arduino
This library enables the developer to evaluate NXP FXLS89xx with Arduino.

> **Note**
This library works with [`I2C_device`](https://github.com/teddokano/I2C_device_Arduino) library together. Please be sure the `I2C_device` library is imported in your environment before trying to build. 

> **Warning**  
Make sure that **J6-2 MUST NOT be connected** when you use this library.  
Since the IOREF pin on Arduino is supplied by 5V, **shorting IOREF (5V) and +3V3 (3.3V) can cause either FXLS89xx board or Arduino board damaged.**  
This driver offers all functions by only I2C communication because all Arduino output H signal is 5V.  
<img src="docs/J6-2.JPG" width="320px" alt="Cut J6-2 in order NOT to short 5V and 3.3V"/>

## Datasheet and Board Information
Device Information|Datasheet|Board Information
---|---|---
[FXLS8961AF](https://www.nxp.com/products/sensors/accelerometers/2g-4g-8g-16g-low-power-12-bit-digital-accelerometer:FXLS8961AF)|[FXLS8961AF.pdf](https://www.nxp.com/docs/en/data-sheet/FXLS8961AF.pdf)|[FRDM-STBA-A8961](https://www.nxp.com/design/software/sensor-toolbox/evaluation-boards/sensor-toolbox-development-board-for-fxls8961af-3-axis-accelerometer:FRDM-STBA-A8961)
[FXLS8964AF](https://www.nxp.com/products/sensors/accelerometers/2g-4g-8g-16g-low-power-12-bit-digital-accelerometer:FXLS8964AF)|[FXLS8964AF.pdf](https://www.nxp.com/docs/en/data-sheet/FXLS8964AF.pdf)|[FRDM-STBA-A8964](https://www.nxp.com/design/software/development-software/sensor-toolbox-sensor-development-ecosystem/sensor-toolbox-development-board-for-fxls8964af-3-axis-accelerometer:FRDM-STBA-A8964)
[FXLS8967AF](https://www.nxp.com/products/sensors/accelerometers/2g-4g-8g-16g-low-power-12-bit-digital-accelerometer:FXLS8967AF)|[FXLS8967AF.pdf](https://www.nxp.com/docs/en/data-sheet/FXLS8967AF.pdf)|[FRDM-STBA-A8967](https://www.nxp.com/design/software/development-software/sensor-toolbox-sensor-development-ecosystem/sensor-toolbox-development-board-for-fxls8967af-3-axis-accelerometer:FRDM-STBA-A8967)
[FXLS8971CF](https://www.nxp.com/products/sensors/accelerometers/2g-4g-8g-16g-low-power-12-bit-digital-accelerometer:FXLS8971CF)|[FXLS8971CF.pdf](https://www.nxp.com/docs/en/data-sheet/FXLS8971CF.pdf)|[FRDM-STBI-A8971](https://www.nxp.com/design/software/sensor-toolbox/evaluation-boards/sensor-toolbox-development-board-for-fxls8971cf-3-axis-iot-accelerometer:FRDM-STBI-A8971)|
[FXLS8974CF](https://www.nxp.com/products/sensors/accelerometers/2g-4g-8g-16g-low-power-12-bit-digital-iot-accelerometer:FXLS8974CF)|[FXLS8974CF.pdf](https://www.nxp.com/docs/en/data-sheet/FXLS8974CF.pdf)|[FRDM-K22F-A8974/FRDM-STBI-A8974](https://www.nxp.com/design/software/sensor-toolbox/evaluation-boards/sensor-toolbox-development-board-for-fxls8974cf-3-axis-iot-accelerometer:FRDM-K22F-A8974)

## How to use
Use Library manager pane in Arduino IDE (the books mark), put this library name (FXLS89xx_Arduino) and click INSTALL button. This library is copied to your Arduino library folder (default: &lt;Your Documents folder&gt;\Arduino\libraries).  
You can find example codes inside this library by selecting from [File]-&gt;[Examples]-&gt;[FXLS89xx_Arduino].  
Please see the documents of [docs/class_f_x_l_s89xx___arduino.html](https://ryraki.github.io/FXLS89xx_Arduino/class_f_x_l_s89xx___arduino.html) in order to get the information of all the functions.

## What are inside in this library?
### List of contents
Category|Folder|Features
---|---|---
Example|[examples/example_DRDY_read/example_DRDY_read.ino](examples/example_DRDY_read/example_DRDY_read.ino)|Example codes to read the XYZ acceleration data every 0.16 sec (=1/6.25Hz), which is the most basic code using this library. Since the power mode is High Performance Mode and the frequency is relatively low, the acceleration data has very low noise. The range of acceleration data is inside +/-2G.
Example|[examples/example_EXT_TRIG/example_EXT_TRIG.ino](examples/example_EXT_TRIG/example_EXT_TRIG.ino)|Example code to use EXT_TRIG mode. Data conversion is NOT automatically done. Arduino MCU always outputs trigger signal to start conversion.
Example|[examples/example_SDCD_fixed/example_SDCD_fixed.ino](examples/example_SDCD_fixed/example_SDCD_fixed.ino)|Example code to utilize SDCD function of fixed threshold. If the Z acceleration is 0.7g>Z>0.3g, then the device outputs INT1 inputting into D2 of Arduino shield.
Example|[examples/example_SDCD_updatedRef/example_SDCD_updatedRef.ino](examples/example_SDCD_updatedRef/example_SDCD_updatedRef.ino)|Example code to utilize SDCD function of updated reference mode. The example also utilizes Sleep mode. Z crosses upper or lower threshold, the device wakes from Sleep mode. However, when 5 measurements are inside the threshold, the device will get into Sleep mode, which measures acceleration data in low frequency.
Documents|docs/&lt;All data inside the folder&gt;|The documents generated by Doxygen. Please see the link: [docs/class_f_x_l_s89xx___arduino.html](https://ryraki.github.io/FXLS89xx_Arduino/class_f_x_l_s89xx___arduino.html)
Source|[src/FXLS89xx_Arduino.cpp](src/FXLS89xx_Arduino.cpp)|The C++ file for this library.
Source|[src/FXLS89xx_Arduino.h](src/FXLS89xx_Arduino.h)|The header file for FXLS89xx_Arduino.cpp.
Source|[src/fxls896x.h](src/fxls896x.h)|The header for register definition made by NXP.
Arduino|keywords.txt|Keywords data only used for Arduino IDE 1.x.
Arduino|library.properties|Library properties used by Arduino and its Library manager.
Other|Doxyfile|For Doxygen
Other|LICENSE|License data. This is BSD 3-Clause.
Other|README.md|This file.
