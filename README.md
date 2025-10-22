# HOJA-ESP32-Baseband
This is based off the Hand Held Legends git at: https://github.com/HandHeldLegend/HOJA-ESP32-Baseband/tree/b72d3987aaee337fc358b71def397a6118dea236

Baseband was designed to take in I2C data from an RP2040 and then emulate a Switch controller using BT and an ESP32.  I converted it to use GPIO directly from the ESP32.  HOJA-LIB-ESP32 is an older version that does this already from HHL, but its buggy and the developer said don't use it.

Pairs to Switch and has a valid N64 button mapping.  Requires a forked ESP-IDF from Hand Held Legend at: https://github.com/HandHeldLegend/esp-idf/tree/hoja_stable

You specifically have to select the hoja_stable branch of the ESP-IDF branch.

Then you need the sdkconfig file from: https://github.com/HandHeldLegend/HOJA-ESP32-Baseband/blob/master/sdkconfig

I'm a hardware guy so please forgive any bad programming practices.
