## SPI master read BME280

This code displays a simple graphics with varying pixel colors on the 320x240 LCD on an ESP-WROVER-KIT board.


Sample code for reading values from a BME280 via ESP-IDF's SPI master driver
====================

To see the main code check [main/main.c] for comments that explain some of implementation details.


----------
About
----------

This sample code implement procedures to read values from BME280 sensor (pressure, temperature, humidity) via ESP-IDF's SPI master driver. 

.. _ESP32 datasheet: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
.. _Bosch's BME280 datasheet: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP280-DS001.pdf