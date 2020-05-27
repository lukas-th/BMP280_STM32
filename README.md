# BMP280 for STM32 #

Interfacing the Bosch Sensortec BMP280 pressure sensor with an STM32 using SPI and HAL.

## Usage ##

This repository was made for usage in [this quadrocopter project](https://github.com/1ukast/QuadrocopterV2 "BMP280_STM32 Documentation") (not yet public), which is utilizing a STM32 microcontroller to make a custom flight controller.

The quadrocopter project makes use of the HAL Drivers for STM32 and the STM32CubeMX code generator provided by ST to initialize hardware peripherals such as SPI.  It is written in C++.

This repository provides a C++ Class called "BMP280" to inteface the BMP280 pressure sensor. The constructor needs the HAL SPI handle as an argument.

There are 4 hardware dependent methods: spiReadWrite, spiCSNhigh, spiCSNlow and delay_ms. Adjust those to your project if necessary.

## Code documentation ##

The code documentation is available [here](https://1ukast.github.io/BMP280_STM32/doc/index.html "BMP280_STM32 Documentation").

It is build automatically using doxygen and deployed to gh-pages on every push to the master branch.<br>
![documentation](https://github.com/1ukast/BMP280_STM32/workflows/documentation/badge.svg)
