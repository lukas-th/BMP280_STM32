#include "BMP280.hpp"
#include "BMP280_defines.h"
#include "stm32f1xx_hal.h"
#include "main.h"

#include <math.h>

BMP280::BMP280(SPI_HandleTypeDef* handle)
{
	spiHandle = handle;
}

/** Initialize the device with desired configuration
 * @return 1, if device is not recognized, 0 otherwise.
 * */
uint8_t BMP280::initialize()
{
	if (getID() != BMP280_CHIP_ID) {
		return 1;
	}

	// Reset device and wait
	reset();
	delay_ms(500);

	// BEGIN OF CONFIGURATION ----------------------------------
	setPressureOversampling(oversampling_x16);
	setTemperatureOversampling(oversampling_x2);

	setPowerMode(mode_normal);
	setFilterCoefficient(filter_coeff_16);
	setStandbyTime(standby_time_500us);
	// END OF CONFIGURATION --------------------------

	readCompensationParameters();
	setReferencePressure(100, 5);

	return 0;
}

/** Perform power-on reset procedure */
void BMP280::reset()
{
	writeRegister(BMP280_REG_RESET, BMP280_RESET_VALUE);
}

/**
 * Read chip identification number.
 * @return chip ID
 * */
uint8_t BMP280::getID()
{
	return readRegister(BMP280_REG_ID);
}

/** Read calibration data from non-volatile sensor registers */
void BMP280::readCompensationParameters()
{
	uint8_t buf[24];
	readMBRegister(BMP280_REG_CALIB, buf, 24);
	compensationParameters.dig_T1 = ((buf[1] << 8) | buf[0]);
	compensationParameters.dig_T2 = ((buf[3] << 8) | buf[2]);
	compensationParameters.dig_T3 = ((buf[5] << 8) | buf[4]);
	compensationParameters.dig_P1 = ((buf[7] << 8) | buf[6]);
	compensationParameters.dig_P2 = ((buf[9] << 8) | buf[8]);
	compensationParameters.dig_P3 = ((buf[11] << 8) | buf[10]);
	compensationParameters.dig_P4 = ((buf[13] << 8) | buf[12]);
	compensationParameters.dig_P5 = ((buf[15] << 8) | buf[14]);
	compensationParameters.dig_P6 = ((buf[17] << 8) | buf[16]);
	compensationParameters.dig_P7 = ((buf[19] << 8) | buf[18]);
	compensationParameters.dig_P8 = ((buf[21] << 8) | buf[20]);
	compensationParameters.dig_P9 = ((buf[23] << 8) | buf[22]);
}

/**
 * Set reference pressure for altitude calculation by averaging pressure measurements.
 * @param samples: Number of measurements to average.
 * @param delay: Delay between measurements (in ms).
 * */
void BMP280::setReferencePressure(uint16_t samples, uint8_t delay)
{
	float sum = 0;
	for(char i = 0; i < samples; i++)
	{
			measure();
			sum += measurement.pressure;
			delay_ms(delay);
	}
	p_reference = sum / samples;

}

/** Configure pressure oversampling */
void BMP280::setPressureOversampling(Oversampling osrs_p)
{
	uint8_t ctrl = readRegister(BMP280_REG_CTRL_MEAS);
	ctrl = (ctrl & 0b11100011) | (osrs_p << 2);
	writeRegister(BMP280_REG_CTRL, ctrl);
}

/** Configure temperature oversampling */
void BMP280::setTemperatureOversampling(Oversampling osrs_t)
{
	uint8_t ctrl = readRegister(BMP280_REG_CTRL_MEAS);
	ctrl = (ctrl & 0b00011111) | (osrs_t << 5);
	writeRegister(BMP280_REG_CTRL, ctrl);
}

/** Configure power mode */
void BMP280::setPowerMode(PowerMode mode)
{
	uint8_t ctrl = readRegister(BMP280_REG_CTRL_MEAS);
	ctrl = (ctrl & 0b11111100) | mode;
	writeRegister(BMP280_REG_CTRL, ctrl);
}

/** Configure standby time */
void BMP280::setStandbyTime(StandbyTime t_sb)
{
	uint8_t conf = readRegister(BMP280_REG_CONFIG);
	conf = (conf & 0b00011111) | (t_sb << 5);
	writeRegister(BMP280_REG_CONFIG, conf);
}

/** Configure IIR filter */
void BMP280::setFilterCoefficient(FilterSetting filter)
{
	uint8_t conf = readRegister(BMP280_REG_CONFIG);
	conf = (conf & 0b11100011) | (filter << 2);
	writeRegister(BMP280_REG_CONFIG, conf);
}

/* ---------------------------------------------------------------------------
 * Measurements and compensation ---------------------------------------------
 * ------------------------------------------------------------------------ */

/**
 * Calculate sensor temperature from measurement and compensation parameters.
 * @param adc_T: Raw temperature measurement.
 * @return Temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
 * */
//Returns
int32_t BMP280::compensate_temperature(int32_t adc_T) {
	int32_t var1, var2;
	var1 = ((((adc_T >> 3) - ((int32_t) compensationParameters.dig_T1 << 1)))
			* (int32_t) compensationParameters.dig_T2) >> 11;
	var2 = (((((adc_T >> 4) - (int32_t) compensationParameters.dig_T1) *
			((adc_T >> 4) - (int32_t) compensationParameters.dig_T1)) >> 12)
			* (int32_t) compensationParameters.dig_T3) >> 14;
	t_fine = var1 + var2;
	return (t_fine * 5 + 128) >> 8;
}


/**
 * Calculate pressure from measurement and compensation parameters.
 * @param adc_P: Raw pressure measurement.
 * @return Pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 * */
uint32_t BMP280::compensate_pressure(int32_t adc_P) {
	int64_t var1, var2, p;
	var1 = (int64_t) t_fine - 128000;
	var2 = var1 * var1 * (int64_t) compensationParameters.dig_P6;
	var2 = var2 + ((var1 * (int64_t) compensationParameters.dig_P5) << 17);
	var2 = var2 + (((int64_t) compensationParameters.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) compensationParameters.dig_P3) >> 8) +
			((var1 * (int64_t) compensationParameters.dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) compensationParameters.dig_P1) >> 33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) compensationParameters.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) compensationParameters.dig_P8 * p) >> 19;
	p = ((p + var1 + var2) >> 8) + ((int64_t) compensationParameters.dig_P7 << 4);
	return p;
}

/**
 * Read latest measurement from sensor and execute compensation.
 * Stores the results in measurement member variable.
 * */
void BMP280::measure() {
	uint8_t data[6];
	readMBRegister(BMP280_REG_DATA, data, 6);

	int32_t adc_P = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	int32_t adc_T = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	measurement.temperature = (float)compensate_temperature(adc_T) / 100.0;
	measurement.pressure = (float)compensate_pressure(adc_P) / 256.0;

	if(p_reference > 0)
	{
		measurement.altitude = (1.0-pow(measurement.pressure/p_reference, 0.1903)) * 4433076.0;
	}
}

/* ---------------------------------------------------------------------------
 * Register read/write definitions -------------------------------------------
 * ------------------------------------------------------------------------ */

/**
 * Read a register
 * @param address: Register address.
 * @return Register value.
 * */
uint8_t BMP280::readRegister(uint8_t address) {
	spiCSNlow();
	spiReadWrite(address);
	uint8_t value = spiReadWrite(0);
	spiCSNhigh();
	return value;
}

/**
 * Write to a register
 * @param address: Register address.
 * @param value: Value to write.
 * */
void BMP280::writeRegister(uint8_t address, uint8_t value) {
	spiCSNlow();
	spiReadWrite(address & BMP280_SPI_MASK_WRITE);
	spiReadWrite(value);
	spiCSNhigh();
}

/**
 * Read a multi-byte register
 * @param address: Register address.
 * @param values: Array pointer to store values in.
 * @param length: Number of bytes to read.
 * */
void BMP280::readMBRegister(uint8_t address, uint8_t *values, uint8_t length) {
	spiCSNlow();
	spiReadWrite(address);
	while (length--) {
		*values++ = spiReadWrite(0);
	}
	spiCSNhigh();
}

/* ---------------------------------------------------------------------------
 * SPI interface definitions -------------------------------------------------
 * (ADAPT THESE METHODS TO YOUR HARDWARE) ------------------------------------
 * ------------------------------------------------------------------------ */

/**
 * SPI transmit and receive one byte simultaneously
 * @param tx_message: Transmit byte.
 * @return Received byte.
 * */
uint8_t BMP280::spiReadWrite(uint8_t tx_message) {
	uint8_t rx_message = 255;
	HAL_SPI_TransmitReceive(this->spiHandle, &tx_message, &rx_message, 1, HAL_MAX_DELAY);
	return rx_message;
}

/** Pull chip select high (inactive) */
void BMP280::spiCSNhigh() {
	HAL_GPIO_WritePin(BMP280_CSN_GPIO_Port, BMP280_CSN_Pin, GPIO_PIN_SET);
}

/** Pull chip select low (active) */
void BMP280::spiCSNlow() {
	HAL_GPIO_WritePin(BMP280_CSN_GPIO_Port, BMP280_CSN_Pin, GPIO_PIN_RESET);
}

/** Millisecond Delay */
void BMP280::delay_ms(uint32_t milliseconds)
{
	HAL_Delay(milliseconds);
}
