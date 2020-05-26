#ifndef BMP280_HPP_
#define BMP280_HPP_

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "BMP280_defines.h"

/**
 *  Interface with an Bosch BMP280 pressure sensor over SPI.
 *  For more information refer to the datasheet:
 *  https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 */
class BMP280
{
public:
	BMP280(SPI_HandleTypeDef* handle);

	struct Measurement {
		float temperature = 0;
		float pressure = 0;
		float altitude = 0;
	} measurement;

	uint8_t initialize();
	void measure();

private:
	SPI_HandleTypeDef* spiHandle;
	float p_reference = 0;
	int32_t t_fine = 0;

	struct CompensationParameters {
		uint16_t dig_T1;
		int16_t dig_T2;
		int16_t dig_T3;
		uint16_t dig_P1;
		int16_t dig_P2;
		int16_t dig_P3;
		int16_t dig_P4;
		int16_t dig_P5;
		int16_t dig_P6;
		int16_t dig_P7;
		int16_t dig_P8;
		int16_t dig_P9;
	} compensationParameters;


	uint8_t readRegister(uint8_t address);
	void writeRegister(uint8_t address, uint8_t value);
	void readMBRegister(uint8_t address, uint8_t *values, uint8_t length);

	void reset();
	uint8_t getID();
	void readCompensationParameters();
	void setReferencePressure(uint16_t samples, uint8_t delay);
	void setPressureOversampling(Oversampling osrs_p);
	void setTemperatureOversampling(Oversampling osrs_t);
	void setPowerMode(PowerMode mode);
	void setStandbyTime(StandbyTime t_sb);
	void setFilterCoefficient(FilterSetting filter);

	int32_t compensate_temperature(int32_t adc_T);
	uint32_t compensate_pressure(int32_t adc_P);

	uint8_t spiReadWrite(uint8_t tx_message);
	void spiCSNhigh();
	void spiCSNlow();
	void delay_ms(uint32_t milliseconds);
};

#endif /* BMP280_HPP_ */
