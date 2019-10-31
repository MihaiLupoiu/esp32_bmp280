#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "bmp280.h"

#define SPI_HOST HSPI_HOST
#define DMA_CHAN 1

#define PIN_NUM_MOSI GPIO_NUM_13
#define PIN_NUM_MISO GPIO_NUM_12
#define PIN_NUM_CLK GPIO_NUM_14
#define PIN_NUM_CS GPIO_NUM_17

int8_t initialize_spi_communication();
int8_t initialize_bmp_device(struct bmp280_dev *dev);

// SPI functions
int8_t read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void delay_ms(uint32_t period);

spi_device_handle_t spi;

int app_main() {
	struct bmp280_dev dev = {0};
	struct bmp280_uncomp_data uncomp_data = {0};
	int ret;

	ret = ENOSYS;
	ret = initialize_spi_communication();
	if (ret < 0) {
		printf("Error: failed to initialize SPI communication\n");
		exit(EXIT_FAILURE);
	}

	ret = initialize_bmp_device(&dev);
	if (ret < 0) {
		printf("Error: failed to initialize bmp280\n");
		exit(EXIT_FAILURE);
	}

	printf("Starting\n");

	ret = bmp280_soft_reset(&dev);
	if (ret < 0) {
		printf("Error: failed to soft reset sensor\n");
		exit(EXIT_FAILURE);
	}

	delay_ms(3000);

	struct bmp280_config bmp280_config = {0};
	if (bmp280_get_config(&bmp280_config, &dev) < 0) {
		printf("Error: failed to get sensor config\n");
		exit(EXIT_FAILURE);
	}

	/*!
	 * @brief This API writes the data to the ctrl_meas register and config register.
	 * It sets the temperature and pressure over-sampling configuration,
	 * power mode configuration, sleep duration and IIR filter coefficient.
	 *
	 * @param[in] conf : Desired configuration to the bmp280
	 * conf.osrs_t, conf.osrs_p = BMP280_OS_NONE, BMP280_OS_1X,
	 *     BMP280_OS_2X, BMP280_OS_4X, BMP280_OS_8X, BMP280_OS_16X
	 * conf.odr = BMP280_ODR_0_5_MS, BMP280_ODR_62_5_MS, BMP280_ODR_125_MS,
	 *     BMP280_ODR_250_MS, BMP280_ODR_500_MS, BMP280_ODR_1000_MS,
	 *     BMP280_ODR_2000_MS, BMP280_ODR_4000_MS
	 * conf.filter = BMP280_FILTER_OFF, BMP280_FILTER_COEFF_2,
	 *     BMP280_FILTER_COEFF_4, BMP280_FILTER_COEFF_8, BMP280_FILTER_COEFF_16
	 * conf.spi3w_en = BMP280_SPI3_WIRE_ENABLE, BMP280_SPI3_WIRE_DISABLE
	 * @param[in] dev : Structure instance of bmp280_dev
	 *
	 * @return Result of API execution
	 * @retval Zero for Success, non-zero otherwise.
	 */

	printf("bmp280_config.os_pres: %d \n", bmp280_config.os_pres);
	printf("bmp280_config.os_temp: %d \n", bmp280_config.os_temp);
	printf("bmp280_config.odr: %d \n", bmp280_config.odr);
	printf("bmp280_config.filter: %d \n", bmp280_config.filter);

	// bmp280_config.odr = BMP280_ODR_4000_MS;
	// bmp280_config.os_pres = BMP280_OS_4X;
	// bmp280_config.os_temp = BMP280_OS_4X;
	// bmp280_config.spi3w_en = BMP280_SPI3_WIRE_DISABLE;

	// if (bmp280_set_config(&bmp280_config, &dev) < 0) {
	// 	printf("Error: failed set sensor config\n");
	// 	exit(EXIT_FAILURE);
	// }

	uint8_t mode = 0;
	ret = bmp280_get_power_mode(&mode, &dev);
	if (ret < 0) {
		printf("Error: failed to read data from sensor\n");
		exit(EXIT_FAILURE);
	}

	switch (mode) {
	case BMP280_SLEEP_MODE:
		printf("Sleep Power Mode\n");
		printf("Setting to Normal Power Mode\n");

		if (bmp280_set_power_mode(BMP280_NORMAL_MODE, &dev) < 0) {
			printf("Error: failed to read data from sensor\n");
			exit(EXIT_FAILURE);
		}

		break;
	case BMP280_NORMAL_MODE:
		printf("Normal Power Mode\n");
		break;
	case BMP280_FORCED_MODE:
		printf("Forced Power Mode\n");
		break;

	default:
		break;
	}

	// uint8_t sensor_comp = 7; // Get all measurements, i.e. temp, hum and pres
	while (1) {

		struct bmp280_status stat = {0};
		ret = bmp280_get_status(&stat, &dev);
		if (ret < 0) {
			printf("Error: failed to read data from sensor\n");
			exit(EXIT_FAILURE);
		}

		if (stat.measuring == BMP280_MEAS_DONE) {
			ret = bmp280_get_uncomp_data(&uncomp_data, &dev);
			if (ret < 0) {
				printf("Error: failed to read data from sensor\n");
				exit(EXIT_FAILURE);
			}

			int32_t temp = 0;
			ret = bmp280_get_comp_temp_32bit(&temp, uncomp_data.uncomp_temp, &dev);
			if (ret < 0) {
				printf("Error: failed to transform temperature data from sensor\n");
				exit(EXIT_FAILURE);
			}

			uint32_t pres = 0;
			ret = bmp280_get_comp_pres_32bit(&pres, uncomp_data.uncomp_temp, &dev);
			if (ret < 0) {
				printf("Error: failed to transform temperature data from sensor\n");
				exit(EXIT_FAILURE);
			}

			printf("temperature = %f DegC\n", (float) temp / 100);
			printf("pressure = %f Pa\n", (float) pres / 256);
		} else {
			printf("Mesuring ongoing\n");

			switch (stat.im_update) {
			case BMP280_IM_UPDATE_DONE:
				printf("Update Done\n");
				break;
			case BMP280_IM_UPDATE_ONGOING:
				printf("Update Ongoing\n");
				break;
			default:
				printf("WTF\n");
				break;
			}
		}
	}

	return ret;
}

int8_t initialize_spi_sensor() {
	int8_t ret;

	spi_bus_config_t buscfg = {.mosi_io_num = PIN_NUM_MOSI,
	                           .miso_io_num = PIN_NUM_MISO,
	                           .sclk_io_num = PIN_NUM_CLK,
	                           .quadwp_io_num = -1,
	                           .quadhd_io_num = -1};

	spi_device_interface_config_t devcfg = {.clock_speed_hz = 10 * 1000 * 1000, // Clock out at 10 MHz
	                                        .mode = 0,
	                                        .spics_io_num = PIN_NUM_CS,
	                                        .queue_size = 7,
	                                        .address_bits = 8};

	// Initialize the SPI bus
	ret = ENOSYS;
	ret = spi_bus_initialize(SPI_HOST, &buscfg, DMA_CHAN);
	if (ret < 0) {
		printf("Error while initializing SPI bus\n");
		ESP_ERROR_CHECK(ret);
		return ret;
	}

	// Attach the device to the SPI bus
	ret = ENOSYS;
	ret = spi_bus_add_device(SPI_HOST, &devcfg, &spi);
	if (ret < 0) {
		ESP_ERROR_CHECK(ret);
		printf("Error while adding device to SPI bus\n");
		return ret;
	}

	return ret;
}

int8_t initialize_spi_communication() {
	int8_t ret;
	ret = ENOSYS;
	ret = initialize_spi_sensor();
	if (ret < 0) {
		ESP_ERROR_CHECK(ret);
		printf("Error while initializing spi sensor\n");
		return ret;
	}

	return ret;
}

int8_t initialize_bmp_device(struct bmp280_dev *dev) {
	int8_t ret;

	dev->intf = BMP280_SPI_INTF;
	dev->read = &read;
	dev->write = &write;
	dev->delay_ms = &delay_ms;

	// Initialize the BMP280
	ret = bmp280_init(dev);
	printf("[initialize_bmp_device] ret = %d\n", ret);
	if (ret < 0) {
		printf("Error: bmp280 device initialization failled\n");
		return ret;
	}

	return ret;
}

int8_t read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int ret;
	static struct spi_transaction_t trans;

	trans.flags = 0;
	trans.addr = reg_addr;
	trans.length = len * 8;
	trans.tx_buffer = heap_caps_malloc(1, MALLOC_CAP_DMA);
	trans.rx_buffer = heap_caps_malloc(len, MALLOC_CAP_DMA);

	/* debug */
	// printf("length = %d\n", len);
	// printf("reg_addr = %x\n", reg_addr);

	// failwith "Students, this is your job!
	ret = ENOSYS;
	ret = spi_device_transmit(spi, &trans);
	if (ret < 0) {
		printf("Error: transaction transmission failled\n");
		return ret;
	}

	memcpy(data, trans.rx_buffer, len);

	/* debug */
	// for (int i = 0; i < len; i++)
	// 	printf("rx_buffer[%d] = %d %x\n", i, ((uint8_t *) trans.rx_buffer)[i], ((uint8_t *) trans.rx_buffer)[i]);
	// delay_ms(1000);

	return ret;
}

int8_t write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t ret;
	static struct spi_transaction_t trans;

	trans.tx_buffer = heap_caps_malloc(len, MALLOC_CAP_DMA);

	trans.flags = 0;
	trans.addr = reg_addr;
	trans.length = len * 8;

	memcpy((void *) trans.tx_buffer, data, len);

	ret = spi_device_transmit(spi, &trans);
	if (ret < 0) {
		printf("Error: transaction transmission failled\n");
		return ret;
	}

	return ret;
}

void delay_ms(uint32_t period) {
	vTaskDelay(period / portTICK_RATE_MS);
}