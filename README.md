# ad7739
C code driver for Ad773x series (spetialy for ad7739)

Define your processor
#ifndef STM32
#define STM32
#endif

Define your processor nesesary comand in the top of file

#ifdef	STM32
#define	SPI_HANDLER_TYPE	SPI_HandleTypeDef // 	 USE YOUR PORT
#define	PORT				GPIO_TypeDef*
#define	PIN					uint16_t
#define SYNC_PIN_SET	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
#define SYNC_PIN_RESET	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
#define MAX_DELAY			HAL_MAX_DELAY
#define PIN_SET			GPIO_PIN_SET
#define PIN_RESET		GPIO_PIN_RESET
#define	DELAY(a) HAL_Delay(a);
#define	SPI_SEND(spi, data, len, timeout)				HAL_SPI_Transmit(spi, data, len, timeout)
#define	SPI_TRANSMITE(spi, data, recive, len, timeout)	HAL_SPI_TransmitReceive(spi, data, recive, len, timeout)
#define ADC_SELECT(port, pin)		HAL_GPIO_WritePin(port, pin, PIN_RESET)
#define ADC_DESELECT(port, pin)		HAL_GPIO_WritePin(port, pin, PIN_SET)
#define IS_RDY(port, pin)			(HAL_GPIO_ReadPin(port, pin) == PIN_SET)
#endif



