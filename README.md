# ad7739
C code driver for Ad773x series (spetialy for ad7739)

MAKE THE PROCESSOR SPECIFIC DEFINITIONS
---------------------------------------------------------------------------------

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



FOR SINGLE CONVERTION
--------------------------------------------------------------------------------------------------------
AD773x ad7739;
	createADC(&ad7739, &hspi2, GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_0);
	setADCP1Input(&ad7739);
AD773xChannel *one = createChannel(&ad7739, 0);
	//setChannelEnabledInContinuesReadMode(one);
	//setChannelContinuesRead(one);
	setChannel24Bit(one);
	setChannelAllowChoping(one);
	setChannelConvertionTime(one, 30);
	setChannelPowerRange(one, V2_5);
	setChannelDumpEnabled(one);
	//setChannelClockDisable(one);
  
  initADC(&ad7739);
  
    startSingleConvertion(&ad7739, one);
		while (IS_RDY(ad7739.ADC_RDY_PORT, ad7739.ADC_RDY_PIN)) {}
    unsigned long value = readAD773xADCValue(&ad7739, i);
    
   FOR CONTINUES CONVERTION
   -------------------------------------------------------------------------------------------------------
   AD773xChannel *one = createChannel(&ad7739, 0);
	//setChannelEnabledInContinuesReadMode(one);
	//setChannelContinuesRead(one);
	setChannel24Bit(one);
	setChannelAllowChoping(one);
	setChannelConvertionTime(one, 30);
	setChannelPowerRange(one, V2_5);
	setChannelDumpEnabled(one);
	//setChannelClockDisable(one);

	AD773xChannel *two = createChannel(&ad7739, 1);
	setChannelEnabledInContinuesReadMode(two); //two->isEnabled = ENABLE;
	//setChannelContinuesRead(two);
	setChannel24Bit(two);
	setChannelAllowChoping(two);
	setChannelConvertionTime(two, 30);
	setChannelPowerRange(two, V2_5);
	setChannelDumpEnabled(two);

	initADC(&ad7739);
  
  startContinuesConvertion(&ad7739, one);
  while(1) {
    while (IS_RDY(ad7739.ADC_RDY_PORT, ad7739.ADC_RDY_PIN)) {}
    for (uint8_t i = 0; i < 8; i++) {
			if (ad7739.channel[i].active == 1) {			
				unsigned long value = readAD773xADCValue(&ad7739, i);
        }
    }
  }
