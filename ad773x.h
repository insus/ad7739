/*
 * adc.h
 *
 *  Created on: Oct 6, 2021
 *      Author: insus
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#define bitset(var,pos) ((var) & (1<<(pos)))

#ifndef STM32
#define STM32
#endif

/*######################### PROCESSOR DEFS ##################################  */
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

/*##############################################################################*/

#define	COMM_ADDR								0x00
#define READSTATE 								0x40
#define ADC_DATA_ADDR 							READSTATE | 0x08 // 0x48 first position of DATA0. DATA1 = (0x48 | 1)

/********************************************/
#define IOPORT  								0x01
/* Configure P0 and P1 pins input/output level;  */
#define P0Pin									0b10000000
#define	P1Pin									0b01000000
/* Configure P0 and P1 direction */
#define	P0Input									0b00100000
#define	P1Input									0b00010000
/* RDYFN */
#define RDYFN									0b00001000
/* RDYPWR */
#define	REDPWR									0b00000100
/* SYNC */
#define	SYNC									0b00000001

/* CHANNEL SETUP REGISTER */
#define CH_SETUP_REG							0x28
#define	AIN_TO_AIN								0b01100000
#define	OPT										0b00010000
#define	ENABLE									0b00001000
#define	V2_5_2_5								0b00000100
#define	V2_5									0b00000101
#define	V1_25_1_25								0b00000000
#define	V1_25									0b00000001
#define	V0_625_0_625							0b00000010
#define	V0_625									0b00000011

/* CHANNEL CONVERSION TIME REGISTER */
#define	CH_CONV_TIME_REG						0x30
#define	CHOP									0b10000000

#define	CH_STATUS_REG							0x20
#define	ADC_STATUS_REG							0x04

/* MODE REGISTER  */
#define	MODE_REG								0x38
#define	IDLE_STATE								0b00000000
#define	CONTINUOUS_CONVERTION					0b00100000
#define	SINGLE_CONVERTION						0b01000000
#define	POWER_DOWN								0b01100000
#define	ADC_ZERO_SCALE_CALIBRATION				0b10000000
#define	ADC_FULL_SCALE_CALIBRATION				0b10100000
#define	CHANNEL_ZERO_SCALE_CALIBRATION			0b11000000
#define	CHANNEL_FULL_SCALE_SYSTEM_CALIBRATION	0b11100000

#define	CLOCK_DISABLE							0b00010000
#define	DUMP_MODE								0b00001000
#define	CONTIUOUSE_READ_MODE					0b00000100
#define CONTIUOUSE_READ_MODE_WITH_DUMP			CONTIUOUSE_READ_MODE | DUMP_MODE
#define	ADC_24_BIT								0b00000010 // esle 16 bit
#define	CLAMP									0b00000001

typedef struct {

	uint8_t active;
	uint8_t channelNum;

	/* MODE */
	uint8_t state;
	uint8_t bits;
	uint8_t clamp;
	uint8_t clockEnable;
	uint8_t continuesRead;
	uint8_t dumpMode;

	/* adc channel setup  */
	uint8_t inputConfig;
	uint8_t isEnabled;
	uint8_t opt;
	uint8_t powerRange;

	/* convertion time  */
	uint8_t chopting;
	uint8_t convertionTime;

	uint8_t infoChannelStatus;
	unsigned long infoChannelZeroScale;
	unsigned long infoChannelFullScale;
	uint8_t infoChannelData;
	uint8_t infoChannelSetup;
	uint8_t infoChannelConvertionTime;
	uint8_t infoMode;

} AD773xChannel;

typedef struct {
	SPI_HANDLER_TYPE *spi;
	PORT *ADC_PORT;
	PIN ADC_CS_PIN;	//	GPIO_PIN_8	// USE YOUR PIN
	PORT *ADC_RDY_PORT; //	GPIOA  // 	 USE YOUR PORT
	PIN ADC_RDY_PIN; //		GPIO_PIN_0	// USE YOUR PIN

	/*  IO REGISTER */
	uint8_t P0Value;
	uint8_t P1Value;
	uint8_t P0Dir;
	uint8_t P1Dir;
	uint8_t ReadyPinMode;
	uint8_t ReducePowerMode;
	uint8_t Sync;
	AD773xChannel channel[8];

	/* status */
	uint8_t infoIOPORT;
	uint8_t infoRevision;
	uint8_t infoADC;
	signed long infoZeroScaleCalibration;
	signed long infoFullScaleCalibration;

} AD773x;

void setADCP0Input(AD773x *adc) {
	adc->P0Dir = P0Input;
}
void setADCP1Input(AD773x *adc) {
	adc->P1Dir = P1Input;
}
void setADCP0High(AD773x *adc) {
	adc->P0Value = P0Pin;
}
void setADCP1High(AD773x *adc) {
	adc->P1Value = P1Pin;
}
void setADCReadyPinLowInAllUnreadedData(AD773x *adc) {
	adc->ReadyPinMode = RDYFN;
}
void setADCReducePowerMode(AD773x *adc) {
	adc->ReducePowerMode = REDPWR;
}
void setADCSyncEnable(AD773x *adc) {
	adc->Sync = SYNC;
}

void setChannelEnabledInContinuesReadMode(AD773xChannel *channel) {
	channel->isEnabled = ENABLE;
}
void setChannelAllowChoping(AD773xChannel *channel) {
	channel->chopting = CHOP;
}
void setChannel24Bit(AD773xChannel *channel) {
	channel->bits = ADC_24_BIT;
}
void setChannelConvertionTime(AD773xChannel *channel, uint8_t time) {
	channel->convertionTime = time;
}
void setChannelClampEnable(AD773xChannel *channel) {
	channel->clamp = CLAMP;
}
void setChannelPowerRange(AD773xChannel *channel, uint8_t powerRangeDef) {
	channel->powerRange = powerRangeDef;
}
void setChannelClockDisable(AD773xChannel *channel) {
	channel->clockEnable = CLOCK_DISABLE;
}
void setChannelDumpEnabled(AD773xChannel *channel) {
	channel->dumpMode = DUMP_MODE;
}
void setChannelContinuesRead(AD773xChannel *channel) {
	channel->continuesRead = CONTIUOUSE_READ_MODE;
}

AD773xChannel* createChannel(AD773x *c, uint8_t chanNum);
void adc_status(AD773x *h);
signed long readAD773xValueWithDump(AD773x *h, uint8_t ch);



void spiSendByte(AD773x *h, uint8_t data) {
	SPI_SEND(h->spi, &data, sizeof(uint8_t), MAX_DELAY);
}

uint8_t spiTransferByte(AD773x *h, uint8_t data) {
	uint8_t recive = 0xFF;
	SPI_TRANSMITE(h->spi, &data, &recive, sizeof(uint8_t), MAX_DELAY);
	return recive;
}

uint8_t readAD773xByte(AD773x *h, uint8_t addr) {
	uint8_t byte = 0x00;
	ADC_SELECT(h->ADC_PORT, h->ADC_CS_PIN);
	spiSendByte(h, READSTATE | addr);
	byte = spiTransferByte(h, 0x00);
	ADC_DESELECT(h->ADC_PORT, h->ADC_CS_PIN);
	return byte;
}

void writeAD773x(AD773x *h, uint8_t addr, uint8_t value) {
	ADC_SELECT(h->ADC_PORT, h->ADC_CS_PIN);
	spiSendByte(h, addr); // to IO PORT
	spiSendByte(h, value);
	ADC_DESELECT(h->ADC_PORT, h->ADC_CS_PIN);
}

void createADC(AD773x *h, SPI_HANDLER_TYPE *spi, PORT spiPort, PIN spiCSPin,
		PORT readyPort, PIN readyPin) {

	h->spi = spi;
	h->ADC_PORT = spiPort;
	h->ADC_RDY_PORT = readyPort;
	h->ADC_CS_PIN = spiCSPin;
	h->ADC_RDY_PIN = readyPin;

	h->P0Value = 0x00;
	h->P1Value = 0x00;
	h->P0Dir = 0x00; //0b00100000;
	h->P1Dir = 0x00; //0b00010000;
	h->ReadyPinMode = 0x00;
	h->ReducePowerMode = 0x00;
	h->Sync = 0x00;

	for (uint8_t i = 0; i < 8; i++) {
		AD773xChannel *c = createChannel(h, i);
		c->active = 0;
		c->channelNum = i;
	}

}

AD773xChannel* createChannel(AD773x *c, uint8_t chanNum) {
	c->channel[chanNum].active = 1;
	c->channel[chanNum].channelNum = chanNum;
	c->channel[chanNum].inputConfig = 0x00;
	c->channel[chanNum].opt = 0x00;
	c->channel[chanNum].isEnabled = 0x00;
	c->channel[chanNum].powerRange = V2_5;

	c->channel[chanNum].chopting = CHOP;
	c->channel[chanNum].convertionTime = 60;

	c->channel[chanNum].bits = ADC_24_BIT;
	c->channel[chanNum].clamp = 0x00;
	c->channel[chanNum].clockEnable = 0x00;
	c->channel[chanNum].continuesRead = 0x00;
	c->channel[chanNum].dumpMode = 0x00;
	c->channel[chanNum].state = IDLE_STATE;

	c->channel[chanNum].infoChannelStatus = 0x00;

	return (&c->channel[chanNum]);
}

void initADC(AD773x *h) {

	ADC_SELECT(h->ADC_PORT, h->ADC_CS_PIN);
	spiSendByte(h->spi, 0x00);
	spiSendByte(h->spi, 0xFF);
	spiSendByte(h->spi, 0xFF);
	spiSendByte(h->spi, 0xFF);
	spiSendByte(h->spi, 0xFF);
	ADC_DESELECT(h->ADC_PORT, h->ADC_CS_PIN);

	// WAIT 32 CYCVLES

	writeAD773x(h, IOPORT,
			h->P0Value | h->P1Value | h->P0Dir | h->P1Dir | h->ReadyPinMode
					| h->ReducePowerMode | h->Sync /*0b00010000*/);

	adcFullScaleCalibration(h);
	adcZeroScaleCalibration(h);
	for (uint8_t i = 0; i < 8; i++) {
		initChannel(h, &h->channel[i]);
	}
}

void initChannel(AD773x *h, AD773xChannel *channel) {
	writeAD773x(h, CH_SETUP_REG | channel->channelNum,
			channel->inputConfig | channel->isEnabled | channel->powerRange
					| channel->opt);

	writeAD773x(h, CH_CONV_TIME_REG | channel->channelNum,
			channel->chopting | channel->convertionTime);

	writeAD773x(h, MODE_REG | channel->channelNum,
			IDLE_STATE | channel->clockEnable | channel->dumpMode
					| channel->continuesRead | channel->bits | channel->clamp);
	DELAY(50);

}

void startSingleConvertion(AD773x *h, AD773xChannel *channel) {

	writeAD773x(h, MODE_REG | channel->channelNum,
			SINGLE_CONVERTION | channel->clockEnable | channel->dumpMode
					| channel->continuesRead | channel->bits | channel->clamp);

}

void startContinuesConvertion(AD773x *h, AD773xChannel *channel) {
	writeAD773x(h, MODE_REG | channel->channelNum,
			CONTINUOUS_CONVERTION | channel->clockEnable | channel->dumpMode
					| channel->continuesRead | channel->bits | channel->clamp);
}

void startContinuesReadConvertion(AD773x *h, AD773xChannel *channel) {
	setChannelContinuesRead(channel);
	h->ReadyPinMode = 0x00;
	initADC(h);
	startSingleConvertion(h, channel);
}



signed long readAD773xValue(AD773x *h, uint8_t reg) {

	static unsigned char b1, b2, b3;
		static signed long adc_value;
		ADC_SELECT(h->ADC_PORT, h->ADC_CS_PIN);
		spiSendByte(h, reg);
		b1 = spiTransferByte(h, 0x00);
		b2 = spiTransferByte(h, 0x00);
		b3 = spiTransferByte(h, 0x00);
		ADC_DESELECT(h->ADC_PORT, h->ADC_CS_PIN);

		adc_value = 0;
		adc_value = b1;
		adc_value = (adc_value << 8) | b2;
		adc_value = (adc_value << 8) | b3;
		return adc_value;

}


signed long readAD773xADCValue(AD773x *h, uint8_t ch) {

	if (bitset(h->channel[ch].dumpMode, 3)) {
		return readAD773xValueWithDump(h, ch);
	}

return readAD773xValue(h, ADC_DATA_ADDR | ch);

}

signed long readAD773xValueWithDump(AD773x *h, uint8_t ch) {
	static unsigned char bt0, bt1, bt2, bt3;
	static signed long adc_value;
	ADC_SELECT(h->ADC_PORT, h->ADC_CS_PIN);
	spiSendByte(h, ADC_DATA_ADDR | ch);
	bt0 = spiTransferByte(h, 0x00);
	bt1 = spiTransferByte(h, 0x00);
	bt2 = spiTransferByte(h, 0x00);
	bt3 = spiTransferByte(h, 0x00);
	ADC_DESELECT(h->ADC_PORT, h->ADC_CS_PIN);

	adc_value = bt0;
	adc_value = (adc_value << 8) | bt1;
	adc_value = (adc_value << 8) | bt2;
	adc_value = (adc_value << 8) | bt3;

	h->channel[ch].infoChannelStatus = (adc_value & 0xFF000000);

	return adc_value & 0x00FFFFFF;
}

const char* byte_to_binary(uint8_t x) {
	static char b[9];
	b[0] = '\0';

	int z;
	for (z = 128; z > 0; z >>= 1) {
		strcat(b, ((x & z) == z) ? "1" : "0");
	}

	return b;
}

void channelZeroScaleCalibration(AD773x *h, uint8_t chan) {
	writeAD773x(h, MODE_REG | chan, CHANNEL_ZERO_SCALE_CALIBRATION );
	DELAY(0.001);
	while (IS_RDY(h->ADC_RDY_PORT, h->ADC_RDY_PIN)) {};
}

void channelFullScaleCalibration(AD773x *h, uint8_t chan) {
	writeAD773x(h, MODE_REG | chan, CHANNEL_FULL_SCALE_SYSTEM_CALIBRATION);

	DELAY(0.001);
	while (IS_RDY(h->ADC_RDY_PORT, h->ADC_RDY_PIN)) {};
}

void adcFullScaleCalibration(AD773x *h) {
	writeAD773x(h, MODE_REG,
			ADC_FULL_SCALE_CALIBRATION /*| channel->clockEnable
					| channel->dumpMode | channel->continuesRead | channel->bits
					| channel->clamp*/);

	DELAY(1);
	while (IS_RDY(h->ADC_RDY_PORT, h->ADC_RDY_PIN)) {};
}

void adcZeroScaleCalibration(AD773x *h) {
	writeAD773x(h, MODE_REG,
			ADC_ZERO_SCALE_CALIBRATION/* | channel->clockEnable
					| channel->dumpMode | channel->continuesRead | channel->bits
					| channel->clamp*/);

	DELAY(1);
	while (IS_RDY(h->ADC_RDY_PORT, h->ADC_RDY_PIN)) {};
}

unsigned long readSingleChannel(AD773x *h, AD773xChannel *channel,
		int delayTime) {
	startSingleConvertion(h, channel);
	DELAY(delayTime);
	while (IS_RDY(h->ADC_RDY_PORT, h->ADC_RDY_PIN)) {};

	return readAD773xADCValue(h, channel);
}

void adc_status(AD773x *h)	// Read the ADC Status Register
{


	h->infoIOPORT  = readAD773xByte(h, 0x40 | 0x01);
	h->infoADC = readAD773xByte(h, 0x40 | 0x04);
	h->infoZeroScaleCalibration = readAD773xValue(h, 0x06);
	h->infoFullScaleCalibration = readAD773xValue(h, 0x07);


	for(uint8_t i = 0; i < 8; i++) {
		h->channel[i].infoMode = readAD773xByte(h, 0x38 | i);
		h->channel[i].infoChannelSetup = readAD773xByte(h, 0x28 | i);
		h->channel[i].infoChannelData = readAD773xByte(h, 0x08 | i);
		h->channel[i].infoChannelConvertionTime = readAD773xByte(h, 0x30 | i);
		h->channel[i].infoChannelStatus = readAD773xByte(h, 0x20 | i);
		h->channel[i].infoChannelZeroScale = readAD773xValue(h, 0x10 | i);
		h->channel[i].infoChannelFullScale = readAD773xValue(h, 0x18 | i);
	}

}


#endif /* INC_ADC_H_ */
