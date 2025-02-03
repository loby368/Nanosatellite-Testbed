
// NAU7802 Functions (Custom) ------------------/
// NAU7802 Datasheet: https://www.nuvoton.com/resource-files/NAU7802%20Data%20Sheet%20V1.7.pdf
// https://github.com/chidx/pbd/blob/master/Src/NUC1xx-LB_002/NAU7802.c
// https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library/blob/master/src/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.cpp

uint8_t NAU7802_Reset_PowerUp() {
	//Define data buffer
	uint8_t NAU7802_PU_CTRL = 0x00;
	uint8_t buffer[2] = { NAU7802_PU_CTRL, 0 };

	// Reset
	// Set RR=1 Bit in PU_CTRL (R0x00) Register
	uint8_t  reset_PU_CTRL= 0b00000001;
	buffer[1] = reset_PU_CTRL; //power register, data to write
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);
	HAL_Delay(1); // Takes 200us

	// Enter normal opertaion
	// Set RR=0, PUA=1 and PUD=1 in PU_Control Register
	uint8_t normalMode_PU_CTRL = 0b00001110;
	buffer[1] = normalMode_PU_CTRL; //power register, data to write
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);

	// Check PWRUP mode
	uint8_t PU_CTRL_Read = 0;
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, &NAU7802_PU_CTRL, 1, 10); //Point to PWR register
	HAL_I2C_Master_Receive(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, &PU_CTRL_Read, 1, 10); //Read Register value
	uint8_t PUR_Status = (PU_CTRL_Read >> 3) & 0b00000001; //PUR is 3rd bit

	// Startup
	// Set CS=1 PU_Control Register
	uint8_t Start_PU_CTRL = 0b00111110;
	buffer[1] = Start_PU_CTRL;
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);

	return(PUR_Status);
}


void NAU7802_Config() {
	//Define data buffer
	uint8_t buffer[2] = {0, 0};

	// Gain
	uint8_t NAU7802_CTRL_1 = 0x01;
	uint8_t gain = 0b00000111; // 128x Gain Byte structure:[00 LDO GAI]
	buffer[0] = NAU7802_CTRL_1; // Register
	buffer[1] = gain; // Data
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);

	// PGA Outputbuffer Enable and PGA Chopper Disable
	uint8_t PGA_REG = 0x1B; //
	uint8_t PGA_Config= 0b00100001;
	buffer[0] = PGA_REG; // Register
	buffer[1] = PGA_Config; // Data
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);

	// PGA Output Bypass Capacitor
	uint8_t PWR_CTRL_REG = 0x1C; // Power Control Register
	uint8_t PWR_CTRL_Config = 0b10000000; // PGA_CAP_EN
	buffer[0] = PWR_CTRL_REG; // Register
	buffer[1] = PWR_CTRL_Config; // Data
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);

	// PGA Outputbuffer and PGACHPS
	uint8_t ADC_CTRL_REG = 0x15; //
	uint8_t ADC_CTRL = 0b00110001; // Last 1 is REG_CHP (EXPERIMENAL)
	buffer[0] = ADC_CTRL_REG; // Register
	buffer[1] = ADC_CTRL; // Data
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);

	// Sampling Rate and Internal Calibration and ###CHANNEL SELECT###
	uint8_t NAU7802_CTRL_2 = 0x02;
	uint8_t SPS = 0b01110100; // 320 SPS
	buffer[0] = NAU7802_CTRL_2; // Register
	buffer[1] = SPS; // Data
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);
	HAL_Delay(10);

	// Gain Calibration
	uint8_t gain_Calibration = 0b01110111; // 320 SPS
	buffer[0] = NAU7802_CTRL_2; // Register
	buffer[1] = gain_Calibration; // Data
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);
	HAL_Delay(10);

	// Offset Calibration
	uint8_t offset_Calibration = 0b01110100; // 320 SPS
	buffer[0] = NAU7802_CTRL_2; // Register
	buffer[1] = offset_Calibration; // Data
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);
	HAL_Delay(10);

}


int NAU7802_ReadVoltage() {

		uint8_t NAU7802_ADCO_B2 = 0x12;
		uint8_t ADCO_B0_Read;
		uint8_t ADCO_B1_Read;
		uint8_t ADCO_B2_Read;

		// Point to ADC Register
		HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, &NAU7802_ADCO_B2, 1, 10);

		// Read Register
		HAL_I2C_Master_Receive(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1,	&ADCO_B2_Read, 1, 10);
		HAL_I2C_Master_Receive(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1,	&ADCO_B1_Read, 1, 10);
		HAL_I2C_Master_Receive(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1,	&ADCO_B0_Read, 1, 10);

		uint32_t Read_Voltage_U32 = (ADCO_B2_Read << 16) | (ADCO_B1_Read << 8) | ADCO_B0_Read;

		// 24-bit number, sign bit resides on bit 23.
		// Shift and cast to a signed int32_t to recover sign
		int32_t Read_Voltage_I32_Shifted = (int32_t) (Read_Voltage_U32 << 8);
		// shift the number back right to recover magnitude
		int32_t Read_Voltage_I32 = (Read_Voltage_I32_Shifted >> 8);
		return (Read_Voltage_I32);

}




int NAU7802_ReadVoltage_Average(uint8_t averageAmount) {

	uint8_t NAU7802_ADCO_B2 = 0x12;

	uint8_t ADCO_B0_Read;
	uint8_t ADCO_B1_Read;
	uint8_t ADCO_B2_Read;

	uint32_t Read_Voltage_U32_Sum = 0;
	int i;

	for (i=0; i<averageAmount; i++){
		// Point to ADC Register
		HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, &NAU7802_ADCO_B2, 1, 10); //Point to PWR register

		// Read Register
		HAL_I2C_Master_Receive(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, &ADCO_B2_Read, 1, 10);
		HAL_I2C_Master_Receive(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, &ADCO_B1_Read, 1, 10);
		HAL_I2C_Master_Receive(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, &ADCO_B0_Read, 1, 10);

		Read_Voltage_U32_Sum += (ADCO_B2_Read << 16) | (ADCO_B1_Read << 8) | ADCO_B0_Read;
	}

	uint32_t Read_Voltage_U32 = Read_Voltage_U32_Sum / i;

	// 24-bit number, sign bit resides on bit 23.
	// Shift and cast to a signed int32_t to recover sign
	int32_t Read_Voltage_I32_Shifted = (int32_t) (Read_Voltage_U32 << 8);
	// shift the number back right to recover magnitude
	int32_t Read_Voltage_I32 = (Read_Voltage_I32_Shifted >> 8);

	return (Read_Voltage_I32);
}



uint8_t NAU7802_ADC_Ready(){
	uint8_t NAU7802_PU_CTRL = 0x00;
	HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, &NAU7802_PU_CTRL, 1, 10); //Point to PWR register
	uint8_t ADC_Ready = 0;
	HAL_I2C_Master_Receive(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, &ADC_Ready, 1, 10);
	ADC_Ready = ADC_Ready >> 5 | 0b00000000;
	return(ADC_Ready);
}








/* Call Initialisation Fucntions */
// NAU7802 Power Up ----------------------------------------------------/
printf("\n\n**NAU7802 Power Up**\n");
NAU7802_Reset_PowerUp();
NAU7802_Config();






// Function Calls in main.c
// Read Voltage and Switch between channel 1 and Channel 2 on NAU

// Define Buffer
uint8_t buffer[2] = {0, 0};
uint8_t NAU7802_CTRL_2 = 0x02;


// Channel A
uint8_t channel1_Data = 0b01110000; // 320 SPS - No Calibration
buffer[0] = NAU7802_CTRL_2; // Register
buffer[1] = channel1_Data; // Data
HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);
HAL_Delay(10);
if (NAU7802_ADC_Ready() == 1)
{
	//voltageA = (NAU7802_ReadVoltage() - voltage_init) / 10000;
	voltageA = (NAU7802_ReadVoltage_Average(32) - voltage_init) / 10000; //10Hz
	//printf("Voltage A: %f\n", voltageA);
}


// Channel B
uint8_t channel2_Data = 0b11110000; // 320 SPS - No Calibration
buffer[0] = NAU7802_CTRL_2; // Register
buffer[1] = channel2_Data; // Data
HAL_I2C_Master_Transmit(NAU7802_I2C_PORT, NAU7802_I2C_ADDR << 1, buffer, 2, 10);
HAL_Delay(10);
if (NAU7802_ADC_Ready() == 1)
{
	//voltageB = (NAU7802_ReadVoltage() - voltage_init) / 10000;
	voltageB = (NAU7802_ReadVoltage_Average(32) - voltage_init) / 10000; //10Hz
	//printf("voltage: %f\n", voltageB);
}


