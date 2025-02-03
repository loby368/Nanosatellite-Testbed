/* USER CODE BEGIN Header */
/*
 * Version 8: Ground Station Comms and New Brass RW
 * Author: Loui Byrne
 * Date: 11 August 2023
 * Details: Masters Thesis Project - Developing an ADCS CubeSat Testbed
 * Location: University College Dublin, Ireland
 * USER SETUP STEPS:
 * "Project Properties > C/C++ Build > Settings > MCU Settings > Check Box: "Use float with printf from newlib-nano"
 *
 */

/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Custom Includes --------------------------------------------*/
#include <stdio.h> 					// Include for printf
#include <Embedded_Coder_Test1.h> 	// Include Simulink Algorithm Header
#include <rtwtypes.h> 				// Include Simulink C code types header

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// I2C Slave Addresses  ---------------------------------------*/
#define BNO055_I2C_ADDR	0x28
#define ARDUINO_NANO_ADDR 0x03

// BNO055 Regiser Addresses -----------------------------------*/
#define BNO055_OPR_MODE 0x3D
#define BNO055_UNIT_SEL 0x3B
#define BNO055_CALIB_STAT 0x35
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_EUL_HEADING_MSB 0x1B

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

// Reconfigure printf to ST-Link Serial Debug ------------------*/
// See https://community.st.com/t5/stm32-mcus/how-to-redirect-the-printf-function-to-a-uart-for-debug-messages/ta-p/49865
// MUST ENABLE: "Project Properties > C/C++ Build > Settings > MCU Settings > Check Box: "Use float with printf from newlib-nano"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE
int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE
int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
	// ST-Link Serial - USART 2 Default Pins
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY); // [Defnied in CubeMx] - USART2 = ST-Link
	return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	// Declare Global Variables ----------------------------------------------*/
	//uint8_t CAN_Rx_Buffer[4];
	//CAN_RxHeaderTypeDef pRxHeader;
	//int32_t register_storage[5];
	int SerialRx_Counter = 0;
	uint8_t STM32_Register_Adddress;
	int32_t STM32_Regiter_Data;

	// BNO055 Variables
	uint8_t Operating_Mode = 0; 				// IMU Operating Mode
	uint8_t Register_Calibration_Status = 0; 	// Store Calibration Status
	uint8_t Mag_Calibration_Status = 0; 		// Store Magnetometer Calibration Status
	uint8_t Euler_Heading_U8[2]; 				// Unsigned byte
	int16_t Euler_Heading_S16; 					// Signed 2 bytes
	float LSB_to_Rad = 1 / 900.0; 				// BNO055 Datasheet pg.35 - LSB to Radians
	float Euler_Heading_Rad = 0; 				// Store current inertial Euler Heading as float
	float Initial_Euler_Heading_Rad = 0; 		// Initial Inertial Euler Heading in Radians as float

	// Motor Variables
	//float Heading_Difference = 0; 				// Difference between clock cycle headings - {replaced with Simulink}
	//float Heading_to_RPM = 0; 					// [Tunable parameter] - {replaced with Simulink}
	float Motor_Speed_RPM = 0; 					// Current Motor Speed sent to mc_api
	int Ramp_Time = 10; 						// [Tunable parameter] - 10ms should be fine?

	// Nano Variables
	float LoadCellRead_Initial[2] = { 0, 0 };		// Initial Load Cell readings in Newtons
	float calibrationFactor = 3040000.0; 		// [Tunable parameter] - Converts Voltage to Newtons
	float LoadCellRead[2] = { 0, 0 }; 			// Current Load Cell Readings in Newtons

	// I2C Address Definitions -----------------------------------------------*/
	I2C_HandleTypeDef *BNO055_I2C_PORT = &hi2c1; 	// [Defnied in CubeMx] - I2C 1
	I2C_HandleTypeDef *ARDUINO_NANO_PORT = &hi2c1; 	// [Defnied in CubeMx] - I2C 1

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	// Simulink Initialisation -----------------------------------------------*/
	Embedded_Coder_Test1_initialize(); // Simulink Embedded Coder Init Function

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_MotorControl_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

	// Arduino Serial Debug Functions ----------------------------------------------*/
	void Arduino_SerialTx() {

		// IMU Data
		int IMU_S32 = (Euler_Heading_Rad*10000); //Scale by 10,000 to preserve 4 decimal places
		uint8_t IMU[4]; // Element 0 is LSB of IMU_S32, Element 3 is MSB
		IMU[0] = IMU_S32 & 0xFF; // LSB (0xFF = 0b11111111)
		IMU[1] = (IMU_S32 >> 8) & 0xFF;
		IMU[2] = (IMU_S32 >> 16) & 0xFF;
		IMU[3] = (IMU_S32 >> 24) & 0xFF; // MSB

		// Load Cell 1 Data
		int LC1_S32 = LoadCellRead[0]*100000; //Scale by 100,000 to preserve 5 decimal places
		uint8_t LC1[4]; // Element 0 is LSB of LC1_S32, Element 3 is MSB
		LC1[0] = LC1_S32 & 0xFF; // LSB (0xFF = 0b11111111)
		LC1[1] = (LC1_S32 >> 8) & 0xFF;
		LC1[2] = (LC1_S32 >> 16) & 0xFF;
		LC1[3] = (LC1_S32 >> 24) & 0xFF; // MSB

		// Load Cell 2 Data
		int LC2_S32 = LoadCellRead[1]*100000; //Scale by 100,000 to preserve 5 decimal places
		uint8_t LC2[4]; // Element 0 is LSB of LC1_S32, Element 3 is MSB
		LC2[0] = LC2_S32 & 0xFF; // LSB (0xFF = 0b11111111)
		LC2[1] = (LC2_S32 >> 8) & 0xFF;
		LC2[2] = (LC2_S32 >> 16) & 0xFF;
		LC2[3] = (LC2_S32 >> 24) & 0xFF; // MSB

		// Reaction Wheel Speed Data
		int RW_Speed_S32 = (Motor_Speed_RPM); // No decimal points
		uint8_t RW_Speed[4]; // Element 0 is LSB of LC1_S32, Element 3 is MSB
		RW_Speed[0] = RW_Speed_S32 & 0xFF; // LSB (0xFF = 0b11111111)
		RW_Speed[1] = (RW_Speed_S32 >> 8) & 0xFF;
		RW_Speed[2] = (RW_Speed_S32 >> 16) & 0xFF;
		RW_Speed[3] = (RW_Speed_S32 >> 24) & 0xFF; // MSB


		uint8_t TxBuffer_Serial_Debug[16] = { IMU[0], IMU[1], IMU[2], IMU[3], LC1[0], LC1[1],
				LC1[2], LC1[3], LC2[0], LC2[1], LC2[2], LC2[3], RW_Speed[0], RW_Speed[1],
				RW_Speed[2], RW_Speed[3] }; // 16 bytes holding 4 uint32_t (Little Edian LSB stored First)
		HAL_UART_Transmit(&huart1, TxBuffer_Serial_Debug, 16, 10);

	}

	// Arduino Serial Uplink Functions ----------------------------------------------*/
	// Call this every X loop cycles or else every 5 seconds
	// Call once every 72MHz cycles to get 1 call per second - When counter = 72e6)
	void Arduino_SerialRx() {
		SerialRx_Counter++;
		printf("\nSerialRx_Counter = %d\n", SerialRx_Counter);
		int time = 1; //Seconds
		int clockFreq = 72;
		int numSteps = time*clockFreq;
		if (SerialRx_Counter % numSteps == 0){
			printf("\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n");
			printf("\nSerial Uplink Activated, SerialRx_Counter = %d\n", SerialRx_Counter);
			uint8_t RxBuffer[5]; // 8 bit register address and 4 x bytes = int32 register value
			HAL_UART_Receive(&huart1, RxBuffer, 5, 10);
			// Decode RxBuffer
			STM32_Register_Adddress = RxBuffer[0]; // Should be the first byte that was sent - Byte 1
			STM32_Regiter_Data = RxBuffer[4] << 24 | RxBuffer[3] << 16 | RxBuffer[2] << 8 | RxBuffer[1]; // Byte 2:5
			printf("\nRegister Address = %d\n", STM32_Register_Adddress);
			printf("\nSTM32_Regiter_Data = %d\n", (int)STM32_Regiter_Data);
			printf("\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n*\n");
		}
	}

	// BNO055 Functions ------------------------------------------------------*/
	// Inspired by: https://github.com/ivyknob/bno055_stm32/tree/master
	// BNO055 Datasheet: https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
	// Read from a BNO055 register
	void BNO055_Write(uint8_t reg, uint8_t data) {
		uint8_t txdata[2] = { reg, data };
		uint8_t statusTx;
		statusTx = HAL_I2C_Master_Transmit(BNO055_I2C_PORT,
		BNO055_I2C_ADDR << 1, txdata, sizeof(txdata), 10);
		if (statusTx != HAL_OK) {
			printf("\n### Error Tx: BNO055\n");
		}
		return;
	}

	// Write to BNO055 Register
	void BNO055_Read(uint8_t reg, uint8_t *data, uint8_t len) {
		uint8_t statusTx;
		uint8_t statusRx;
		statusTx = HAL_I2C_Master_Transmit(BNO055_I2C_PORT,
		BNO055_I2C_ADDR << 1, &reg, 1, 10);
		statusRx = HAL_I2C_Master_Receive(BNO055_I2C_PORT, BNO055_I2C_ADDR << 1,
				data, len, 10);
		if (statusTx != HAL_OK) {
			printf("\n### Error Tx: BNO055\n");
		}
		if (statusRx != HAL_OK) {
			printf("\n### Error Rx: BNO055\n");
		}
		return;
	}

	// Read Euler heading register as 2 consecutive signed bytes
	void Read_Euler_Heading() {
		for (int i = 0; i < 2; i++) {
			BNO055_Read(BNO055_EUL_HEADING_LSB + i, &Euler_Heading_U8[i], 1);
		}
		Euler_Heading_S16 = (Euler_Heading_U8[1] << 8) | Euler_Heading_U8[0]; // Units = 1/900 radians
		Euler_Heading_Rad = Euler_Heading_S16 * LSB_to_Rad; // Convert to Radians
		printf(" IMU: %.3f \n", Euler_Heading_Rad);
	}

	// Initialisation configuration to put BNO055 into best performing mode for the testbed
	void BNO055_Init() {
		printf("\n*** BNO055 Initialisation ***\n");

		// Enter CONFIG Mode ---------------------------*/
		printf("\n CONFIG MODE\n");
		BNO055_Write(BNO055_OPR_MODE, 0); // Enter Config Mode (0b0000)
		HAL_Delay(19); //Time taken to to switch to CONFIG Mode

		BNO055_Read(BNO055_OPR_MODE, &Operating_Mode, 1); //Check Operating Mode (Data Sheet pg.21)
		printf(" Operating_Mode: %d \n", Operating_Mode);

		// Units Selection -----------------------------*/
		BNO055_Write(BNO055_UNIT_SEL, 0b0110); // Heading (Rad) Angular Rate (Rad/s) (Data Sheet pg.30)

		// Mode Select ---------------------------------*/
		printf("\n NDOF MODE\n");
		BNO055_Write(BNO055_OPR_MODE, 0b1100); // Enter NDOF Mode (0b1100) [Fast Mag Calibration]
		HAL_Delay(7); //Time to switch to Other Modes
		// Check Mode
		BNO055_Read(BNO055_OPR_MODE, &Operating_Mode, 1); //Check Operating Mode
		HAL_Delay(7); //Time to switch to Other Modes
		printf(" Operating_Mode: %d \n", Operating_Mode); // (Data Sheet pg.21)

		// Calibrate IMU ------------------------------*/
		// Calibration Status is the first 2 LSB in BNO055_CALIB_STAT (Data Sheet pg.67)
		// 0b00000000 OR 0x0 = Uncalibrated
		// 0b00000011 OR 0x3 = Calibrated
		BNO055_Read(BNO055_CALIB_STAT, &Register_Calibration_Status, 1);
		Mag_Calibration_Status = Register_Calibration_Status & 0b00000011; //Bitmask 6 MSB and keep 2 LSB
		printf("\n Mag_Calibration_Status: %d \n", Mag_Calibration_Status);
		if (Mag_Calibration_Status != 3) {
			while (Mag_Calibration_Status != 3) {
				BNO055_Read(BNO055_CALIB_STAT, &Register_Calibration_Status, 1);
				Mag_Calibration_Status = Register_Calibration_Status
						& 0b00000011; //Bitmask 6 MSB and keep 2 LSB
				if (Mag_Calibration_Status == 0) {
					printf(
							"\n CALIBRATING IMU - Please Shake IMU to Calibrate\n");
				}
				HAL_Delay(10);
			}
		}
		if (Mag_Calibration_Status == 3) {
			printf("\n SUCCESS - IMU Calibrated\n");
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); // ON Notify that Magnetometer is Calibrated
		}

		// Wait for user to place IMU into stable position to take initial headings
		printf("\n Put IMU on Table\n\n Wait 5 Seconds\n");
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); // OFF
		printf(" .\n");
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); // ON
		printf(" .\n");
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); // OFF
		printf(" .\n");
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); // ON
		printf(" .\n");
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); // OFF
		printf(" .\n\n");
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); // ON


		// Take initial heading
		Read_Euler_Heading();
		Initial_Euler_Heading_Rad = Euler_Heading_Rad;
	}

	// Arduino Nano Functions ------------------------------------------------*/

	// Read Avearge Raw Voltage from Load cell 1
	float Nano_ReadAvg_1(float samples) {
		int sum = 0;
		for (int i = 0; i < samples; i++) {
			int32_t LoadCellRead_Raw_S32[2];
			HAL_I2C_Master_Receive(ARDUINO_NANO_PORT, ARDUINO_NANO_ADDR << 1,
					LoadCellRead_Raw_S32, 8, 100);
			sum += LoadCellRead_Raw_S32[0];
		}
		float average = sum / samples;
		float averageNewtons = average / calibrationFactor;
		return averageNewtons;
	}

	// Read Avearge Raw Voltage from Load cell 2
	float Nano_ReadAvg_2(float samples) {
		int sum = 0;
		for (int i = 0; i < samples; i++) {
			int32_t LoadCellRead_Raw_S32[2];
			HAL_I2C_Master_Receive(ARDUINO_NANO_PORT, ARDUINO_NANO_ADDR << 1,
					LoadCellRead_Raw_S32, 8, 100);
			sum += LoadCellRead_Raw_S32[1];
		}
		float average = sum / samples;
		float averageNewtons = average / calibrationFactor;
		return averageNewtons;
	}

	// Read Function to convert load cell Voltage to Newtons
	void Read_Load_Cells(int samples) {
		LoadCellRead[0] = (Nano_ReadAvg_1(samples) - LoadCellRead_Initial[0]);
		LoadCellRead[1] = (Nano_ReadAvg_2(samples) - LoadCellRead_Initial[1]);
		printf("\n Load Cells: %0.4f %0.4f ", LoadCellRead[0], LoadCellRead[1]); //Only The first 4 decimal places are useful!
	}

	// Take iniital Load Cell raw voltage readings
	void Nano_Init() {
		printf("\n*** Arduino Nano Initialisation ***\n");
		LoadCellRead_Initial[0] = Nano_ReadAvg_1(100);
		LoadCellRead_Initial[1] = Nano_ReadAvg_2(100);
	}

	// Motor Functions ------------------------------------------------------------*/

	// Initialise Motor by aknowledging any faults and putting it into START mode
	void Motor_Init() {
		MC_AcknowledgeFaultMotor1();
		MC_ProgramSpeedRampMotor1(0, 0);
		MC_StartMotor1();
		printf("\n\n*** Motor Initialisation ***\n");
	}

	// Write Simulink motor speed output to mc_api {replace speed limits and deadzone with Simulink}
	void Simulink_Motor_Control(float Motor_Speed_RPM) {

		// Define Motor Performace Parameters {replace with Simulink}
		float Motor_Speed_Limit = 3000;
		float Motor_Dead_Zone = 0;
		Ramp_Time = 1; // [Tunable Parameter]

		// Motor Control
		if (Motor_Speed_RPM > Motor_Speed_Limit) { //Speed Limit
			Motor_Speed_RPM = Motor_Speed_Limit;
		}
		if (Motor_Speed_RPM < -Motor_Speed_Limit) { //Speed Limit
			Motor_Speed_RPM = -Motor_Speed_Limit;
		}
		if (Motor_Speed_RPM < Motor_Dead_Zone && Motor_Speed_RPM > 0) { //Dead Zone Filter
			Motor_Speed_RPM = Motor_Dead_Zone;
		}
		if (Motor_Speed_RPM < 0 && Motor_Speed_RPM > -Motor_Dead_Zone) { //Dead Zone Filter
			Motor_Speed_RPM = -Motor_Dead_Zone;
		}

		MC_AcknowledgeFaultMotor1(); // Aknowledge any Faults (Usually Overcurrent faults)
		MC_StartMotor1(); // Restart motor if it has faulted

		MC_ProgramSpeedRampMotor1_F(Motor_Speed_RPM, Ramp_Time); //Speed is rpm
		// Previously had: MC_ProgramSpeedRampMotor1(Motor_Speed_RPM / 6, Ramp_Time); //Speed is in 1/6 rpm
		HAL_Delay(Ramp_Time);
		printf("Motor Speed in RPM: %.3f    Ramp Time (ms): %d\n",
				Motor_Speed_RPM, Ramp_Time);

	}


//	void Can_Recieve(){
//		//HAL_CAN_Start(&hcan);
//		HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &pRxHeader, CAN_Rx_Buffer);
//		//HAL_CAN_Stop(&hcan);
//		printf("CAN Message: %d %d %d %d", CAN_Rx_Buffer[0], CAN_Rx_Buffer[1], CAN_Rx_Buffer[2], CAN_Rx_Buffer[3]);
//	}

	/* Run Initialisation Functions -----------------------------------------------*/
	printf("\n-------- STM32 Initialisation Begin --------\n\n");
	BNO055_Init();
	Nano_Init();
	Motor_Init();
	printf("\n\n-------- STM32 Initialisation Complete -----\n\n");
	HAL_Delay(3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {

		/* Read Load cells --------------------------------------------------------*/
		Read_Load_Cells(2);
		rtU.Load_Cell_1_Input = LoadCellRead[0];//- LoadCellRead_Initial[0] / calibrationFactor;
		rtU.Load_Cell_2_Input = LoadCellRead[1];//- LoadCellRead_Initial[1] / calibrationFactor;

		// Read IMU ---------------------------------------------------------------*/
		Read_Euler_Heading();
		rtU.Euler_Heading_Input = Euler_Heading_Rad - Initial_Euler_Heading_Rad;

		// Simulink Algorithm Timestep --------------------------------------------*/
		Embedded_Coder_Test1_step();

		// Motor Control ----------------------------------------------------------*/
		Motor_Speed_RPM = rtY.Motor_Speed_Output;
		Simulink_Motor_Control(Motor_Speed_RPM);

		// MKR1000 Serial Debug Output --------------------------------------------*/
		Arduino_SerialTx();
		Arduino_SerialRx();

		// CAN Bus Uplink from MKR1000 --------------------------------------------*/
		//Can_Recieve();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 4;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ((PWM_PERIOD_CYCLES) / 4);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.BreakFilter = 3;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_LOW;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = M1_HALL_TIM_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_HALL_IC_FILTER;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M1_PWM_EN_U_Pin|M1_PWM_EN_V_Pin|M1_PWM_EN_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_PWM_EN_U_Pin M1_PWM_EN_V_Pin M1_PWM_EN_W_Pin */
  GPIO_InitStruct.Pin = M1_PWM_EN_U_Pin|M1_PWM_EN_V_Pin|M1_PWM_EN_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
