/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "icache.h"
#include "gpio.h"
#include "app_tof.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l4cd_api.h" // VL53L4CD 传感器 API
#include "custom_ranging_sensor.h" // 自定义测距传感器头文件
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int status;// 存储操作状态
volatile int IntCount;// 中断计数器
uint8_t p_data_ready;// 数据准备标志
uint16_t dev, sensor_id;// 设备 ID 和传感器 ID
VL53L4CD_ResultsData_t results;		/* Results data from VL53L4CD */
VL53L4CD_Version_t sw_version;	/* Driver version */


/* Below code has to be updated depending on customer setup parameters */
/* Start: Measure liquid level application*/
/* Consider X indicator levels and define a structure to store level, expected result and compensation value*/
// 液位标志的数量，定义有 10 个级别
#define MAX_LABEL 10 /* Number of level indicators – To be updated depending on the number of levels in the OffGainVal.csv file */
typedef struct ogalgo_data
{
  uint8_t level ;// 液位级别
  uint16_t  expected_res;// 预期测量结果
  uint16_t  og_val;// 补偿值
}ogalgo_data;

/* Design a lookup table with a structure of compensate value data received from GUI once characterization is done*/
/* First column : indicator level , Second column: Expected result , Third column: compensate value*/
/* Example of values given below - To be updated with values from the GUI*/

/* 使用测量补偿数据初始化查找表 */
ogalgo_data ogalgo_data_inst[MAX_LABEL]={
		{ 9, 21, 4 },
		{ 8, 42, 4 },
		{ 7, 63, 2 },
		{ 6, 84, 1 },
		{ 5, 105, 5 },
		{ 4, 126, 14 },
		{ 3, 147, 25 },
		{ 2, 168, 30 },
		{ 1, 189, 20 },
	};


/* Ranging 函数的原型声明 */
int Ranging(Dev_t dev,uint16_t *ouputranging);

/* 开启算法标志 */
uint8_t algo_enable=1; /* algo_enable=1 to apply the lookup table - algo_enable=0 to not apply */
uint16_t totaldistance=270; /* 传感器到容器底部的高度，单位为毫米 */
uint16_t meanranging=0;// 平均测距值
uint16_t rangevalue_out=0;// 最终补偿后的测距值
uint16_t invalid_range=999;// 无效测距值


void get_data_by_polling(Dev_t dev);// 数据轮询获取函数
	

/* Ranging 函数 - 获取测量数据并计算平均值 */
int Ranging(Dev_t dev,uint16_t *ouputranging)
{
	uint8_t cnt=0;uint16_t mean_distance=0;uint16_t meanranging=0;
	 /* 进行 10 次测量，取平均值 */
	for (cnt = 0; cnt < 10;)
	{
		status = VL53L4CD_CheckForDataReady(dev, &p_data_ready);
//		printf("status11=%d,data_ready=%d\n",status,p_data_ready);
		HAL_Delay(3);
		if (p_data_ready)
		{
			VL53L4CD_ClearInterrupt(dev);
			VL53L4CD_GetResult(dev, &results);


				if(results.range_status ==0)
				{
					mean_distance+=results.distance_mm;
					cnt++;
					HAL_Delay(3);
				}


		}
	}
//	printf("cnt=%d\n",cnt);
	meanranging=mean_distance/10;// 计算平均值
	*ouputranging =meanranging;
	return status;
}

/* 液位测量误差补偿函数 */
void Liquidlevelmeasureerrorcomponsate(uint16_t rangevalue,uint16_t *rangevalue_out)
{

		uint16_t pos=0;
		uint16_t i=0; uint16_t value;

		for (i=0;i<MAX_LABEL;i++)
		{
			if  (rangevalue <totaldistance+5) //5 buffer
			{
				if (rangevalue < ogalgo_data_inst[i].expected_res)
				{

					//Check first
					if (i >0)
					{
						value = (ogalgo_data_inst[i].expected_res + ogalgo_data_inst[i-1].expected_res)/2;
					}
					else
						value = ogalgo_data_inst[i].expected_res;
					if(rangevalue <= value)
					{
						/* First position */
						if (i ==0 )
						{
							pos=1;
							break;
						}
						pos=1+(i-1); // adding one bcz position starts from 1
						break;
					}
					else
					{
						pos=1+i;
						break;
					}
				}
				else
				{
					if (rangevalue > totaldistance+5)
						pos=99;
					else
						pos=6;  // Need to find which one is max OG value or store it directly here

				}
			}
			else
				pos=99;

		}

		 /* 根据位置补偿测量值 */
		switch(pos)
		{
			case 1:
					*rangevalue_out=rangevalue + ogalgo_data_inst[pos].og_val; // Category Underranging,
				break;
			case 2:
					*rangevalue_out=rangevalue + ogalgo_data_inst[pos].og_val; // Category Underranging, 30mm average b/w C3&C2
				break;
			case 3:
					*rangevalue_out=rangevalue + ogalgo_data_inst[pos].og_val; // Category Underranging, 25mm average b/w C4&C3
				break;
			case 4:
					*rangevalue_out=rangevalue + ogalgo_data_inst[pos].og_val; // Category Underranging, 12mm average b/w C2&C2
				break;
			case 5:
					*rangevalue_out=rangevalue - ogalgo_data_inst[pos].og_val; // Category Overranging, 4mm average b/w C2&C2
				break;
			case 6:
					*rangevalue_out=(uint16_t)(rangevalue - ogalgo_data_inst[pos].og_val); // Category Overranging,

				break;
			case 7:
					*rangevalue_out=(uint16_t)(rangevalue - ogalgo_data_inst[pos].og_val); //Category Overranging,
				break;
			case 8:
					*rangevalue_out=(uint16_t)(rangevalue - ogalgo_data_inst[pos].og_val); // Category Overranging,

				break;
			case 9:
					*rangevalue_out=(uint16_t)(rangevalue - ogalgo_data_inst[pos].og_val); // Category Overranging,
				break;
			default:
					printf("Valid water level not found\n");
			}

}
/* End:Measure liquid level application */
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_TOF_Init();
  /* USER CODE BEGIN 2 */
	/* Print the software version */
	status = VL53L4CD_GetSWVersion(&sw_version);
	printf("Starting VL53L4CD driver version %u.%u.%u\n",
			sw_version.major,
			sw_version.minor,
			sw_version.build);
			
	
	
  VL53L4CD_IO_t              IOCtx;
  uint32_t                   id;
  static VL53L4CD_Object_t   VL53L4CDObj;

  /* Configure the ranging sensor driver */
  IOCtx.Address     = RANGING_SENSOR_VL53L4CD_ADDRESS;
  IOCtx.Init        = CUSTOM_VL53L4CD_I2C_INIT;
  IOCtx.DeInit      = CUSTOM_VL53L4CD_I2C_DEINIT;
  IOCtx.WriteReg    = CUSTOM_VL53L4CD_I2C_WRITEREG;
  IOCtx.ReadReg     = CUSTOM_VL53L4CD_I2C_READREG;
  IOCtx.GetTick     = BSP_GetTick;	
	

	VL53L4CD_RegisterBusIO(&VL53L4CDObj, &IOCtx);
	/* Check if VL53L4CD is connected. 0xebaa is the sensor id. */
	status = VL53L4CD_ReadID(&VL53L4CDObj, &id);
	printf("VL53L4CD_ID=0x%x\n",id);
	if(status || (id != 0xebaa))
	{
		printf("VL53L4CD not detected\n");
		return status;
	}	
	
	status = VL53L4CD_SetRangeTiming(&VL53L4CDObj, 100, 0); //100ms			
	printf("status=%d\n",status);
			
	status = VL53L4CD_SensorInit(&VL53L4CDObj);		// 初始化传感器

	/*******************************************************************
	 * User can chose a configuration below for a specific use case
	 ******************************************************************/

	// #define VL53L4CD_CONFIGURATION_LOW_POWER
#ifdef VL53L4CD_CONFIGURATION_LOW_POWER
	/* The examples below allows using the low power mode. The InterMeasurement
	 * value defines the measurements period, and needs to be greater than the
	 * TimingBudget.
	 */

	// Timing budget of 50ms, and ranging period 100ms (50% active ranging and
	// 50% low power)
	// status = VL53L4CD_SetRangeTiming(50, 100);

	// Timing budget of 100ms, and ranging period 1000ms (10% active ranging and
	// 90% low power)
	status = VL53L4CD_SetRangeTiming(dev, 100, 1000);
#endif

	// #define VL53L4CD_CONFIGURATION_HIGH_ACCURACY
#ifdef VL53L4CD_CONFIGURATION_HIGH_ACCURACY
	/* Program the highest possible TimingBudget, without enabling the
	 * low power mode. This should give the best accuracy */
	status = VL53L4CD_SetRangeTiming(dev, 200, 0);
#endif

	// #define VL53L4CD_CONFIGURATION_FAST_RANGING
#ifdef VL53L4CD_CONFIGURATION_FAST_RANGING
	/* Program the lowest possible TimingBudget, without enabling the
	 * low power mode. This gives the highest ranging frequency (100Hz) */
	status = VL53L4CD_SetRangeTiming(dev, 10, 0);
#endif
	/*******************************************************************
	 * User can chose a configuration below for a specific use case
	 ******************************************************************/

	// #define VL53L4CD_CONFIGURATION_LOW_POWER
#ifdef VL53L4CD_CONFIGURATION_LOW_POWER
	/* The examples below allows using the low power mode. The InterMeasurement
	 * value defines the measurements period, and needs to be greater than the
	 * TimingBudget.
	 */

	// Timing budget of 50ms, and ranging period 100ms (50% active ranging and
	// 50% low power)
	// status = VL53L4CD_SetRangeTiming(50, 100);

	// Timing budget of 100ms, and ranging period 1000ms (10% active ranging and
	// 90% low power)
	status = VL53L4CD_SetRangeTiming(dev, 100, 1000);
#endif

	// #define VL53L4CD_CONFIGURATION_HIGH_ACCURACY
#ifdef VL53L4CD_CONFIGURATION_HIGH_ACCURACY
	/* Program the highest possible TimingBudget, without enabling the
	 * low power mode. This should give the best accuracy */
	status = VL53L4CD_SetRangeTiming(dev, 200, 0);
#endif

	// #define VL53L4CD_CONFIGURATION_FAST_RANGING
#ifdef VL53L4CD_CONFIGURATION_FAST_RANGING
	/* Program the lowest possible TimingBudget, without enabling the
	 * low power mode. This gives the highest ranging frequency (100Hz) */
	status = VL53L4CD_SetRangeTiming(dev, 10, 0);
#endif






	
	status = VL53L4CD_StartRanging(&VL53L4CDObj);		// 开始测距
			
			
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		get_data_by_polling(&VL53L4CDObj);// 轮询获取数据
    /* USER CODE END WHILE */

  MX_TOF_Process();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* USER CODE BEGIN 4 */

/* 通过轮询获取数据 */
void get_data_by_polling(Dev_t dev){
	do
	{
		/* Liquid level Measure application */
		status=Ranging(dev,&meanranging);
			 if (!status)
			 {
				 if (algo_enable == 1)
				 {
					 Liquidlevelmeasureerrorcomponsate(meanranging,&rangevalue_out);
					 //printf("rg=%d %d\n",meanranging, rangevalue_out);
					 if (rangevalue_out>0)
					 	 printf("Liquid level is = %d mm\n",rangevalue_out);
					 else
						 printf("Invalid range is = %d mm\n",invalid_range);
				 }
				 else
					 printf("Mean ranging is = %d mm\n",meanranging);
			 }

	}
	while(1);
}


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
#ifdef USE_FULL_ASSERT
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
