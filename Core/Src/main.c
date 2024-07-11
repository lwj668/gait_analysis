/* USER CODE BEGIN Header */
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
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "Delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#ifdef __GNUC__
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif
//	
//PUTCHAR_PROTOTYPE
//{
//  HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xFFFF);//??????,??1
//  return ch;
//}

//重定向printf多个串口输出
#ifndef __CK_H
#define __CK_H
#include "stm32f1xx_hal.h"	
#define  P_USART1   1;
#define  P_USART2   2;
#define  P_USART3   3;
uint8_t  P_Flag = P_USART1;
#define  UART_START_USART1()   {P_Flag = P_USART1;}
#define  UART_START_USART2()   {P_Flag = P_USART2;}
#define  UART_START_USART3()   {P_Flag = P_USART3;}

int fputc(int ch,FILE *stream)
{
  switch(P_Flag)
  {
    case 1:HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,2);
          break;
    case 3:HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,2);
          break;
    default: break;
  }
  return ch;
}
int fputc(int ch,FILE *stream);
#endif

//AT指令定义
#define AT_start  "+++a"       //进入串口AT指令模式
#define AT_slave  "AT+MODE=S"  //切换为从设备模式，切换完重启
#define AT_flag   "AT+LINK"    //主设备连接后，从设备发送link可查询是否连接成功
#define AT_end    "AT+ENTM"    //退出指令模式
#define AT_mode   "AT+WMODE=0" //配置为透传模式

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adcBuf[16];    //adc采集储存数组
uint8_t REG_configuration[2]={0x12,0};  //开始转换寄存器指令
uint16_t capBuf[9];     //16位cap采集储存数组
double capValue[9];     //真实电容 单位PF
double adcVoc[16];      //adc真实电压 单位mV
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void AD7150_Init(void);
void select_res_unit(int Row);
void select_cap_unit(int Row,int Col);
void BLE106_Init(void);
void for_delay_us(uint32_t nus);

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  AD7150_Init();
	BLE106_Init();
	HAL_Delay(10000);   //等待10s再采集
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
				//电阻采集
		for(int i=0;i<4;i++)
		{
			uint8_t low,high;
			select_res_unit(i+1);
			HAL_Delay(3);                //延迟稳定电平
			for(int j=0;j<4;j++)
      {
			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1,50);
			  adcBuf[i*4+j]=HAL_ADC_GetValue(&hadc1);   //获取ADC数值
        high=(uint8_t)(adcBuf[i*4+j]>>8);         //拆分成高低2个u8数据
        low=(uint8_t)(adcBuf[i*4+j]&0xFF);
				 
				HAL_UART_Transmit(&huart3 , &high, 1, 0xff);
        HAL_UART_Transmit(&huart3 , &low, 1, 0xff);
								
			  //adcVoc[i*4+j]=(double)((double)((int)adcBuf[i*4+j]*330)/4096);   //单位mV
				//UART_START_USART3();	
			  //printf("Resistance %u = %f\r\n", (i*4+j+1) ,adcVoc[i*4+j]);
				
		  }
			HAL_ADC_Stop(&hadc1);
		}

		
		//电容采集
		uint8_t data[2];
		uint8_t cap_low;
		uint8_t cap_high;
		
		//测单个单元的数据
//		select_cap_unit(2,2);
//		HAL_Delay(1);
//		HAL_I2C_Mem_Write(&hi2c1,0x90,0x0F,I2C_MEMADD_SIZE_8BIT,(uint8_t*)REG_configuration,1,1000);
//		HAL_Delay(10);
//		HAL_I2C_Mem_Read(&hi2c1,0x90,0x01,I2C_MEMADD_SIZE_8BIT,data,2,1000);
//		capBuf[0]=((uint16_t)data[0] << 8) | data[1];          //将两个高低8位二进制拼成16位
//		capValue[0]=(double)((double)((int)data[0]*256+(int)data[1])-12288)/40944*4; 
//		UART_START_USART3();		
//		printf("Capacitance=%f\r\n",capValue[0]);   //通过串口3蓝牙传输出去，一个单元数据
//		//HAL_UART_Transmit(&huart3,(uint8_t *)data, 2, 0xffff);//传输2字节数据
		
		//测所有单元的数据
		for(int i=1;i<4;i++)
		{
			for(int j=1;j<4;j++)
			{
				select_cap_unit(i,j);
				HAL_Delay(1);             //延迟稳定电平
				HAL_I2C_Mem_Write(&hi2c1,0x90,0x0F,I2C_MEMADD_SIZE_8BIT,(uint8_t*)REG_configuration,1,1000);
				HAL_Delay(5);             //转换需要5ms
				HAL_I2C_Mem_Read(&hi2c1,0x90,0x01,I2C_MEMADD_SIZE_8BIT,data,2,1000);
				//cap_low=((data[0]<<4)+(data[1]>>4));
				//cap_high=data[0]>>4;
				
				//capBuf[(i-1)*3+(j-1)]=((uint16_t)data[0] << 8) | data[1];          //将两个高低8位二进制拼成16位
				//capValue[(i-1)*3+(j-1)]=(double)((double)((int)data[0]*256+(int)data[1])-12288)/40944*4; 							
				
				//printf("%u\r\n",(int)data[0]*256+(int)data[1]);
				//UART_START_USART3();		
				//printf("Capacitance %u = %f\r\n", ((i-1)*3+j) ,capValue[(i-1)*3+(j-1)]);   //通过串口3蓝牙传输出去，所有单元数据
				
				HAL_UART_Transmit(&huart3,(uint8_t *)data, 2, 0xffff);//传输2字节数据
       }
    }
//		count=0;
//		HAL_UART_Transmit(&huart3,(uint8_t *)data1, 50, 0xffff);//传输50字节数据
				
				//printf("0x%04x\r\n",capBuf[(i-1)*3+(j-1)]);传输16进制数据
				//HAL_UART_Transmit(&huart3,(uint8_t *)data, 2, 0xffff);//传输2字节数据
			//if(i==2)
//			{
//				UART_START_USART1();  //通过串口1输出到电脑上
//				printf("Capacitance %u = %f\r\n", 5 ,capValue[4]);
//			}
			}
}
		
		//蓝牙传输
		
		
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//AD7150初始化
void AD7150_Init(void)
{
	uint8_t REG_setup[2] = {0xCB,0x00};
	uint8_t REG_dcap[2] = {0xC0,0x00};
	uint8_t REG_configuration1[2] = {0x10,0x00};
	//uint8_t REG_setup=0xCB;
	//uint8_t REG_dcap=0xC0;
	//uint8_t REG_configuration1=0x10;
	HAL_I2C_Mem_Write(&hi2c1,0x90,0x0B,I2C_MEMADD_SIZE_8BIT,(uint8_t*)REG_setup,1,1000);
	HAL_Delay(20);
	HAL_I2C_Mem_Write(&hi2c1,0x90,0x11,I2C_MEMADD_SIZE_8BIT,(uint8_t*)REG_dcap,1,1000);
	HAL_I2C_Mem_Write(&hi2c1,0x90,0x0F,I2C_MEMADD_SIZE_8BIT,(uint8_t*)REG_configuration1,1,1000);
}

//电阻行电极选择函数说明
void select_res_unit(int Row)
{
	switch(Row)
	{
		//Rcontrol1 PA5;Rcontrol2 PA6;
		//0 0->Row1;1 0->Row2;0 1->Row3;1 1->Row4
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);break;
		case 3:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);break;
		case 4:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);break;
	}
}

//电容单元选择函数说明
void select_cap_unit(int Row,int Col)
{
	switch(Row)
	{
		//Ccontrol1 PB5;Ccontrol2 PB4;
		//1 0->Row1;0 1->Row2;1 1->Row3
		case 1:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);break;
		case 2:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);break;
		case 3:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);break;
	}
	switch(Col)
	{
		//Ccontrol3 PB3;Ccontrol4 PD2;
		//1 0->Col1;0 1->Col2;1 1->Col3
		case 1:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);break;
		case 2:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);break;
		case 3:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);break;
	}
}

//蓝牙模块初始化-AT配置
void BLE106_Init(void)
{
//#define AT_start  "+++a"       //进入串口AT指令模式
//#define AT_slave  "AT+MODE=S"  //切换为从设备模式，切换完重启
//#define AT_flag   "AT+LINK"    //主设备连接后，从设备发送link可查询是否连接成功
//#define AT_end    "AT+ENTM"    //退出指令模式
//#define AT_mode   "AT+WMODE=0" //配置为透传模式
	  char *AT_Start="+++a\r\n";
    char *AT_Slave="AT+MODE=S\r\n"; 
    char *AT_Flag="AT+LINK\r\n";    
    char *AT_End="AT+ENTM\r\n";    
    char *AT_Mode="AT+WMODE=0"; 
    char *AT_Speed="AT+SPD=HIGH";

	
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_Start, strlen(AT_Start), 0xffff);
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_Slave, strlen(AT_Slave), 0xffff);
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_Start, strlen(AT_Start), 0xffff);
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_Flag, strlen(AT_Flag), 0xffff);
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_Mode, strlen(AT_Mode), 0xffff);
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_Speed, strlen(AT_Speed), 0xffff);
		HAL_UART_Transmit(&huart3,(uint8_t *)AT_End, strlen(AT_End), 0xffff);
}

void for_delay_us(uint32_t nus)
{
    uint32_t Delay = nus * 168/4;
    do
    {
        __NOP();
    }
    while (Delay --);
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
