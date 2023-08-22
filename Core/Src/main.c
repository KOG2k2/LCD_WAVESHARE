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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD_lib.h"
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

//color define
#define LCD_WIDTH    	240
#define LCD_HEIGHT   	320

#define FONT_1206    	12
#define FONT_1608    	16
#define FONT_GB2312  	16

#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0XF81F
#define GRED 		   	   0XFFE0
#define GBLUE		   		 0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN 		   	 0XBC40
#define BRRED 		   	 0XFC07
#define GRAY  		   	 0X8430

#define LCD_DC_H()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, 1)
#define LCD_DC_L()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, 0)

#define LCD_CS_H()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, 1)
#define LCD_CS_L()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, 0)

#define LCD_BKL_H()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1)
#define LCD_BKL_L()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0)

#define LCD_RST_H()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1)
#define LCD_RST_L()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0)

#define LCD_CMD                0
#define LCD_DATA               1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t chRetry = 0;
uint8_t chTemp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void spi1_communication(uint8_t send_char)
{
	while (HAL_SPI_GetState(&hspi1) != SPI_FLAG_TXE) {
		if (++ chRetry > 200) {

		}
	}

	//SPI_I2S_SendData(SPI1, send_char);
	HAL_SPI_Transmit(&hspi1, &send_char, 1, 100);

	chRetry=0;
	while (HAL_SPI_GetState(&hspi1) != SPI_FLAG_RXNE){
		if (++ chRetry > 200) {

		}
	}

	chTemp = HAL_SPI_Receive(&hspi1, &send_char, 1, 100);

	/* Wait until the BSY flag is set */
	while(HAL_SPI_GetState(&hspi1) == SPI_FLAG_BSY) {

	}
}

void lcd_set_cursor(uint16_t hwXpos, uint16_t hwYpos)
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}
#ifdef HX8347D_DEVICE
	lcd_write_command(0x02, hwXpos >> 8);
	lcd_write_command(0x03, hwXpos & 0xFF);
	lcd_write_command(0x06, hwYpos >> 8);
	lcd_write_command(0x07, hwYpos & 0xFF);
#elif defined ST7789_DEVICE
	lcd_write_byte(0x2A,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(hwXpos,LCD_DATA);
	lcd_write_byte(0x2B,LCD_CMD);
	lcd_write_byte((hwYpos>>8)&0xff,LCD_DATA);
	lcd_write_byte(hwYpos&0xff,LCD_DATA);
#endif
}

void lcd_clear_screen(uint16_t hwColor)
{
	uint32_t i, wCount = LCD_WIDTH;
	wCount *= LCD_HEIGHT;

#ifdef HX8347D_DEVICE
	lcd_set_cursor(0, 0);
	lcd_write_byte(0x22, LCD_CMD);
#elif defined ST7789_DEVICE
	lcd_write_byte(0x2A,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte((LCD_WIDTH-1)&0xff,LCD_DATA);
	lcd_write_byte(0x2B,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(((LCD_HEIGHT-1)>>8)&0xff,LCD_DATA);
	lcd_write_byte((LCD_HEIGHT-1)&0xff,LCD_DATA);
	lcd_write_byte(0x2C,LCD_CMD);
#endif
	LCD_CS_L();
	LCD_DC_H();
	for(i=0;i<wCount;i++){
#ifdef HX8347D_DEVICE
//		spi1_communication((uint8_t)(hwColor&0xff));
//		spi1_communication(hwColor>>8);

		spi1_communication(hwColor>>8);
		spi1_communication((uint8_t)(hwColor&0xff));
#elif defined ST7789_DEVICE
		spi1_communication(hwColor>>8);
		spi1_communication((uint8_t)(hwColor&0xff));
#endif
	}
	LCD_CS_H();
}

void lcd_init(void)
{
	//lcd_ctrl_port_init();
	LCD_RST_H();
	//spi_init();

	LCD_CS_H();
	LCD_BKL_H();
#ifdef 	ST7789_DEVICE
	LCD_RST_H();
	HAL_Delay(5);
	LCD_RST_L();
	HAL_Delay(5);
	LCD_RST_H();
	HAL_Delay(5);
	LCD_CS_H();
#endif

#ifdef HX8347D_DEVICE
	lcd_write_command(0xEA,0x00);
	lcd_write_command(0xEB,0x20);
	lcd_write_command(0xEC,0x0C);
	lcd_write_command(0xED,0xC4);
	lcd_write_command(0xE8,0x38);
	lcd_write_command(0xE9,0x10);
	lcd_write_command(0xF1,0x01);
	lcd_write_command(0xF2,0x10);
	lcd_write_command(0x40,0x01);
	lcd_write_command(0x41,0x00);
	lcd_write_command(0x42,0x00);
	lcd_write_command(0x43,0x10);
	lcd_write_command(0x44,0x0E);
	lcd_write_command(0x45,0x24);
	lcd_write_command(0x46,0x04);
	lcd_write_command(0x47,0x50);
	lcd_write_command(0x48,0x02);
	lcd_write_command(0x49,0x13);
	lcd_write_command(0x4A,0x19);
	lcd_write_command(0x4B,0x19);
	lcd_write_command(0x4C,0x16);
	lcd_write_command(0x50,0x1B);
	lcd_write_command(0x51,0x31);
	lcd_write_command(0x52,0x2F);
	lcd_write_command(0x53,0x3F);
	lcd_write_command(0x54,0x3F);
	lcd_write_command(0x55,0x3E);
	lcd_write_command(0x56,0x2F);
	lcd_write_command(0x57,0x7B);
	lcd_write_command(0x58,0x09);
	lcd_write_command(0x59,0x06);
	lcd_write_command(0x5A,0x06);
	lcd_write_command(0x5B,0x0C);
	lcd_write_command(0x5C,0x1D);
	lcd_write_command(0x5D,0xCC);
	lcd_write_command(0x1B,0x1B);
	lcd_write_command(0x1A,0x01);
	lcd_write_command(0x24,0x2F);
	lcd_write_command(0x25,0x57);
	lcd_write_command(0x23,0x88);
	lcd_write_command(0x18,0x34);
	lcd_write_command(0x19,0x01);
	lcd_write_command(0x01,0x00);
	lcd_write_command(0x1F,0x88);
	lcd_write_command(0x1F,0x80);
	lcd_write_command(0x1F,0x90);
	lcd_write_command(0x1F,0xD0);
	lcd_write_command(0x17,0x05);
	lcd_write_command(0x36,0x02);
	lcd_write_command(0x28,0x38);
	lcd_write_command(0x28,0x3F);
	lcd_write_command(0x16,0x18);
	lcd_write_command(0x02,0x00);
	lcd_write_command(0x03,0x00);
	lcd_write_command(0x04,0x00);
	lcd_write_command(0x05,0xEF);
	lcd_write_command(0x06,0x00);
	lcd_write_command(0x07,0x00);
	lcd_write_command(0x08,0x01);
	lcd_write_command(0x09,0x3F);

#elif defined ST7789_DEVICE
	lcd_write_byte(0x11,LCD_CMD);
	HAL_Delay(10);
	lcd_write_command(0x36,0x00);
	lcd_write_command(0x3a,0x05);
	lcd_write_byte(0xb2,LCD_CMD);
	lcd_write_byte(0x0c,LCD_DATA);
	lcd_write_byte(0x0c,LCD_DATA);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(0x33,LCD_DATA);
	lcd_write_byte(0x33,LCD_DATA);
	lcd_write_command(0xb7,0x35);
	lcd_write_command(0xbb,0x28);
	lcd_write_command(0xc0,0x3c);
	lcd_write_command(0xc2,0x01);
	lcd_write_command(0xc3,0x0b);
	lcd_write_command(0xc4,0x20);
	lcd_write_command(0xc6,0x0f);
	lcd_write_byte(0xD0,LCD_CMD);
	lcd_write_byte(0xa4,LCD_DATA);
	lcd_write_byte(0xa1,LCD_DATA);
	lcd_write_byte(0xe0,LCD_CMD);
	lcd_write_byte(0xd0,LCD_DATA);
	lcd_write_byte(0x01,LCD_DATA);
	lcd_write_byte(0x08,LCD_DATA);
	lcd_write_byte(0x0f,LCD_DATA);
	lcd_write_byte(0x11,LCD_DATA);
	lcd_write_byte(0x2a,LCD_DATA);
	lcd_write_byte(0x36,LCD_DATA);
	lcd_write_byte(0x55,LCD_DATA);
	lcd_write_byte(0x44,LCD_DATA);
	lcd_write_byte(0x3a,LCD_DATA);
	lcd_write_byte(0x0b,LCD_DATA);
	lcd_write_byte(0x06,LCD_DATA);
	lcd_write_byte(0x11,LCD_DATA);
	lcd_write_byte(0x20,LCD_DATA);
	lcd_write_byte(0xe1,LCD_CMD);
	lcd_write_byte(0xd0,LCD_DATA);
	lcd_write_byte(0x02,LCD_DATA);
	lcd_write_byte(0x07,LCD_DATA);
	lcd_write_byte(0x0a,LCD_DATA);
	lcd_write_byte(0x0b,LCD_DATA);
	lcd_write_byte(0x18,LCD_DATA);
	lcd_write_byte(0x34,LCD_DATA);
	lcd_write_byte(0x43,LCD_DATA);
	lcd_write_byte(0x4a,LCD_DATA);
	lcd_write_byte(0x2b,LCD_DATA);
	lcd_write_byte(0x1b,LCD_DATA);
	lcd_write_byte(0x1c,LCD_DATA);
	lcd_write_byte(0x22,LCD_DATA);
	lcd_write_byte(0x1f,LCD_DATA);
	lcd_write_byte(0x29,LCD_CMD);
	lcd_write_command(0x51,0xff);
	lcd_write_command(0x55,0xB0);
#endif

	lcd_clear_screen(WHITE);
}

void lcd_write_byte(uint8_t chByte, uint8_t chCmd)
{
    if(chCmd) {
        LCD_DC_H();
    } else {
        LCD_DC_L();
    }

    LCD_CS_L();
    spi1_communication(chByte);
    LCD_CS_H();
}

void lcd_write_command(uint8_t chRegister, uint8_t chValue)
{
	lcd_write_byte(chRegister, LCD_CMD);
	lcd_write_byte(chValue, LCD_DATA);
}

void lcd_write_word(uint16_t hwData)
{
    LCD_DC_H();
    LCD_CS_L();
    spi1_communication(hwData >> 8);
    spi1_communication(hwData & 0xFF);
    LCD_CS_H();
}

void lcd_draw_dot(uint16_t hwXpos, uint16_t hwYpos, uint16_t hwColor)
{
	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}
	lcd_set_cursor(hwXpos, hwYpos);
#ifdef HX8347D_DEVICE
	lcd_write_byte(0x22, LCD_CMD);
#elif defined ST7789_DEVICE
	lcd_write_byte(0x2C, LCD_CMD);
#endif
	lcd_write_word(hwColor);

}

void lcd_display_char(	 uint16_t hwXpos,
                         uint16_t hwYpos,
                         uint8_t chChr,
                         uint8_t chSize,
                         uint16_t hwColor)
{
	uint8_t i, j, chTemp;
	uint16_t hwYpos0 = hwYpos, hwColorVal = 0;

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}
#ifdef HX8347D_DEVICE

#elif defined ST7789_DEVICE
	lcd_write_byte(0x2A,LCD_CMD);
	lcd_write_byte(0x00,LCD_DATA);
	lcd_write_byte(hwXpos,LCD_DATA);
	lcd_write_byte((hwXpos) >> 8,LCD_DATA);
	lcd_write_byte((hwXpos) & 0xFF,LCD_DATA);

	lcd_write_byte(0x2B,LCD_CMD);
	lcd_write_byte(hwYpos >> 8,LCD_DATA);
	lcd_write_byte(hwYpos & 0xFF,LCD_DATA);
	lcd_write_byte((hwYpos) >> 8,LCD_DATA);
	lcd_write_byte((hwYpos) & 0xFF,LCD_DATA);
	lcd_write_byte(0x2C, LCD_CMD);
#endif
    for (i = 0; i < chSize; i ++) {
				if (FONT_1206 == chSize) {
					chTemp = c_chFont1206[chChr - 0x20][i];
				}
				else if (FONT_1608 == chSize) {
					chTemp = c_chFont1608[chChr - 0x20][i];
				}
        for (j = 0; j < 8; j ++) {
					if (chTemp & 0x80) {
						hwColorVal = hwColor;
						lcd_draw_dot(hwXpos, hwYpos, hwColorVal);
					}
					chTemp <<= 1;
					hwYpos ++;
					if ((hwYpos - hwYpos0) == chSize) {
						hwYpos = hwYpos0;
						hwXpos ++;
						break;
					}
				}
    }
}

void lcd_display_string(uint16_t hwXpos,uint16_t hwYpos,
													const uint8_t *pchString,
													uint8_t chSize,uint16_t hwColor)
{

	if (hwXpos >= LCD_WIDTH || hwYpos >= LCD_HEIGHT) {
		return;
	}

    while (*pchString != '\0') {
        if (hwXpos > (LCD_WIDTH - chSize / 2)) {
					hwXpos = 0;
					hwYpos += chSize;
					if (hwYpos > (LCD_HEIGHT - chSize)) {
						hwYpos = hwXpos = 0;
						lcd_clear_screen(0x00);
					}
				}

        lcd_display_char(hwXpos, hwYpos, (uint8_t)*pchString, chSize, hwColor);
        hwXpos += chSize / 2;
        pchString ++;
    }
}

void lcd_display_GB2312(  uint8_t gb, uint16_t color_front,
													uint16_t postion_x,uint16_t postion_y )
{
	uint8_t i, j,chTemp;
	uint16_t hwYpos0 = postion_y, hwColorVal = 0;

	if (postion_x >= LCD_WIDTH || postion_y >= LCD_HEIGHT) {
		return;
	}

	for (i = 0; i < 32; i++) {
		chTemp = GB2312[gb][i];
		for (j = 0; j < 8; j++) {
			if (chTemp & 0x80) {
					hwColorVal = color_front;
				if(i<15)
					lcd_draw_dot(postion_x, postion_y, hwColorVal);
				else
					lcd_draw_dot(postion_x-16, postion_y+8, hwColorVal);
			}
			chTemp <<= 1;
			postion_y ++;
			if ((postion_y - hwYpos0) == 8) {
				postion_y = hwYpos0;
				postion_x ++;
				break;
			}
		}
	}
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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t i;
  	lcd_init();
  	lcd_display_string( 60,60,(uint8_t* )"CPU:STM32F405RGT6          ", FONT_1206, RED );
    lcd_display_string( 60,80,(uint8_t* )"www.WaveShare.net          ", FONT_1206, RED );
    for(i = 0;i<7;i++){
    	lcd_display_GB2312( i,RED,60+i*16,100);
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RESET_Pin|GPIO_PIN_6|LCD_CS_Pin|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RESET_Pin PB6 LCD_CS_Pin PB8
                           PB9 */
  GPIO_InitStruct.Pin = LCD_RESET_Pin|GPIO_PIN_6|LCD_CS_Pin|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
