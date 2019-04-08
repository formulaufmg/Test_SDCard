
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Define endereços dos dados (measureID) de acordo com o protocolo FTCAN2.0*/
#define TPS_ADDR 0x0002
#define RPM_ADDR 0x0084
#define BAT_V_ADDR 0x0012
#define TPM_MOT_ADDR 0x0008
#define OILP_ADDR 0x000A
#define SPEEDFR_ADDR 0x001A
#define FUELP_ADDR 0x000C
#define MAP_ADDR 0x0004
#define DRSFRPM 0x0020


//#define TEST_CAN 1
/*Variaveis para gravação de dados*/
char buffer[512];
static FATFS g_sFatFs;
FRESULT fresult;
FIL file;
int len;
UINT bytes_written;

/* Define endereços das temperaturas do Freio*/

/*Variaveis globais dos dados*/
uint16_t RPM = 0,  SPEEDFR = 0, TIMERCOUNT=0, POSVOL = 0, PFREIOT = 0, PFREIOD =0, TEMPPDU = 0,TPS, ECT, BAT, OILP,FUELP, CORRENTE=0, SUSP = 0,TEMPBREAK_1=0, TEMPBREAK_2=0;
float  MAP;
uint8_t BOMBA, VENT, SPARKC, BEACON = 0;

CanTxMsgTypeDef TxTemp;
CanTxMsgTypeDef TxSpeed;
CanTxMsgTypeDef adc1;

/* UART */
char tx_buffer[100];
char tx_buffer2[80];
char tx_buffer3[80];
char tx_buffer4[100];
uint8_t pack3_cnt = 0, pack2_cnt = 0, PACKNO;

uint8_t buff1[15], buff2[18], buff3[13] ,buff4[28];

/*I2c acelerometro*/
//SD_MPU6050 mpu1;
char acel_result[100];
int16_t g_x, g_y, g_z, a_x, a_y, a_z;

/* Recepcao de pacotes CAN */
uint32_t canID =0, callbackCnt = 0;
uint8_t  datafieldID;
uint16_t payloadLength, payloadCnt = 0;
uint8_t segPackNo = 0;
uint8_t payloadData[80];
uint8_t canData[8];

/* ADC */
uint32_t adc_buffer[600], ADC[6]= {0,0,0,0,0,0};
uint32_t cont = 0;

/* Extensometros */
uint8_t dados_ext[8], RECEIVED_ETX = 0;
uint8_t EXT_ARRAY[8];
unsigned long EXT_DATA[8] = {0,0,0,0,0,0,0,0};
uint8_t CANTX = 0, CANRX = 0;


/*  vent = B4
  bomba = A15
beacon = a8
sensor de corrente = b0
 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void CAN_Config(void);
void getMeasure(uint8_t address, uint16_t value);
void writeFile();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */


    //Delay para esperar FT500 estabelecer comunicacao com WB-O2
    HAL_Delay(4000);

    CAN_Config();


  	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
  	{
  	  // Reception Error
  	  Error_Handler();
  	}

    // Sinaliza para timer começar a contagem
    HAL_TIM_Base_Start_IT(&htim3);


    //LED C13 pisca para sinalizar que o progrma começou
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(500);



/*
    //-------------SD DEMO START-----------------
        char buffer[512];
        static FATFS g_sFatFs;
        FRESULT fresult;
        FIL file;
        int len;
        UINT bytes_written;
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        fresult = f_mount(&g_sFatFs, "0:", 0);        //mount SD card
        fresult = f_open(&file, "Formula.txt", FA_OPEN_ALWAYS | FA_WRITE); //open file on SD card
        fresult = f_lseek(&file, file.fsize);         //go to the end of the file
        len = sprintf(buffer, "Jesus!\r\n Ao que parece deu certo\r\n");    //generate some string
        fresult = f_write(&file, buffer, len, &bytes_written);     //write data to the file
        fresult = f_close(&file);
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    //-------------SD DEMO END-----------------
*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	 // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_5TQ;
  hcan.Init.BS2 = CAN_BS2_6TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|CS_SD_CARD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_ERROR_Pin */
  GPIO_InitStruct.Pin = LED_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ERROR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin CS_SD_CARD_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|CS_SD_CARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


static void CAN_Config(void){

  CAN_FilterConfTypeDef  sFilterConfig;
  static CanTxMsgTypeDef        TxMessage;     //--
  static CanRxMsgTypeDef        RxMessage;     //-- codigo original

  /*##-1- Configure the CAN peripheral #######################################*/
   hcan.pTxMsg = &TxMessage;     //-- codigo original
   hcan.pRxMsg = &RxMessage;	  //--



  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Configure Transmission process #####################################*/
  hcan.pTxMsg->StdId = 0x001;
  hcan.pTxMsg->ExtId = 0x001;
  hcan.pTxMsg->RTR = CAN_RTR_DATA;
  /*configurando como extended id (29 bits)*/
  hcan.pTxMsg->IDE = CAN_ID_EXT;
  hcan.pTxMsg->DLC = 2;
}


#ifdef TEST_CAN

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan){

	//CANRX = 1;

	uint8_t i, dataLength;

	/*Identificação da mensagem enviada pela FT*/
		if(((hcan->pRxMsg->ExtId) >> 19) == 0x280){
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		}

	/* Armazena a quantidade de bytes do frame na variavel dataLength e a ID em canID */
	dataLength = hcan->pRxMsg->DLC;
	canID = hcan->pRxMsg->ExtId;

	/* Armazena dados recebidos em array canData*/
	for (i=0;i<dataLength;i++){
		canData[i] = hcan->pRxMsg->Data[i];
	}

	/*Se o ProductID for relativo a FT500 (0x280)*/
   if ((((hcan->pRxMsg->ExtId) >> 19) == 0x280) && (hcan->pRxMsg->IDE == CAN_ID_EXT)){

	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

	datafieldID = (canID>>11)& 0x7;

		//writeFile();

		if (datafieldID != 0x2){
			//while(1){}
		}

		/* Testa se pacote é do tipo Single Packet*/
		if (canData[0] == 0xff){
		}

		/* Testa se pacote é do tipo segmented packet */
		/* Testa se é o primeiro pacote do segmented packet (valor 0 no primeiro byte)*/
		if (canData[0] == 0){

			/* Zera variável que conta o indice da payload */
			payloadCnt = 0;

			/* Seta segPackNo como 1, indicando que o proximo pacote a ser recebido tem valor do primeiro byte como 1 */
			segPackNo = 1;

			/* Armazena tamanho da payload na variavel payloadLength. Tamanho da payload sao o segundo e
			 * terceiros bytes do primeiro pacote */
			payloadLength = ((canData[1] & 0x7) << 8) | canData[2];

			/* Nos 5 bytes restantes, armazena payload no array */
			for (i=3;i<dataLength;i++){
				payloadData[payloadCnt] = canData[i];
				payloadCnt++;
			}
		}

		/* Caso o pacote tenha o primeiro byte diferente de 0 */
		else{
			/* Caso seja o pacote com o número certo */
			if(canData[0] == segPackNo){
				segPackNo++;
				/* Armazena os dados recebidos na variavel payloadData, começando do byte 1 e indo até o ultimo byte recebido*/
				for (i=1;i<dataLength;i++){
					payloadData[payloadCnt] = canData[i];

					/*Dados sao enviados de 4 em 4 bytes, sendo os 2 primeiro relativos ao endereço especifico de cada um
					  e os 2 ultimos o dado
					  Faz divisao por 4 e pega o resto.
					*/
					if(payloadCnt % 4 == 3){
						uint8_t add = payloadData[payloadCnt-2];
						/*Junta os 2 bytes correspondentes ao valor do dado*/
						uint16_t value = (payloadData[payloadCnt-1]<<8)|payloadData[payloadCnt];
						getMeasure(add, value);
					}

					/* Chegou no final da payload */
					if((payloadCnt == payloadLength-1)){
						writeFile();
						break;
					}

					payloadCnt++;
				}
			}
		}
	  }


	/* Receive */
	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
	{
		/* Reception Error */
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		Error_Handler();
	}
}

#endif

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{

	CANRX = 1;

	/* LED 13 muda de estado toda vez que é chamada a funcão callback, ou seja, toda vez que recebe um pacote CAN */
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);


	uint8_t i, dataLength;

	/* Armazena a quantidade de bytes do frame na variavel dataLength e a ID em canID */
	dataLength = hcan->pRxMsg->DLC;
	canID = hcan->pRxMsg->ExtId;

	/* Armazena dados recebidos em array canData*/
	for (i=0;i<dataLength;i++){
		canData[i] = hcan->pRxMsg->Data[i];
	}

	/*Se o ProductID for relativo a FT500 (0x280)*/
   if ((((hcan->pRxMsg->ExtId) >> 19) == 0x280) && (hcan->pRxMsg->IDE == CAN_ID_EXT)){

	datafieldID = (canID>>11)& 0x7;

	if (datafieldID != 0x2){
		//while(1){}
	}

	/* Testa se pacote é do tipo Single Packet*/
	if (canData[0] == 0xff){
	}

	/* Testa se pacote é do tipo segmented packet */
	/* Testa se é o primeiro pacote do segmented packet (valor 0 no primeiro byte)*/
	if ( canData[0] == 0){

		/* Zera variável que conta o indice da payload */
		payloadCnt = 0;

		/* Seta segPackNo como 1, indicando que o proximo pacote a ser recebido tem valor do primeiro byte como 1 */
		segPackNo = 1;

		/* Armazena tamanho da payload na variavel payloadLength. Tamanho da payload sao o segundo e
		 * terceiros bytes do primeiro pacote */
		payloadLength = ((canData[1] & 0x7) << 8) | canData[2];

		/* Nos 5 bytes restantes, armazena payload no array */
		for (i=3;i<dataLength;i++){
			payloadData[payloadCnt] = canData[i];
			payloadCnt++;
		}
	}

	/* Caso o pacote tenha o primeiro byte diferente de 0 */
	else{
		/* Caso seja o pacote com o número certo */
		if(canData[0] == segPackNo){
			segPackNo++;
			/* Armazena os dados recebidos na variavel payloadData, começando do byte 1 e indo até o ultimo byte recebido*/
			for (i=1;i<dataLength;i++){
				payloadData[payloadCnt] = canData[i];

				/*Dados sao enviados de 4 em 4 bytes, sendo os 2 primeiro relativos ao endereço especifico de cada um
				  e os 2 ultimos o dado
				  Faz divisao por 4 e pega o resto.
				*/
				if(payloadCnt % 4 == 3){
					uint8_t add = payloadData[payloadCnt-2];

					/*Junta os 2 bytes correspondentes ao valor do dado*/
					uint16_t value = (payloadData[payloadCnt-1]<<8)|payloadData[payloadCnt];

					getMeasure(add, value);
				}

				/* Chegou no final da payload */
				if((payloadCnt == payloadLength-1)){
					break;
				}

				payloadCnt++;
			}
		}
	}
  }

  callbackCnt++; //Contador de quantos callbacks foram chamados

  /* Chama funcao para receber novo pacote */
  if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
  {
    Error_Handler();
  }
  CANRX = 0;

}


/* Recebe endereço do measureID do dado recebido pelo barramento e seu valor e atualiza a variavel global correspondente*/
void getMeasure(uint8_t address, uint16_t value){

	switch(address){
	case TPS_ADDR:
		//TPS = value*0.1;
		TPS = value;
		break;
	case RPM_ADDR:
		RPM = value;
		break;
	case  OILP_ADDR:
		//OILP = 0.001*value;
		OILP = value;
		break;
	case  MAP_ADDR:
		MAP = 0.001*value;
		break;
	case  FUELP_ADDR:
//		FUELP = 0.001*value;
		FUELP = value;
		break;
	case SPEEDFR_ADDR:
		SPEEDFR = value;
		break;
	case BAT_V_ADDR:
//		BAT = 0.01*value;
		BAT = value;
		break;
	case TPM_MOT_ADDR:
		//ECT = 0.1*value;
		ECT = value;
		break;
	case DRSFRPM:
		RPM = value;
		break;
	default:
		break;
	}

}

/*
 *  Função para o arquivo.
 */
void writeFile(){

	/* Writing a File*/
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

	/* Mount SD card */
	if(f_mount(&g_sFatFs, "0:", 0) != FR_OK){
		/* Mount Error */
		Error_Handler();
	}

	/* Open file on SD card */
	if(f_open(&file, "CANTest.txt", FA_OPEN_ALWAYS | FA_WRITE) != FR_OK){
		/* Open File Error */
		Error_Handler();
	}

	/*Go to the end of the file*/
	if(f_lseek(&file, file.fsize) != FR_OK){
		/* Open File Error */
		Error_Handler();
	}

	//len = sprintf(buffer, "Jesus!\r\n Ao que parece deu certo\r\n");    //generate some string
	len = sprintf(buffer, "\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", TPS, RPM, OILP, MAP, FUELP, SPEEDFR, BAT, ECT);    //generate some string

	/*Write data to the file*/
	if(f_write(&file, buffer, len, &bytes_written)){
		/* Write data Error */
		Error_Handler();
	}

	/* Close file on SD card */
	if(f_close(&file) != FR_OK){
		/* Close file Error */
		Error_Handler();
	}

	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	writeFile();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */

	  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);

  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_Delay(500);
	  break;
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
