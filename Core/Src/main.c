/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
typedef struct {
    uint8_t AIR_POS_cmd_isActive : 1;
    uint8_t AIR_POS_isClosed : 1;
    uint8_t AIR_NEG_cmd_isActive : 1;
    uint8_t AIR_NEG_isClosed : 1;
    uint8_t DCBUS_PRECH_RLY_cmd_isActive : 1;
    uint8_t DCBUS_PRECH_RLY_isClosed : 1;
    uint8_t AMS_errIsActive : 1;
    uint8_t IMD_errIsActive : 1;
    uint8_t TSAL_green_isActive : 1;
    uint8_t DCBUS_isOver60V : 1;
    uint8_t IMP_any_isActive : 1;
    uint8_t IMP_HV_Relays_Signals_isActive : 1;
    uint8_t IMP_AIRS_Signals_isActive : 1;
    uint8_t AIR_POS_STG_mechStateSignal_isActive : 1;
    uint8_t AIR_NEG_STG_mechStateSignal_isActive : 1;
} CAN_MessageTypeDef; //each field takes up 1 bit to pack them later

CAN_MessageTypeDef canMessage; //to declare the struct globally

void InitializeCANMessage(void){
  canMessage.AIR_POS_cmd_isActive = 0;
  canMessage.AIR_POS_isClosed = 0;
  canMessage.AIR_NEG_cmd_isActive = 0;
  canMessage.AIR_NEG_isClosed = 0;
  canMessage.DCBUS_PRECH_RLY_cmd_isActive = 0;
  canMessage.DCBUS_PRECH_RLY_isClosed = 0;
  canMessage.AMS_errIsActive = 0;
  canMessage.IMD_errIsActive = 0;
  canMessage.TSAL_green_isActive = 1;  //only TSAL green light is active
  canMessage.DCBUS_isOver60V = 0;
  canMessage.IMP_any_isActive = 0;
  canMessage.IMP_HV_Relays_Signals_isActive = 0;
  canMessage.IMP_AIRS_Signals_isActive = 0;
  canMessage.AIR_POS_STG_mechStateSignal_isActive = 0;
  canMessage.AIR_NEG_STG_mechStateSignal_isActive = 0;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  InitializeCANMessage();

  HAL_TIM_Base_Start_IT(&htim2);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t PackCANMessage(CAN_MessageTypeDef* message) {
  uint32_t packedData = 0; //now let's left shift every single bit individually
  packedData |= (message->AIR_POS_cmd_isActive) << 0;
  packedData |= (message->AIR_POS_isClosed) << 1;
  packedData |= (message->AIR_NEG_cmd_isActive) << 2;
  packedData |= (message->AIR_NEG_isClosed) << 3;
  packedData |= (message->DCBUS_PRECH_RLY_cmd_isActive) << 4;
  packedData |= (message->DCBUS_PRECH_RLY_isClosed) << 5;
  packedData |= (message->AMS_errIsActive) << 6;
  packedData |= (message->IMD_errIsActive) << 7;
  packedData |= (message->TSAL_green_isActive) << 8;
  packedData |= (message->DCBUS_isOver60V) << 9;
  packedData |= (message->IMP_any_isActive) << 10;
  packedData |= (message->IMP_HV_Relays_Signals_isActive) << 11;
  packedData |= (message->IMP_AIRS_Signals_isActive) << 12;
  packedData |= (message->AIR_POS_STG_mechStateSignal_isActive) << 13;
  packedData |= (message->AIR_NEG_STG_mechStateSignal_isActive) << 14;

  return packedData;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance==TIM2){
    uint32_t packedData=PackCANMessage(&canMessage);//we packed the canMessage

    uint8_t dataPerByte[4]; //contains 4 bytes of 8 bits each
    dataPerByte[0] = (uint8_t)(packedData & 0xFF);//LSB because little-endian
    dataPerByte[1] = (uint8_t)((packedData >> 8) & 0xFF);
    dataPerByte[2] = (uint8_t)((packedData >> 16) & 0xFF);
    dataPerByte[3] = (uint8_t)((packedData >> 24) & 0xFF); //MSB 
    //dataPerByte=0x00000100

    uint32_t TxMailbox;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.DLC=4;//size in bytes
    TxHeader.IDE=CAN_ID_STD;
    TxHeader.RTR=CAN_RTR_DATA;
    TxHeader.StdId=0x4; //msg ID

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, dataPerByte, &TxMailbox) != HAL_OK) {
      
      Error_Handler();
    }
  }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t receivedData[4]; // Buffer to store received data (4 bytes)

    // Read the received message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, receivedData) == HAL_OK) {
        // Check if the received message ID matches what you sent
        if (RxHeader.StdId == 0x4 && RxHeader.DLC == 4) {
            // reverse it because it's in little endian
            uint32_t receivedPackedData = (receivedData[3] << 24) | (receivedData[2] << 16) | (receivedData[1] << 8) | receivedData[0];

            // Print to check the receivedPackedData
            char msg[50];
            sprintf(msg, "Received: 0x%08lX\r\n", receivedPackedData);
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *);

void CAN_ErrorHandler(CAN_HandleTypeDef *hcan) {

    char buf[20];
    uint32_t error = HAL_CAN_GetError(hcan);

    #define tmp_printf(X)                                           \
    do {                                                                   \
        HAL_UART_Transmit(&huart2, (uint8_t *)(X), strlen(X), HAL_MAX_DELAY); \
        HAL_UART_Transmit(&huart2, (uint8_t *)("\r\n"), strlen("\r\n"), HAL_MAX_DELAY); \
    } while (0)
 
    if (error & HAL_CAN_ERROR_EWG)
        tmp_printf("Protocol Error Warning");
    if (error & HAL_CAN_ERROR_EPV)
        tmp_printf("Error Passive");
    if (error & HAL_CAN_ERROR_BOF)
        tmp_printf("Bus-off Error");
    if (error & HAL_CAN_ERROR_STF)
        tmp_printf("Stuff Error");
    if (error & HAL_CAN_ERROR_FOR)
        tmp_printf("Form Error");
    if (error & HAL_CAN_ERROR_ACK)
        tmp_printf("ACK Error");
    if (error & HAL_CAN_ERROR_BR)
        tmp_printf("Bit Recessive Error");
    if (error & HAL_CAN_ERROR_BD)
        tmp_printf("Bit Dominant Error");
    if (error & HAL_CAN_ERROR_CRC)
        tmp_printf("CRC Error");
    if (error & HAL_CAN_ERROR_RX_FOV0)
        tmp_printf("FIFO0 Overrun");
    if (error & HAL_CAN_ERROR_RX_FOV1)
        tmp_printf("FIFO1 Overrun");
    if (error & HAL_CAN_ERROR_TX_ALST0)
        tmp_printf("Mailbox 0 TX failure (arbitration lost)");
    if (error & HAL_CAN_ERROR_TX_TERR0)
        tmp_printf("Mailbox 0 TX failure (tx error)");
    if (error & HAL_CAN_ERROR_TX_ALST1)
        tmp_printf("Mailbox 1 TX failure (arbitration lost)");
    if (error & HAL_CAN_ERROR_TX_TERR1)
        tmp_printf("Mailbox 1 TX failure (tx error)");
    if (error & HAL_CAN_ERROR_TX_ALST2)
        tmp_printf("Mailbox 2 TX failure (arbitration lost)");
    if (error & HAL_CAN_ERROR_TX_TERR2)
        tmp_printf("Mailbox 2 TX failure (tx error)");
    if (error & HAL_CAN_ERROR_TIMEOUT)
        tmp_printf("Timeout Error");
    if (error & HAL_CAN_ERROR_NOT_INITIALIZED)
        tmp_printf("Peripheral not initialized");
    if (error & HAL_CAN_ERROR_NOT_READY)
        tmp_printf("Peripheral not ready");
    if (error & HAL_CAN_ERROR_NOT_STARTED)
        tmp_printf("Peripheral not strated");
    if (error & HAL_CAN_ERROR_PARAM)
        tmp_printf("Parameter Error");
     
    uint16_t rec = (uint16_t)((hcan->Instance->ESR && CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
    uint16_t tec = (uint16_t)((hcan->Instance->ESR && CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);

    sprintf(buf,"rec %u, tec %u",rec,tec);
    tmp_printf(buf);

    HAL_CAN_ResetError(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  if(hcan == &hcan1){
    CAN_ErrorHandler(hcan);
  }
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
