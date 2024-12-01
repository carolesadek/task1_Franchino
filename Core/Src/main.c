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
#include "mcb.h"

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
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t buffer_tx[8]={0};
uint8_t buffer_rx[8]={0};
double static air_neg_cmd_is_active = 0.0, air_neg_is_closed = 0.0, air_neg_stg_mech_state_signal_is_active = 0.0,
                  air_pos_cmd_is_active = 0.0, air_pos_is_closed = 0.0, air_pos_stg_mech_state_signal_is_active = 0.0,
                  ams_err_is_active = 0.0, dcbus_is_over60_v = 0.0, dcbus_prech_rly_cmd_is_active = 0.0,
                  dcbus_prech_rly_is_closed = 0.0, imd_err_is_active = 0.0, imp_dcbus_is_active = 0.0, imp_any_is_active = 0.0,
                  imp_hv_relays_signals_is_active = 0.0, tsal_green_is_active = 1;


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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance==TIM2){
    struct mcb_tlb_bat_signals_status_t static tlb_signals;
    mcb_tlb_bat_signals_status_init(&tlb_signals);

    tlb_signals.air_neg_cmd_is_active = mcb_tlb_bat_signals_status_air_neg_cmd_is_active_encode(air_neg_cmd_is_active);
    tlb_signals.air_neg_is_closed     = mcb_tlb_bat_signals_status_air_neg_is_closed_encode(air_neg_is_closed);
    tlb_signals.air_neg_stg_mech_state_signal_is_active =mcb_tlb_bat_signals_status_air_neg_stg_mech_state_signal_is_active_encode(air_neg_stg_mech_state_signal_is_active);
    tlb_signals.air_pos_cmd_is_active = mcb_tlb_bat_signals_status_air_pos_cmd_is_active_encode(air_pos_cmd_is_active);
    tlb_signals.air_pos_is_closed     = mcb_tlb_bat_signals_status_air_pos_is_closed_encode(air_pos_is_closed);
    tlb_signals.air_pos_stg_mech_state_signal_is_active =mcb_tlb_bat_signals_status_air_pos_stg_mech_state_signal_is_active_encode(air_pos_stg_mech_state_signal_is_active);
    tlb_signals.ams_err_is_active = mcb_tlb_bat_signals_status_ams_err_is_active_encode(ams_err_is_active);
    tlb_signals.dcbus_is_over60_v = mcb_tlb_bat_signals_status_dcbus_is_over60_v_encode(dcbus_is_over60_v);
    tlb_signals.dcbus_prech_rly_cmd_is_active =mcb_tlb_bat_signals_status_dcbus_prech_rly_cmd_is_active_encode(dcbus_prech_rly_cmd_is_active);
    tlb_signals.dcbus_prech_rly_is_closed =mcb_tlb_bat_signals_status_dcbus_prech_rly_is_closed_encode(dcbus_prech_rly_is_closed);
    tlb_signals.tsal_green_is_active = mcb_tlb_bat_signals_status_tsal_green_is_active_encode(tsal_green_is_active);
    tlb_signals.imd_err_is_active = mcb_tlb_bat_signals_status_imd_err_is_active_encode(imd_err_is_active);
    tlb_signals.imp_any_is_active = mcb_tlb_bat_signals_status_imp_any_is_active_encode(imp_any_is_active);
    tlb_signals.imp_dcbus_is_active =mcb_tlb_bat_signals_status_imp_dcbus_is_active_encode(imp_dcbus_is_active);
    //tlb_signals.imp_hv_relays_signals_is_active =mcb_tlb_bat_signals_status_imp_hv_relays_signals_is_active_encode(imp_hv_relays_signals_is_active);
    

    mcb_tlb_bat_signals_status_pack(buffer_tx,&tlb_signals,MCB_TLB_BAT_SIGNALS_STATUS_LENGTH);  
    TxHeader.DLC=sizeof(buffer_tx);
    TxHeader.StdId=MCB_TLB_BAT_SIGNALS_STATUS_FRAME_ID;

    can_send(&hcan1,buffer_tx,&TxHeader,CAN_TX_MAILBOX0) != HAL_OK;            

  }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        CAN_RxHeaderTypeDef RxHeader;
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, buffer_rx) == HAL_OK) {
            if (RxHeader.StdId == MCB_TLB_BAT_SIGNALS_STATUS_FRAME_ID && RxHeader.DLC == MCB_TLB_BAT_SIGNALS_STATUS_LENGTH) {
                struct mcb_tlb_bat_signals_status_t tlb_signals_rx;
                if (mcb_tlb_bat_signals_status_unpack(&tlb_signals_rx, &buffer_rx, RxHeader.DLC) == 0) {
                    double air_pos_cmd_is_active = mcb_tlb_bat_signals_status_air_pos_cmd_is_active_decode(tlb_signals_rx.air_pos_cmd_is_active);
                    double air_pos_is_closed = mcb_tlb_bat_signals_status_air_pos_is_closed_decode(tlb_signals_rx.air_pos_is_closed);
                    double air_neg_cmd_is_active = mcb_tlb_bat_signals_status_air_neg_cmd_is_active_decode(tlb_signals_rx.air_neg_cmd_is_active);
                    double air_neg_is_closed = mcb_tlb_bat_signals_status_air_neg_is_closed_decode(tlb_signals_rx.air_neg_is_closed);
                    double dcbus_prech_rly_cmd_is_active = mcb_tlb_bat_signals_status_dcbus_prech_rly_cmd_is_active_decode(tlb_signals_rx.dcbus_prech_rly_cmd_is_active);
                    double dcbus_prech_rly_is_closed = mcb_tlb_bat_signals_status_dcbus_prech_rly_is_closed_decode(tlb_signals_rx.dcbus_prech_rly_is_closed);
                    double ams_err_is_active = mcb_tlb_bat_signals_status_ams_err_is_active_decode(tlb_signals_rx.ams_err_is_active);
                    double imd_err_is_active = mcb_tlb_bat_signals_status_imd_err_is_active_decode(tlb_signals_rx.imd_err_is_active);
                    double tsal_green_is_active = mcb_tlb_bat_signals_status_tsal_green_is_active_decode(tlb_signals_rx.tsal_green_is_active);
                    double dcbus_is_over60_v = mcb_tlb_bat_signals_status_dcbus_is_over60_v_decode(tlb_signals_rx.dcbus_is_over60_v);
                    double imp_any_is_active = mcb_tlb_bat_signals_status_imp_any_is_active_decode(tlb_signals_rx.imp_any_is_active);
                    double imp_hv_relays_state_is_active = mcb_tlb_bat_signals_status_imp_hv_relays_state_is_active_decode(tlb_signals_rx.imp_hv_relays_state_is_active);
                    double air_pos_stg_mech_state_signal_is_active = mcb_tlb_bat_signals_status_air_pos_stg_mech_state_signal_is_active_decode(tlb_signals_rx.air_pos_stg_mech_state_signal_is_active);
                    double air_neg_stg_mech_state_signal_is_active = mcb_tlb_bat_signals_status_air_neg_stg_mech_state_signal_is_active_decode(tlb_signals_rx.air_neg_stg_mech_state_signal_is_active);
                    double imp_dcbus_is_active = mcb_tlb_bat_signals_status_imp_dcbus_is_active_decode(tlb_signals_rx.imp_dcbus_is_active);

                    // Comparing buffer_tx and buffer_rx to check if transmission was successful
                    int buffer_match = 1; // Initialize match flag
                    for (int i = 0; i < RxHeader.DLC; i++) {
                        if (buffer_tx[i] != buffer_rx[i]) {
                            buffer_match = 0; // Mismatch detected
                            break;
                        }
                    }
                    // Handle buffer match or mismatch
                    if (buffer_match) {
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Turn LED ON if match
                    } else {
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Turn LED OFF if mismatch
                    }
                }
            }
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
