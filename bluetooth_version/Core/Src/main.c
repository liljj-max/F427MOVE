/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_protocol.h"
#include "CAN_motor.h"
#include "kinematics.h"
#include "pid.h"
#include "usbd_cdc_if.h" 
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
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

static ChassisCmd_t target_cmd = {0};
static uint32_t last_cmd_time = 0;

static PID_t speed_pid[MOTOR_NUM];
static ChassisFeedback_t chassis_fb = {0};

//手机接单片机版本
static uint8_t uart_rx_buf[8];

// CAN错误状态
volatile uint8_t can2_error_status = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CAN_Filter_Config(){
	CAN_FilterTypeDef sFilterConfig;

	
	// ========== CAN2过滤器：只收电机反馈 ID=0x201~0x204 ==========
    //uint32_t motor_ids[] = {0x201, 0x202, 0x203, 0x204};
    

        sFilterConfig.FilterBank = 14 ;//用第十四个过滤器
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterIdHigh = (CAN2_M3508_CMD_BASE<< 5);
        sFilterConfig.FilterIdLow = 0x0000;
			sFilterConfig.FilterMaskIdHigh = 0x7F0<<5|0x3;//IDE和RTR匹配
        sFilterConfig.FilterMaskIdLow = 0;
        sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
        sFilterConfig.FilterActivation = ENABLE;
        HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	
}

//USB CDC接收回调（工控机发给stm速度和角度信息)
/*  USB为什么和 UART 不一样
UART 的接收回调 HAL_UART_RxCpltCallback() 是个 weak 函数（弱定义），
HAL 库里已经给你写好了默认实现（空的），
你在 main.c 里重新定义一个同名的，编译器就会用你的版本代替默认的——这叫弱符号覆盖。
USB CDC 不一样。它的接收回调 CDC_Receive_FS() 是通过函数指针注册到
这个结构体被 USB 协议栈在初始化时读进去，后续有数据过来时就通过这个指针去调用 CDC_Receive_FS()。
它不是 weak 函数，不能靠同名覆盖，你必须直接在 usbd_cdc_if.c 里改 CDC_Receive_FS() 的函数体。
*/


void USB_CMD_Process(uint8_t *buf, uint32_t len)
{
    if(len >= sizeof(ChassisCmd_t)) {
        ChassisCmd_t *cmd = (ChassisCmd_t*)buf;
        target_cmd.vx = cmd->vx;
        target_cmd.vy = cmd->vy;
        target_cmd.wz = cmd->wz;
        target_cmd.enable = cmd->enable;
        last_cmd_time = HAL_GetTick();
    }
}






// CAN2接收回调（电机反馈），反馈电流，转速，角度，温度
void CAN2_RxCallback(uint32_t rx_id, uint8_t *data, uint8_t dlc)
{
    if(rx_id >= 0x201 && rx_id <= 0x204) {
        MotorCAN2_ProcessRx(rx_id, data);
    }
}


//当错误数累积到255后你将会得到

// ============ CAN2 SCE中断回调（错误处理）============
void CAN2_SCE_Callback(void)
{
    uint32_t esr = hcan2.Instance->ESR;
    
    if(esr & CAN_ESR_BOFF) {
        can2_error_status = 3;
        // CAN2 BusOff，尝试恢复电机通信
        MotorCAN2_ResetBus();
    } else if(esr & CAN_ESR_EPVF) {
        can2_error_status = 2;
    } else if(esr & CAN_ESR_EWGF) {
        can2_error_status = 1;
    }
    
    __HAL_CAN_CLEAR_FLAG(&hcan2, CAN_FLAG_BOF);
    __HAL_CAN_CLEAR_FLAG(&hcan2, CAN_FLAG_EPV);
    __HAL_CAN_CLEAR_FLAG(&hcan2, CAN_FLAG_EWG);
}

/*CAN收发数据的时候都要做一个  r/tx_header和对应的data数组存储数据*/

// ============ HAL库CAN中断回调（CubeMX自动调用这些函数）============


// CAN2接收中断
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN2) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK) {
            CAN2_RxCallback(rx_header.StdId, rx_data, rx_header.DLC);
        }
    }
}

//CAN错误中断
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
   
   if(hcan->Instance == CAN2) {
        CAN2_SCE_Callback();
    }
}

//CAN初始化函数
void CAN_Start(){
	  //配置过滤器
	  CAN_Filter_Config();
	  HAL_Delay(10);
	 
	
	// 3. 启用 FIFO0 消息挂起中断（有数据来时触发）
	
	  //启动CAN2
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    // 启用 FIFO1 消息挂起中断（有数据到达时触发）
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    // 启用错误中断（用于监控总线错误）
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR) != HAL_OK)
    {
        Error_Handler();
    }
}

// ============ 更新里程计 ============
void UpdateOdometry(void)
{
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    if(dt > 0.1f) dt = 0.01f;
    

    int16_t wheel_rpm[MOTOR_NUM];
    ChassisSpeed_t actual_speed;
    for(int i = 0; i < MOTOR_NUM; i++) {
        wheel_rpm[i] = motor_fb[i].speed_rpm;
    }
    Kinematics_Forward(wheel_rpm, &actual_speed);
// 速度反馈
    chassis_fb.vx_actual = actual_speed.vx;
    chassis_fb.vy_actual = actual_speed.vy;


    chassis_fb.timestamp_ms = now;
    
    last_time = now;
}





// ============ 发送底盘状态到工控机（USB）============
void SendChassisFeedback(void)
{
	 chassis_fb.header = FRAME_HEADER;  
   chassis_fb.error_code = can2_error_status;  // 只有CAN2的状态了
   chassis_fb.can2_status = can2_error_status;
    
    // 通过USB CDC虚拟串口发送
    CDC_Transmit_FS((uint8_t*)&chassis_fb, sizeof(ChassisFeedback_t));
}



// ============ 定时器中断（控制周期10ms）============
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM3){

		 static int16_t target_rpm[MOTOR_NUM];
     static float current_cmd[MOTOR_NUM];
		
		
        Motor_CheckOnline();  // ← 新增：更新丢帧计数
		
		 // 超时或急停检查
        if((HAL_GetTick() - last_cmd_time > CMD_TIMEOUT_MS) || (target_cmd.enable == 0)) {                MotorCAN2_SendCurrent(0,0,0, 0);
            return;
        }
		  // 运动学逆解（线速度到转速）
        Kinematics_Calculate(*(ChassisSpeed_t*)&target_cmd, target_rpm);
			
			 // PID控制
        for(int i = 0; i < MOTOR_NUM; i++)
        {
            if(Motor_IsOnline(i + 1))                     // ← 在线才跑 PID
            {
                current_cmd[i] = (int16_t)PID_Calculate(
                    &speed_pid[i],
                    (float)target_rpm[i],
                    (float)motor_fb[i].speed_rpm);
            }
            else
            {
                current_cmd[i] = 0;                       // ← 离线电机输出 0
                PID_Reset(&speed_pid[i]);                 // ← 清空积分，防止恢复时猛冲
            }
        }

        MotorCAN2_SendCurrent(current_cmd[0], current_cmd[1],
                              current_cmd[2], current_cmd[3]);

        UpdateOdometry();
				
				// 每100ms发送一次状态反馈
        static uint8_t fb_counter = 0;
        if(++fb_counter >= 10) {
            SendChassisFeedback();
            fb_counter = 0;
        }
	}
	
}
	
	
	



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
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_4);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_4);
	HAL_Delay(100);
	
	CAN_Start();
  for(int i = 0; i < MOTOR_NUM; i++) {
        PID_Init(&speed_pid[i], PID_KP, PID_KI, PID_KD, PID_OUT_MAX, PID_INT_LIMIT,PID_INTEGRAL_SEPARATE_THR);
    }
	
	// 启动定时器
    HAL_TIM_Base_Start_IT(&htim3);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		HAL_UART_Receive_IT(&huart1, uart_rx_buf, sizeof(ChassisCmd_t));
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		
        ChassisCmd_t *cmd = (ChassisCmd_t*)uart_rx_buf;
        target_cmd.vx = cmd->vx;
        target_cmd.vy = cmd->vy;
        target_cmd.wz = cmd->wz;
        target_cmd.enable = cmd->enable;
        last_cmd_time = HAL_GetTick();
        HAL_UART_Receive_IT(&huart1, uart_rx_buf, 8);
		/*
		什么时候才会需要清理缓冲区
		情况	                                要不要清	                   为什么
正常收到一帧，处理完了	                  ✅ 要清	             把已处理的 12 字节从缓冲区删掉，腾空间
收到垃圾数据，永远找不到帧头	            ✅ 要清	             防止缓冲区无限涨，设个上限（比如 1024 字节）就清空重来
用 HAL_UART_Receive_IT() 收固定长度	      ❌ 不用	             HAL 内部都管好了
		
		*/
		
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
