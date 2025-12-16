/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_PWM_MAX 1800  /* PWM最大值*/
#define TARGET_SPEED 1900    /* 基础目标速度*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* PID参数 */
float KP_turn = 90.0f;      /* 转向环比例系数*/
float KD_turn = -60.0f;       /* 转向环微分系数*/
float KP_speed = 30.0f;      /* 速度环比例系数  */
float KI_speed = 0.3f;     /* 速度环积分系数*/

/* 角速度环参数 */
float KP_gyro = 8.0f;       /* 角速度环比例系数*/
float KD_gyro = 0.08f;       /* 角速度环微分系数*/
float target_gyro_rate = 0.0f; /* 目标角速度*/
float last_gyro_error = 0.0f;  /* 上一次角速度误差*/

/* 全局变量 */
int32_t left_encoder = 0;  /* 左编码器值*/
int32_t right_encoder = 0; /* 右编码器值*/
int32_t last_left_encoder = 0;
int32_t last_right_encoder = 0;
float left_speed = 0;      /* 左电机实际速度*/
float right_speed = 0;     /* 右电机实际速度*/
float last_deviation = 0;  /* 上一次偏差*/
float left_integral = 0;   /* 左电机速度积分*/
float right_integral = 0;  /* 右电机速度积分*/
float left_pwm = 0;
float right_pwm = 0;

/* 新增变量 */
float last_left_speed = 0;     /* 左电机上一次速度（用于滤波）*/
float last_right_speed = 0;    /* 右电机上一次速度（用于滤波）*/
uint32_t last_speed_time = 0;  /* 上一次速度计算时间*/

/* 光电管权值定义 */
int phototube_weights[12] = {11, 9, 7, 5, 3, 1, -1, -3, -5, -7, -9, -11};

float gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z; /*陀螺仪数据*/
float integrated_angle = 0.0f; /* 积分角度*/
float last_gyro_z = 0.0f;     /* 上一次陀螺仪Z轴数据*/
uint32_t last_gyro_time = 0;  /* 上一次陀螺仪采样时间*/	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
float Calculate_Deviation(void);
void Motor_Control(float left_pwm, float right_pwm);
void PID_Control(void);
void Control_Init(void);
void Update_Gyro_Data(void);
float Gyro_Rate_Control(float target_rate, float current_rate); /* 新增角速度环控制函数*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/* 更新陀螺仪数据并积分 */
void Update_Gyro_Data(void)
{
  dodo_BMI270_get_data(); /*获取陀螺仪数据*/
    
  /* 转换陀螺仪数据*/
  gyro_z = BMI270_gyro_transition(BMI270_gyro_z); /*角速度*/
    
  /* 角速度积分得到角度*/
  uint32_t current_time = HAL_GetTick();
  if(last_gyro_time > 0) {
      float dt = (current_time - last_gyro_time) / 1000.0f; /* 转换为秒*/
      integrated_angle += gyro_z * dt;
        
      /* 角度限制在180度范围内*/
      if(integrated_angle > 180.0f) integrated_angle = 180.0f;
      if(integrated_angle < -180.0f) integrated_angle = -180.0f;
  }
  last_gyro_time = current_time;
  last_gyro_z = gyro_z;
}

/* 预测偏差函数 - 基于历史数据 */
float Predict_Deviation(float history[], int size)
{
    float sum = 0;
    int valid_count = 0;
    int i;
    
    /* 计算有效历史数据的平均值*/
    for(i = 0; i < size; i++) {
        if(history[i] != 0) {
            sum += history[i];
            valid_count++;
        }
    }
    
    if(valid_count > 0) {
        return sum / valid_count;
    } else {
        return 0.0f;
    }
}

/* 计算偏差趋势函数 */
float Calculate_Deviation_Trend(float history[], int size)
{
    if(size < 2) return 0;
    
    float sum = 0;
    int count = 0;
    int i;
    
    /* 计算相邻点的变化趋势*/
    for(i = 1; i < size; i++) {
        if(history[i] != 0 && history[i-1] != 0) {
            sum += (history[i] - history[i-1]);
            count++;
        }
    }
    
    if(count > 0) {
        return sum / count;
    } else {
        return 0;
    }
}

/* 计算光电管偏差值 - 增强虚线处理逻辑 */
float Calculate_Deviation(void)
{
    uint16_t mux_value;
    MUX_get_value(&mux_value);
    
    float deviation = 0;
    int count = 0;
    int left_count = 0;
    int right_count = 0;
    int i;
    
    /* 检查所有光电管状态*/
    for(i = 0; i < 12; i++) {
        if(MUX_GET_CHANNEL(mux_value, i)) {
            deviation += phototube_weights[i];
            count++;
            
            /* 统计左右侧检测到的数量*/
            if(i < 6) left_count++;
            else right_count++;
        }
    }
    
    /* 根据不同情况处理偏差*/
    if(count > 0) {
        /* 正常情况：计算加权平均偏差*/
        deviation = deviation / count;
        
        /* 普通转弯情况：如果只有单侧检测到，适当增强偏差*/
        if(left_count > 0 && right_count == 0) {
            deviation -= 1.5f; /* 向左增强*/
        } else if(right_count > 0 && left_count == 0) {
            deviation += 1.5f; /* 向右增强*/
        }
    } else {
        /* 没有检测到线的处理策略 - 简单搜索*/
        if(last_deviation > 3.0f) {
            /* 上次偏右，继续向右寻找*/
            deviation = 8.0f;
        } else if(last_deviation < -3.0f) {
            /* 上次偏左，继续向左寻找*/
            deviation = -8.0f;
        } else {
            /* 默认向右转寻找*/
            deviation = -8.0f;
        }
    }
    
    /* 限制偏差范围*/
    if(deviation > 11.0f) deviation = 11.0f;
    if(deviation < -11.0f) deviation = -11.0f;
    
    /* 调试输出*/
    static uint32_t debug_count = 0;
    if(debug_count % 50 == 0) {
        printf("Line: L:%d R:%d | Dev:%.1f\n", 
               left_count, right_count, deviation);
    }
    debug_count++;
    
    return deviation;
}


/* 角速度环控制函数 */
float Gyro_Rate_Control(float target_rate, float current_rate)
{
    float error = target_rate - current_rate;
    float derivative = error - last_gyro_error;
    last_gyro_error = error;
    
    /* PD控制*/
    float output = KP_gyro * error + KD_gyro * derivative;
    
    return output;
}

/* 电机控制函数 */
void Motor_Control(float left_pwm, float right_pwm)
{
    /* 限制PWM范围*/
    if(left_pwm > MOTOR_PWM_MAX) left_pwm = MOTOR_PWM_MAX;
    if(left_pwm < -MOTOR_PWM_MAX) left_pwm = -MOTOR_PWM_MAX;
    if(right_pwm > MOTOR_PWM_MAX) right_pwm = MOTOR_PWM_MAX;  
    if(right_pwm < -MOTOR_PWM_MAX) right_pwm = -MOTOR_PWM_MAX;
    
    /* 设置电机方向和PWM*/
    if(left_pwm >= 0) {
        HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, GPIO_PIN_RESET); /* 正转*/
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)left_pwm);
    } else {
        HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin, GPIO_PIN_SET); /* 反转*/
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(-left_pwm));
    }
    
    if(right_pwm >= 0) {
        HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, GPIO_PIN_SET); /* 正转*/
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)right_pwm);
    } else {
        HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin, GPIO_PIN_RESET); /* 反转*/
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(-right_pwm));
    }
}

/* 主PID控制函数 - 简化版：只有速度环和转向环 */
void PID_Control(void)
{
    float current_target_speed = 500.0f; /* 暂时使用固定值，实际应从Calculate_Deviation获取*/
	
    Update_Gyro_Data();	
		
    /* 1. 读取编码器值计算实际速度*/
    left_encoder = __HAL_TIM_GET_COUNTER(&htim4);
    right_encoder = __HAL_TIM_GET_COUNTER(&htim3);
    
    /* 更新编码器记录*/
    last_left_encoder = left_encoder;
    last_right_encoder = right_encoder;
    
    /* 2. 转向环PD控制*/
    float current_deviation = Calculate_Deviation();
    float turn_output = KP_turn * current_deviation + 
                       KD_turn * (current_deviation - last_deviation);
    last_deviation = current_deviation;
    
    /* 3. 角速度环控制 - 将转向环输出作为目标角速度*/
    target_gyro_rate = turn_output;
    float gyro_output = Gyro_Rate_Control(target_gyro_rate, gyro_z);
		
    /* 4. 计算左右电机目标速度（转向环输出直接叠加到目标速度）*/
    float left_target = current_target_speed - gyro_output;
    float right_target = current_target_speed + gyro_output;
    
    /* 5. 速度环PI控制*/
    float left_error = left_target - left_speed;
    float right_error = right_target - right_speed;
    
    /* 积分项*/
    left_integral += left_error;
    right_integral += right_error;
    
    /* 积分限幅*/
    if(left_integral > 1000)   left_integral = 1000;
    if(left_integral < -1000)  left_integral = -1000;
    if(right_integral > 1000)  right_integral = 1000;  
    if(right_integral < -1000) right_integral = -1000;
    
    /* 计算最终PWM输出*/
    left_pwm = KP_speed * left_error + KI_speed * left_integral;
    right_pwm = KP_speed * right_error + KI_speed * right_integral;
    
    /* 电机输出*/
    Motor_Control(left_pwm, right_pwm);
}

/* 初始化函数 */
void Control_Init(void)
{
    /* 启动PWM*/
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    
    /* 启动编码器模式*/
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    
    /* 启动定时器中断（控制周期，比如1kHz）*/
    HAL_TIM_Base_Start_IT(&htim2);
    
    /* 初始化变量*/
    last_left_encoder = __HAL_TIM_GET_COUNTER(&htim4);
    last_right_encoder = __HAL_TIM_GET_COUNTER(&htim3);
    last_gyro_time = HAL_GetTick();
    last_speed_time = HAL_GetTick(); /* 新增速度时间初始化*/
    
    /* 初始读取一次光电管值，避免last_deviation为0*/
    last_deviation = Calculate_Deviation();
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
    dodo_BMI270_init(); /*初始化陀螺仪*/
  Control_Init(); /* 初始化控制系统*/
  
  printf("Car Racing System Started!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
        int i;
        /* 调试输出：可以定期输出传感器数据用于调试*/
        static uint32_t last_debug_time = 0;
        if(HAL_GetTick() - last_debug_time > 500) { /* 每500ms输出一次*/
            printf("GyroZ: %.2f deg/s, Angle: %.2f deg, PhotoDev: %.2f\r\n", 
                   gyro_z, integrated_angle, last_deviation);
            last_debug_time = HAL_GetTick();
        }
		
        uint16_t mux_value;
        MUX_get_value(&mux_value);
        for(i = 0; i <= 11; i++){
            printf("%d,", MUX_GET_CHANNEL(mux_value, i)); /*获取第i个光电管的数值并输出*/
        }
        printf("\n");
   /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2) { /* 使用TIM2作为控制周期*/
        PID_Control();
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|R_DIR_Pin|MUX_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_DIR_Pin|MUX_1_Pin|MUX_2_Pin|MUX_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L_DIR_Pin */
  GPIO_InitStruct.Pin = L_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R_DIR_Pin */
  GPIO_InitStruct.Pin = R_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_READ_Pin */
  GPIO_InitStruct.Pin = MUX_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_READ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_0_Pin */
  GPIO_InitStruct.Pin = MUX_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MUX_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_1_Pin MUX_2_Pin MUX_3_Pin */
  GPIO_InitStruct.Pin = MUX_1_Pin|MUX_2_Pin|MUX_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
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
void assert_failed(uint8_t *file, uint32_t *line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */