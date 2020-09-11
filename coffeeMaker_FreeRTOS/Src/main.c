/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t xPosition;
  uint8_t yPosition;
  char * textColor;
  char * textStr;
}queueCfg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//ANSI Escape Sequqnces for VT100 terminal emulation
#define RED_ON_BLK  "31;40"
#define WHT_ON_BLK  "37;40"
#define YEL_ON_BLK  "33;40"
#define GRN_ON_BLK  "32;40"
#define BLU_ON_BLK  "34;40"

#define BLK_ON_RED  "30;41"
#define BLK_ON_WHT  "30;47"
#define BLK_ON_YEL  "30;43"
#define BLK_ON_GRN  "30;42"

#define NORMAL WHT_ON_BLK

#define EVENT_GROUP_TEMPERATURE_SET (1<<0)
#define EVENT_GROUP_VOLUME_SET (1<<1)
#define EVENT_GROUP_BREW_ENABLED (1<<2)

#define BLUE_LED_Pin GPIO_PIN_9
#define BLUE_LED_GPIO_Port GPIOC
#define SET_BLUE_LED(enable) HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, !(enable))

#define SET_WARM_MODE(enable) HAL_GPIO_WritePin(ENABLE_HEATER_GPIO_Port, ENABLE_HEATER_Pin, !enable)
#define SET_BREW_MODE(enable) HAL_GPIO_WritePin(ENABLE_BREW_GPIO_Port, ENABLE_BREW_Pin, !enable)

#define STATE_INIT             0
#define STATE_SET_MODE         1
#define STATE_SET_VOLUME       2
#define STATE_SET_TEMPERATURE  3
#define STATE_BREW             4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
QueueHandle_t xMsgQueue;
SemaphoreHandle_t xButtonClickSemaphore;
ADC_ChannelConfTypeDef sConfig;
SemaphoreHandle_t xMutexADC1;
EventGroupHandle_t xEventGroup;
BaseType_t xHigherPriorityTaskWoken;
TimerHandle_t xBrewTimerAutoReload;
TimerHandle_t xUserInputFlashTimerOneShot;

uint32_t tempSetpoint=10;
uint32_t levelSetpoint=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t outputMsgCounter = 0;
static void updateUserInterface(int y, int x, char * clr, char * txt)
{
  queueCfg_t queueMsg;
  queueMsg.yPosition=y;
  queueMsg.xPosition=x;
  queueMsg.textColor=clr;
  queueMsg.textStr=txt;
  xQueueSendToBack(xMsgQueue, &queueMsg, portMAX_DELAY);
  //this delay intentionally slow queue TX processing
  //20msec is a 50Hz update rate
  vTaskDelay(pdMS_TO_TICKS(20));
  outputMsgCounter++;
}
//talkTag1
static void prvUserInputFlashTimerOneShot(TimerHandle_t xTimer)
{
  //turn LED OFF
  //since this GPIO is also controlled by an interrupt handler
  //need to designate this as a critical section for GPIO resourse management.
  taskENTER_CRITICAL();
  SET_BLUE_LED(0);
  taskEXIT_CRITICAL();
  (void)xTimer;
}

//talkTag2
uint32_t uptime=0;
static void prvBrewTimerAutoReload(TimerHandle_t xTimer)
{
  char buf[80];
  sprintf(buf, "%li secs", uptime);
  updateUserInterface(25, 27, YEL_ON_BLK,  buf);
  uptime++;
  (void)xTimer;
}

//talkTag3
static void prvTaskConsoleOutput(void* pvParameters)
{
  char buf[80];
  queueCfg_t queueMsg;

  //init output
  snprintf(buf, sizeof(buf), "\033[2J");
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 1000);

  for (;; )
  {
    //Wait for a user defined data element to become available on a queue.  If queue is not empty:
    uxQueueMessagesWaiting(xMsgQueue);
    while (uxQueueMessagesWaiting(xMsgQueue))
    {
      //Pull data element from the queue.
      xQueueReceive(xMsgQueue, &queueMsg, portMAX_DELAY);
      //Format the output char array
      sprintf(buf, "\033[%d;%dH\033[%sm%s", queueMsg.xPosition, queueMsg.yPosition, queueMsg.textColor, queueMsg.textStr);
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 1000);
      sprintf(buf, "\033[1D");
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 1000);
    }
  }
}
static void prvTaskProcessUserInput(void* pvParameters)
{
  uint8_t enableBrew = 0;
  uint8_t ySelectorPosition;
  uint8_t previous_ySelectorPosition;
  uint32_t adcReadVal;
  uint8_t buttonClicked = 0;

  char buf[80];

  //init state mechine to STATE_INIT
  uint8_t userInterfaceState = STATE_INIT;

  for (;;)
  {
    //hard code bypass of all userSelect code
    //shutdown the user interface in brew mode

    if(enableBrew == 0)
    {
      //Take mutex to access ADC.
      xSemaphoreTake(xMutexADC1, portMAX_DELAY);
      {
        sConfig.Channel = ADC_CHANNEL_1;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
        HAL_ADC_Start(&hadc1);
        while(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY)!=HAL_OK);
        adcReadVal = HAL_ADC_GetValue(&hadc1);
      }
      //Give mutex
      xSemaphoreGive(xMutexADC1);

      if(xSemaphoreTake(xButtonClickSemaphore,0))
      {
        //selects state active behavior
        buttonClicked=1;
      }
      else
      {
        //selects state default behavior
        buttonClicked=0;
      }

      //main process state machine
      //goes idle when brewEable = 1
      //states will reset buttonClicked to 0
      switch(userInterfaceState)
      {
        //state:STATE_INIT
        //one time init to the screen
        case(STATE_INIT):
        {
          //state:STATE_INIT
          //one time init to the screen
          //Perform a one-time initialization of the VT100 console.
          updateUserInterface(0, 1, BLU_ON_BLK,  "Welcome to Embedded Real-Time Operating Systems (RTOS)");
          updateUserInterface(0, 2, BLU_ON_BLK,  "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
          updateUserInterface(0, 3, BLU_ON_BLK,  "~            Assignment: Final Project               ~");
          updateUserInterface(0, 4, BLU_ON_BLK,  "~         Course Number: ECE-40290                   ~");
          updateUserInterface(0, 5, BLU_ON_BLK,  "~            Section ID: 146369                      ~");
          updateUserInterface(0, 6, BLU_ON_BLK,  "~          Student Name: Chris Isabelle              ~");
          updateUserInterface(0, 7, BLU_ON_BLK,  "~                   SID: U01136665                   ~");
          updateUserInterface(0, 8, BLU_ON_BLK,  "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

          updateUserInterface(4, 10, YEL_ON_BLK,  "  Coffee Level (oz)");
          updateUserInterface(16, 11, YEL_ON_BLK,  "   Set:");
          updateUserInterface(24, 11, RED_ON_BLK,  "unknown");
          updateUserInterface(16, 12, YEL_ON_BLK,  "Actual:");

          updateUserInterface(4, 14, YEL_ON_BLK,  "  Coffee Temperature");
          updateUserInterface(16, 15, YEL_ON_BLK,  "   Set:");
          updateUserInterface(24, 15, RED_ON_BLK,  "unknown");
          updateUserInterface(16, 16, YEL_ON_BLK,  "Actual:");

          updateUserInterface(4, 18, YEL_ON_BLK,  "  Start Brew");
          updateUserInterface(4, 20, GRN_ON_BLK,  ">>Use Joy-Stick to Select");
          //changes state to STATE_SET_MODE
          userInterfaceState = STATE_SET_MODE;
          //selects state default behavior
          buttonClicked=0;
          break;
        }

        //state:STATE_SET_MODE
        //Joy stick scrolls UP/DOWN and selects userInterfaceState
        case(STATE_SET_MODE):
        {
          //Scale and limit screen y value to line-up with screen options
          //1790 & 512 are manual cal adjust for specific platform
          ySelectorPosition += (((int16_t)adcReadVal)-1790)/512;
          ySelectorPosition = ySelectorPosition > 0xf ? 0xf : ySelectorPosition;
          ySelectorPosition = ySelectorPosition < 0 ? 0 : ySelectorPosition;
          ySelectorPosition &= 0xc;

          //default behavior : scroll up an down with current selection in GRN all other option are in YEL
          if(ySelectorPosition!=previous_ySelectorPosition)
          {
            updateUserInterface(4, 10, YEL_ON_BLK,  "  Coffee Level (oz)");
            updateUserInterface(4, 14, YEL_ON_BLK,  "  Coffee Temperature");
            updateUserInterface(4, 18, YEL_ON_BLK,  "  Start Brew");
            updateUserInterface(4, 20, YEL_ON_BLK,  "  Use Joy-Stick to Select");
            switch(ySelectorPosition)
            {
              case(0x0):updateUserInterface(4, 10, GRN_ON_BLK,  ">>Coffee Level (oz)"); break;
              case(0x4):updateUserInterface(4, 14, GRN_ON_BLK,  ">>Coffee Temperature"); break;
              case(0x8):updateUserInterface(4, 18, GRN_ON_BLK,  ">>Start Brew"); break;
              case(0xc):updateUserInterface(4, 20, GRN_ON_BLK,  ">>Use Joy-Stick to Select"); break;
            }
            previous_ySelectorPosition = ySelectorPosition;
            //add additional delay to slow slew rate
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          //active behavior : detect user button click and set userInterfaceState.
          if(buttonClicked)
          {
            //selects state default behavior
            buttonClicked=0;
            switch(ySelectorPosition)
            {
                            //user selection adjust coffee volume
                case(0x0):  userInterfaceState = STATE_SET_VOLUME;
                            break;
                            //user selection adjust coffee temperature
                case(0x4):  userInterfaceState = STATE_SET_TEMPERATURE;
                            break;
                case(0x8):  userInterfaceState = STATE_BREW;
                            updateUserInterface(4, 18, YEL_ON_BLK,  "  Start Brew");
                            updateUserInterface(6, 24, GRN_ON_BLK,  "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
                            updateUserInterface(6, 25, GRN_ON_BLK,  "~    Coffee Brew :          ~");
                            updateUserInterface(6, 26, GRN_ON_BLK,  "~  Coffee Warmer :          ~");
                            updateUserInterface(6, 27, GRN_ON_BLK,  "~      Brew Time :          ~");
                            updateUserInterface(6, 28, GRN_ON_BLK,  "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
                            break;
            }
          }
          break;
        }

       //state:STATE_SET_VOLUME
       //Joy stick scrolls UP/DOWN and selects levelSetpoint
        case(STATE_SET_VOLUME):
        {
          //active behavior : set userInterfaceState to STATE_SET_MODE, locks levelSetpoint
          if(buttonClicked)
          {
            //selects state default behavior
            buttonClicked=0;
            userInterfaceState = STATE_SET_MODE;
            xEventGroupSetBits( xEventGroup, EVENT_GROUP_VOLUME_SET);
            updateUserInterface(4, 10, YEL_ON_BLK,  "  Coffee Level (oz)");
            updateUserInterface(4, 20, GRN_ON_BLK,  ">>Use Joy-Stick to Select ");
            break;
          }
          //default behavior : scroll UP/DOWN in volume level in oz

          //adcReadVal of 3096 (~75% of ADC full scale) is a calibrated threshold for levelSetPoint decrement
          if(adcReadVal > 3096)
            levelSetpoint--;

          //adcReadVal of 1024 (~25% of ADC full scale) is a calibrated threshold for levelSetPoint increment
          if(adcReadVal < 1024)
            levelSetpoint++;

          //levelSetpoint maximum size is 20oz
          if(levelSetpoint > 20)
            levelSetpoint=20;

          //levelSetpoint minimum (and default) size is 4oz
          if(levelSetpoint < 4)
            levelSetpoint=4;

          //formats msg for real time ASNI updates to the trminal
          sprintf(buf, "%li oz    ", levelSetpoint);
          updateUserInterface(24, 11, GRN_ON_BLK,  buf);

          //add additional 100ms delay to slow slew rate
          vTaskDelay(pdMS_TO_TICKS(100));

          break;
        }

        //state:STATE_SET_TEMPERATURE
        //Joy stick scrolls UP/DOWN and selects tempSetpoint
        case(STATE_SET_TEMPERATURE):
        {
          //active behavior : set userInterfaceState to STATE_SET_MODE, locks tempSetpoint
          if(buttonClicked)
          {
            //selects state default behavior
            buttonClicked=0;
            userInterfaceState = STATE_SET_MODE;
            xEventGroupSetBits( xEventGroup, EVENT_GROUP_TEMPERATURE_SET);
            updateUserInterface(4, 14, YEL_ON_BLK,  "  Coffee Temperature (degC)");
            updateUserInterface(4, 20, GRN_ON_BLK,  ">>Use Joy-Stick to Select ");
            break;
          }

          //default behavior : scroll UP/DOWN in volume level in oz

          //adcReadVal of 3096 (~75% of ADC full scale) is a calibrated threshold for levelSetPoint decrement
          if(adcReadVal > 3096)
            tempSetpoint--;

          //adcReadVal of 1024 (~25% of ADC full scale) is a calibrated threshold for levelSetPoint increment
          if(adcReadVal < 1024)
            tempSetpoint++;

          //levelSetpoint maximum size is 60degC
          if(tempSetpoint>60)
            tempSetpoint=60;

          //tempSetpoint minimum (and default) size is 24degC
          if(tempSetpoint<24)
            tempSetpoint=24;

          //formats msg for real time ASNI updates to the trminal
          sprintf(buf, "%li degC / %li degF ", tempSetpoint, (uint32_t)((double)tempSetpoint * 1.8) + 32);
          updateUserInterface(24, 15, GRN_ON_BLK,  buf);

          //add additional 100msec delay to slow slew rate
          vTaskDelay(pdMS_TO_TICKS(100));

          break;
        }

        //state:STATE_SET_BREW
        //Joy stick selects BREW
        //xEventGroup EVENT_GROUP_BREW_ENABLED=1, releases  prvTaskProcessTemperature & prvTaskProcessLevel
        case(STATE_BREW):
        {
          //selects state default behavior
          buttonClicked=0;
          xEventGroupSetBits( xEventGroup, EVENT_GROUP_BREW_ENABLED);

          //start Brew timer
          xTimerStart(xBrewTimerAutoReload, 0);

          updateUserInterface(4, 20, GRN_ON_BLK,  ">>Use Joy-Stick to Select ");
          //set task to idle
          enableBrew = 1;
          break;
        }

        //state:INVALID
        default:
          //exception
          while(1);
      }
    }
    //process user interface ~ 10HZ
    //100msec is a 10Hz update rate
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void prvTaskProcessTemperature(void* pvParameters)
{
  char buf[80];
  uint32_t adcTemperature;
  for (;;)
  {
    //Check event group for OK to monitor & control temperature.
    xEventGroupWaitBits(xEventGroup,
        EVENT_GROUP_TEMPERATURE_SET | EVENT_GROUP_BREW_ENABLED,
        pdFALSE, //do not clear bits
        pdTRUE,  //wait for all bits
        portMAX_DELAY);

    //Take mutex to access ADC.
    xSemaphoreTake(xMutexADC1, portMAX_DELAY);
    {
      //Read ADC to acquire coffee temperature.
      sConfig.Channel = ADC_CHANNEL_3;
      HAL_ADC_ConfigChannel(&hadc1, &sConfig);
      HAL_ADC_Start(&hadc1);
      while(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY)!=HAL_OK);
      adcTemperature = (uint32_t)( ((float)HAL_ADC_GetValue(&hadc1)/4096) * 100);
    }
    //Give mutex
    xSemaphoreGive(xMutexADC1);

    //Send output message to pvTaskConsoleOutput by placing a user defined data element on a queue.
    sprintf(buf, "%li degC / %li degF ", adcTemperature, (uint32_t)((double)adcTemperature * 1.8) + 32);
    updateUserInterface(24, 16, GRN_ON_BLK, buf);

    //Enable coffee warmer if coffee temperature below set point.
    if(adcTemperature > (tempSetpoint+1))
    {
      SET_WARM_MODE(0);
      updateUserInterface(25, 26, YEL_ON_BLK, "OFF");
    }

    //Disable coffee warmer if coffee temperature above the set point.
    if(adcTemperature < (tempSetpoint-1))
    {
      SET_WARM_MODE(1);
      updateUserInterface(25, 26, RED_ON_BLK, "ON ");
    }

    //update temperature at  1hz rate
    //delay 1000msec
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
static void prvTaskProcessLevel(void* pvParameters)
{
  char buf[80];
  uint32_t adcLevel;
  for (;;)
  {
    //Check event group for OK to monitor & control level.
    xEventGroupWaitBits(xEventGroup,
        EVENT_GROUP_VOLUME_SET | EVENT_GROUP_BREW_ENABLED,
        pdFALSE, //do not clear bits
        pdTRUE,  //wait for all bits
        portMAX_DELAY);

    //Take mutex to access ADC.
    xSemaphoreTake(xMutexADC1, portMAX_DELAY);
    {
      //Read ADC to acquire coffee level.
      sConfig.Channel = ADC_CHANNEL_2;
      HAL_ADC_ConfigChannel(&hadc1, &sConfig);
      HAL_ADC_Start(&hadc1);
      while(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY)!=HAL_OK);
      adcLevel = (uint32_t)( ((float)HAL_ADC_GetValue(&hadc1)/4096) * 24);
    }
    //Give mutex
    xSemaphoreGive(xMutexADC1);

    //Send output message to pvTaskConsoleOutput by placing a user defined data element on a queue.
    sprintf(buf, "%li oz  ", adcLevel);
    updateUserInterface(24, 12, GRN_ON_BLK, buf);

    if(adcLevel >= levelSetpoint)
    {
      SET_BREW_MODE(0);
      updateUserInterface(25, 25, RED_ON_BLK, "READY");
//      //this terminates the coffee level processing until a reset/restart
//      xEventGroupSetBits( xEventGroup, EVENT_GROUP_VOLUME_SET);
    }
    else
    {
      SET_BREW_MODE(1);
      updateUserInterface(25, 25, GRN_ON_BLK, "ON   ");
    }
    //update volume at 1hz rate
    //delay 1000msec
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
//C callback function to trap interrupt for Blue <USER> button press
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  xHigherPriorityTaskWoken = pdFALSE;
  UBaseType_t unSavedInterruptStatus;

  if(GPIO_Pin == BUTTON_EXTI13_Pin)
  {
    //turn LED ON
    //since this GPIO is also controlled by a task
    //need to designate this as a critical section for GPIO resourse management.
    unSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    SET_BLUE_LED(1);
    taskEXIT_CRITICAL_FROM_ISR(unSavedInterruptStatus);

    //start one shot timer that turns LED off.
    xTimerStartFromISR(xUserInputFlashTimerOneShot, 0);
    xSemaphoreGiveFromISR(xButtonClickSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  xEventGroup = xEventGroupCreate();
  //this sequence resets and initializes the simulator
  SET_WARM_MODE(1);
  SET_BREW_MODE(1);
  HAL_Delay(500);
  SET_WARM_MODE(0);
  SET_BREW_MODE(0);
  //initialize Blue LED to off
  SET_BLUE_LED(0);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  xMutexADC1 = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  xButtonClickSemaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

  //Toggles LED every one second
  xBrewTimerAutoReload = xTimerCreate("AutoReload", /* The text name assigned to the timer. */
    pdMS_TO_TICKS(1000),            /* Timer delay. */
    pdTRUE,                         /* AutoRestart = TRUE. */
    0,                              /* pvTimerID */
    prvBrewTimerAutoReload);        /* The function that implements the timer. */

  //Turns off LED aftr .5 seconds
  xUserInputFlashTimerOneShot = xTimerCreate("OneShot", /* The text name assigned to the timer. */
    pdMS_TO_TICKS(500),             /* Timer delay. */
    pdFALSE,                        /* AutoRestart = TRUE. */
    0,                              /* pvTimerID */
    prvUserInputFlashTimerOneShot);        /* The function that implements the timer. */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  //create a queue 5 deep of user defined queueCfg_t data elements
  xMsgQueue = xQueueCreate(10, sizeof(queueCfg_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  BaseType_t rtnVal;
  rtnVal = xTaskCreate(prvTaskConsoleOutput, "cons-out task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  if(rtnVal == -1)  //exception
    while(1);

  rtnVal = xTaskCreate(prvTaskProcessUserInput, "proc-user task", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
  if(rtnVal == -1)  //exception
    while(1);

  rtnVal = xTaskCreate(prvTaskProcessTemperature, "pro-temp task", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
  if(rtnVal == -1)  //exception
    while(1);

  rtnVal = xTaskCreate(prvTaskProcessLevel, "proc-vol task", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
  if(rtnVal == -1)  //exception
    while(1);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ENABLE_HEATER_Pin|ENABLE_BREW_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin 
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE_HEATER_Pin ENABLE_BREW_Pin VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = ENABLE_HEATER_Pin|ENABLE_BREW_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin 
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin 
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin 
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin 
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
