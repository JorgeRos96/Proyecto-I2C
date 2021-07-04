 /**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   Proyecto que emplea el I2C para obtener el valor de la temperatura
	* 				 y el valor del acelerometro de los ejes x,y o z
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Driver_I2C.h"
#include <string.h>
#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS2                  // when RTE component CMSIS RTOS2 is used
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#endif

#ifdef RTE_CMSIS_RTOS2_RTX5
/**
  * Override default HAL_GetTick function
  */
uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  if (osKernelGetState () == osKernelRunning) {
    return ((uint32_t)osKernelGetTickCount ());
  }

  /* If Kernel is not running wait approximately 1 ms then increment 
     and return auxiliary tick counter value */
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
}
#endif

/* Private define ------------------------------------------------------------*/
#define LM75B_I2C_ADDR 0x48
#define ACC_ADR_W 0x4C

/* Private variables ---------------------------------------------------------*/
extern ARM_DRIVER_I2C Driver_I2C1;
static ARM_DRIVER_I2C *I2Cdev = &Driver_I2C1;

uint8_t buf [2];
float temperatura;
float ejex;
float ejey;
float ejez;
uint16_t aux;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void MPU_Config(void);


/* Private functions ---------------------------------------------------------*/
float leerTemperatura();
float leeraxis(uint8_t eje );
void initI2c();
void retardo();
void init_acc();
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* This project template calls firstly two functions in order to configure MPU feature 
     and to enable the CPU Cache, respectively MPU_Config() and CPU_CACHE_Enable().
     These functions are provided as template implementation that User may integrate 
     in his application, to enhance the performance in case of use of AXI interface 
     with several masters. */ 
  
  /* Configure the MPU attributes as Write Through */
  MPU_Config();

  /* Enable the CPU Cache */


  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the System clock to have a frequency of 216 MHz */
  SystemClock_Config();
  SystemCoreClockUpdate();


  /* Add your application code here
     */
	initI2c();
	init_acc();
#ifdef RTE_CMSIS_RTOS2
  /* Initialize CMSIS-RTOS2 */
  osKernelInitialize ();

  /* Create thread functions that start executing, 
  Example: osThreadNew(app_main, NULL, NULL); */

  /* Start thread execution */
  osKernelStart();
#endif

  /* Infinite loop */
  while (1)
  {
		
		
		temperatura = leerTemperatura();
		retardo(20);
		ejex = leeraxis(0x00);
		retardo(20);
		ejey = leeraxis(0x01);
		retardo(20);
		ejez = leeraxis(0x02);
		retardo(20);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 226;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes as Write Through Internal SRAM1/SRAM2.
  * @note   The Base Address is 0x20000000 and Region Size 512KB.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress      = 0x20000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */


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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief Función de inicialización del I2C
	* @param None
  * @retval None
  */
void initI2c(){
	
	I2Cdev -> Initialize (NULL);
	I2Cdev -> PowerControl (ARM_POWER_FULL);
	I2Cdev -> Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
	I2Cdev -> Control (ARM_I2C_BUS_CLEAR,0);	
	
}
/**
  * @brief Función para realizar un wait del tiempo introducido  
	* @param n_milisegundos: tiempo en milisegundos que realiza el wait
  * @retval None
  */
void retardo(uint32_t n_milisegundos){
	uint32_t i=0;
	uint32_t cuenta = 0;
	cuenta = n_milisegundos*1000*80;
	for(i=0;i < cuenta; i++);
	
}
/**
  * @brief Función de inicialización del acelerometro
	* @param 
  * @retval None
  */
void init_acc()
	{
	uint8_t a[2];
	
	a[1] = 0x01;
	a[0] = 0x07;


	I2Cdev -> MasterTransmit(ACC_ADR_W, a, 2, false);
	while(I2Cdev -> GetStatus().busy);	
	
}
/**
  * @brief Función que lee el valor del eje que se pasa por parametro
	* @param eje: eje del cual se quiere obtener el valor del acelerometro
  * @retval None
  */
float leeraxis(uint8_t eje )
	{
	uint8_t  ax;
		float axis;

	uint8_t buff;

		
	if (eje == 0){
	I2Cdev -> MasterTransmit(ACC_ADR_W, &eje, 1, true);
	while(I2Cdev -> GetStatus().busy);
	
	I2Cdev -> MasterReceive(ACC_ADR_W, &buff, 1, false);
	while(I2Cdev -> GetStatus().busy);
	}
	else if (eje == 1){
	I2Cdev -> MasterTransmit(ACC_ADR_W,&eje, 1, true);
	while(I2Cdev -> GetStatus().busy);
	
	I2Cdev -> MasterReceive(ACC_ADR_W, &buff, 1, false);
	while(I2Cdev -> GetStatus().busy);
	}
	else{
	I2Cdev -> MasterTransmit(ACC_ADR_W, &eje, 1, true);
	while(I2Cdev -> GetStatus().busy);
	
	I2Cdev -> MasterReceive(ACC_ADR_W, &buff, 1, false);
	while(I2Cdev -> GetStatus().busy);
	}
	
		axis = ((char)(buff));
	
	return axis;
}

/**
  * @brief Función que permite obtener el valor de la temperatura del sensor de temperatura
	* @param None
  * @retval None
  */
float leerTemperatura(){
	float temp;
	
	I2Cdev -> MasterTransmit(LM75B_I2C_ADDR, 0x00, 1, true);
	while(I2Cdev -> GetStatus().busy);
	
	I2Cdev -> MasterReceive(LM75B_I2C_ADDR, buf, 2, true);
	while(I2Cdev -> GetStatus().busy);

	aux = ((buf[0]<<8)|buf[1])>>5;
	
	if ( aux & (1 << 10) )
		temp = -((~aux)+1)*0.125;
	else 
		temp = aux *0.125;
	return temp;
}
/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
