 /**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   Proyecto que emplea el I2C para obtener el valor de la temperatura
	* 				 y el valor del acelerometro de los ejes x,y o z.
	*					 Se configura el I2C en el RTE_Device.h utilizando el I2C1 con los 
	*					 pines:
	*					 - Pin SCL: PB8
	*					 - Pin SDA: PB9
	*					 Para utilizar los dos preifericos con la misma comunicación I2C, a la 
	*					 hora de realizar la comunicación se identifican mediante su dirección.
	*					 En el sensor de temperatura se indica en su datasheet que se dejan
	*					 los últimos tres bits para seleccionar el periférico. Como el 
	*					 acelerometro viene descrita por defecto una dirección en su datasheet, 
	*					 las direcciones de ambos perifericos son:
	*					 - Dirección del sensor de temperatura: 0x48
	*					 - Dirección del acelerómetro: 0x4C
	*
	*					 Se emplea la USART para enviar los datos de la temperatura y el valor
	*					 de los tres ejes al terminal cada dos segundos.
  *
	*					 Se configura el reloj del sistema para que trabaje a una frecuencia 
	*					 de 180 MHz utilizando como fuente de reloj el PLL con el HSI. Con 
	*					 esta frecuencia del sistema se configuran las siguientes frecuencias:
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
#include "sensor_temp.h"
#include "sensor_acc.h"
#include "Delay.h"
#include "USART.h"

#include <string.h>
#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif

/*Private variables*/
float temperatura;
int ejex;
int ejey;
int ejez;
char buf[100];
int size = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(int fallo);


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  
  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  if (HAL_Init() != HAL_OK)
		Error_Handler(0);

  /* Inicialización y configuración del reloj a 168 MHz */
  SystemClock_Config();
  SystemCoreClockUpdate();

	/*Inicialización del I2C*/
	initI2c();
	/*Inicialización del acelerometro*/
	init_acc();
	
	/* Inicialización de la USART a traves de la función init_USART de la libreria USART
	*	 y habilitación de la transmisión
	*							- Baudrate = 9600 baud
	*							- Word length = 8 bits
	*							- Un bit de stop
	*							- Sin bit de paridad
	*							- Sin control de flujo
	*/
	if (init_USART() != 0)
		Error_Handler(2);
		
	/*Inicializaciñon del Delay*/
	Init_Delay(180,4);


  /* Infinite loop */
  while (1)
  {
		
		/*Lectura del valor de la temperatura*/
		temperatura = leerTemperatura();
		/* Texto que se desea enviar*/
		size = sprintf(buf,"\rLa temperatura es de %f ºC\n", temperatura);
		/* Envío del array al terminal a traves de la función tx_USART de la librería USART*/
		if (tx_USART(buf, size) != 0)
		Error_Handler(3);
		Delay_ms(20);
		
		/*Lectura del valor del eje x*/
		ejex = leeraxis(0x00);
		/* Texto que se desea enviar*/
		size = sprintf(buf,"\rEl eje x esta en %d\n", ejex);
		/* Envío del array al terminal a traves de la función tx_USART de la librería USART*/
		if (tx_USART(buf, size) != 0)
		Error_Handler(3);
		Delay_ms(20);
					
		/*Lectura del valor del eje y*/
		ejey = leeraxis(0x01);
		/* Texto que se desea enviar*/
		size = sprintf(buf,"\rEl eje y esta en %d\n", ejey);
		/* Envío del array al terminal a traves de la función tx_USART de la librería USART*/
		if (tx_USART(buf, size) != 0)
		Error_Handler(3);
		Delay_ms(20);
				
		/*Lectura del valor del eje z*/
		ejez = leeraxis(0x02);
		/* Texto que se desea enviar*/
		size = sprintf(buf,"\rEl eje z esta en %d\n", ejez);
		/* Envío del array al terminal a traves de la función tx_USART de la librería USART*/
		if (tx_USART(buf, size) != 0)
		Error_Handler(3);
		Delay_ms(2000);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 8
  *            PLL_N                          = 180
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
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
	
  /** Se configura el HSI como fuente de reloj del PLL y se configuran
	* 	los parametros del PLL para ajusta la frecuencia a 180 MHz con una
	* 	frecuencia del HSI de 16 MHZ (por defecto).
	* 	SYSCLK =[(16MHz(frecuencia HSI)/8(PLLM))*180 (PLLN)]/2 (PLLP) = 180 MHz
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(1);
  }
  /** Se activa el modo de Over Drive para poder alcanzar los 180 MHz
	* 	como frecuencia del sistema
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler(1);
  }
  /** Se selecciona el PLL como fuente de reloj del sistema y se configuran los parametros
	*		para configurar el HCLK, PCLK1 y PCLK2. La frecuencia máxima del HCLK es 180 MHZ, la 
	*		frecuencia máxima del PCLK1 es de 45 MHZ y la frecuencia máxima del PCLK2 es de 90 MHz
	*		HCLK = SYSCK/AHB = 180 MHz / 1 = 180 MHz
	*		PCLK1 = HCLK/APB1 = 180 MHz / 4 = 45 MHZ
	*		PCLK2 = HCLK/APB2 = 180 MHz / 2 = 90 MHZ
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler(1);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(int fallo)
{
	if(fallo == 0)
		/* Mensaje si se ha producido un error en la inicializacón de la librería HAL*/
		printf(buf,"\r Se ha producido un error al inicializar la librería HAL\n");
	else if (fallo == 1)
		/* Mensaje si se ha producido un error en la inicializacón del reloj del sistema*/
		printf(buf,"\r Se ha producido un error al inicializar el reloj del sistema\n");
	else if(fallo == 2)
		/* Mensaje si se ha producido un error en la inicializacón de la USART*/
		printf(buf,"\r Se ha producido un error al inicializar la USART\n");
	else if (fallo == 3)
		/* Mensaje si se ha producido un error en el envío de datos de la USART*/
		printf(buf,"\r Se ha producido un error al enviar datos por la USART\n");
  while(1)
  {
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
