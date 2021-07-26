/**
  ******************************************************************************
  * @file    Templates/Src/sensor_acc.c
  * @author  MCD Application Team
  * @brief   Fichero de inicialización del acelerometro de la tarjeta 
	*					 de aplicaciones. 
	*					 	
	*					 En el datasheet del acelerometro aparece su dirección definida como:
	*					 Dirección del acelerometro ->0x4C	
  *					 
	*					 Para configurar el I2C se utiliza el CMSIS Driver, a traves de sus 
	*					 sus funciones se inicializa y configura para trabajar en modo FAST.
	*				 	 Los pines del I2C se configuran a traves del fichero RTE_Device.h:
	*					 PIN SDL-> 	 PB8	
	*					 PIN SDA->	 PB9
	*
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * 
  ******************************************************************************
  */
	
#include "sensor_acc.h"
#include "Driver_I2C.h"

#define ACC_ADR_W 0x4C

extern ARM_DRIVER_I2C Driver_I2C1;
static ARM_DRIVER_I2C *I2Cdev = &Driver_I2C1;



void initI2c();
void init_acc();
float leeraxis(uint8_t eje );

/**
  * @brief Función de inicialización del I2C
	* @param None
  * @retval None
  */
void I2C_init(){
	
	I2Cdev -> Initialize (NULL);
	I2Cdev -> PowerControl (ARM_POWER_FULL);
	I2Cdev -> Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
	I2Cdev -> Control (ARM_I2C_BUS_CLEAR,0);	
	
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

	/*Envío de 2 bytes, el primero con el registro a modificar (modo) y 
		el segundo poniendo el modo activo (1 en el bit 0)*/
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

	uint8_t axis;
	float ax;
	uint8_t buff;

	/*Si se quiere leer el eje x*/	
	if (eje == 0){
	/*Se envía el byte indicando que se quiere leer registro del eje x*/
	I2Cdev -> MasterTransmit(ACC_ADR_W, &eje, 1, true);
	while(I2Cdev -> GetStatus().busy);
	/*Se recibe el byte del registro con el valor del eje x*/
	I2Cdev -> MasterReceive(ACC_ADR_W, &buff, 1, false);
	while(I2Cdev -> GetStatus().busy);
	}
		
	/*Si se quiere leer el eje y*/	
	else if (eje == 1){
	/*Se envía el byte indicando que se quiere leer registro del eje y*/
	I2Cdev -> MasterTransmit(ACC_ADR_W,&eje, 1, true);
	while(I2Cdev -> GetStatus().busy);
	/*Se recibe el byte del registro con el valor del eje y*/
	I2Cdev -> MasterReceive(ACC_ADR_W, &buff, 1, false);
	while(I2Cdev -> GetStatus().busy);
	}
		
	/*Si se quiere leer el eje z*/	
	else{
	/*Se envía el byte indicando que se quiere leer registro del eje z*/
	I2Cdev -> MasterTransmit(ACC_ADR_W, &eje, 1, true);
	while(I2Cdev -> GetStatus().busy);
	/*Se recibe el byte del registro con el valor del eje z*/
	I2Cdev -> MasterReceive(ACC_ADR_W, &buff, 1, false);
	while(I2Cdev -> GetStatus().busy);
	}
	
	/*El valor del eje se encuentra entr los valores 31:-32. El valor del eje se encuentra
	en los bits 0 a 5, done el bit 5 a 1 indica si es un valor negativo */
	axis = ((char)(buff));
	/*En caso de que sea negativo*/
	if(axis > 31){
		/*El valor esta en complemento a 2 y se convierte para obtener su valor negativo*/
		axis = ~axis +1;
		axis = ( axis << 2);
		axis = axis >> 2;
		ax = -axis;
		return ax;
	}
	ax = axis;
	return axis;
}