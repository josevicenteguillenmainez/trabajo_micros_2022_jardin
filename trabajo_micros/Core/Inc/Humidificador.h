/*
 * Humidificador.h
 *
 *  Created on: 15 jun. 2022
 *      Author: Juanmi y Vicen
 */

#ifndef INC_HUMIDIFICADOR_H_
#define INC_HUMIDIFICADOR_H_
#include "main.h"

TIM_HandleTypeDef htim6;
uint8_t Temp_byte1, Temp_byte2;
uint16_t SUM, TEMP;
float Temperature = 0;
uint8_t Presence = 0;

int temperatura_referencia=40;
int estado_humidificador=0; //0 off, 1 on, 2 auto
uint32_t tickstart_humidificador=0, counter_humidificador=0;
uint32_t tickstart_humidificador_on=0, counter_humidificador_on=0;

/******************FUNCIONES DE HUMIDIFICADOR**************************/

void setEstadoHumidificador(int n){
	estado_humidificador=n;
	if(estado_humidificador==0){	//0 off, 1 on, 2 auto
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);//5 on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//6 auto
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);//7 humidificador
	}else if(estado_humidificador==1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	}else if(estado_humidificador==2){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	}
}

int getEstadoHumidificado(){
	return estado_humidificador;
}

void setTemperaturaReferencia(int t){
	temperatura_referencia=t;
}

void Humidificador(){
	if(estado_humidificador==2 && counter_humidificador>2000){
		counter_humidificador=0;
		tickstart_humidificador=HAL_GetTick();
		lectura_dht11();
		if(Temperature>(temperatura_referencia-1)){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		}else if(Temperature<(temperatura_referencia+1)) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}
	}else{counter_humidificador=HAL_GetTick()-tickstart_humidificador;}
		/*if(estado_humidificador==1 && counter_humidificador_on>=20000){
		counter_humidificador_on=0;
		tickstart_humidificador_on=HAL_GetTick();
		setEstadoHumidificador(0);
	}else if(estado_humidificador==1 && counter_humidificador_on<20000){
		counter_humidificador_on=HAL_GetTick()-tickstart_humidificador_on;
	}*/
}



void delay(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim6)) < time);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


/******************FUNCIONES DE DHT11**************************/

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

void DHT11_Start (void)
{
	Set_Pin_Output(DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	delay(18000);   // wait for 18ms
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);	// pull the pin high
	delay(20); //	wait for 20us
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	delay(40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		delay(80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		delay(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}

void lectura_dht11() {
	  	DHT11_Start();
	  	Presence = DHT11_Check_Response();
	  	Temp_byte1 = DHT11_Read();
	  	Temp_byte2 = DHT11_Read();
	  	SUM = DHT11_Read();
	  	TEMP = Temp_byte1;
	  	Temperature = (float) TEMP;
	  	Temperature = (17-Temperature)+25;
	  }

#endif /* INC_HUMIDIFICADOR_H_ */
