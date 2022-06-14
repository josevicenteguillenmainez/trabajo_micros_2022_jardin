/*
 * Luces.h
 *
 *  Created on: 14 jun. 2022
 *      Author: Juanmi y Vicen
 */

#ifndef INC_LUCES_H_
#define INC_LUCES_H_

#include "main.h"

ADC_HandleTypeDef hadc1;
int estado_luces=0;//0 off 1 on 2 auto
int8_t adcval[10];
uint32_t counter_luces=0, tickstart_luces=0;
int umbral_luces=50;
uint32_t media_ldr=0;

void setLuces(int n){
	if(n==0){ //off
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //0 ON
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //1 AUTO
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); //2 luces del patio
	}else if(n==1){ //on
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	}else{ //auto
	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	estado_luces=n;
}

int getEstadoLuces(){
	return estado_luces;
}

void setUmbralLuces(int u){
	umbral_luces=u;
}

int getMediaLDR(){
	return media_ldr;
}

void medirLDR(){
	if(estado_luces==2 && counter_luces>1000){
		counter_luces=0;
		media_ldr=0;
		tickstart_luces=HAL_GetTick();
		int i=0;
		for(i=0;i<5;i++){
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			adcval[i]=HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			media_ldr+=adcval[i];
		}
		media_ldr=media_ldr/5;
			if(media_ldr>(umbral_luces+5)){
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
			}else if(media_ldr<(umbral_luces-5)) {
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
			}
	}else{
		counter_luces=HAL_GetTick()-tickstart_luces;
	}
}

void luces(){
	medirLDR();
}

#endif /* INC_LUCES_H_ */
