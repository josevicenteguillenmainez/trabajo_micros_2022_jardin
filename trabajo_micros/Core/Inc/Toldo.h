/*
 * Toldo.h
 *
 *  Created on: 14 jun. 2022
 *      Author: Juanmi y Vicen
 */

#ifndef INC_TOLDO_H_
#define INC_TOLDO_H_

#include "main.h"

int estado_toldo=0; //0 parado, 1 subiendo, 2 bajando
int flag_subida=0;	//flags que indican si el toldo est� totalmente subido o bajado
int flag_bajada=0;

void setEstadoToldo(int n){
	estado_toldo=n;
	if(estado_toldo==0){//parado
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);//3 subiendo
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);//4 bajando
	}else if(estado_toldo==1){//subiendo
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	}else if(estado_toldo==2){//bajando
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	}
}

void cambiarEstadoToldo(int n){
	if(n==0){
		setEstadoToldo(0);
	}else if(n==1 && flag_subida==0){
		setEstadoToldo(1);
		flag_bajada=0;
	}else if(n==2 && flag_bajada==0){
		setEstadoToldo(2);
		flag_subida=0;
	}else if(n==3){
		setEstadoToldo(0);
		flag_subida=1;
	}else if(n==4){
		setEstadoToldo(0);
		flag_bajada=1;
	}
}

int getEstadoToldo(){
	return estado_toldo;
}

#endif /* INC_TOLDO_H_ */
