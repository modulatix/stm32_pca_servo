/*
Copyright (c) 2020, Oleg Pereverzev
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Oleg Pereverzev nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <string.h>
#include "pca_servo.h"

struct _pca_servo_ledaddr {
	uint8_t on_l;
	uint8_t on_h;
	uint8_t off_l;
	uint8_t off_h;
};

struct pca_servo_st_ {
	I2C_HandleTypeDef *i2cport;
	uint16_t timeout;
	uint8_t datasheet_address;
	float prev_degree[16];
};

struct _pca_servo_ledaddr _pca_leds[16];

pca_servo_st *pca_servo_init(I2C_HandleTypeDef *i2cport, uint16_t timeout, uint8_t datasheet_address) {
	int i;
	pca_servo_st *ret;
	uint8_t resetarr[] = {0, 0};
	uint8_t pwm_speedarr[] = {0xFE, 122};

	if(HAL_I2C_Master_Transmit(i2cport, datasheet_address << 1, resetarr, 2, timeout)!=HAL_OK)
		return NULL;

	if(HAL_I2C_Master_Transmit(i2cport, datasheet_address << 1, pwm_speedarr, 2, timeout)!=HAL_OK)
		return NULL;

	for(i=0; i<16; i++) {
		_pca_leds[i].on_l=6+i*4;
		_pca_leds[i].on_h=7+i*4;
		_pca_leds[i].off_l=8+i*4;
		_pca_leds[i].off_h=9+i*4;
	}

	ret=malloc(sizeof(pca_servo_st));
	if(!ret) return NULL;
	memset(ret, 0, sizeof(pca_servo_st));

	ret->datasheet_address=datasheet_address;
	ret->i2cport=i2cport;
	ret->timeout=timeout;

	return ret;
}

int pca_servo_set_raw(pca_servo_st *ps, uint8_t LEDn, uint16_t from, uint16_t to) {
	uint8_t data[2];

	if(!ps
			|| ( LEDn>15
					|| ( from>4095 || to>4095) ) )
		return 1;


	data[0]=_pca_leds[LEDn].on_l;
	data[1]=(uint8_t)(from & 0xFF);
	if(HAL_I2C_Master_Transmit(ps->i2cport, (ps->datasheet_address) << 1, data, 2, ps->timeout)!=HAL_OK)
		return 1;
	data[0]=_pca_leds[LEDn].on_h;
	data[1]=(uint8_t)(from >> 8);
	if(HAL_I2C_Master_Transmit(ps->i2cport, (ps->datasheet_address) << 1, data, 2, ps->timeout)!=HAL_OK)
		return 1;

	data[0]=_pca_leds[LEDn].off_l;
	data[1]=(uint8_t)(to & 0xFF);
	if(HAL_I2C_Master_Transmit(ps->i2cport, (ps->datasheet_address) << 1, data, 2, ps->timeout)!=HAL_OK)
		return 1;
	data[0]=_pca_leds[LEDn].off_h;
	data[1]=(uint8_t)(to >> 8);
	if(HAL_I2C_Master_Transmit(ps->i2cport, (ps->datasheet_address) << 1, data, 2, ps->timeout)!=HAL_OK)
		return 1;

	return 0;
}

int pca_servo_set_degree(pca_servo_st *ps, uint8_t LEDn, float degree) {
	uint8_t data[2];

	if(!ps || ( LEDn>15 || (degree > 180.0 || degree < 0.0)) )
		return 1;

	uint16_t newpos = (uint16_t)(degree*(((float)PCA_SERVO_MAX - PCA_SERVO_MIN)/180.0))+PCA_SERVO_MIN; //no rounding as servos are not that precise

	data[0]=_pca_leds[LEDn].on_l;
	data[1]=0;
	if(HAL_I2C_Master_Transmit(ps->i2cport, (ps->datasheet_address) << 1, data, 2, ps->timeout)!=HAL_OK)
		return 1;
	data[0]=_pca_leds[LEDn].on_h;
	data[1]=0;
	if(HAL_I2C_Master_Transmit(ps->i2cport, (ps->datasheet_address) << 1, data, 2, ps->timeout)!=HAL_OK)
		return 1;

	data[0]=_pca_leds[LEDn].off_l;
	data[1]=(uint8_t)(newpos & 0xFF);
	if(HAL_I2C_Master_Transmit(ps->i2cport, (ps->datasheet_address) << 1, data, 2, ps->timeout)!=HAL_OK)
		return 1;
	data[0]=_pca_leds[LEDn].off_h;
	data[1]=(uint8_t)(newpos >> 8);
	if(HAL_I2C_Master_Transmit(ps->i2cport, (ps->datasheet_address) << 1, data, 2, ps->timeout)!=HAL_OK)
		return 1;

	float degree_diff_mod = ((ps->prev_degree[LEDn]-degree)>0.0)?(ps->prev_degree[LEDn]-degree):((ps->prev_degree[LEDn]-degree)*-1.0);
	float dynamic_multiplier = 250*(1.0-degree_diff_mod/180.0);
	HAL_Delay((uint32_t)((degree_diff_mod/60.0)*(250.0+dynamic_multiplier)));
	ps->prev_degree[LEDn]=degree;
	return 0;
}
