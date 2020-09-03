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

#ifndef INC_PCA_SERVO_H_
#define INC_PCA_SERVO_H_

#include <stm32f4xx_hal.h> //or whatever HAL is for your STM32

typedef struct pca_servo_st_ pca_servo_st;

//PCA_SERVO_MIN and PCA_SERVO_MAX can be used for fine-tuning 0 and 180 degrees respectively to suit your servo (presumably SG90)
#ifndef PCA_SERVO_MIN
#define PCA_SERVO_MIN 260
#endif

#ifndef PCA_SERVO_MAX
#define PCA_SERVO_MAX 1860
#endif

//Structure creation and PCA9685 initialization
pca_servo_st *pca_servo_init(I2C_HandleTypeDef *i2cport, uint16_t timeout, uint8_t datasheet_address);
//A way to set pulsewidth manually
int pca_servo_set_raw(pca_servo_st *ps, uint8_t LEDn, uint16_t from, uint16_t to);
//Function to set servo degree (accepatble float degree range is 0:180) with automatic waiting
int pca_servo_set_degree(pca_servo_st *ps, uint8_t LEDn, float degree);

#endif /* INC_PCA_SERVO_H_ */
