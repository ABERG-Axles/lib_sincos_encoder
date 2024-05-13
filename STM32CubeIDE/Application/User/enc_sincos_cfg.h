/*
 * enc_sincos_cfg.h
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */

#ifndef APPLICATION_USER_ENC_SINCOS_CFG_H_
#define APPLICATION_USER_ENC_SINCOS_CFG_H_

#define V_REG					3.3
#define ADC_VOLTS( adc_val )	( ( float )adc_val / 4095.0 * V_REG )

#define SINCOS_MIN_AMPLITUDE        0.7         // sqrt(sin^2 + cos^2) has to be larger than this
#define SINCOS_MAX_AMPLITUDE        1.3         // sqrt(sin^2 + cos^2) has to be smaller than this

#endif /* APPLICATION_USER_ENC_SINCOS_CFG_H_ */
