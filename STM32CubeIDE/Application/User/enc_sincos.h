/*
 * sincos_encoder.h
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */

#ifndef APPLICATION_USER_ENC_SINCOS_H_
#define APPLICATION_USER_ENC_SINCOS_H_

#include <stdbool.h>
#include <stdint.h>

#include "adc.h"

typedef struct{
	uint32_t signal_below_min_error_cnt;
	uint32_t signal_above_max_error_cnt;
	float signal_low_error_rate;
	float signal_above_max_error_rate;
	float last_enc_angle;
	float sin_filtered;
	float cos_filtered;
	uint64_t last_update_time;
}EncSinCosStateT;

typedef struct{
	// The gain is 1/amplutide. The reason it is stored like that
	// is to avoid two divisions when reading the encoder.
	float 			s_gain;
	float 			c_gain;
	float 			s_offset;
	float 			c_offset;
	float 			filter_constant;
	float 			phase_corrrection;
	float 			sph; // sin of the phase_correction angle
	float 			cph; // cos of the phase_correction angle
	ADC_TypeDef* 	adcx1;
	uint32_t 		injected_channel_1;
	ADC_TypeDef* 	adcx2;
	uint32_t 		injected_channel_2;
	EncSinCosStateT	state;
}EncSinCosConfigT;

bool enc_sincos_get_defaults( EncSinCosConfigT* pcfg );
void enc_sincos_shutdown( EncSinCosConfigT* pcfg );
float enc_sincos_read_deg( EncSinCosConfigT* pcfg, uint32_t adc_value_sin, uint32_t adc_value_cos );
void enc_sincos_calibrate( /*EncSinCosConfigT* pcfg,*/ uint32_t adc_value_sin, uint32_t adc_value_cos );
void enc_sincos_read_values( EncSinCosConfigT* pcfg );

#endif /* APPLICATION_USER_ENC_SINCOS_H_ */
