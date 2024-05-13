/*
 * enc_sincos.c
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */
#include "enc_sincos.h"
#include <string.h>
#include <stdlib.h>
#include "enc_sincos_cfg.h"
#include "utils.h"
#include "ms_timer.h"

bool enc_sincos_init( EncSinCosConfigT* pcfg ){
	memset( &pcfg->state, 0, sizeof( EncSinCosStateT ) );
	return true;
}

void enc_sincos_shutdown( EncSinCosConfigT* pcfg ){
	memset( &pcfg->state, 0, sizeof( EncSinCosStateT ) );
}

/**
 * s_offset
 * */
float enc_sincos_read_deg( EncSinCosConfigT* pcfg, uint32_t adc_value_sin, uint32_t adc_value_cos ){
	float sin = ( ADC_VOLTS( adc_value_sin ) - pcfg->s_offset ) * pcfg->s_gain;
	float cos = ( ADC_VOLTS( adc_value_cos ) - pcfg->c_offset ) * pcfg->c_gain;
	UTILS_LP_FAST( pcfg->state.sin_filter, sin, pcfg->filter_constant );
	UTILS_LP_FAST( pcfg->state.cos_filter, cos, pcfg->filter_constant );
	sin = pcfg->state.sin_filter;
	cos = pcfg->state.cos_filter;

	//phase correction
	cos = (cos + sin * pcfg->sph) / pcfg->cph;
	float module = SQ( sin ) + SQ( cos );
	float time_ellapsed = ms_timer_seconds_elapsed_since( pcfg->state.last_update_time );

	if( time_ellapsed > 1.0f ){
		time_ellapsed = 1.f;
	}

	pcfg->state.last_update_time = ms_timer_get_now();

	// signals vector outside of the valid area. Increase error count and discard measurement
	if( module > SQ( SINCOS_MAX_AMPLITUDE ) ){
		++ pcfg->state.signal_above_max_error_cnt;
		UTILS_LP_FAST( pcfg->state.signal_above_max_error_rate, 1.0f, time_ellapsed );
	}else if( module < SQ( SINCOS_MIN_AMPLITUDE ) ){
		++ pcfg->state.signal_below_min_error_cnt;
		UTILS_LP_FAST( pcfg->state.signal_low_error_rate, 1.0f, time_ellapsed );
	}else{
		UTILS_LP_FAST( pcfg->state.signal_above_max_error_rate, 0.0f, time_ellapsed );
		UTILS_LP_FAST( pcfg->state.signal_low_error_rate, 0.0f, time_ellapsed );
		pcfg->state.last_enc_angle = RAD2DEG_f( utils_fast_atan2( sin, cos ) ) + 180.0f;
	}

	return pcfg->state.last_enc_angle;
}

void enc_sincos_calibrate( EncSinCosConfigT* pcfg, uint32_t adc_value_sin, uint32_t adc_value_cos ){

}

