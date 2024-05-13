/*
 * enc_sincos.c
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */
#include "enc_sincos.h"
#include <string.h>
#include <math.h>

#include "enc_sincos_cfg.h"
#include "utils.h"
#include "ms_timer.h"
#include "adc.h"


// ---------------------------------------------- members ----------------------------------------------
static float out_sin = 0.0f;
static float out_cos = 0.0f;
static float out_mod = 0.0f;

static float s_amp = 1.0f;
static float c_amp = 1.0f;
static float s_gain = 1.0f;
static float c_gain = 1.0f;
static float s_ofs = 1.65f;
static float c_ofs = 1.65f;

static float s_max = 0.0f;
static float s_min = UPPER_BOUND_V;
static float c_max = 0.0f;
static float c_min = UPPER_BOUND_V;
static float raw_sin = 0.0f;
static float raw_cos = 0.0f;

volatile uint32_t InjADC_Reading = 0;
volatile uint32_t InjADC_Reading2 = 0;
volatile float last_deg = 0.0f;

#define FILTER_CONST 0.8f




// ---------------------------------------------- private ----------------------------------------------
static void calculate_amp_off(){
	s_ofs = 0.5f * ( s_max + s_min );
	c_ofs = 0.5f * ( c_max + c_min );
	s_amp = 0.5f * ( s_max - s_min );
	c_amp = 0.5f * ( c_max - c_min );
	s_gain = 1.0f / s_amp;
	c_gain = 1.0f / c_amp;
}

// ---------------------------------------------- interface ----------------------------------------------
bool enc_sincos_init( EncSinCosConfigT* pcfg ){
	memset( &pcfg->state, 0, sizeof( EncSinCosStateT ) );
	out_sin = 0.0f;
	out_cos = 0.0f;
	out_mod = 0.0f;
	s_amp = 1.0f;
	c_amp = 1.0f;
	s_ofs = 1.65f;
	c_ofs = 1.65f;
	s_max = 0.0f;
	s_min = UPPER_BOUND_V;
	c_max = 0.0f;
	c_min = UPPER_BOUND_V;

	pcfg->s_gain 			= 1.0f / ENCODER_SIN_AMP;
	pcfg->s_offset			= ENCODER_SIN_OFFSET;
	pcfg->c_gain			= 1.0f / ENCODER_COS_AMP;
	pcfg->c_offset			= ENCODER_COS_OFFSET;
	pcfg->filter_constant	= ENCODER_SINCOS_FILTER;
	pcfg->sph 				= sinf( DEG2RAD( ENCODER_SINCOS_PHASE ) );
	pcfg->cph 				= cosf( DEG2RAD( ENCODER_SINCOS_PHASE ) );
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
	LP_FAST( pcfg->state.sin_filtered, sin, pcfg->filter_constant );
	LP_FAST( pcfg->state.cos_filtered, cos, pcfg->filter_constant );
	sin = pcfg->state.sin_filtered;
	cos = pcfg->state.cos_filtered;

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
		LP_FAST( pcfg->state.signal_above_max_error_rate, 1.0f, time_ellapsed );
	}else if( module < SQ( SINCOS_MIN_AMPLITUDE ) ){
		++ pcfg->state.signal_below_min_error_cnt;
		LP_FAST( pcfg->state.signal_low_error_rate, 1.0f, time_ellapsed );
	}else{
		LP_FAST( pcfg->state.signal_above_max_error_rate, 0.0f, time_ellapsed );
		LP_FAST( pcfg->state.signal_low_error_rate, 0.0f, time_ellapsed );
		pcfg->state.last_enc_angle = RAD2DEG( utils_fast_atan2( sin, cos ) ) + 180.0f;
	}

	return pcfg->state.last_enc_angle;
}

void enc_sincos_calibrate( /*EncSinCosConfigT* pcfg,*/ uint32_t adc_value_sin, uint32_t adc_value_cos ){
	LP_FAST( raw_sin, ADC_VOLTS( adc_value_sin ), FILTER_CONST );
	LP_FAST( raw_cos, ADC_VOLTS( adc_value_cos ), FILTER_CONST );

	if( raw_sin > s_max ){
		s_max = raw_sin;
	}

	if( raw_sin < s_min ){
		s_min = raw_sin;
	}

	if( raw_cos > c_max ){
		c_max = raw_cos;
	}

	if( raw_cos < c_min ){
		c_min = raw_cos;
	}

	out_sin = ( raw_sin - s_ofs ) / s_amp;
	out_cos = ( raw_cos - c_ofs ) / c_amp;
	out_mod = sqrtf( SQ( out_sin ) + SQ( out_cos ) );
	float sdiff = s_max - s_min;
	float cdiff = c_max - c_min;

	if( sdiff > 0.1f && cdiff > 0.1f ){
		calculate_amp_off();
	}
}

//------------------------------------------------------------------------
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
	if( hadc == &hadc1 ){
		InjADC_Reading = HAL_ADCEx_InjectedGetValue( &hadc1, ADC_INJECTED_RANK_1 ); // Read The Injected Channel Result
	}else{
		InjADC_Reading2 = HAL_ADCEx_InjectedGetValue( &hadc2, ADC_INJECTED_RANK_1 ); // Read The Injected Channel Result
	}
}

void enc_sincos_adc12_callback( EncSinCosConfigT* pcfg ){
//	enc_sincos_calibrate( InjADC_Reading, InjADC_Reading2 );
	last_deg = enc_sincos_read_deg( pcfg, InjADC_Reading, InjADC_Reading2 );
}
