/*
 * ms_timer.h
 *
 *  Created on: May 13, 2024
 *      Author: t.shypytiak
 */

#ifndef APPLICATION_USER_MS_TIMER_H_
#define APPLICATION_USER_MS_TIMER_H_
#include <stdint.h>

void ms_timer_handler( void );
float ms_timer_seconds_elapsed_since( uint64_t time_ms );
uint64_t ms_timer_get_now( void );

#endif /* APPLICATION_USER_MS_TIMER_H_ */
