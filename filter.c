/*
 * filter.c
 *
 *  Created on: 28 мая 2020 г.
 *      Author: spide
 */

#include "filter.h"

uint16_t filter(uint16_t filt, uint16_t signal)
{
	return ((F_A * filt) + (F_B * signal)) >> F_K;
}
