/*
 * filter.h
 *
 *  Created on: 28 мая 2020 г.
 *      Author: spide
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <stdint.h>

#define F_A 2
#define F_B 2
#define F_K 2

uint16_t filter(uint16_t filt, uint16_t signal);

#endif /* FILTER_H_ */
