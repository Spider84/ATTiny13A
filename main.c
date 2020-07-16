/*
 * main.c
 *
 *  Created on: 25 мая 2020 г.
 *      Author: spide
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "filter.h"

//Собственно кнопка
#define BUTTON_PIN  (_BV(PB4))
#define BUTTON_PORT PORTB
#define BUTTON_DDR  DDRB
#define BUTTON_PRESSED() (!(PINB & BUTTON_PIN))

#define LED_PIN     (_BV(PB1))
#define LED_PORT    PORTB
#define LED_DDR     DDRB

#define DEFAULT_VALUE_1 500
#define DEFAULT_VALUE_2 500

typedef struct {
	uint16_t value1;
	uint16_t value2;
	uint8_t crc;
} ee_config;

static ee_config EEMEM addr_config;
static ee_config config;

static uint8_t calcCRC(ee_config *config)
{
	uint8_t crc = 0x55;
	for (uint8_t i=0; i<sizeof(ee_config); i++) {
		crc ^= ((uint8_t *)config)[i];
	}
	return crc;
}

static void writeConfig(void)
{
	config.crc = calcCRC(&config);
	eeprom_busy_wait();
	eeprom_write_block ((const void *)&config, (void *)&addr_config, sizeof(ee_config));
}

static void readConfig(void)
{
	eeprom_read_block ((void *)&config, (const void *)&addr_config, sizeof(ee_config));
	if (config.crc!=calcCRC(&config)) {
		config.value1 = DEFAULT_VALUE_1;
		config.value2 = DEFAULT_VALUE_2;
		writeConfig();
	}
}

int main (void)
{
	wdt_enable(WDTO_2S);

	BUTTON_PORT |= BUTTON_PIN;
	BUTTON_DDR &= ~BUTTON_PIN;

	LED_PORT &= ~LED_PIN;
	LED_DDR |= LED_PIN;

    PORTB &= ~(_BV(PB2));
	DDRB &= ~(_BV(PB2));
	DIDR0 = _BV(ADC1D);

	ADCSRB = 0;
	ADMUX = _BV(MUX0);
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);
#ifndef NO_NOISE_REDUCTION
	ADCSRA |= _BV(ADIE);
#endif

	readConfig();

	static uint16_t filt1 = 0;
	static uint16_t compare_with;

	compare_with = config.value1;

	wdt_reset();
	sei();

	while (1) {
		wdt_reset();

#ifndef NO_NOISE_REDUCTION
		set_sleep_mode(SLEEP_MODE_ADC);
		sleep_enable();
#else
		ADCSRA |= _BV(ADIF);
#endif

		ADCSRA &= ~_BV(ADEN);
		ADCSRA |= _BV(ADEN);
#ifndef NO_NOISE_REDUCTION
		// DS Page 31: If the ADC is enabled, a conversion starts automatically when this mode is entered
		sleep_cpu();
#else
		ADCSRA |= _BV(ADSC);
		do {} while (!(ADCSRA & _BV(ADIF)));
#endif
		filt1 = filter(filt1,ADCW);

		if (filt1>=compare_with) {
			LED_PORT |= LED_PIN;
		} else {
			LED_PORT &= ~LED_PIN;
		}

		static uint8_t button_state = 0;
		if (BUTTON_PRESSED()) {
			button_state = 1;
		} else
		if (button_state) {
			compare_with = config.value2 = filt1;
			//writeConfig();
			button_state = 0;
		}
	}
}

#ifndef NO_NOISE_REDUCTION
ISR(ADC_vect)
{
	//Reset Interrupt flag
	ADCSRA |= _BV(ADIF);
}
#endif
