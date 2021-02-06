/* Blink the LED on the board and chat on the serial port. */

#define _GNU_SOURCE

#include "main.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>

#include <sys/types.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>

static void clock_setup(void)
{
	// Enable peripheral clock to gpio ports A, B, F
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOF);

	// Enable peripheral clock to timer2
	rcc_periph_clock_enable(RCC_TIM2);

#ifdef ENABLE_DEBUG_USART
	// Enable peripheral clock to usart1
	rcc_periph_clock_enable(RCC_USART1);
#else
	// Enable peripheral clock to i2c1
	rcc_periph_clock_enable(RCC_I2C1);
#endif // ENABLE_DEBUG_USART
}

static void gpio_setup(void)
{
	// configure status LED GPIO
	gpio_mode_setup(LEDPORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LEDPIN);

	// configure button input GPIO
	gpio_mode_setup(SWPORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SWPIN);
	
	// configure presence sense GPIO
	gpio_mode_setup(SENSEPORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SENSEPIN);

	// configure segment GPIOs
	gpio_mode_setup(SEGPORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, SEGPIN_ALL);

	// configure grid drive output (AF1: TIM2_CH1)
	gpio_mode_setup(SEGPORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SEGPIN_G);
	gpio_set_af(SEGPORT, GPIO_AF1, SEGPIN_G);
	gpio_set_output_options(SEGPORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, SEGPIN_G);

	// configure 5V regulator enable GPIO
	gpio_mode_setup(EN5VPORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EN5VPIN);

	// configure I2C enable GPIO
	gpio_mode_setup(ENI2CPORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ENI2CPIN);

	// configure filament drive output (AF10: TIM2_CH3)
	gpio_mode_setup(FILPORT, GPIO_MODE_AF, GPIO_PUPD_NONE, FILPIN);
	gpio_set_af(FILPORT, GPIO_AF10, FILPIN);
	gpio_set_output_options(FILPORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, FILPIN);

#ifdef ENABLE_DEBUG_USART
	// configure USART port IOs (AF7: USART1_RX, USART1_TX)
	gpio_mode_setup(USARTPORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USARTPIN_RX);
	gpio_mode_setup(USARTPORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USARTPIN_TX);
	gpio_set_output_options(USARTPORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, USARTPIN_RX);
	gpio_set_output_options(USARTPORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, USARTPIN_TX);
	gpio_set_af(USARTPORT, GPIO_AF7, USARTPIN_RX);
	gpio_set_af(USARTPORT, GPIO_AF7, USARTPIN_TX);
#else
	// configure I2C port IOs (AF2: I2C1_SCL, I2C1_SDA)
	gpio_mode_setup(I2CPORT, GPIO_MODE_AF, GPIO_PUPD_NONE, I2CPIN_SCL);
	gpio_mode_setup(I2CPORT, GPIO_MODE_AF, GPIO_PUPD_NONE, I2CPIN_SDA);
	gpio_set_output_options(I2CPORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, I2CPIN_SCL);
	gpio_set_output_options(I2CPORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, I2CPIN_SDA);
    gpio_set_af(I2CPORT, GPIO_AF4, I2CPIN_SCL);
    gpio_set_af(I2CPORT, GPIO_AF4, I2CPIN_SDA);
#endif // ENABLE_DEBUG_USART
}

#ifdef ENABLE_DEBUG_USART
static FILE* usart_setup(void)
{
	// Configure USART1: 9600 8N1
	usart_set_baudrate(USART1, 9600);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	// Enable USART1 peripheral
	usart_enable(USART1);

	cookie_io_functions_t io_func = { _iord, _iowr, NULL, NULL };
	FILE *fp = fopencookie((void *)USART1, "rw+", io_func);

    // Don't buffer the serial line
	setvbuf(fp, NULL, _IONBF, 0);
	return fp;
}

static ssize_t _iord(void *_cookie, char *_buf, size_t _n)
{
	// Don't implement reading for now
	(void)_cookie;
	(void)_buf;
	(void)_n;
	return 0;
}

static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n)
{
	uint32_t dev = (uint32_t)_cookie;

	int written = 0;
	while (_n-- > 0) {
		usart_send_blocking(dev, *_buf++);
		written++;
	};
	return written;
}
#else
static void i2c_setup(void) {
	i2c_reset(I2C1);
	i2c_peripheral_disable(I2C1);

	// Assuming the defaults work here
	i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);

	// Enable I2C1 peripheral
	i2c_peripheral_enable(I2C1);
}
#endif // ENABLE_DEBUG_USART

static void timer_setup(void)
{
	// Enable the timer2 interrupt and reset the peripheral
	rcc_periph_reset_pulse(RST_TIM2);

	// These are the defaults after reset; no division, edge alignment, count up
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	// Set the prescaler to (peripheral clock frequency / 1000000) - 1 , so it'll tick at 1MHz
	// for a peripheral clock of 8MHz, this makes the prescaler 7
	timer_set_prescaler(TIM2, (rcc_apb1_frequency / 1000000) - 1);

	// Set the timer period to 100 so we'll overflow every 200us, making the overflow frequency 5kHz
	timer_set_period(TIM2, 99);

	// Enable continuous mode so the timer will restart at zero after overflow
	timer_continuous_mode(TIM2);

	// TIM2_OC3 config: controls the filament drive waveform
	// Disable outputs
	timer_disable_oc_output(TIM2, TIM_OC3);

	// Configure output compare channel 3
	timer_disable_oc_clear(TIM2, TIM_OC3);
	timer_enable_oc_preload(TIM2, TIM_OC3);
	timer_set_oc_slow_mode(TIM2, TIM_OC3);
	timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
	timer_set_oc_polarity_high(TIM2, TIM_OC3);
	timer_set_oc_idle_state_set(TIM2, TIM_OC3);

	// Set TIM2_OC3 to 50% duty cycle
	timer_set_oc_value(TIM2, TIM_OC3, 50);

	timer_enable_oc_output(TIM2, TIM_OC3);

	// TIM2_OC1 config: controls the grid drive waveform
	// Disable outputs
	timer_disable_oc_output(TIM2, TIM_OC1);

	// Configure output compare channel 1
	timer_disable_oc_clear(TIM2, TIM_OC1);
	timer_enable_oc_preload(TIM2, TIM_OC1);
	timer_set_oc_slow_mode(TIM2, TIM_OC1);
	timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1);	
	timer_set_oc_polarity_low(TIM2, TIM_OC1);
	timer_set_oc_idle_state_set(TIM2, TIM_OC1);

	// Set TIM2_OC1 to 20% duty cycle
	_set_grid_duty_cycle(20);

	timer_enable_oc_output(TIM2, TIM_OC1);

	// Start counting
	timer_enable_counter(TIM2);
}

static void systick_setup(void) {
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8); // 1MHz
	systick_set_reload(999); // period = 1000, so we overflow at 1kHz
	systick_interrupt_enable();
	systick_counter_enable();
}

void sys_tick_handler(void) {
	isr_flag = 1;
}

static void _set_grid_duty_cycle(uint8_t duty_cycle) {
	duty_cycle = 100 - duty_cycle;

	// due to rise/fall time on the grid driver circuitry, values lower than 10
	// or higher than 90 aren't reliable.  crowbar to these values
	// this could be mitigated by driving the grid at a lower frequency
	if (duty_cycle < 10)
		duty_cycle = 10;

	if (duty_cycle > 90)
		duty_cycle = 90;
	
	timer_set_oc_value(TIM2, TIM_OC1, duty_cycle);
}

static void _set_digit(uint8_t value) {
	gpio_clear(SEGPORT, SEGPIN_SEGMENTS);

	switch(value) {
		case 0:
			gpio_set(SEGPORT, 0xbe);
			break;
		case 1:
			gpio_set(SEGPORT, 0xa0);
			break;
		case 2:
			gpio_set(SEGPORT, 0xda);
			break;
		case 3:
			gpio_set(SEGPORT, 0xf2);
			break;
		case 4:
			gpio_set(SEGPORT, 0xe4);
			break;
		case 5:
			gpio_set(SEGPORT, 0x76);
			break;
		case 6:
			gpio_set(SEGPORT, 0x7e);
			break;
		case 7:
			gpio_set(SEGPORT, 0xa2);
			break;
		case 8:
			gpio_set(SEGPORT, 0xfe);
			break;
		case 9:
			gpio_set(SEGPORT, 0xe6);
			break;
		default:
			gpio_set(SEGPORT, 0x00);
	}
}

int main(void) {
	uint8_t display_digit = 0;
	uint8_t display_digit_next = 0;

	stime_t time = { .milliseconds = 0, .seconds = 0, .minutes = 0, .hours = 0,
					 .days = 0, .months = 0, .years = 0 };

	clock_setup();
	gpio_setup();
    timer_setup();
	systick_setup();

#ifdef ENABLE_DEBUG_USART
	FILE *fp;
	fp = usart_setup();
#else
	i2c_setup();
#endif // ENABLE_DEBUG_USART

	// Light everything up
	gpio_set(SEGPORT, SEGPIN_ALL);
	gpio_set(EN5VPORT, EN5VPIN);
	gpio_set(ENI2CPORT, ENI2CPIN);

    // Spin forever
	while (1) {
        if (isr_flag) {
			isr_flag = 0;

			time.milliseconds++;
			srtc_update(&time);

		    // Toggle LED GPIO
			if (time.milliseconds == 0) {
	    	    gpio_toggle(LEDPORT, LEDPIN);
				_set_grid_duty_cycle((time.seconds % 10) * 10);

				display_digit = display_digit_next;
				display_digit_next = (display_digit + 1) % 10;
#ifdef ENABLE_DEBUG_USART
//				fprintf(fp, "grid_duty_cycle: %d\n", grid_duty_cycle);
				fprintf(fp, "display_digit: %d\n", display_digit);
#endif
			}

			// transition to the next digit
			// todo: generalize this, it's uuuugly
			if (time.milliseconds > 250) {
				_set_digit(display_digit_next);
			} else {
				if (time.milliseconds < 25) {
					if (time.milliseconds % 10 < 2) {
						_set_digit(display_digit_next);
					} else {
						_set_digit(display_digit);
					}
				} else if (time.milliseconds < 50) {
					if (time.milliseconds % 10 < 3) {
						_set_digit(display_digit_next);
					} else {
						_set_digit(display_digit);
					}
				} else if (time.milliseconds < 75) {
					if (time.milliseconds % 10 < 4) {
						_set_digit(display_digit_next);
					} else {
						_set_digit(display_digit);
					}
				} else if (time.milliseconds < 100) {
					if (time.milliseconds % 10 < 5) {
						_set_digit(display_digit_next);
					} else {
						_set_digit(display_digit);
					}
				} else if (time.milliseconds < 125) {
					if (time.milliseconds % 10 < 6) {
						_set_digit(display_digit_next);
					} else {
						_set_digit(display_digit);
					}
				} else if (time.milliseconds < 150) {
					if (time.milliseconds % 10 < 7) {
						_set_digit(display_digit_next);
					} else {
						_set_digit(display_digit);
					}
				} else if (time.milliseconds < 175) {
					if (time.milliseconds % 10 < 8) {
						_set_digit(display_digit_next);
					} else {
						_set_digit(display_digit);
					}
				}
			}
		}
	}

	return 0;
}
