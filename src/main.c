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
	rcc_osc_bypass_enable(RCC_HSE);
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_HSE);

	rcc_apb1_frequency = 25000000;
	rcc_apb2_frequency = 25000000;
	rcc_ahb_frequency = 25000000;

	// TODO: Set flash wait states?  This seems to work without any tinkering,
	// 		 but it might be an important stability/reliability thing.

	// Enable peripheral clock to gpio ports A, B, F
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOF);

	// Enable peripheral clock to timer2
	rcc_periph_clock_enable(RCC_TIM2);

	// Enable peripheral clock to usart1
	rcc_periph_clock_enable(RCC_USART1);

	// Enable peripheral clock to i2c1
	rcc_periph_clock_enable(RCC_I2C1);
}

static void gpio_setup(void)
{
	// configure status LED GPIO
	gpio_mode_setup(LEDPORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LEDPIN);

	// configure button input GPIO
	gpio_mode_setup(SWPORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SWPIN);
	
	// configure presence sense GPIO
	gpio_mode_setup(SENSEPORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SENSEPIN);

	// configure presence signal GPIO
	gpio_mode_setup(SIGNALPORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, SIGNALPIN);

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

	// configure USART port IOs (AF7: USART1_RX, USART1_TX)
	gpio_mode_setup(USARTPORT_RX, GPIO_MODE_AF, GPIO_PUPD_NONE, USARTPIN_RX);
	gpio_mode_setup(USARTPORT_TX, GPIO_MODE_AF, GPIO_PUPD_NONE, USARTPIN_TX);
	gpio_set_af(USARTPORT_RX, GPIO_AF7, USARTPIN_RX);
	gpio_set_af(USARTPORT_TX, GPIO_AF7, USARTPIN_TX);

	// configure I2C port IOs (AF4: I2C1_SCL, I2C1_SDA)
	gpio_mode_setup(I2CPORT_SCL, GPIO_MODE_AF, GPIO_PUPD_NONE, I2CPIN_SCL);
	gpio_mode_setup(I2CPORT_SDA, GPIO_MODE_AF, GPIO_PUPD_NONE, I2CPIN_SDA);
	gpio_set_output_options(I2CPORT_SCL, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, I2CPIN_SCL);
	gpio_set_output_options(I2CPORT_SDA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, I2CPIN_SDA);
    gpio_set_af(I2CPORT_SCL, GPIO_AF4, I2CPIN_SCL);
    gpio_set_af(I2CPORT_SDA, GPIO_AF4, I2CPIN_SDA);
}

static FILE* usart_setup(void)
{
	// Configure USART1: 115200 8N1
	usart_set_baudrate(USART1, 115200);
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

static void i2c_setup(void) {
	nvic_enable_irq(NVIC_I2C1_EV_EXTI23_IRQ);

	i2c_reset(I2C1);
	i2c_peripheral_disable(I2C1);

	// Assuming the defaults work here
	i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);
	i2c_set_own_7bit_slave_address(I2C1, I2C_DEFAULT_ADDRESS);

	// There's a bug in libopencm3 that doesn't set this correctly, which is why we're doing it here.
	I2C_OAR1(I2C1) |= I2C_OAR1_OA1EN_ENABLE;

	i2c_enable_interrupt(I2C1, I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_ADDRIE);

	// Enable I2C1 peripheral
	i2c_peripheral_enable(I2C1);
}

static void timer_setup(void)
{
	// Enable the timer2 interrupt and reset the peripheral
	rcc_periph_reset_pulse(RST_TIM2);

	// These are the defaults after reset; no division, edge alignment, count up
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	// Set the prescaler to (peripheral clock frequency / 1000000) - 1 , so it'll tick at 1MHz
	// for a peripheral clock of 25MHz, this makes the prescaler 24
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
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB); // 25MHz
	systick_set_reload(24999); // period = 25000, so we overflow at 1kHz
	systick_interrupt_enable();
	systick_counter_enable();
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
		case 0xa:
			gpio_set(SEGPORT, 0xee);
			break;
		case 0xb:
			gpio_set(SEGPORT, 0x7c);
			break;
		case 0xc:
			gpio_set(SEGPORT, 0x1e);
			break;
		case 0xd:
			gpio_set(SEGPORT, 0xf8);
			break;
		case 0xe:
			gpio_set(SEGPORT, 0x5e);
			break;
		case 0xf:
			gpio_set(SEGPORT, 0x4e);
			break;
		default:
			gpio_set(SEGPORT, 0x00);
	}
}

/* This function's job is to ensure that transitions between digits are as smooth as
 * possible.  It does this by computing the sequence of length (l) with (n) bits set,
 * for which the average transition density is maximized. It then returns position (p)
 * of the computed sequence.
 * 
 * For example, with l=7, n=3, p=1, the computed sequence is 0010101 (or a rotation 
 * thereof) and the returned value is 0.  For l=7, n=4, p=1, the computed sequence is
 * 1010101 (or a rotation thereof) and the returned value is 1.
 * 
 * The digit transition function works on chunks of (l) one-millisecond timeslices.
 * Each chunk will have a given target of 'on' time (n). Once we've computed this
 * sequence, we can use it to time the digit transitions in a way that produces the
 * least possible amount of flicker for a particular (l) and (n).
 * 
 * I don't know what this operation is called - I suppose it's some sort of one-
 * dimensional (temporal) dithering.
 * 
 * TODO: Rewrite this with floating point?  Does anyone care?
 */
static uint8_t _select_digit( uint8_t length, uint8_t set_bits, uint8_t pos ) {
	uint8_t gap_mod = length % set_bits;
	uint8_t gap_base = ( ( length - set_bits ) - gap_mod ) / set_bits;

	uint8_t accum_mod = gap_mod;
	uint8_t accum_base = gap_base;

	uint8_t digit = 0;

	// Degenerate cases
	if ( set_bits == 0 ) {
		return 0;
	}

	if ( set_bits == length ) {
		return 1;
	}

	for (;;) {
		if ( accum_base == 0 ) {
			digit = 1;
			accum_mod  += gap_mod;
			accum_base += gap_base;
		} else {
			digit = 0;
			accum_base--;
		}

		if ( --pos == 0 ) {
			return digit;
		}

		if ( accum_mod >= set_bits ) {
			digit = 0;
			accum_mod -= set_bits;

			if ( --pos == 0 ) {
				return digit;
			}
		}
	}
}

static void update_display(void) {
	if ( display_digit == root->display_data )
		return;

	if ( root->transition_type == TRANSITION_TYPE_DEFAULT ) {
		display_digit = root->display_data;
		_set_digit( display_digit );
	}

	if ( root->transition_type == TRANSITION_TYPE_FADE ) {
		if ( transition_counter < ( N_TRANSITION_STEPS * T_STEP_PERIOD ) ) {
			// The current brightness step in this transition
			uint8_t current_step = ( ( transition_counter - ( transition_counter % T_STEP_PERIOD ) ) / T_STEP_PERIOD ) + 1;
			// The current position within that brightness step
			uint8_t current_position = ( transition_counter % T_STEP_PERIOD ) + 1;

			// The number of bits set in the current transition step; (n_bits / t_step_period) is the
			// duty cycle for the new display data and determines how bright the new symbol appears
			// compared to the old one
			uint8_t n_bits = ( T_STEP_PERIOD / N_TRANSITION_STEPS ) * current_step;

			if ( _select_digit( T_STEP_PERIOD, n_bits, current_position ) ) {
				_set_digit( root->display_data );
			} else {
				_set_digit( display_digit );
			}

			transition_counter++;
		} else {
			transition_counter = 0;
			display_digit = root->display_data;
			_set_digit( display_digit );
		}
	}
}

static void update_button(void) {
	if ( gpio_get(SWPORT, SWPIN) ) {
		if ( button_counter > BUTTON_OFF )
			button_counter--;
	} else {
		if ( button_counter < BUTTON_ON )
			button_counter++;
	}
}

static uint8_t button_state(void) {
	if ( button_counter == BUTTON_ON ) {
		if ( button_holdoff_counter == 0 ) {
			button_holdoff_counter = BUTTON_HOLDOFF;
			return BUTTON_ON;
		}
		button_holdoff_counter--;
	}

	if ( button_counter == BUTTON_OFF )
		button_holdoff_counter = 0;

	return BUTTON_OFF;
}

void sys_tick_handler(void) {
	isr_flag = 1;

	time.milliseconds++;
	srtc_update(&time);
}

void i2c1_ev_exti23_isr(void) {
	i2c_flag = 1;

	uint32_t isr = I2C_ISR(I2C1);

	if (isr & I2C_ISR_ADDR) {
		read_ptr = buf;
		read_bytes = 0;

		// clear address match
		I2C_ICR(I2C1) |= I2C_ICR_ADDRCF;
	}
	else if (isr & I2C_ISR_RXNE) {
		if (read_bytes > 3)
			return;
		*read_ptr++ = I2C_RXDR(I2C1) & 0xff;
		read_bytes++;
	}
	else if (isr & I2C_ISR_TXIS) {
		I2C_TXDR(I2C1) = val;
	}
	else if (isr & I2C_ISR_STOPF) {
		i2c_peripheral_enable(I2C1);

		switch(buf[0]) {
			case I2C_CMD_SET_ADDR:
				// OA1EN_ENABLE must be cleared before we can update our own address
				I2C_OAR1(I2C1) &= ~(I2C_OAR1_OA1EN_ENABLE);
				i2c_set_own_7bit_slave_address(I2C1, buf[1]);
				I2C_OAR1(I2C1) |= I2C_OAR1_OA1EN_ENABLE;
				break;
			case I2C_CMD_SET_INDEX:
				root->node_index = buf[1];
				break;
			case I2C_CMD_GET_INDEX:
				val = root->node_index;
				break;
			case I2C_CMD_SET_TRANSITION_TYPE:
				root->transition_type = buf[1];
				break;
			case I2C_CMD_GET_TRANSITION_TYPE:
				val = root->transition_type;
				break;
			case I2C_CMD_SET_DISPLAY_DATA:
				root->display_data = buf[1];
				break;
			case I2C_CMD_GET_DISPLAY_DATA:
				val = root->display_data;
				break;
			case I2C_CMD_SET_SIGNAL:
				if (buf[1])
					gpio_set(SIGNALPORT, SIGNALPIN);
				else
					gpio_clear(SIGNALPORT, SIGNALPIN);
				break;
			case I2C_CMD_GET_SIGNAL:
				val = (uint8_t) gpio_get(SIGNALPORT, SIGNALPIN);
				break;
			default:
				break;
		}

		// clear TXDR - we can't write to it once it's got a value in it
		I2C_ISR(I2C1) |= I2C_ISR_TXE;

		// clear stop flag
		I2C_ICR(I2C1) |= I2C_ICR_STOPCF; 
	}
	else if ( isr & I2C_ISR_NACKF ) {
		val = I2C_TRANSFER_NACK;

		// clear nack flag
		I2C_ICR(I2C1) |= I2C_ICR_NACKCF;
	}
}

void i2c1_er_isr(void) {
	i2c_flag = 1;
}

int main(void) {
	uint8_t state = STATE_UNKNOWN;
	uint8_t role = ROLE_UNKNOWN;
	uint16_t temp = 0;

	display_digit = 0;
	transition_counter = 0;

	button_counter = BUTTON_OFF;
	button_holdoff_counter = BUTTON_HOLDOFF;

	clock_setup();
	gpio_setup();
    timer_setup();
	systick_setup();
	i2c_setup();

	FILE *fp;
	fp = usart_setup();

	root = add_node();

	state = STATE_CONF;
	fprintf(fp, "[state] 0x%02x\n", state);

	fprintf(fp, "[init] Starting init\n");

	// Light everything up
	gpio_set(SEGPORT, SEGPIN_ALL);			// Enable all segments
	gpio_set(EN5VPORT, EN5VPIN);			// Enable 5V regulator
	gpio_set(SIGNALPORT, SIGNALPIN);		// Enable presence detect
	gpio_set(LEDPORT, LEDPIN);				// Turn on user LED

	gpio_clear(ENI2CPORT, ENI2CPIN);		// Ensure I2C buffer is disabled

	fprintf(fp, "[init] Port init complete\n");

	// Wait for a second to see if another board has control; all role selection should be
	// complete during this period. If this times out, this device will elect itself leader
	temp = time.milliseconds - 1;

	while (temp != time.milliseconds) {
		if (isr_flag) {
			isr_flag = 0;

			if (gpio_get(SENSEPORT, SENSEPIN)) {
				fprintf(fp, "[init] Cohort detected (%d)\n", time.milliseconds - temp);
				role = ROLE_COHORT;
				temp = time.milliseconds;
			}
		}
	}

#ifdef DEBUG_ROLE_COHORT
	fprintf(fp, "[debug] Forcing role to ROLE_COHORT\n");
	role = ROLE_COHORT;
#endif

	if (role == ROLE_UNKNOWN) {
		fprintf(fp, "[init] No upstream cohort detected, assuming leader role\n");
		role = ROLE_LEADER;
	}

	// Clear the presence detect signal; this'll be used in a moment to instruct other nodes
	// to turn on their i2c buffers so they can be configured
	gpio_clear(SIGNALPORT, SIGNALPIN);

	fprintf(fp, "[init] Role selection complete (%d)\n", role);

	if (role == ROLE_LEADER) {
		I2C_OAR1(I2C1) &= ~(I2C_OAR1_OA1EN_ENABLE);
		i2c_set_own_7bit_slave_address(I2C1, I2C_LEADER_ADDRESS);
		root->i2c_address = I2C_LEADER_ADDRESS;
		root->node_index = 0;
		root->display_data = 0;
		root->transition_type = TRANSITION_TYPE_FADE;
		root->next_node = NULL;

		fprintf(fp, "[init] Own i2c address set to 0x%02x\n", I2C_LEADER_ADDRESS);
	} else {
		root->i2c_address = I2C_DEFAULT_ADDRESS;
		root->node_index = 0;
		root->display_data = 0;
		root->transition_type = TRANSITION_TYPE_DEFAULT;
		root->next_node = NULL;

		fprintf(fp, "[init] Own i2c address set to 0x%02x\n", I2C_DEFAULT_ADDRESS);
	}

	if (role == ROLE_LEADER) {
		// This tells the next node up in the chain to turn on its I2C buffer so we can talk
		// to it on the bus.  Since this is will be the first device (besides the leader) on
		// the I2C bus, it'll have address I2C_DEFAULT_ADDRESS.  Our first goal will be to assign
		// it a new address and verify that we can still chat with it

		fprintf(fp, "[init] Signaling cohort to enable i2c buffer\n");

		// Lazy busy-wait
		for (int i = 0; i < 10000; i++)
			__asm__("nop");

		gpio_set(SIGNALPORT, SIGNALPIN);

		for (int i = 0; i < 10000; i++)
			__asm__("nop");

		gpio_clear(SIGNALPORT, SIGNALPIN);
	}

	if (role == ROLE_COHORT) {
		// Cohorts must wait until they recieve a signal on SENSEPIN before enabling their
		// I2C buffers.  This allows the leader to assign new addresses one at a time without
		// multiple I2C nodes listening at the same address

		// Since this must only be done once during init, using a busy-wait is okay

		fprintf(fp, "[init] Waiting for i2c buffer-enable signal\n");

#ifndef DEBUG_ROLE_COHORT
		while(!gpio_get(SENSEPORT, SENSEPIN)) {
			for (int i = 0; i < 1000; i++)
				__asm__("nop");			
		}
#endif

		gpio_set(ENI2CPORT, ENI2CPIN);

		fprintf(fp, "[init] Enabled i2c buffer\n");
	} else {
		// Leader sits this one out
		for (int i = 0; i < 10000; i++)
			__asm__("nop");
	}

	/*
	 * At this point, this device will be in one of the following three states:
	 * - Leader (at I2C_LEADER_ADDRESS)
	 * - Cohort (at I2C_DEFAULT_ADDRESS) with its I2C buffer enabled, ready for configuration
	 * - Cohort (at I2C_DEFAULT_ADDRESS) with its I2C buffer disabled, waiting to enable
	 * 
	 * The leader's job is now to do the following:
	 * - Assign any waiting cohort the next available I2C address
	 * - Assign an index representing a physical position
	 * - Instruct cohort to set and then clear SIGNALPIN to put the next device on the bus
	 * - Repeat until there are no more unconfigured cohorts
	 */

	if (role == ROLE_LEADER) {
		// cohort interrupts interfere with the leader sending this probe.  leader doesn't need
		// anything but the nack interrupt anyhow
		i2c_disable_interrupt(I2C1, I2C_CR1_STOPIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_ADDRIE);

		fprintf(fp, "[init] Starting bus probe\n");

		bool device_probe_failed = false;
		uint8_t index = 0;

		while (device_probe_failed == false) {
			fprintf(fp, "[init] Probing default address 0x%02x\n", I2C_DEFAULT_ADDRESS);

			// XXX: The leader finishes each loop iteration in a weird state, which is cleared here
			i2c_peripheral_disable(I2C1);
			i2c_peripheral_enable(I2C1);

			// This is kinda piecemeal, and much of it is stolen from i2c_common_v2.c - however the
			// library assumes that a device will be present and does no error checking; in order to
			// probe for a device, we need to issue a speculative read to I2C_DEFAULT_ADDRESS and then
			// wait to see whether the transaction generated a NACK, which is something we can detect
			i2c_set_7bit_address(I2C1, I2C_DEFAULT_ADDRESS);
			i2c_set_read_transfer_dir(I2C1);
			i2c_set_bytes_to_transfer(I2C1, 1);
			i2c_enable_autoend(I2C1);
			I2C_CR2(I2C1) |= I2C_CR2_START;

			// Stall for a few hundred nanoseconds to let the transaction begin
			for (int i = 0; i < 50; i++)
				__asm__("nop");

			// Wait until the transaction is complete
			while(i2c_busy(I2C1));

			if (val == I2C_TRANSFER_NACK) {
				fprintf(fp, "[init] No device at 0x%02x\n", I2C_DEFAULT_ADDRESS);
				device_probe_failed = true;
			} else {
				fprintf(fp, "[init] Device present at 0x%02x\n", I2C_DEFAULT_ADDRESS);

				index++;

				// Set the new node's address
				buf[0] = I2C_CMD_SET_ADDR;
				buf[1] = I2C_LEADER_ADDRESS + index;
				i2c_transfer7(I2C1, I2C_DEFAULT_ADDRESS, (uint8_t*) &buf, 2, (uint8_t*) &buf, 0);

				fprintf(fp, "[init] (%d) Relocating: 0x%02x -> 0x%02x\n", index, I2C_DEFAULT_ADDRESS, buf[1]);

				// Physical index
				buf[0] = I2C_CMD_SET_INDEX;
				buf[1] = index;
				i2c_transfer7(I2C1, I2C_LEADER_ADDRESS + index, (uint8_t*) &buf, 2, (uint8_t*) &buf, 0);

				fprintf(fp, "[init] (%d) Physical index: %d\n", index, buf[1]);

				// Display data
				buf[0] = I2C_CMD_SET_DISPLAY_DATA;
				buf[1] = index;
				i2c_transfer7(I2C1, I2C_LEADER_ADDRESS + index, (uint8_t*) &buf, 2, (uint8_t*) &buf, 0);

				fprintf(fp, "[init] (%d) Display data: %d\n", index, buf[1]);

				// And finally signal the next device to get on the bus
				buf[0] = I2C_CMD_SET_SIGNAL;
				buf[1] = 1;
				i2c_transfer7(I2C1, I2C_LEADER_ADDRESS + index, (uint8_t*) &buf, 2, (uint8_t*) &buf, 0);

				for (int i = 0; i < 10000; i++)
					__asm__("nop");

				buf[0] = I2C_CMD_SET_SIGNAL;
				buf[1] = 0;
				i2c_transfer7(I2C1, I2C_LEADER_ADDRESS + index, (uint8_t*) &buf, 2, (uint8_t*) &buf, 0);

				// Now that the device is configured, update our own bookkeeping
				node_t* node = add_node();
				node->i2c_address = I2C_LEADER_ADDRESS + index;
				node->node_index = index;
				node->display_data = index;
				node->next_node = NULL;

				node_t* current = root;

				while (current->next_node != NULL) {
					current = current->next_node;
				}

				current->next_node = node;

				fprintf(fp, "[init] (%d) Finished configuration\n", index);
			}
		}

		// XXX: Leader ends this transaction block with STOPF set, which is cleared here
		I2C_ICR(I2C1) |= I2C_ICR_STOPCF;
	}

	fprintf(fp, "[init] Bus probe complete\n");
	fprintf(fp, "[init] Summary of configured devices:\n");

	node_t* current = root;
	uint8_t n_nodes = 1;

	fprintf(fp, "[init]   %d address: 0x%02x\n",  current->node_index, current->i2c_address);

	while (current->next_node != NULL) {
		current = current->next_node;
		n_nodes++;

		fprintf(fp, "[init]   %d address: 0x%02x\n",  current->node_index, current->i2c_address);
	}

	fprintf(fp, "[init]   (%d total)\n", n_nodes);

	fprintf(fp, "[init] Init complete\n");

	state = STATE_RUN;
	fprintf(fp, "[state] 0x%02x\n", state);

	fprintf(fp, "[run] Starting runloop\n");

    // Main runloop
	while (1) {
		if (i2c_flag) {
			fprintf(fp, "[i2c] i2c interrupt fired!\n");
			fprintf(fp, "[i2c] val: %d\n", val);
			fprintf(fp, "[i2c] I2C_ISR: 0x%08lx I2C_ICR: 0x%08lx I2C_OAR1: 0x%08lx\n", I2C_ISR(I2C1), I2C_ICR(I2C1), I2C_OAR1(I2C1));

			i2c_flag = 0;
		}

		// the systick handler sets this flag every millisecond
        if (isr_flag) {
			isr_flag = 0;

		    // Do some stuff - toggle LED GPIO, set the next digit to display from timekeeping register
			if (time.milliseconds == 0) {
	    	    gpio_toggle(LEDPORT, LEDPIN);

				_set_grid_duty_cycle(100);
				root->display_data = ((time.seconds % 16) + 1) % 16;

				fprintf(fp, "[run] Tick (%02d)\n", time.seconds);
			}

			update_button();
			update_display();

			if ( button_state() == BUTTON_ON )
				fprintf(fp, "button on!\n");
		}
	}

	return 0;
}
