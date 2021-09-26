#ifndef _MAIN_H
#define _MAIN_H

#include <stdint.h>
#include <stdio.h>

#include "timekeeping.h"
#include "ansible.h"

#define DEBUG_ROLE_COHORT   1

#define LEDPORT				(GPIOB)
#define LEDPIN				(GPIO5)

#define SWPORT				(GPIOB)
#define SWPIN				(GPIO4)

#define SIGNALPORT          (GPIOB)
#define SIGNALPIN           (GPIO0)

#define SENSEPORT			(GPIOB)
#define SENSEPIN			(GPIO1)

#define SEGPORT				(GPIOA)
#define SEGPIN_G			(GPIO0)
#define SEGPIN_1			(GPIO1)
#define SEGPIN_2			(GPIO2)
#define SEGPIN_3			(GPIO3)
#define SEGPIN_4			(GPIO4)
#define SEGPIN_5			(GPIO5)
#define SEGPIN_6			(GPIO6)
#define SEGPIN_7			(GPIO7)
#define SEGPIN_ALL          (SEGPIN_G | SEGPIN_1 | SEGPIN_2 | SEGPIN_3 | SEGPIN_4 | SEGPIN_5 | SEGPIN_6 | SEGPIN_7)
#define SEGPIN_SEGMENTS     (SEGPIN_1 | SEGPIN_2 | SEGPIN_3 | SEGPIN_4 | SEGPIN_5 | SEGPIN_6 | SEGPIN_7)

#define FILPORT				(GPIOA)
#define FILPIN				(GPIO9)

#define EN5VPORT			(GPIOF)
#define EN5VPIN				(GPIO1)

#define ENI2CPORT			(GPIOA)
#define ENI2CPIN			(GPIO11)

#define I2CPORT_SCL			(GPIOA)
#define I2CPIN_SCL			(GPIO15)

#define I2CPORT_SDA			(GPIOB)
#define I2CPIN_SDA			(GPIO7)

#define USARTPORT_TX        (GPIOB)
#define USARTPIN_TX         (GPIO6)

#define USARTPORT_RX        (GPIOA)
#define USARTPIN_RX         (GPIO10)

// State machine definitions
#define STATE_UNKNOWN       0x00
#define STATE_CONF          0x01
#define STATE_RUN           0x02

// Role definitions
#define ROLE_UNKNOWN        0x00
#define ROLE_LEADER         0x01
#define ROLE_COHORT         0x02

volatile uint8_t isr_flag;
volatile uint8_t i2c_flag;
volatile uint32_t status;
volatile stime_t time;

// I2C buffers and such
volatile uint8_t* read_ptr;
volatile uint8_t read_bytes;
volatile uint8_t buf[3];
volatile uint8_t val;

// Root node for device state.  The root node is always the local device,
// and the only device with more than one entry in this list is the leader
node_t* root;

static void clock_setup(void);
static void timer_setup(void);
static void gpio_setup(void);
static void i2c_setup(void);

static FILE *usart_setup(void);
static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);

static void _set_grid_duty_cycle(uint8_t duty_cycle);
static void _set_digit(uint8_t value);
static uint8_t _select_digit(uint8_t length, uint8_t set_bits, uint8_t pos);

#endif // _MAIN_H