#ifndef _MAIN_H
#define _MAIN_H

#include <stdint.h>
#include <stdio.h>

#define LEDPORT				(GPIOB)
#define LEDPIN				(GPIO5)

#define SWPORT				(GPIOB)
#define SWPIN				(GPIO4)

#define SENSEPORT			(GPIOB)
#define SENSEPIN			(GPIO0)

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

#define I2CPORT				(GPIOB)
#define I2CPIN_SCL			(GPIO6)
#define I2CPIN_SDA			(GPIO7)

#define USARTPORT           (GPIOB)
#define USARTPIN_TX         (GPIO6)
#define USARTPIN_RX         (GPIO7)

#define ERR_ISR_TIMEOUT     (1<<31);

#define ENABLE_DEBUG_USART

volatile uint8_t isr_flag;
volatile uint32_t status;

static void clock_setup(void);
static void timer_setup(void);
static void gpio_setup(void);

#ifdef ENABLE_DEBUG_USART
static FILE *usart_setup(void);
static ssize_t _iord(void *_cookie, char *_buf, size_t _n);
static ssize_t _iowr(void *_cookie, const char *_buf, size_t _n);
#else
static void i2c_setup(void);
#endif // ENABLE_DEBUG_USART

static void _set_grid_duty_cycle(uint8_t duty_cycle);
static void _set_digit(uint8_t value);

#endif // _MAIN_H