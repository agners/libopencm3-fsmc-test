/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2013 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

int _write(int file, char *ptr, int len);

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_send_blocking(USART3, ptr[i]);
		return i;
	}

	errno = EIO;
	return -1;
}

static void clock_setup(void)
{
	/* Enable GPIOD clock for LED & USARTs. */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);

	/* Enable clocks for USART3. */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);
}

static void usart_setup(void)
{
	/* Enable the USART3 interrupt. */
	nvic_enable_irq(NVIC_USART3_IRQ);

	/* Setup GPIO pins for USART3 transmit. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

	/* Setup GPIO pins for USART3 receive. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO11);

	/* Setup USART3 TX and RX pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF7, GPIO10 | GPIO11);

	/* Setup USART3 parameters. */
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Enable USART3 Receive interrupt. */
	usart_enable_rx_interrupt(USART3);

	/* Finally enable the USART. */
	usart_enable(USART3);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO12 on GPIO port D for LED. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

char test_to_run = 'A';

int main(void)
{
	clock_setup();
	gpio_setup();
	usart_setup();

        printf("Startup complete\r\n");
        printf("Select test:\r\n");
        printf(" - [w]rite\r\n");
        printf(" - [r]ead\r\n");


	while (1) {
		switch (test_to_run)
		{
		case 'r':
			printf("Running read test...\r\n");
			test_to_run = '\0';
			break;
		case 'w':
			printf("Running write test...\r\n");
			test_to_run = '\0';
			break;
                case '\0':
			__asm__("NOP");
			break;
		}
	}

	return 0;
}

void usart3_isr(void)
{

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOD, GPIO12);

		/* Retrieve the data from the peripheral. */
		test_to_run = usart_recv(USART3);
	}

}
