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
#include <libopencm3/stm32/fsmc.h>
#include <libopencm3/stm32/exti.h>

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
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);

        /* FSMC */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPFEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPGEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPEEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPIEN);

	/* Enable clocks for USART3. */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);

        /* FSMC */
	rcc_peripheral_enable_clock(&RCC_AHB3ENR, RCC_AHB3ENR_FSMCEN);
}

static void usart_setup(void)
{
	/* Enable the USART3 interrupt. */
	nvic_enable_irq(NVIC_USART3_IRQ);

	/* Setup GPIO pins for USART3 transmit. */
        gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO10);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

	/* Setup GPIO pins for USART3 receive. */
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO11);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);

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

#define FSMC_BCR_RESERVED (1 << 7)

static void fsmc_setup(void)
{
        /* Address Lines 0-9 on Port F */
        uint16_t portf_gpios = GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5 |
          GPIO12 | GPIO13 | GPIO14 | GPIO15;
        gpio_set_af(GPIOF, GPIO_AF12, portf_gpios);
	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, portf_gpios);
	gpio_set_output_options(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, portf_gpios);

        /* Address Lines 10-13 on Port G */
        uint16_t portg_gpios = GPIO0 | GPIO1 | GPIO2 | GPIO3;
        gpio_set_af(GPIOG, GPIO_AF12, portg_gpios);
	gpio_mode_setup(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, portg_gpios);
	gpio_set_output_options(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, portg_gpios);

        /* Data Lines 0-3, 13-15, OE, WE, E1 on Port D */
        uint16_t portd_gpios = GPIO14 | GPIO15 | GPIO0 | GPIO1 |
          GPIO8 | GPIO9 | GPIO10 | GPIO4 | GPIO5 | GPIO7;
        gpio_set_af(GPIOD, GPIO_AF12, portd_gpios);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, portd_gpios);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, portd_gpios);

        /* Data Lines 4-12, NBL0, NBL1 on Port E */
        uint16_t porte_gpios = GPIO7 | GPIO8 | GPIO9 | GPIO10 | GPIO11 |
          GPIO12 | GPIO13 | GPIO14 | GPIO15 | GPIO0 | GPIO1;
        gpio_set_af(GPIOE, GPIO_AF12, porte_gpios);
	gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, porte_gpios);
	gpio_set_output_options(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, porte_gpios);

        /* Special inputs Busy, Int on Port I */
        uint16_t porti_gpios = GPIO8 | GPIO10;
	gpio_mode_setup(GPIOI, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, porti_gpios);
	//gpio_set_output_options(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, porti_gpios);

        /* Special output Sem on Port I */
        porti_gpios = GPIO11;
	gpio_mode_setup(GPIOI, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, porti_gpios);
	gpio_set_output_options(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, porti_gpios);

        /* Configure in Mode 1 according to Datasheet p1527 ff. */
        FSMC_BCR(0) = FSMC_BCR_WREN | FSMC_BCR_RESERVED | FSMC_BCR_MWID | FSMC_BCR_MBKEN;
        FSMC_BTR(0) = FSMC_BTR_BUSTURNx(0) | FSMC_BTR_DATASTx(0) | FSMC_BTR_ADDSETx(0);

	nvic_enable_irq(NVIC_EXTI9_5_IRQ);
        exti_select_source(EXTI8, GPIOI);

	exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
	exti_reset_request(EXTI8);
	exti_enable_request(EXTI8);
}

void exti9_5_isr(void)
{
	exti_reset_request(EXTI8);
        printf("Got interrupt! We need to read something...\r\n");
}

char test_to_run = 'A';
unsigned int* test = (unsigned int*)0x60000000;
unsigned int data = 0xc0ffeeba;

static uint32_t readhex(void)
{
        uint32_t out = 0;
        int bytes = 8;
        while(bytes)
        {
                char new, print;

                /* Wait for character */
                while (!test_to_run);

                new = print = test_to_run;
                test_to_run = '\0';

                if (new >= '0' && new <= '9')
                         new -= '0';
                else if (new >= 'a' && new <= 'f')
                         new -= 'a' + 10;
                else
                          continue;

                out <<= 4;
                out |= new;
                printf("%c", print);
                fflush(stdout);
                bytes--;
        }
        printf("\r\n");
        return out;
}

static void help(void)
{
        printf("Startup complete\r\n");
        printf("Select test:\r\n");
        printf(" - [d]estination (currently %08x)\r\n", (unsigned int)test);
        printf(" - [e]dit data (currently %08x)\r\n", (unsigned int)data);
        printf(" - [w]rite\r\n");
        printf(" - [r]ead\r\n");
        printf(" - [s]et semaphore\r\n");
        printf(" - [f]alsify semaphore\r\n");
        printf(" - [o]ut test (write to all addresses)\r\n");
        printf(" - [i]n test (read all addresses)\r\n");
        printf(" - b[c]r register\r\n");
        printf(" - b[t]r register\r\n");
        printf(" - [h]elp\r\n");
}

int main(void)
{
        unsigned int *addr;

	clock_setup();
	gpio_setup();
	usart_setup();
        fsmc_setup();

        help();

	while (1) {
		switch (test_to_run)
		{
                case 'h':
                        help();
                        test_to_run = '\0';
                        break;
                case 'd':
                        printf("Define new destination address: 0x");
                        fflush(stdout);
			test_to_run = '\0';
                        test = (unsigned int*)readhex();
                        break;
                case 'e':
                        printf("Define new data: 0x");
                        fflush(stdout);
                        test_to_run = '\0';
                        data = (int)readhex();
                        break;
		case 'r':
			printf("Running read test from (%08x)...\r\n", (unsigned int)test);
                        printf("0x%08x\r\n", *test);
			test_to_run = '\0';
			break;
		case 'w':
			printf("Running write (%08x)...\r\n", (unsigned int)data);
                        *test = data;
			test_to_run = '\0';
			break;
                case 's':
                        printf("Set semaphore...\r\n");
                        gpio_set(GPIOI, GPIO11);
			test_to_run = '\0';
                        break;
                case 'f':
                        printf("Clear semaphore...\r\n");
                        gpio_clear(GPIOI, GPIO11);
			test_to_run = '\0';
                        break;
                case 'o':
                        printf("Write addresses 0x60000000..0x60003fff...\r\n");
                        for (addr = (unsigned int *)0x60000000; addr < (unsigned int *)0x60003fff; addr++)
                                  *addr = (unsigned int)addr;
                        printf("Done...\r\n");
			test_to_run = '\0';
                        break;
                case 'i':
                        printf("Read addresses...\r\n");
                        for (addr = (unsigned int *)0x60000000; addr < (unsigned int *)0x60003fff; addr++) {
                                  if (*addr != (unsigned int)addr)
                                          printf("Wrong data (0x%08x) at address  0x%08x\r\n",
                                              *addr, (unsigned int)addr);
                                  *addr = (unsigned int)addr;
                        }
                        printf("Done...\r\n");
			test_to_run = '\0';
                        break;
                case 'c':
                        printf("BCR register: 0x%08x\r\n", (unsigned int)FSMC_BCR(0));
			test_to_run = '\0';
                        break;
                case 't':
                        printf("BTR register: 0x%08x\r\n", (unsigned int)FSMC_BTR(0));
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
