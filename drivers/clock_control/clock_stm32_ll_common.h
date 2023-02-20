/*
 *
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_CONTROL_STM32_LL_CLOCK_H_
#define ZEPHYR_DRIVERS_CLOCK_CONTROL_STM32_LL_CLOCK_H_

#include <stdint.h>

#include <zephyr/device.h>

#include <stm32_ll_utils.h>

#if CONFIG_CLOCK_STM32_MCO1_SRC_NOCLOCK
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_NOCLOCK
#elif CONFIG_CLOCK_STM32_MCO1_SRC_EXT_HSE
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_EXT_HSE
#elif CONFIG_CLOCK_STM32_MCO1_SRC_LSE
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_LSE
#elif CONFIG_CLOCK_STM32_MCO1_SRC_HSE
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_HSE
#elif CONFIG_CLOCK_STM32_MCO1_SRC_LSI
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_LSI
#elif CONFIG_CLOCK_STM32_MCO1_SRC_MSI
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_MSI
#elif CONFIG_CLOCK_STM32_MCO1_SRC_HSI
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_HSI
#elif CONFIG_CLOCK_STM32_MCO1_SRC_HSI16
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_HSI
#elif CONFIG_CLOCK_STM32_MCO1_SRC_HSI48
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_HSI48
#elif CONFIG_CLOCK_STM32_MCO1_SRC_PLLCLK
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_PLLCLK
#elif CONFIG_CLOCK_STM32_MCO1_SRC_PLLCLK_DIV2
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_PLLCLK_DIV_2
#elif CONFIG_CLOCK_STM32_MCO1_SRC_PLL2CLK
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_PLL2CLK
#elif CONFIG_CLOCK_STM32_MCO1_SRC_PLLI2SCLK
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_PLLI2SCLK
#elif CONFIG_CLOCK_STM32_MCO1_SRC_PLLI2SCLK_DIV2
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_PLLI2SCLK_DIV2
#elif CONFIG_CLOCK_STM32_MCO1_SRC_SYSCLK
	#define MCO1_SOURCE		LL_RCC_MCO1SOURCE_SYSCLK
#endif

#if CONFIG_CLOCK_STM32_MCO2_SRC_SYSCLK
	#define MCO2_SOURCE		LL_RCC_MCO2SOURCE_SYSCLK
#elif CONFIG_CLOCK_STM32_MCO2_SRC_PLLI2S
	#define MCO2_SOURCE		LL_RCC_MCO2SOURCE_PLLI2S
#elif CONFIG_CLOCK_STM32_MCO2_SRC_HSE
	#define MCO2_SOURCE		LL_RCC_MCO2SOURCE_HSE
#elif CONFIG_CLOCK_STM32_MCO2_SRC_PLLCLK
	#define MCO2_SOURCE		LL_RCC_MCO2SOURCE_PLLCLK
#endif

/* Macros to fill up multiplication and division factors values */
#define z_pllm(v) LL_RCC_PLLM_DIV_ ## v
#define pllm(v) z_pllm(v)

#define z_pllp(v) LL_RCC_PLLP_DIV_ ## v
#define pllp(v) z_pllp(v)

#define z_pllq(v) LL_RCC_PLLQ_DIV_ ## v
#define pllq(v) z_pllq(v)

#define z_pllr(v) LL_RCC_PLLR_DIV_ ## v
#define pllr(v) z_pllr(v)

#if defined(STM32_PLLSAI_ENABLED)

#define z_pllsaim(v) LL_RCC_PLLSAIM_DIV_ ## v
#define pllsaim(v) z_pllsaim(v)

#define z_pllsaip(v) LL_RCC_PLLSAIP_DIV_ ## v
#define pllsaip(v) z_pllsaip(v)

#define z_pllsaiq(v) LL_RCC_PLLSAIQ_DIV_ ## v
#define pllsaiq(v) z_pllsaiq(v)

#define z_pllsair(v) LL_RCC_PLLSAIR_DIV_ ## v
#define pllsair(v) z_pllsair(v)

#define z_pllsaidivq(v) LL_RCC_PLLSAIDIVQ_DIV_ ## v
#define pllsaidivq(v) z_pllsaidivq(v)

#define z_pllsaidivr(v) LL_RCC_PLLSAIDIVR_DIV_ ## v
#define pllsaidivr(v) z_pllsaidivr(v)

#endif

#if defined(STM32_PLLI2S_ENABLED)

#define z_plli2sm(v) LL_RCC_PLLI2SM_DIV_ ## v
#define plli2sm(v) z_plli2sm(v)

#define z_plli2sp(v) LL_RCC_PLLI2SP_DIV_ ## v
#define plli2sp(v) z_plli2sp(v)

#define z_plli2sq(v) LL_RCC_PLLI2SQ_DIV_ ## v
#define plli2sq(v) z_plli2sq(v)

#define z_plli2sr(v) LL_RCC_PLLI2SR_DIV_ ## v
#define plli2sr(v) z_plli2sr(v)

#define z_plli2sdivq(v) LL_RCC_PLLI2SDIVQ_DIV_ ## v
#define plli2sdivq(v) z_plli2sdivq(v)

#define z_plli2sdivr(v) LL_RCC_PLLI2SDIVR_DIV_ ## v
#define plli2sdivr(v) z_plli2sdivr(v)

#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(STM32_PLL_ENABLED)
void config_pll_sysclock(void);
uint32_t get_pllout_frequency(void);
uint32_t get_pllsrc_frequency(void);
#endif
#if defined(STM32_PLL2_ENABLED)
void config_pll2(void);
#endif
void config_enable_default_clocks(void);

/* function exported to the soc power.c */
int stm32_clock_control_init(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_CLOCK_CONTROL_STM32_LL_CLOCK_H_ */
