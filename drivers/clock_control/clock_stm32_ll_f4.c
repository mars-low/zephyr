/*
 *
 * Copyright (c) 2023 Marek Słowiński
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_utils.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include "stm32_hsem.h"

/* Macros to fill up prescaler values */
#define z_ahb_prescaler(v) LL_RCC_SYSCLK_DIV_ ## v
#define ahb_prescaler(v) z_ahb_prescaler(v)

#define z_apb1_prescaler(v) LL_RCC_APB1_DIV_ ## v
#define apb1_prescaler(v) z_apb1_prescaler(v)

#define z_apb2_prescaler(v) LL_RCC_APB2_DIV_ ## v
#define apb2_prescaler(v) z_apb2_prescaler(v)

/* Macros to fill up multiplication and division factors values */
#define z_pllm(v) LL_RCC_PLLM_DIV_ ## v
#define pllm(v) z_pllm(v)

#define z_pllp(v) LL_RCC_PLLP_DIV_ ## v
#define pllp(v) z_pllp(v)

#define z_pllq(v) LL_RCC_PLLQ_DIV_ ## v
#define pllq(v) z_pllq(v)

#define z_pllr(v) LL_RCC_PLLR_DIV_ ## v
#define pllr(v) z_pllr(v)

/* Macros to check for clock feasibility */

/* Choose PLL SRC */
#if defined(STM32_PLL_SRC_HSI)
#define PLLSRC_FREQ  ((STM32_HSI_FREQ)/(STM32_HSI_DIVISOR))
#elif defined(STM32_PLL_SRC_HSE)
#define PLLSRC_FREQ  STM32_HSE_FREQ
#else
#define PLLSRC_FREQ 0
#endif

/* Given source clock and dividers, computed the output frequency of PLLP */
#define PLLP_FREQ(pllsrc_freq, divm, divn, divp)	(((pllsrc_freq)*\
							(divn))/((divm)*(divp)))

/* PLL P output frequency value */
#define PLLP_VALUE	PLLP_FREQ(\
				PLLSRC_FREQ,\
				STM32_PLL_M_DIVISOR,\
				STM32_PLL_N_MULTIPLIER,\
				STM32_PLL_P_DIVISOR)

#if defined(STM32_SYSCLK_SRC_PLL)
#define SYSCLKSRC_FREQ	PLLP_VALUE
#elif defined(STM32_SYSCLK_SRC_HSI)
#define SYSCLKSRC_FREQ	((STM32_HSI_FREQ)/(STM32_HSI_DIVISOR))
#elif defined(STM32_SYSCLK_SRC_HSE)
#define SYSCLKSRC_FREQ	STM32_HSE_FREQ
#endif

/* ARM Sys CPU Clock */
#define SYSCLK_FREQ	SYSCLKSRC_FREQ
#define AHB_FREQ	((SYSCLK_FREQ)/(STM32_AHB_PRESCALER))
#define APB1_FREQ	((AHB_FREQ)/(STM32_APB1_PRESCALER))
#define APB2_FREQ	((AHB_FREQ)/(STM32_APB2_PRESCALER))

/* end of clock feasability check */


__unused
static uint32_t get_pllout_frequency(uint32_t pllsrc_freq,
				     int pllm_div,
				     int plln_mul,
				     int pllout_div)
{
	__ASSERT_NO_MSG(pllm_div && pllout_div);

	return (pllsrc_freq / pllm_div) * plln_mul / pllout_div;
}

static uint32_t get_bus_clock(uint32_t clock, uint32_t prescaler)
{
	return clock / prescaler;
}

#if defined(STM32_PLL_ENABLED)
__unused
static uint32_t get_pll_source(void)
{
	if (IS_ENABLED(STM32_PLL_SRC_HSI)) {
		return LL_RCC_PLLSOURCE_HSI;
	} else if (IS_ENABLED(STM32_PLL_SRC_HSE)) {
		return LL_RCC_PLLSOURCE_HSE;
	}

	__ASSERT(0, "Invalid source");
	return 0;
}

__unused
uint32_t get_pllsrc_frequency(void)
{
	if (IS_ENABLED(STM32_PLL_SRC_HSI)) {
		return STM32_HSI_FREQ;
	} else if (IS_ENABLED(STM32_PLL_SRC_HSE)) {
		return STM32_HSE_FREQ;
	}

	__ASSERT(0, "Invalid source");
	return 0;
}

/**
 * @brief Set up pll configuration
 */
__unused
void config_pll_sysclock(void)
{
	LL_RCC_PLL_ConfigDomain_SYS(get_pll_source(),
				    pllm(STM32_PLL_M_DIVISOR),
				    STM32_PLL_N_MULTIPLIER,
				    pllp(STM32_PLL_P_DIVISOR));
}

#endif /* defined(STM32_PLL_ENABLED) */

/**
 * @brief Activate default clocks
 */
void config_enable_default_clocks(void)
{
	/* Power Interface clock enabled by default */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
}

__unused
static uint32_t get_hclk_frequency(void)
{
	uint32_t sysclk = 0;

	/* Get the current system clock source */
	switch (LL_RCC_GetSysClkSource()) {
	case LL_RCC_SYS_CLKSOURCE_STATUS_HSI:
		sysclk = STM32_HSI_FREQ/STM32_HSI_DIVISOR;
		break;
	case LL_RCC_SYS_CLKSOURCE_STATUS_HSE:
		sysclk = STM32_HSE_FREQ;
		break;
#if defined(STM32_PLL_ENABLED)
	case LL_RCC_SYS_CLKSOURCE_STATUS_PLL:
		sysclk = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLL_M_DIVISOR,
					      STM32_PLL_N_MULTIPLIER,
					      STM32_PLL_P_DIVISOR);
		break;
#endif /* STM32_PLL_ENABLED */
	}

	return get_bus_clock(sysclk, STM32_AHB_PRESCALER);
}

/** @brief Verifies clock is part of active clock configuration */
__unused
static int enabled_clock(uint32_t src_clk)
{

	if ((src_clk == STM32_SRC_SYSCLK) ||
	    ((src_clk == STM32_SRC_HSE) && IS_ENABLED(STM32_HSE_ENABLED)) ||
	    ((src_clk == STM32_SRC_HSI) && IS_ENABLED(STM32_HSI_ENABLED)) ||
	    ((src_clk == STM32_SRC_LSE) && IS_ENABLED(STM32_LSE_ENABLED)) ||
	    ((src_clk == STM32_SRC_LSI) && IS_ENABLED(STM32_LSI_ENABLED)) ||
	    ((src_clk == STM32_SRC_PLL_P) && IS_ENABLED(STM32_PLL_P_ENABLED)) ||
	    ((src_clk == STM32_SRC_PLL_Q) && IS_ENABLED(STM32_PLL_Q_ENABLED)) ||
	    ((src_clk == STM32_SRC_PLL_R) && IS_ENABLED(STM32_PLL_R_ENABLED)) ||
	    ((src_clk == STM32_SRC_PLLI2S_P) && IS_ENABLED(STM32_PLLI2S_P_ENABLED)) ||
	    ((src_clk == STM32_SRC_PLLI2S_Q) && IS_ENABLED(STM32_PLLI2S_Q_ENABLED)) ||
	    ((src_clk == STM32_SRC_PLLI2S_R) && IS_ENABLED(STM32_PLLI2S_R_ENABLED)) ||
	    ((src_clk == STM32_SRC_PLLSAI_P) && IS_ENABLED(STM32_PLLSAI_P_ENABLED)) ||
	    ((src_clk == STM32_SRC_PLLSAI_Q) && IS_ENABLED(STM32_PLLSAI_Q_ENABLED)) ||
	    ((src_clk == STM32_SRC_PLLSAI_R) && IS_ENABLED(STM32_PLLSAI_R_ENABLED))) {
		return 0;
	}

	return -ENOTSUP;
}

static inline int stm32_clock_control_on(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);

	ARG_UNUSED(dev);

	if (IN_RANGE(pclken->bus, STM32_PERIPH_BUS_MIN, STM32_PERIPH_BUS_MAX) == 0) {
		/* Attemp to toggle a wrong periph clock bit */
		return -ENOTSUP;
	}

	sys_set_bits(DT_REG_ADDR(DT_NODELABEL(rcc)) + pclken->bus, pclken->enr);

	return 0;
}

static inline int stm32_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);

	ARG_UNUSED(dev);

	if (IN_RANGE(pclken->bus, STM32_PERIPH_BUS_MIN, STM32_PERIPH_BUS_MAX) == 0) {
		/* Attemp to toggle a wrong periph clock bit */
		return -ENOTSUP;
	}

	sys_clear_bits(DT_REG_ADDR(DT_NODELABEL(rcc)) + pclken->bus, pclken->enr);

	return 0;
}

static inline int stm32_clock_control_configure(const struct device *dev,
						clock_control_subsys_t sub_system,
						void *data)
{
#if defined(STM32_SRC_SYSCLK)
	/* At least one alt src clock available */
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	int err;

	ARG_UNUSED(dev);
	ARG_UNUSED(data);

	err = enabled_clock(pclken->bus);
	if (err < 0) {
		/* Attempt to configure a src clock not available or not valid */
		return err;
	}

	if (pclken->enr == NO_SEL) {
		/* Domain clock is fixed. Nothing to set. Exit */
		return 0;
	}

	sys_set_bits(DT_REG_ADDR(DT_NODELABEL(rcc)) + STM32_CLOCK_REG_GET(pclken->enr),
		     STM32_CLOCK_VAL_GET(pclken->enr) << STM32_CLOCK_SHIFT_GET(pclken->enr));

	return 0;
#else
	/* No src clock available: Not supported */
	return -ENOTSUP;
#endif
}

static int stm32_clock_control_get_subsys_rate(const struct device *clock,
					       clock_control_subsys_t sub_system,
					       uint32_t *rate)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	/*
	 * Get AHB Clock (= SystemCoreClock = SYSCLK/prescaler)
	 * SystemCoreClock is preferred to CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
	 * since it will be updated after clock configuration and hence
	 * more likely to contain actual clock speed
	 */
	uint32_t ahb_clock = get_bus_clock(SystemCoreClock, STM32_AHB_PRESCALER);
	uint32_t apb1_clock = get_bus_clock(ahb_clock, STM32_APB1_PRESCALER);
#if DT_NODE_HAS_PROP(DT_NODELABEL(rcc), apb2_prescaler)
	uint32_t apb2_clock = get_bus_clock(ahb_clock, STM32_APB2_PRESCALER);
#elif defined(STM32_CLOCK_BUS_APB2)
	/* APB2 bus exists, but w/o dedicated prescaler */
	uint32_t apb2_clock = apb1_clock;
#endif
	ARG_UNUSED(clock);

	switch (pclken->bus) {
	case STM32_CLOCK_BUS_AHB1:
	case STM32_CLOCK_BUS_AHB2:
	case STM32_CLOCK_BUS_AHB3:
		*rate = ahb_clock;
		break;
	case STM32_CLOCK_BUS_APB1:
		*rate = apb1_clock;
		break;
	case STM32_CLOCK_BUS_APB2:
		*rate = apb2_clock;
		break;
	case STM32_SRC_SYSCLK:
		*rate = get_hclk_frequency();
		break;
#if defined(STM32_HSE_ENABLED)
	case STM32_SRC_HSE:
		*rate = STM32_HSE_FREQ;
		break;
#endif /* STM32_HSE_ENABLED */
#if defined(STM32_LSE_ENABLED)
	case STM32_SRC_LSE:
		*rate = STM32_LSE_FREQ;
		break;
#endif /* STM32_LSE_ENABLED */
#if defined(STM32_LSI_ENABLED)
	case STM32_SRC_LSI:
		*rate = STM32_LSI_FREQ;
		break;
#endif /* STM32_LSI_ENABLED */
#if defined(STM32_HSI_ENABLED)
	case STM32_SRC_HSI:
		*rate = STM32_HSI_FREQ;
		break;
#endif /* STM32_HSI_ENABLED */
#if defined(STM32_PLL_ENABLED)
	case STM32_SRC_PLL_P:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLL_M_DIVISOR,
					      STM32_PLL_N_MULTIPLIER,
					      STM32_PLL_P_DIVISOR);
		break;
	case STM32_SRC_PLL_Q:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLL_M_DIVISOR,
					      STM32_PLL_N_MULTIPLIER,
					      STM32_PLL_Q_DIVISOR);
		break;
	case STM32_SRC_PLL_R:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLL_M_DIVISOR,
					      STM32_PLL_N_MULTIPLIER,
					      STM32_PLL_R_DIVISOR);
		break;
#endif /* STM32_PLL_ENABLED */
#if defined(STM32_PLLI2S_ENABLED)
	case STM32_SRC_PLLI2S_P:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLLI2S_M_DIVISOR,
					      STM32_PLLI2S_N_MULTIPLIER,
					      STM32_PLLI2S_P_DIVISOR);
		break;
	case STM32_SRC_PLLI2S_Q:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLLI2S_M_DIVISOR,
					      STM32_PLLI2S_N_MULTIPLIER,
					      STM32_PLLI2S_Q_DIVISOR);
		break;
	case STM32_SRC_PLLI2S_R:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLLI2S_M_DIVISOR,
					      STM32_PLLI2S_N_MULTIPLIER,
					      STM32_PLLI2S_R_DIVISOR);
		break;
#endif /* STM32_PLLI2S_ENABLED */
#if defined(STM32_PLLSAI_ENABLED)
	case STM32_SRC_PLLSAI_P:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLLSAI_M_DIVISOR,
					      STM32_PLLSAI_N_MULTIPLIER,
					      STM32_PLLSAI_P_DIVISOR);
		break;
	case STM32_SRC_PLLSAI_Q:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLLSAI_M_DIVISOR,
					      STM32_PLLSAI_N_MULTIPLIER,
					      STM32_PLLSAI_Q_DIVISOR);
		break;
	case STM32_SRC_PLLSAI_R:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLLSAI_M_DIVISOR,
					      STM32_PLLSAI_N_MULTIPLIER,
					      STM32_PLLSAI_R_DIVISOR);
		break;
	case STM32_SRC_PLLSAI_DIVR:
		*rate = get_pllout_frequency(get_pllsrc_frequency(),
					      STM32_PLLSAI_M_DIVISOR,
					      STM32_PLLSAI_N_MULTIPLIER,
					      STM32_PLLSAI_R_DIVISOR) / LL_RCC_PLLSAIDIVR_DIV_4;
		break;
#endif /* STM32_PLLSAI_ENABLED */
	default:
		return -ENOTSUP;
	}

	return 0;
}

static struct clock_control_driver_api stm32_clock_control_api = {
	.on = stm32_clock_control_on,
	.off = stm32_clock_control_off,
	.get_rate = stm32_clock_control_get_subsys_rate,
	.configure = stm32_clock_control_configure,
};

__unused
static void set_up_fixed_clock_sources(void)
{
	if (IS_ENABLED(STM32_HSE_ENABLED)) {
		/* Enable HSE oscillator */
		if (IS_ENABLED(STM32_HSE_BYPASS)) {
			LL_RCC_HSE_EnableBypass();
		} else {
			LL_RCC_HSE_DisableBypass();
		}

		LL_RCC_HSE_Enable();
		while (LL_RCC_HSE_IsReady() != 1) {
		}
	}

	if (IS_ENABLED(STM32_HSI_ENABLED)) {
		/* Enable HSI if not enabled */
		if (LL_RCC_HSI_IsReady() != 1) {
			/* Enable HSI */
			LL_RCC_HSI_Enable();
			while (LL_RCC_HSI_IsReady() != 1) {
			/* Wait for HSI ready */
			}
		}
	}

	if (IS_ENABLED(STM32_LSI_ENABLED)) {
		/* Enable LSI oscillator */
		LL_RCC_LSI_Enable();
		while (LL_RCC_LSI_IsReady() != 1) {
		}
	}

	if (IS_ENABLED(STM32_LSE_ENABLED)) {
		/* Enable backup domain */
		LL_PWR_EnableBkUpAccess();

		/* Configure driving capability */
#if STM32_LSE_DRIVING
		/* Configure driving capability */
		LL_RCC_LSE_EnableHighDriveMode();
#endif

		if (IS_ENABLED(STM32_LSE_BYPASS)) {
			/* Configure LSE bypass */
			LL_RCC_LSE_EnableBypass();
		}

		/* Enable LSE oscillator */
		LL_RCC_LSE_Enable();
		while (LL_RCC_LSE_IsReady() != 1) {
		}
	}

	if (IS_ENABLED(STM32_HSI_ENABLED)) {
		LL_RCC_HSI_Enable();
		while (LL_RCC_HSI_IsReady() != 1) {
		}
	}
}


/*
 * Unconditionally switch the system clock source to HSI.
 */
__unused
static void stm32_clock_switch_to_hsi(void)
{
	/* Enable HSI if not enabled */
	if (LL_RCC_HSI_IsReady() != 1) {
		/* Enable HSI */
		LL_RCC_HSI_Enable();
		while (LL_RCC_HSI_IsReady() != 1) {
		/* Wait for HSI ready */
		}
	}

	/* Set HSI as SYSCLCK source */
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
	}
}

__unused
static int set_up_plls(void)
{
#if defined(STM32_PLL_ENABLED)
	/*
	 * Case of chain-loaded applications:
	 * Switch to HSI and disable the PLL before configuration.
	 * (Switching to HSI makes sure we have a SYSCLK source in
	 * case we're currently running from the PLL we're about to
	 * turn off and reconfigure.)
	 *
	 */
	if (LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
		LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
		stm32_clock_switch_to_hsi();
	}
	LL_RCC_PLL_Disable();

#endif

#if defined(STM32_PLL_ENABLED) || defined(STM32_PLLI2S_ENABLED) || defined(STM32_PLLSAI_ENABLED)
	/* Configure PLL source */

	/* Can be HSE or HSI*/
	LL_RCC_PLL_SetMainSource(get_pll_source());

#if defined(STM32_PLLI2S_ENABLED)
	if (IS_ENABLED(STM32_PLLI2S_P_ENABLED)) {
		// Check if PLLI2SP is supported / enabled before configuring domain
	}

	if (IS_ENABLED(STM32_PLLI2S_Q_ENABLED)) {
		LL_RCC_PLLI2S_ConfigDomain_SAI(get_pll_source(),
					       STM32_PLLI2S_M_DIVISOR,
					       STM32_PLLI2S_N_MULTIPLIER,
					       STM32_PLLI2S_Q_DIVISOR,
						   LL_RCC_PLLI2SDIVQ_DIV_1);
	}

	if (IS_ENABLED(STM32_PLLI2S_R_ENABLED)) {
		LL_RCC_PLLI2S_ConfigDomain_I2S(get_pll_source(),
					       STM32_PLLI2S_M_DIVISOR,
					       STM32_PLLI2S_N_MULTIPLIER,
					       STM32_PLLI2S_R_DIVISOR);
	}

	LL_RCC_PLLI2S_Enable();
	while (LL_RCC_PLLI2S_IsReady() != 1U) {
	}

#endif /* STM32_PLLI2S_ENABLED */

#if defined(STM32_PLLSAI_ENABLED)
	if (IS_ENABLED(STM32_PLLSAI_P_ENABLED)) {
		LL_RCC_PLLSAI_ConfigDomain_48M(get_pll_source(),
					       STM32_PLLSAI_M_DIVISOR,
					       STM32_PLLSAI_N_MULTIPLIER,
					       STM32_PLLSAI_P_DIVISOR);
	}

	if (IS_ENABLED(STM32_PLLSAI_Q_ENABLED)) {
		LL_RCC_PLLSAI_ConfigDomain_SAI(get_pll_source(),
					       STM32_PLLSAI_M_DIVISOR,
					       STM32_PLLSAI_N_MULTIPLIER,
					       STM32_PLLSAI_Q_DIVISOR,
						   LL_RCC_PLLSAIDIVQ_DIV_1);
	}

	if (IS_ENABLED(STM32_PLLSAI_R_ENABLED)) {
		LL_RCC_PLLSAI_ConfigDomain_LTDC(get_pll_source(),
					       STM32_PLLSAI_M_DIVISOR,
					       STM32_PLLSAI_N_MULTIPLIER,
					       STM32_PLLSAI_R_DIVISOR,
						   LL_RCC_PLLSAIDIVR_DIV_4);
	}

	LL_RCC_PLLSAI_Enable();
	while (LL_RCC_PLLSAI_IsReady() != 1U) {
	}

#endif /* STM32_PLLSAI_ENABLED */

#if defined(STM32_PLL_ENABLED)
	if (IS_ENABLED(STM32_PLL_P_ENABLED)) {
		config_pll_sysclock();
	}

	if (IS_ENABLED(STM32_PLL_Q_ENABLED)) {
		LL_RCC_PLL_ConfigDomain_48M(get_pll_source(),
				    pllm(STM32_PLL_M_DIVISOR),
				    STM32_PLL_N_MULTIPLIER,
				    pllq(STM32_PLL_Q_DIVISOR));
	}

	if (IS_ENABLED(STM32_PLL_R_ENABLED)) {
		// Check if PLLR is supported / enabled before configuring domain
		// LL_RCC_PLL_ConfigDomain_DSI(get_pll_source(),
		// 		    STM32_PLL_M_DIVISOR,
		// 		    STM32_PLL_N_MULTIPLIER,
		// 		    STM32_PLL_R_DIVISOR);
	}

	LL_RCC_PLL_Enable();
	while (LL_RCC_PLL_IsReady() != 1U) {
	}

#endif /* STM32_PLL_ENABLED */

#else
	/* Init PLL source to HSI */
	LL_RCC_PLL_SetMainSource(LL_RCC_PLLSOURCE_HSI);

#endif /* STM32_PLL_ENABLED || STM32_PLLI2S_ENABLED || STM32_PLLSAI_ENABLED */

	return 0;
}

static int stm32_clock_control_init(const struct device *dev)
{
	uint32_t old_hclk_freq = 0;
	uint32_t new_hclk_freq = 0;
	int r;

	ARG_UNUSED(dev);

	/* Some clocks would be activated by default */
	config_enable_default_clocks();

	z_stm32_hsem_lock(CFG_HW_RCC_SEMID, HSEM_LOCK_DEFAULT_RETRY);

	/* Set up indiviual enabled clocks */
	set_up_fixed_clock_sources();

	/* Set up PLLs */
	r = set_up_plls();
	if (r < 0) {
		return r;
	}

	/* Current hclk value */
	old_hclk_freq = get_hclk_frequency();
	/* AHB is HCLK clock to configure */
	new_hclk_freq = get_bus_clock(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
				      STM32_AHB_PRESCALER);

	/* Set flash latency */
	/* AHB/AXI/HCLK clock is SYSCLK / HPRE */
	/* If freq increases, set flash latency before any clock setting */
	if (new_hclk_freq > old_hclk_freq) {
		LL_SetFlashLatency(new_hclk_freq);
	}

	/* Preset the prescalers prior to chosing SYSCLK */
	/* Prevents APB clock to go over limits */
	/* Set buses (AHB, APB1, APB2) prescalers */
	LL_RCC_SetAHBPrescaler(ahb_prescaler(STM32_AHB_PRESCALER));
	LL_RCC_SetAPB1Prescaler(apb1_prescaler(STM32_APB1_PRESCALER));
	LL_RCC_SetAPB2Prescaler(apb2_prescaler(STM32_APB2_PRESCALER));

	/* Set up sys clock */
	if (IS_ENABLED(STM32_SYSCLK_SRC_PLL)) {
		/* Set PLL as System Clock Source */
		LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
		while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
		}
	} else if (IS_ENABLED(STM32_SYSCLK_SRC_HSE)) {
		/* Set sysclk source to HSE */
		LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
		while (LL_RCC_GetSysClkSource() !=
					LL_RCC_SYS_CLKSOURCE_STATUS_HSE) {
		}
	} else if (IS_ENABLED(STM32_SYSCLK_SRC_HSI)) {
		/* Set sysclk source to HSI */
		LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
		while (LL_RCC_GetSysClkSource() !=
					LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
		}
	} else {
		return -ENOTSUP;
	}

	/* Set FLASH latency */
	/* AHB/AXI/HCLK clock is SYSCLK / HPRE */
	/* If freq not increased, set flash latency after all clock setting */
	if (new_hclk_freq <= old_hclk_freq) {
		LL_SetFlashLatency(new_hclk_freq);
	}

	z_stm32_hsem_unlock(CFG_HW_RCC_SEMID);

	/* Update CMSIS variable */
	SystemCoreClock = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;

	return r;
}

/**
 * @brief RCC device, note that priority is intentionally set to 1 so
 * that the device init runs just after SOC init
 */
DEVICE_DT_DEFINE(DT_NODELABEL(rcc),
		    &stm32_clock_control_init,
		    NULL,
		    NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &stm32_clock_control_api);
