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
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>

#include <stm32_ll_utils.h>

/* Macros to fill up multiplication and division factors values */
#define pllm(v) CONCAT(LL_RCC_PLLM_DIV_, v)
#define pllp(v) CONCAT(LL_RCC_PLLP_DIV_, v)
#define pllq(v) CONCAT(LL_RCC_PLLQ_DIV_, v)
#define pllr(v) CONCAT(LL_RCC_PLLR_DIV_, v)
#define plldivr(v) CONCAT(LL_RCC_PLLDIVR_DIV_, v)

#if defined(RCC_PLLI2SCFGR_PLLI2SM)
/* Some stm32F4 devices have a dedicated PLL I2S with M divider */
#define plli2sm(v) CONCAT(LL_RCC_PLLI2SM_DIV_, v)
#else
/* Some stm32F4 devices (typ. stm32F401) have a dedicated PLL I2S with PLL M divider */
#define plli2sm(v) CONCAT(LL_RCC_PLLM_DIV_, v)
#endif /* RCC_PLLI2SCFGR_PLLI2SM */
#define plli2sp(v) CONCAT(LL_RCC_PLLI2SP_DIV_, v)
#define plli2sq(v) CONCAT(LL_RCC_PLLI2SQ_DIV_, v)
#define plli2sdivq(v) CONCAT(LL_RCC_PLLI2SDIVQ_DIV_, v)
#define plli2sr(v) CONCAT(LL_RCC_PLLI2SR_DIV_, v)
#define plli2sdivr(v) CONCAT(LL_RCC_PLLI2SDIVR_DIV_, v)

#define pllsaim(v) CONCAT(LL_RCC_PLLM_DIV_, v)
#define pllsaip(v) CONCAT(LL_RCC_PLLSAIP_DIV_, v)
#define pllsaiq(v) CONCAT(LL_RCC_PLLSAIQ_DIV_, v)
#define pllsaidivq(v) CONCAT(LL_RCC_PLLSAIDIVQ_DIV_, v)
#define pllsair(v) CONCAT(LL_RCC_PLLSAIR_DIV_, v)
#define pllsaidivr(v) CONCAT(LL_RCC_PLLSAIDIVR_DIV_, v)

#define pllsai1p(v) CONCAT(LL_RCC_PLLSAI1P_DIV_, v)
#define pllsai1q(v) CONCAT(LL_RCC_PLLSAI1Q_DIV_, v)
#define pllsai1r(v) CONCAT(LL_RCC_PLLSAI1R_DIV_, v)

#if defined(RCC_PLLSAI2M_DIV_1_16_SUPPORT)
#define pllsai2m(v) CONCAT(LL_RCC_PLLSAI2M_DIV_, v)
#else
#define pllsai2m(v) CONCAT(LL_RCC_PLLM_DIV_, v)
#endif
#define pllsai2p(v) CONCAT(LL_RCC_PLLSAI2P_DIV_, v)
#define pllsai2q(v) CONCAT(LL_RCC_PLLSAI2Q_DIV_, v)
#define pllsai2r(v) CONCAT(LL_RCC_PLLSAI2R_DIV_, v)
#define pllsai2divr(v) CONCAT(LL_RCC_PLLSAI2DIVR_DIV_, v)

#if IS_ENABLED(CONFIG_CLOCK_STM32_AT32F435_PLL_FR_ENCODING)
/* AT32F435 PLL_FR (post-divider) field: bits 18:16 */
#define AT32F435_CRM_PLLCFG_PLL_FR_MASK		GENMASK(18, 16)
#define AT32F435_CRM_PLLCFG_PLL_FR_POS		16

/**
 * @brief Convert PLL post-divider value to AT32F435 PLL_FR encoding
 * 
 * AT32F435 uses 3-bit PLL_FR field (bits 18:16) with direct divisor encoding:
 * - 010 (0b010) = divide by 4
 * - 011 (0b011) = divide by 8
 * - 100 (0b100) = divide by 16
 * - 101 (0b101) = divide by 32
 * 
 * @param divisor PLL post-divider value (4, 8, 16, or 32)
 * @return Encoded value for PLL_FR field (bits 18:16)
 */
static inline uint32_t at32f435_pll_fr(uint32_t divisor)
{
	uint32_t encoding;

	switch (divisor) {
	case 4:
		encoding = 0b010;
		break;
	case 8:
		encoding = 0b011;
		break;
	case 16:
		encoding = 0b100;
		break;
	case 32:
		encoding = 0b101;
		break;
	default:
		__ASSERT(0, "Invalid AT32F435 PLL post-divider: %u (must be 4, 8, 16, or 32)",
			 divisor);
		encoding = 0b010; /* Default to /4 */
		break;
	}

	return encoding << AT32F435_CRM_PLLCFG_PLL_FR_POS;
}
#endif /* CONFIG_CLOCK_STM32_AT32F435_PLL_FR_ENCODING */

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
#if defined(STM32_PLLI2S_ENABLED)
uint32_t get_plli2ssrc_frequency(void);
void config_plli2s(void);
#endif
#if defined(STM32_PLLSAI_ENABLED)
uint32_t get_pllsaisrc_frequency(void);
void config_pllsai(void);
#endif
#if defined(STM32_PLLSAI1_ENABLED)
uint32_t get_pllsai1src_frequency(void);
void config_pllsai1(void);
#endif
#if defined(STM32_PLLSAI2_ENABLED)
uint32_t get_pllsai2src_frequency(void);
void config_pllsai2(void);
#endif
void config_enable_default_clocks(void);
void config_regulator_voltage(uint32_t hclk_freq);
int enabled_clock(uint32_t src_clk);

#if defined(STM32_CK48_ENABLED)
uint32_t get_ck48_frequency(void);
#endif

/* functions exported to the soc power.c */
int stm32_clock_control_init(const struct device *dev);
void stm32_clock_control_standby_exit(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_CLOCK_CONTROL_STM32_LL_CLOCK_H_ */
