/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOF_DAI_DRIVER_MN_H__
#define __SOF_DAI_DRIVER_MN_H__

#include <stdbool.h>
#include <stdint.h>

#define MN_BASE			0x00078C00

/** \brief Offset of MCLK Divider Control Register. */
#define MN_MDIVCTRL 0x0

/** \brief Enables the output of MCLK Divider. */
#define MN_MDIVCTRL_M_DIV_ENABLE(x) BIT(x)

/** \brief Offset of MCLK Divider x Ratio Register. */
#define MN_MDIVR(x) (0x80 + (x) * 0x4)

/** \brief Bits for setting MCLK source clock. */
#define MCDSS(x)	SET_BITS(17, 16, x)

/** \brief Offset of BCLK x M/N Divider M Value Register. */
#define MN_MDIV_M_VAL(x) (0x100 + (x) * 0x8 + 0x0)

/** \brief Offset of BCLK x M/N Divider N Value Register. */
#define MN_MDIV_N_VAL(x) (0x100 + (x) * 0x8 + 0x4)

/** \brief Bits for setting M/N source clock. */
#define MNDSS(x)	SET_BITS(21, 20, x)

/** \brief Mask for clearing mclk and bclk source in MN_MDIVCTRL */
#define MN_SOURCE_CLKS_MASK 0x3

/**
 * \brief Initializes MN driver.
 */
void z_mn_init();

/**
 * \brief Finds and sets valid combination of MCLK source and divider to
 *	  achieve requested MCLK rate.
 *	  User should release clock when it is no longer needed to allow
 *	  driver to change MCLK M/N source when user count drops to 0.
 *	  M value of M/N is not supported for MCLK, only divider can be used.
 * \param[in] mclk_id id of main clock for which rate should be set.
 * \param[in] mclk_rate main clock frequency.
 * \return 0 on success otherwise a negative error code.
 */
int z_mn_set_mclk(uint16_t mclk_id, uint32_t mclk_rate);

/**
 * \brief set mclk according to blob setting
 * \param[in] mdivc mclk control setting.
 * \param[in] mdivr mclk divider setting.
 * \return 0 on success otherwise a negative error code.
 */
int z_mn_set_mclk_blob(uint32_t mdivc, uint32_t mdivr);

/**
 * \brief Release previously requested MCLK for given MCLK ID.
 * \param[in] mclk_id id of main clock.
 */
void z_mn_release_mclk(uint32_t mclk_id);

#if CONFIG_INTEL_MN
/**
 * \brief Finds and sets valid combination of BCLK source and M/N to
 *	  achieve requested BCLK rate.
 *	  User should release clock when it is no longer needed to allow
 *	  driver to change M/N source when user count drops to 0.
 * \param[in] dai_index DAI index (SSP port).
 * \param[in] bclk_rate Bit clock frequency.
 * \param[out] out_scr_div SCR divisor that should be set by caller to achieve
 *			   requested BCLK rate.
 * \param[out] out_need_ecs If set to true, the caller should configure ECS.
 * \return 0 on success otherwise a negative error code.
 * \see mn_release_bclk()
 */
int z_mn_set_bclk(uint32_t dai_index, uint32_t bclk_rate,
		  uint32_t *out_scr_div, bool *out_need_ecs);

/**
 * \brief Release previously requested BCLK for given DAI.
 * \param[in] dai_index DAI index (SSP port).
 */
void z_mn_release_bclk(uint32_t dai_index);

/**
 * \brief Resets M & N values of M/N divider for given DAI index.
 * \param[in] dai_index DAI index (SSP port).
 */
void z_mn_reset_bclk_divider(uint32_t dai_index);
#endif

static inline uint32_t mn_reg_read(uint32_t reg, uint32_t id)
{
	return *((volatile uint32_t*)(MN_BASE + reg));
}

static inline void mn_reg_write(uint32_t reg, uint32_t id, uint32_t val)
{
	*((volatile uint32_t*)(MN_BASE + reg)) = val;
}

#endif /* __SOF_DRIVERS_MN_H__ */
