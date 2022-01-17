/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <spinlock.h>
#include <devicetree.h>
#define LOG_DOMAIN sof_dai_ssp
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_DOMAIN);

#include "ssp.h"
#include "dai-params-intel.h"
#include "dai-params-intel-ipc4.h"

#define DT_DRV_COMPAT intel_ssp_dai

#define dai_set_drvdata(dai, data) \
        (dai->priv_data = data)

#define dai_get_drvdata(dai) \
        dai->priv_data

#define dai_get_mn(dai) \
        dai->plat_data.mn_inst

#define dai_get_ftable(dai) \
        dai->plat_data.ftable

#define dai_get_fsources(dai) \
        dai->plat_data.fsources

#define dai_mn_base(dai) \
        dai->plat_data.mn_inst->base

#define dai_base(dai) \
        dai->plat_data.base

/* stream PCM frame format */
enum sof_ipc_frame {
	SOF_IPC_FRAME_S16_LE = 0,
	SOF_IPC_FRAME_S24_4LE,
	SOF_IPC_FRAME_S32_LE,
	SOF_IPC_FRAME_FLOAT,
	/* other formats here */
	SOF_IPC_FRAME_S24_3LE,
};

/* stream direction */
enum sof_ipc_stream_direction {
	SOF_IPC_STREAM_PLAYBACK = 0,
	SOF_IPC_STREAM_CAPTURE,
};

struct ssp_pdata {
	uint32_t sscr0;
	uint32_t sscr1;
	uint32_t psp;
	uint32_t state[2];		/* SSP_STATE_ for each direction */
	uint32_t clk_active;
	struct dai_config config;
	struct dai_properties props;
	struct sof_dai_ssp_params params;
};

static uint32_t io_reg_read(uint32_t reg)
{
	return *((volatile uint32_t*)reg);
}

static void io_reg_write(uint32_t reg, uint32_t val)
{
	*((volatile uint32_t*)reg) = val;
}

static void io_reg_update_bits(uint32_t reg, uint32_t mask, uint32_t value)
{
	io_reg_write(reg, (io_reg_read(reg) & (~mask)) | (value & mask));
}

static inline void mn_write(struct dai *dai, uint32_t reg, uint32_t val)
{
	io_reg_write(dai_mn_base(dai) + reg, val);
}

static inline uint32_t mn_read(struct dai *dai, uint32_t reg)
{
	return io_reg_read(dai_mn_base(dai) + reg);
}

static void ssp_write(struct dai *dai, uint32_t reg, uint32_t value)
{
	io_reg_write(dai_base(dai) + reg, value);
}

static uint32_t ssp_read(struct dai *dai, uint32_t reg)
{
	return io_reg_read(dai_base(dai) + reg);
}

static void ssp_update_bits(struct dai *dai, uint32_t reg, uint32_t mask,
			    uint32_t value)
{
	LOG_INF("ssp_update_bits base %x, reg %x, mask %x, value %x",
		dai_base(dai), reg, mask, value);
	io_reg_update_bits(dai_base(dai) + reg, mask, value);
}

#if CONFIG_SOC_SERIES_INTEL_CAVS_V15
#define SHIM_BASE		0x00001000
#define SHIM_CLKCTL		0x78
#define SHIM_CLKCTL_I2SFDCGB(x)		BIT(20 + x)
#define SHIM_CLKCTL_I2SEFDCGB(x)	BIT(18 + x)

static inline uint32_t shim_read(uint32_t reg)
{
        return *((volatile uint32_t*)(SHIM_BASE + reg));
}

static inline void shim_write(uint32_t reg, uint32_t val)
{
        *((volatile uint32_t*)(SHIM_BASE + reg)) = val;
}
#endif

static int gcd(int a, int b)
{
	if (a == 0)
		return b;

	if (b == 0)
		return a;

	/* If the numbers are negative, convert them to positive numbers
	 * gcd(a, b) = gcd(-a, -b) = gcd(-a, b) = gcd(a, -b)
	 */

	if (a < 0)
		a = -a;

	if (b < 0)
		b = -b;

	int aux;
	int k;

	/* Find the greatest power of 2 that devides both a and b */
	for (k = 0; ((a | b) & 1) == 0; k++) {
		a >>= 1;
		b >>= 1;
	}

	/* divide by 2 until a becomes odd */
	while ((a & 1) == 0)
		a >>= 1;

	do {
		/*if b is even, remove all factors of 2*/
		while ((b & 1) == 0)
			b >>= 1;

		/* both a and b are odd now. Swap so a <= b
		 * then set b = b - a, which is also even
		 */
		if (a > b) {
			aux = a;
			a = b;
			b = aux;
		}

		b = b - a;

	} while (b != 0);

	/* restore common factors of 2 */
	return a << k;
}

/**
 * \brief Checks if given clock is used as source for any MCLK.
 *
 * \return true if any port use given clock source, false otherwise.
 */
static bool is_mclk_source_in_use(struct dai *dai)
{
	struct mn *mn = dai_get_mn(dai);
	bool ret = false;
	int i;

	for (i = 0; i < ARRAY_SIZE(mn->mclk_sources_ref); i++) {
		if (mn->mclk_sources_ref[i] > 0) {
			ret = true;
			break;
		}
	}

	return ret;
}

/**
 * \brief Configures source clock for MCLK.
 *	  All MCLKs share the same source, so it should be changed
 *	  only if there are no other ports using it already.
 * \param[in] mclk_rate main clock frequency.
 * \return 0 on success, error code otherwise.
 */
static int setup_initial_mclk_source(struct dai *dai, uint32_t mclk_id, uint32_t mclk_rate)
{
	struct freq_table *ft = dai_get_ftable(dai);
	uint32_t *fs = dai_get_fsources(dai);
	struct mn *mn = dai_get_mn(dai);
	int clk_index = -1;
	uint32_t mdivc;
	int ret = 0;
	int i;

	if (mclk_id >= DAI_NUM_SSP_MCLK) {
		LOG_ERR("can't configure MCLK %d, only %d mclk[s] existed!",
		       mclk_id, DAI_NUM_SSP_MCLK);
		ret = -EINVAL;
		goto out;
	}

	/* searching the smallest possible mclk source */
	for (i = 0; i <= MAX_SSP_FREQ_INDEX; i++) {
		if (ft[i].freq % mclk_rate == 0) {
			clk_index = i;
			break;
		}
	}

	if (clk_index < 0) {
		LOG_ERR("MCLK %d, no valid source", mclk_rate);
		ret = -EINVAL;
		goto out;
	}

	mn->mclk_source_clock = clk_index;

	mdivc = mn_read(dai, MN_MDIVCTRL);

	/* enable MCLK divider */
	mdivc |= MN_MDIVCTRL_M_DIV_ENABLE(mclk_id);

	/* clear source mclk clock - bits 17-16 */
	mdivc &= ~MCDSS(MN_SOURCE_CLKS_MASK);

	/* select source clock */
	mdivc |= MCDSS(fs[clk_index]);

	mn_write(dai, MN_MDIVCTRL, mdivc);

	mn->mclk_sources_ref[mclk_id]++;
out:

	return ret;
}

/**
 * \brief Checks if requested MCLK can be achieved with current source.
 * \param[in] mclk_rate main clock frequency.
 * \return 0 on success, error code otherwise.
 */
static int check_current_mclk_source(struct dai *dai, uint16_t mclk_id, uint32_t mclk_rate)
{
	struct freq_table *ft = dai_get_ftable(dai);
	struct mn *mn = dai_get_mn(dai);
	uint32_t mdivc;
	int ret = 0;

        LOG_INF("MCLK %d, source = %d",	mclk_rate, mn->mclk_source_clock);

	if (ft[mn->mclk_source_clock].freq % mclk_rate != 0) {
		LOG_ERR("MCLK %d, no valid configuration for already selected source = %d",
		       mclk_rate, mn->mclk_source_clock);
		ret = -EINVAL;
	}

	/* if the mclk is already used, can't change its divider, just increase ref count */
	if (mn->mclk_sources_ref[mclk_id] > 0) {
		if (mn->mclk_rate[mclk_id] != mclk_rate) {
			LOG_ERR("Can't set MCLK %d to %d, it is already configured to %d",
			       mclk_id, mclk_rate, mn->mclk_rate[mclk_id]);
			return -EINVAL;
		}

		mn->mclk_sources_ref[mclk_id]++;
	} else {
		mdivc = mn_read(dai, MN_MDIVCTRL);

		/* enable MCLK divider */
		mdivc |= MN_MDIVCTRL_M_DIV_ENABLE(mclk_id);
		mn_write(dai, MN_MDIVCTRL, mdivc);

		mn->mclk_sources_ref[mclk_id]++;
	}

	return ret;
}

/**
 * \brief Sets MCLK divider to given value.
 * \param[in] mclk_id ID of MCLK.
 * \param[in] mdivr_val divider value.
 * \return 0 on success, error code otherwise.
 */
static int set_mclk_divider(struct dai *dai, uint16_t mclk_id, uint32_t mdivr_val)
{
	uint32_t mdivr;

	LOG_INF("mclk_id %d mdivr_val %d", mclk_id, mdivr_val);
	switch (mdivr_val) {
	case 1:
		mdivr = 0x00000fff; /* bypass divider for MCLK */
		break;
	case 2 ... 8:
		mdivr = mdivr_val - 2; /* 1/n */
		break;
	default:
		LOG_ERR("invalid mdivr_val %d", mdivr_val);
		return -EINVAL;
	}

	mn_write(dai, MN_MDIVR(mclk_id), mdivr);
	return 0;
}

static int mn_set_mclk(struct dai *dai, uint16_t mclk_id, uint32_t mclk_rate)
{
	struct freq_table *ft = dai_get_ftable(dai);
	struct mn *mn = dai_get_mn(dai);
	k_spinlock_key_t key;
	int ret = 0;

	if (mclk_id >= DAI_NUM_SSP_MCLK) {
		LOG_ERR("mclk ID (%d) >= %d", mclk_id, DAI_NUM_SSP_MCLK);
		return -EINVAL;
	}

	key = k_spin_lock(&mn->lock);

	if (is_mclk_source_in_use(dai))
		ret = check_current_mclk_source(dai, mclk_id, mclk_rate);
	else
		ret = setup_initial_mclk_source(dai, mclk_id, mclk_rate);

	if (ret < 0)
		goto out;

	LOG_INF("mclk_rate %d, mclk_source_clock %d",
		mclk_rate, mn->mclk_source_clock);

	ret = set_mclk_divider(dai, mclk_id,
			       ft[mn->mclk_source_clock].freq /
			       mclk_rate);
	if (!ret)
		mn->mclk_rate[mclk_id] = mclk_rate;

out:

	k_spin_unlock(&mn->lock, key);

	return ret;
}

static int mn_set_mclk_blob(struct dai *dai, uint32_t mdivc, uint32_t mdivr)
{
	mn_write(dai, MN_MDIVCTRL, mdivc);
	mn_write(dai, MN_MDIVR(0), mdivr);

	return 0;
}

static void mn_release_mclk(struct dai *dai, uint32_t mclk_id)
{
	struct mn *mn = dai_get_mn(dai);
	k_spinlock_key_t key;
	uint32_t mdivc;

	key = k_spin_lock(&mn->lock);

	mn->mclk_sources_ref[mclk_id]--;

	/* disable MCLK divider if nobody use it */
	if (!mn->mclk_sources_ref[mclk_id]) {
		mdivc = mn_read(dai, MN_MDIVCTRL);

		mdivc &= ~MN_MDIVCTRL_M_DIV_ENABLE(mclk_id);
		mn_write(dai, MN_MDIVCTRL, mdivc);
	}

	/* release the clock source if all mclks are released */
	if (!is_mclk_source_in_use(dai)) {
		mdivc = mn_read(dai, MN_MDIVCTRL);

		/* clear source mclk clock - bits 17-16 */
		mdivc &= ~MCDSS(MN_SOURCE_CLKS_MASK);

		mn_write(dai, MN_MDIVCTRL, mdivc);

		mn->mclk_source_clock = 0;
	}
	k_spin_unlock(&mn->lock, key);
}

#if CONFIG_INTEL_MN
/**
 * \brief Finds valid M/(N * SCR) values for given frequencies.
 * \param[in] freq SSP clock frequency.
 * \param[in] bclk Bit clock frequency.
 * \param[out] out_scr_div SCR divisor.
 * \param[out] out_m M value of M/N divider.
 * \param[out] out_n N value of M/N divider.
 * \return true if found suitable values, false otherwise.
 */
static bool find_mn(uint32_t freq, uint32_t bclk, uint32_t *out_scr_div, uint32_t *out_m,
		    uint32_t *out_n)
{
	uint32_t m, n, mn_div;
	uint32_t scr_div = freq / bclk;

	LOG_INF("find_mn for freq %d bclk %d", freq, bclk);
	/* check if just SCR is enough */
	if (freq % bclk == 0 && scr_div < (SSCR0_SCR_MASK >> 8) + 1) {
		*out_scr_div = scr_div;
		*out_m = 1;
		*out_n = 1;

		return true;
	}

	/* M/(N * scr_div) has to be less than 1/2 */
	if ((bclk * 2) >= freq)
		return false;

	/* odd SCR gives lower duty cycle */
	if (scr_div > 1 && scr_div % 2 != 0)
		--scr_div;

	/* clamp to valid SCR range */
	scr_div = MIN(scr_div, (SSCR0_SCR_MASK >> 8) + 1);

	/* find highest even divisor */
	while (scr_div > 1 && freq % scr_div != 0)
		scr_div -= 2;

	/* compute M/N with smallest dividend and divisor */
	mn_div = gcd(bclk, freq / scr_div);

	m = bclk / mn_div;
	n = freq / scr_div / mn_div;

	/* M/N values can be up to 24 bits */
	if (n & (~0xffffff))
		return false;

	*out_scr_div = scr_div;
	*out_m = m;
	*out_n = n;

	LOG_INF("find_mn m %d n %d", m, n);
	return true;
}

/**
 * \brief Finds index of clock valid for given BCLK rate.
 *	  Clock that can use just SCR is preferred.
 *	  M/N other than 1/1 is used only if there are no other possibilities.
 * \param[in] bclk Bit clock frequency.
 * \param[out] scr_div SCR divisor.
 * \param[out] m M value of M/N divider.
 * \param[out] n N value of M/N divider.
 * \return index of suitable clock if could find it, -EINVAL otherwise.
 */
static int find_bclk_source(struct dai *dai, uint32_t bclk, uint32_t *scr_div, uint32_t *m,
			    uint32_t *n)
{
	struct freq_table *ft = dai_get_ftable(dai);
	struct mn *mn = dai_get_mn(dai);
	int i;

	/* check if we can use MCLK source clock */
	if (is_mclk_source_in_use(dai)) {
		if (find_mn(ft[mn->mclk_source_clock].freq, bclk, scr_div, m, n))
			return mn->mclk_source_clock;

		LOG_WRN("BCLK %d warning: cannot use MCLK source %d",
			bclk, ft[mn->mclk_source_clock].freq);
	}

	/* searching the smallest possible bclk source */
	for (i = 0; i <= MAX_SSP_FREQ_INDEX; i++)
		if (ft[i].freq % bclk == 0) {
			*scr_div = ft[i].freq / bclk;
			return i;
		}

	/* check if we can get target BCLK with M/N */
	for (i = 0; i <= MAX_SSP_FREQ_INDEX; i++)
		if (find_mn(ft[i].freq, bclk,
			    scr_div, m, n))
			return i;

	return -EINVAL;
}

/**
 * \brief Finds index of SSP clock with the given clock source encoded index.
 * \return the index in ssp_freq if could find it, -EINVAL otherwise.
 */
static int find_clk_ssp_index(struct dai *dai, uint32_t src_enc)
{
	uint32_t *fs = dai_get_fsources(dai);
	int i;

	/* searching for the encode value matched bclk source */
	for (i = 0; i <= MAX_SSP_FREQ_INDEX; i++)
		if (fs[i] == src_enc)
			return i;

	return -EINVAL;
}

/**
 * \brief Checks if given clock is used as source for any BCLK.
 * \param[in] clk_src Bit clock source.
 * \return true if any port use given clock source, false otherwise.
 */
static bool is_bclk_source_in_use(struct dai *dai, enum bclk_source clk_src)
{
	struct mn *mn = dai_get_mn(dai);
	bool ret = false;
	int i;

	for (i = 0; i < ARRAY_SIZE(mn->bclk_sources); i++) {
		if (mn->bclk_sources[i] == clk_src) {
			ret = true;
			break;
		}
	}

	return ret;
}

/**
 * \brief Configures M/N source clock for BCLK.
 *	  All ports that use M/N share the same source, so it should be changed
 *	  only if there are no other ports using M/N already.
 * \param[in] bclk Bit clock frequency.
 * \param[out] scr_div SCR divisor.
 * \param[out] m M value of M/N divider.
 * \param[out] n N value of M/N divider.
 * \return 0 on success, error code otherwise.
 */
static int setup_initial_bclk_mn_source(struct dai *dai, uint32_t bclk, uint32_t *scr_div,
					uint32_t *m, uint32_t *n)
{
	uint32_t *fs = dai_get_fsources(dai);
	struct mn *mn = dai_get_mn(dai);
	uint32_t mdivc;
	int clk_index = find_bclk_source(dai, bclk, scr_div, m, n);

	if (clk_index < 0) {
		LOG_ERR("BCLK %d, no valid source", bclk);
		return -EINVAL;
	}

	mn->bclk_source_mn_clock = clk_index;

	mdivc = mn_read(dai, MN_MDIVCTRL);

	/* clear source bclk clock - 21-20 bits */
	mdivc &= ~MNDSS(MN_SOURCE_CLKS_MASK);

	/* select source clock */
	mdivc |= MNDSS(fs[clk_index]);

	mn_write(dai, MN_MDIVCTRL, mdivc);

	return 0;
}

/**
 * \brief Reset M/N source clock for BCLK.
 *	  If no port is using bclk, reset to use SSP_CLOCK_XTAL_OSCILLATOR
 *	  as the default clock source.
 */
static void reset_bclk_mn_source(struct dai *dai)
{
	struct mn *mn = dai_get_mn(dai);
	uint32_t mdivc;
	int clk_index = find_clk_ssp_index(dai, SSP_CLOCK_XTAL_OSCILLATOR);

	if (clk_index < 0) {
		LOG_ERR("BCLK reset failed, no SSP_CLOCK_XTAL_OSCILLATOR source!");
		return;
	}

	mdivc = mn_read(dai, MN_MDIVCTRL);

	/* reset to use XTAL Oscillator */
	mdivc &= ~MNDSS(MN_SOURCE_CLKS_MASK);
	mdivc |= MNDSS(SSP_CLOCK_XTAL_OSCILLATOR);

	mn_write(dai, MN_MDIVCTRL, mdivc);

	mn->bclk_source_mn_clock = clk_index;
}

/**
 * \brief Finds valid M/(N * SCR) values for source clock that is already locked
 *	  because other ports use it.
 * \param[in] bclk Bit clock frequency.
 * \param[out] scr_div SCR divisor.
 * \param[out] m M value of M/N divider.
 * \param[out] n N value of M/N divider.
 * \return 0 on success, error code otherwise.
 */
static int setup_current_bclk_mn_source(struct dai *dai, uint32_t bclk, uint32_t *scr_div,
					uint32_t *m, uint32_t *n)
{
	struct freq_table *ft = dai_get_ftable(dai);
	struct mn *mn = dai_get_mn(dai);
	int ret = 0;

	/* source for M/N is already set, no need to do it */
	if (find_mn(ft[mn->bclk_source_mn_clock].freq, bclk, scr_div, m, n))
		goto out;

	LOG_ERR("BCLK %d, no valid configuration for already selected source = %d",
	       bclk, mn->bclk_source_mn_clock);
	ret = -EINVAL;

out:

	return ret;
}

static bool check_bclk_xtal_source(uint32_t bclk, bool mn_in_use,
				   uint32_t *scr_div)
{
	/* since cAVS 2.0 bypassing XTAL (ECS=0) is not supported */
	return false;
}
#endif

static int mn_set_bclk(struct dai *dai, uint32_t dai_index, uint32_t bclk_rate,
		       uint32_t *out_scr_div, bool *out_need_ecs)
{
	struct mn *mn = dai_get_mn(dai);
	k_spinlock_key_t key;
	uint32_t m = 1;
	uint32_t n = 1;
	int ret = 0;
	bool mn_in_use;

	key = k_spin_lock(&mn->lock);

	mn->bclk_sources[dai_index] = MN_BCLK_SOURCE_NONE;

	mn_in_use = is_bclk_source_in_use(dai, MN_BCLK_SOURCE_MN);

	if (check_bclk_xtal_source(bclk_rate, mn_in_use, out_scr_div)) {
		mn->bclk_sources[dai_index] = MN_BCLK_SOURCE_XTAL;
		*out_need_ecs = false;
		goto out;
	}

	*out_need_ecs = true;

	if (mn_in_use)
		ret = setup_current_bclk_mn_source(dai, bclk_rate,
						   out_scr_div, &m, &n);
	else
		ret = setup_initial_bclk_mn_source(dai, bclk_rate,
						   out_scr_div, &m, &n);

	if (ret >= 0) {
		mn->bclk_sources[dai_index] = MN_BCLK_SOURCE_MN;

		LOG_INF("bclk_rate %d, *out_scr_div %d, m %d, n %d",
			bclk_rate, *out_scr_div, m, n);

		mn_write(dai, MN_MDIV_M_VAL(dai_index), m);
		mn_write(dai, MN_MDIV_N_VAL(dai_index), n);
	}

out:

	k_spin_unlock(&mn->lock, key);

	return ret;
}

static void mn_release_bclk(struct dai *dai, uint32_t dai_index)
{
	struct mn *mn = dai_get_mn(dai);
	k_spinlock_key_t key;
	bool mn_in_use;

	key = k_spin_lock(&mn->lock);
	mn->bclk_sources[dai_index] = MN_BCLK_SOURCE_NONE;

	mn_in_use = is_bclk_source_in_use(dai, MN_BCLK_SOURCE_MN);
	/* release the M/N clock source if not used */
	if (!mn_in_use)
		reset_bclk_mn_source(dai);
	k_spin_unlock(&mn->lock, key);
}

static void mn_reset_bclk_divider(struct dai *dai, uint32_t dai_index)
{
	struct mn *mn = dai_get_mn(dai);
	k_spinlock_key_t key;

	key = k_spin_lock(&mn->lock);
	mn_write(dai, MN_MDIV_M_VAL(dai_index), 1);
	mn_write(dai, MN_MDIV_N_VAL(dai_index), 1);
	k_spin_unlock(&mn->lock, key);
}

static inline void pm_runtime_dis_ssp_clk_gating(uint32_t index)
{
#if CONFIG_SOC_SERIES_INTEL_CAVS_V15
        uint32_t shim_reg;

        shim_reg = shim_read(SHIM_CLKCTL) |
                (index < DAI_NUM_SSP_BASE ?
                        SHIM_CLKCTL_I2SFDCGB(index) :
                        SHIM_CLKCTL_I2SEFDCGB(index - DAI_NUM_SSP_BASE));

	shim_write(SHIM_CLKCTL, shim_reg);

        LOG_INF("dis-ssp-clk-gating index %d CLKCTL %08x", index, shim_reg);
#endif
}

static inline void pm_runtime_en_ssp_clk_gating(uint32_t index)
{
#if CONFIG_SOC_SERIES_INTEL_CAVS_V15
        uint32_t shim_reg;

        shim_reg = shim_read(SHIM_CLKCTL) &
		~(index < DAI_NUM_SSP_BASE ?
                        SHIM_CLKCTL_I2SFDCGB(index) :
                        SHIM_CLKCTL_I2SEFDCGB(index - DAI_NUM_SSP_BASE));

        shim_write(SHIM_CLKCTL, shim_reg);

        LOG_INF("en-ssp-clk-gating index %d CLKCTL %08x", index, shim_reg);
#endif
}

static void pm_runtime_en_ssp_power(uint32_t index)
{
#if CONFIG_SOC_SERIES_INTEL_CAVS_V25
        uint32_t reg;

        LOG_INF("en_ssp_power index %d", index);

        io_reg_write(I2SLCTL, io_reg_read(I2SLCTL) | I2SLCTL_SPA(index));

	LOG_INF("en_ssp_power after io_reg_write");

        /* Check if powered on. */
        do {
                reg = io_reg_read(I2SLCTL);
        } while (!(reg & I2SLCTL_CPA(index)));

        LOG_INF("en_ssp_power I2SLCTL %08x", reg);
#endif
}

static void pm_runtime_dis_ssp_power(uint32_t index)
{
#if CONFIG_SOC_SERIES_INTEL_CAVS_V25
        uint32_t reg;

        LOG_INF("dis_ssp_power index %d", index);

        io_reg_write(I2SLCTL, io_reg_read(I2SLCTL) & (~I2SLCTL_SPA(index)));

        /* Check if powered off. */
        do {
                reg = io_reg_read(I2SLCTL);
        } while (reg & I2SLCTL_CPA(index));

        LOG_INF("dis_ssp_power I2SLCTL %08x", reg);
#endif
}

static uint64_t ssp_clock_ticks_per_sample(struct dai *dai, uint32_t sample_rate)
{
	struct freq_table *ft = dai_get_ftable(dai);
	uint32_t ticks_per_msec;
	uint64_t ticks_per_sample;

	ticks_per_msec = ft[SSP_DEFAULT_IDX].ticks_per_msec;
	ticks_per_sample = sample_rate ? ticks_per_msec * 1000 / sample_rate : 0;

	return ticks_per_sample;
}

static inline void ssp_idelay(int n)
{
	while (n--)
		asm volatile("nop");
}

static void ssp_wait_delay(uint64_t number_of_clks)
{
	uint64_t current = k_cycle_get_64();

	while ((k_cycle_get_64() - current) < number_of_clks)
		ssp_idelay(PLATFORM_DEFAULT_DELAY);
}

static int ssp_poll_for_register_delay(uint32_t reg, uint32_t mask,
				       uint32_t val, uint64_t us)
{
	uint64_t tick = k_us_to_ticks_floor64(us);
	uint32_t tries = DEFAULT_TRY_TIMES;
	uint64_t delta = tick / tries;

	if (!delta) {
		/*
		 * If we want to wait for less than DEFAULT_TRY_TIMES ticks then
		 * delta has to be set to 1 and number of tries to that of number
		 * of ticks.
		 */
		delta = 1;
		tries = tick;
	}

	LOG_INF("ssp_poll_for_register_delay(), us %llu", us);
	LOG_INF("ssp_poll_for_register_delay(), tick %llu", tick);
	LOG_INF("ssp_poll_for_register_delay(), tries %u", tries);
	LOG_INF("ssp_poll_for_register_delay(), delta %llu", delta);

	while ((io_reg_read(reg) & mask) != val) {
		if (!tries--) {
			LOG_ERR("poll timeout reg %u mask %u val %u us %u",
			       reg, mask, val, (uint32_t)us);
			return -EIO;
		}
		ssp_wait_delay(delta);
	}

	return 0;
}

/* empty SSP transmit FIFO */
static void ssp_empty_tx_fifo(struct dai *dai)
{
	int ret;
	uint32_t sssr;

	/*
	 * SSSR_TNF is cleared when TX FIFO is empty or full,
	 * so wait for set TNF then for TFL zero - order matter.
	 */
	ret = ssp_poll_for_register_delay(dai_base(dai) + SSSR, SSSR_TNF, SSSR_TNF,
					  SSP_MAX_SEND_TIME_PER_SAMPLE);
	ret |= ssp_poll_for_register_delay(dai_base(dai) + SSCR3, SSCR3_TFL_MASK, 0,
					   SSP_MAX_SEND_TIME_PER_SAMPLE *
					   (SSP_FIFO_DEPTH - 1) / 2);

	if (ret)
		LOG_WRN("ssp_empty_tx_fifo() warning: timeout");

	sssr = ssp_read(dai, SSSR);

	/* clear interrupt */
	if (sssr & SSSR_TUR)
		ssp_write(dai, SSSR, sssr);
}

/* empty SSP receive FIFO */
static void ssp_empty_rx_fifo(struct dai *dai)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	uint64_t sample_ticks = ssp_clock_ticks_per_sample(dai, ssp->params.fsync_rate);
	uint32_t retry = SSP_RX_FLUSH_RETRY_MAX;
	uint32_t entries;
	uint32_t i;

	/*
	 * To make sure all the RX FIFO entries are read out for the flushing,
	 * we need to wait a minimal SSP port delay after entries are all read,
	 * and then re-check to see if there is any subsequent entries written
	 * to the FIFO. This will help to make sure there is no sample mismatched
	 * issue for the next run with the SSP RX.
	 */
	while ((ssp_read(dai, SSSR) & SSSR_RNE) && retry--) {
		entries = SSCR3_RFL_VAL(ssp_read(dai, SSCR3));
		LOG_DBG("ssp_empty_rx_fifo(), before flushing, entries %d", entries);
		for (i = 0; i < entries + 1; i++)
			/* read to try empty fifo */
			ssp_read(dai, SSDR);

		/* wait to get valid fifo status and re-check */
		ssp_wait_delay(sample_ticks);
		entries = SSCR3_RFL_VAL(ssp_read(dai, SSCR3));
		LOG_DBG("ssp_empty_rx_fifo(), after flushing, entries %d", entries);
	}

	/* clear interrupt */
	ssp_update_bits(dai, SSSR, SSSR_ROR, SSSR_ROR);
}

static int ssp_mclk_prepare_enable(struct dai *dai)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	int ret;

	if (ssp->clk_active & SSP_CLK_MCLK_ACTIVE)
		return 0;

	/* MCLK config */
	ret = mn_set_mclk(dai, ssp->params.mclk_id, ssp->params.mclk_rate);
	if (ret < 0)
		LOG_ERR("ssp_mclk_prepare_enable(): invalid mclk_rate = %d for mclk_id = %d",
			ssp->params.mclk_rate, ssp->params.mclk_id);
	else
		ssp->clk_active |= SSP_CLK_MCLK_ACTIVE;

	return ret;
}

static void ssp_mclk_disable_unprepare(struct dai *dai)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);

	if (!(ssp->clk_active & SSP_CLK_MCLK_ACTIVE))
		return;

	mn_release_mclk(dai, ssp->params.mclk_id);

	ssp->clk_active &= ~SSP_CLK_MCLK_ACTIVE;
}

static int ssp_bclk_prepare_enable(struct dai *dai)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	struct dai_config *config = &ssp->config;
	uint32_t sscr0;
	uint32_t mdiv;
	bool need_ecs = false;
	int ret = 0;

	if (ssp->clk_active & SSP_CLK_BCLK_ACTIVE)
		return 0;

	sscr0 = ssp_read(dai, SSCR0);

#if CONFIG_INTEL_MN
	/* BCLK config */
	ret = mn_set_bclk(dai, config->dai_index, ssp->params.bclk_rate,
			  &mdiv, &need_ecs);
	if (ret < 0) {
		LOG_ERR("ssp_bclk_prepare_enable(): invalid bclk_rate = %d for dai_index = %d",
			ssp->params.bclk_rate, config->dai_index);
		goto out;
	}
#else
	if (ft[SSP_DEFAULT_IDX].freq % ssp->params.bclk_rate != 0) {
		LOG_ERR("ssp_bclk_prepare_enable(): invalid bclk_rate = %d for dai_index = %d",
			ssp->params.bclk_rate, config->dai_index);
		ret = -EINVAL;
		goto out;
	}

	mdiv = ft[SSP_DEFAULT_IDX].freq / ssp->params.bclk_rate;
#endif

	if (need_ecs)
		sscr0 |= SSCR0_ECS;

	/* clock divisor is SCR + 1 */
	mdiv -= 1;

	/* divisor must be within SCR range */
	if (mdiv > (SSCR0_SCR_MASK >> 8)) {
		LOG_ERR("ssp_bclk_prepare_enable(): divisor %d is not within SCR range",
			mdiv);
		ret = -EINVAL;
		goto out;
	}

	/* set the SCR divisor */
	sscr0 &= ~SSCR0_SCR_MASK;
	sscr0 |= SSCR0_SCR(mdiv);

	ssp_write(dai, SSCR0, sscr0);

	LOG_INF("ssp_bclk_prepare_enable(): sscr0 = 0x%08x", sscr0);
out:
	if (!ret)
		ssp->clk_active |= SSP_CLK_BCLK_ACTIVE;

	return ret;
}

static void ssp_bclk_disable_unprepare(struct dai *dai)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);

	if (!(ssp->clk_active & SSP_CLK_BCLK_ACTIVE))
		return;
#if CONFIG_INTEL_MN
	mn_release_bclk(dai, dai->index);
#endif
	ssp->clk_active &= ~SSP_CLK_BCLK_ACTIVE;
}

static void log_ssp_data(struct dai *dai)
{
	LOG_INF("log_ssp_data dai index: %u", dai->index);
	LOG_INF("log_ssp_data plat_data base: %u", dai->plat_data.base);
	LOG_INF("log_ssp_data plat_data irq: %u", dai->plat_data.irq);
	LOG_INF("log_ssp_data plat_data fifo playback offset: %u",
		dai->plat_data.fifo[SOF_IPC_STREAM_PLAYBACK].offset);
	LOG_INF("log_ssp_data plat_data fifo playback handshake: %u",
		dai->plat_data.fifo[SOF_IPC_STREAM_PLAYBACK].handshake);
	LOG_INF("log_ssp_data plat_data fifo capture offset: %u",
		dai->plat_data.fifo[SOF_IPC_STREAM_CAPTURE].offset);
	LOG_INF("log_ssp_data plat_data fifo capture handshake: %u",
		dai->plat_data.fifo[SOF_IPC_STREAM_CAPTURE].handshake);
}

/* Digital Audio interface formatting */
static int ssp_set_config_tplg(struct dai *dai, const struct dai_config *config,
			       const void *bespoke_cfg)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	struct freq_table *ft = dai_get_ftable(dai);
	uint32_t sscr0;
	uint32_t sscr1;
	uint32_t sscr2;
	uint32_t sscr3;
	uint32_t sspsp;
	uint32_t sspsp2;
	uint32_t sstsa;
	uint32_t ssrsa;
	uint32_t ssto;
	uint32_t ssioc;
	uint32_t bdiv;
	uint32_t data_size;
	uint32_t frame_end_padding;
	uint32_t slot_end_padding;
	uint32_t frame_len = 0;
	uint32_t bdiv_min;
	uint32_t tft;
	uint32_t rft;
	uint32_t active_tx_slots = 2;
	uint32_t active_rx_slots = 2;
	uint32_t sample_width = 2;

	bool inverted_bclk = false;
	bool inverted_frame = false;
	bool cfs = false;
	bool start_delay = false;
	k_spinlock_key_t key;
	int ret = 0;

	log_ssp_data(dai);

	key = k_spin_lock(&dai->lock);

	/* ignore config if SSP is already configured */
	if (ssp->state[DAI_DIR_PLAYBACK] > DAI_STATE_READY ||
	    ssp->state[DAI_DIR_CAPTURE] > DAI_STATE_READY) {
		LOG_INF("ssp_set_config(): Already configured. Ignore config");
		goto clk;
	}

	LOG_INF("ssp_set_config()");

	/* reset SSP settings */
	/* sscr0 dynamic settings are DSS, EDSS, SCR, FRDC, ECS */
	/*
	 * FIXME: MOD, ACS, NCS are not set,
	 * no support for network mode for now
	 */
	sscr0 = SSCR0_PSP | SSCR0_RIM | SSCR0_TIM;

	/* sscr1 dynamic settings are SFRMDIR, SCLKDIR, SCFR, RSRE, TSRE */
	sscr1 = SSCR1_TTE | SSCR1_TTELP | SSCR1_TRAIL;

	/* sscr2 dynamic setting is LJDFD */
	sscr2 = SSCR2_SDFD | SSCR2_TURM1;

	/* sscr3 dynamic settings are TFT, RFT */
	sscr3 = 0;

	/* sspsp dynamic settings are SCMODE, SFRMP, DMYSTRT, SFRMWDTH */
	sspsp = 0;

	ssp->config = *config;
	memcpy(&ssp->params, bespoke_cfg, sizeof(struct sof_dai_ssp_params));

	/* sspsp2 no dynamic setting */
	sspsp2 = 0x0;

	/* ssioc dynamic setting is SFCR */
	ssioc = SSIOC_SCOE;

	/* ssto no dynamic setting */
	ssto = 0x0;

	/* sstsa dynamic setting is TTSA, default 2 slots */
	sstsa = SSTSA_SSTSA(ssp->params.tx_slots);

	/* ssrsa dynamic setting is RTSA, default 2 slots */
	ssrsa = SSRSA_SSRSA(ssp->params.rx_slots);

	switch (config->format & SOF_DAI_FMT_CLOCK_PROVIDER_MASK) {
	case SOF_DAI_FMT_CBP_CFP:
		sscr1 |= SSCR1_SCLKDIR | SSCR1_SFRMDIR;
		break;
	case SOF_DAI_FMT_CBC_CFC:
		sscr1 |= SSCR1_SCFR;
		cfs = true;
		break;
	case SOF_DAI_FMT_CBP_CFC:
		sscr1 |= SSCR1_SCLKDIR;
		/* FIXME: this mode has not been tested */

		cfs = true;
		break;
	case SOF_DAI_FMT_CBC_CFP:
		sscr1 |= SSCR1_SCFR | SSCR1_SFRMDIR;
		/* FIXME: this mode has not been tested */
		break;
	default:
		LOG_ERR("ssp_set_config(): format & PROVIDER_MASK EINVAL");
		ret = -EINVAL;
		goto out;
	}

	/* clock signal polarity */
	switch (config->format & SOF_DAI_FMT_INV_MASK) {
	case SOF_DAI_FMT_NB_NF:
		break;
	case SOF_DAI_FMT_NB_IF:
		inverted_frame = true; /* handled later with format */
		break;
	case SOF_DAI_FMT_IB_IF:
		inverted_bclk = true; /* handled later with bclk idle */
		inverted_frame = true; /* handled later with format */
		break;
	case SOF_DAI_FMT_IB_NF:
		inverted_bclk = true; /* handled later with bclk idle */
		break;
	default:
		LOG_ERR("ssp_set_config(): format & INV_MASK EINVAL");
		ret = -EINVAL;
		goto out;
	}

	/* supporting bclk idle state */
	if (ssp->params.clks_control &
		SOF_DAI_INTEL_SSP_CLKCTRL_BCLK_IDLE_HIGH) {
		/* bclk idle state high */
		sspsp |= SSPSP_SCMODE((inverted_bclk ^ 0x3) & 0x3);
	} else {
		/* bclk idle state low */
		sspsp |= SSPSP_SCMODE(inverted_bclk);
	}

	sscr0 |= SSCR0_MOD | SSCR0_ACS;

	/* Additional hardware settings */

	/* Receiver Time-out Interrupt Disabled/Enabled */
	sscr1 |= (ssp->params.quirks & SOF_DAI_INTEL_SSP_QUIRK_TINTE) ?
		SSCR1_TINTE : 0;

	/* Peripheral Trailing Byte Interrupts Disable/Enable */
	sscr1 |= (ssp->params.quirks & SOF_DAI_INTEL_SSP_QUIRK_PINTE) ?
		SSCR1_PINTE : 0;

	/* Enable/disable internal loopback. Output of transmit serial
	 * shifter connected to input of receive serial shifter, internally.
	 */
	sscr1 |= (ssp->params.quirks & SOF_DAI_INTEL_SSP_QUIRK_LBM) ?
		SSCR1_LBM : 0;

	if (ssp->params.quirks & SOF_DAI_INTEL_SSP_QUIRK_LBM)
		LOG_INF("ssp_set_config(): loopback baby!");
	else
		LOG_INF("ssp_set_config(): no loopback!");

	/* Transmit data are driven at the same/opposite clock edge specified
	 * in SSPSP.SCMODE[1:0]
	 */
	sscr2 |= (ssp->params.quirks & SOF_DAI_INTEL_SSP_QUIRK_SMTATF) ?
		SSCR2_SMTATF : 0;

	/* Receive data are sampled at the same/opposite clock edge specified
	 * in SSPSP.SCMODE[1:0]
	 */
	sscr2 |= (ssp->params.quirks & SOF_DAI_INTEL_SSP_QUIRK_MMRATF) ?
		SSCR2_MMRATF : 0;

	/* Enable/disable the fix for PSP consumer mode TXD wait for frame
	 * de-assertion before starting the second channel
	 */
	sscr2 |= (ssp->params.quirks & SOF_DAI_INTEL_SSP_QUIRK_PSPSTWFDFD) ?
		SSCR2_PSPSTWFDFD : 0;

	/* Enable/disable the fix for PSP provider mode FSRT with dummy stop &
	 * frame end padding capability
	 */
	sscr2 |= (ssp->params.quirks & SOF_DAI_INTEL_SSP_QUIRK_PSPSRWFDFD) ?
		SSCR2_PSPSRWFDFD : 0;

	if (!ssp->params.mclk_rate ||
	    ssp->params.mclk_rate > ft[MAX_SSP_FREQ_INDEX].freq) {
		LOG_ERR("ssp_set_config(): invalid MCLK = %d Hz (valid < %d)",
			ssp->params.mclk_rate,
			ft[MAX_SSP_FREQ_INDEX].freq);
		ret = -EINVAL;
		goto out;
	}

	if (!ssp->params.bclk_rate ||
	    ssp->params.bclk_rate > ssp->params.mclk_rate) {
		LOG_ERR("ssp_set_config(): BCLK %d Hz = 0 or > MCLK %d Hz",
			ssp->params.bclk_rate, ssp->params.mclk_rate);
		ret = -EINVAL;
		goto out;
	}

	/* calc frame width based on BCLK and rate - must be divisable */
	if (ssp->params.bclk_rate % ssp->params.fsync_rate) {
		LOG_ERR("ssp_set_config(): BCLK %d is not divisable by rate %d",
			ssp->params.bclk_rate, ssp->params.fsync_rate);
		ret = -EINVAL;
		goto out;
	}

	/* must be enough BCLKs for data */
	bdiv = ssp->params.bclk_rate / ssp->params.fsync_rate;
	if (bdiv < ssp->params.tdm_slot_width * ssp->params.tdm_slots) {
		LOG_ERR("ssp_set_config(): not enough BCLKs need %d",
			ssp->params.tdm_slot_width *
			ssp->params.tdm_slots);
		ret = -EINVAL;
		goto out;
	}

	/* tdm_slot_width must be <= 38 for SSP */
	if (ssp->params.tdm_slot_width > 38) {
		LOG_ERR("ssp_set_config(): tdm_slot_width %d > 38",
			ssp->params.tdm_slot_width);
		ret = -EINVAL;
		goto out;
	}

	bdiv_min = ssp->params.tdm_slots *
		   (ssp->params.tdm_per_slot_padding_flag ?
		    ssp->params.tdm_slot_width : ssp->params.sample_valid_bits);
	if (bdiv < bdiv_min) {
		LOG_ERR("ssp_set_config(): bdiv(%d) < bdiv_min(%d)",
			bdiv, bdiv_min);
		ret = -EINVAL;
		goto out;
	}

	frame_end_padding = bdiv - bdiv_min;
	if (frame_end_padding > SSPSP2_FEP_MASK) {
		LOG_ERR("ssp_set_config(): frame_end_padding too big: %u",
			frame_end_padding);
		ret = -EINVAL;
		goto out;
	}

	/* format */
	switch (config->format & SOF_DAI_FMT_FORMAT_MASK) {
	case SOF_DAI_FMT_I2S:

		start_delay = true;

		sscr0 |= SSCR0_FRDC(ssp->params.tdm_slots);

		if (bdiv % 2) {
			LOG_ERR("ssp_set_config(): bdiv %d is not divisible by 2",
				bdiv);
			ret = -EINVAL;
			goto out;
		}

		/* set asserted frame length to half frame length */
		frame_len = bdiv / 2;

		/*
		 * handle frame polarity, I2S default is falling/active low,
		 * non-inverted(inverted_frame=0) -- active low(SFRMP=0),
		 * inverted(inverted_frame=1) -- rising/active high(SFRMP=1),
		 * so, we should set SFRMP to inverted_frame.
		 */
		sspsp |= SSPSP_SFRMP(inverted_frame);

		/*
		 *  for I2S/LEFT_J, the padding has to happen at the end
		 * of each slot
		 */
		if (frame_end_padding % 2) {
			LOG_ERR("ssp_set_config(): frame_end_padding %d is not divisible by 2",
				frame_end_padding);
			ret = -EINVAL;
			goto out;
		}

		slot_end_padding = frame_end_padding / 2;

		if (slot_end_padding > SOF_DAI_INTEL_SSP_SLOT_PADDING_MAX) {
			/* too big padding */
			LOG_ERR("ssp_set_config(): slot_end_padding > %d",
				SOF_DAI_INTEL_SSP_SLOT_PADDING_MAX);
			ret = -EINVAL;
			goto out;
		}

		sspsp |= SSPSP_DMYSTOP(slot_end_padding);
		slot_end_padding >>= SSPSP_DMYSTOP_BITS;
		sspsp |= SSPSP_EDMYSTOP(slot_end_padding);

		break;

	case SOF_DAI_FMT_LEFT_J:

		/* default start_delay value is set to false */

		sscr0 |= SSCR0_FRDC(ssp->params.tdm_slots);

		/* LJDFD enable */
		sscr2 &= ~SSCR2_LJDFD;

		if (bdiv % 2) {
			LOG_ERR("ssp_set_config(): bdiv %d is not divisible by 2",
				bdiv);
			ret = -EINVAL;
			goto out;
		}

		/* set asserted frame length to half frame length */
		frame_len = bdiv / 2;

		/*
		 * handle frame polarity, LEFT_J default is rising/active high,
		 * non-inverted(inverted_frame=0) -- active high(SFRMP=1),
		 * inverted(inverted_frame=1) -- falling/active low(SFRMP=0),
		 * so, we should set SFRMP to !inverted_frame.
		 */
		sspsp |= SSPSP_SFRMP(!inverted_frame);

		/*
		 *  for I2S/LEFT_J, the padding has to happen at the end
		 * of each slot
		 */
		if (frame_end_padding % 2) {
			LOG_ERR("ssp_set_config(): frame_end_padding %d is not divisible by 2",
				frame_end_padding);
			ret = -EINVAL;
			goto out;
		}

		slot_end_padding = frame_end_padding / 2;

		if (slot_end_padding > 15) {
			/* can't handle padding over 15 bits */
			LOG_ERR("ssp_set_config(): slot_end_padding %d > 15 bits",
				slot_end_padding);
			ret = -EINVAL;
			goto out;
		}

		sspsp |= SSPSP_DMYSTOP(slot_end_padding);
		slot_end_padding >>= SSPSP_DMYSTOP_BITS;
		sspsp |= SSPSP_EDMYSTOP(slot_end_padding);

		break;
	case SOF_DAI_FMT_DSP_A:

		start_delay = true;

		/* fallthrough */

	case SOF_DAI_FMT_DSP_B:

		/* default start_delay value is set to false */

		sscr0 |= SSCR0_MOD | SSCR0_FRDC(ssp->params.tdm_slots);

		/* set asserted frame length */
		frame_len = 1; /* default */

		if (cfs && ssp->params.frame_pulse_width > 0 &&
		    ssp->params.frame_pulse_width <=
		    SOF_DAI_INTEL_SSP_FRAME_PULSE_WIDTH_MAX) {
			frame_len = ssp->params.frame_pulse_width;
		}

		/* frame_pulse_width must less or equal 38 */
		if (ssp->params.frame_pulse_width >
			SOF_DAI_INTEL_SSP_FRAME_PULSE_WIDTH_MAX) {
			LOG_ERR("ssp_set_config(): frame_pulse_width > %d",
				SOF_DAI_INTEL_SSP_FRAME_PULSE_WIDTH_MAX);
			ret = -EINVAL;
			goto out;
		}
		/*
		 * handle frame polarity, DSP_B default is rising/active high,
		 * non-inverted(inverted_frame=0) -- active high(SFRMP=1),
		 * inverted(inverted_frame=1) -- falling/active low(SFRMP=0),
		 * so, we should set SFRMP to !inverted_frame.
		 */
		sspsp |= SSPSP_SFRMP(!inverted_frame);

		active_tx_slots = popcount(ssp->params.tx_slots);
		active_rx_slots = popcount(ssp->params.rx_slots);

		/*
		 * handle TDM mode, TDM mode has padding at the end of
		 * each slot. The amount of padding is equal to result of
		 * subtracting slot width and valid bits per slot.
		 */
		if (ssp->params.tdm_per_slot_padding_flag) {
			frame_end_padding = bdiv - ssp->params.tdm_slots *
				ssp->params.tdm_slot_width;

			slot_end_padding = ssp->params.tdm_slot_width -
				ssp->params.sample_valid_bits;

			if (slot_end_padding >
				SOF_DAI_INTEL_SSP_SLOT_PADDING_MAX) {
				LOG_ERR("ssp_set_config(): slot_end_padding > %d",
					SOF_DAI_INTEL_SSP_SLOT_PADDING_MAX);
				ret = -EINVAL;
				goto out;
			}

			sspsp |= SSPSP_DMYSTOP(slot_end_padding);
			slot_end_padding >>= SSPSP_DMYSTOP_BITS;
			sspsp |= SSPSP_EDMYSTOP(slot_end_padding);
		}

		sspsp2 |= (frame_end_padding & SSPSP2_FEP_MASK);

		break;
	default:
		LOG_ERR("ssp_set_config(): invalid format 0x%04x",
			config->format);
		ret = -EINVAL;
		goto out;
	}

	if (start_delay)
		sspsp |= SSPSP_FSRT;

	sspsp |= SSPSP_SFRMWDTH(frame_len);

	data_size = ssp->params.sample_valid_bits;

	if (data_size > 16)
		sscr0 |= (SSCR0_EDSS | SSCR0_DSIZE(data_size - 16));
	else
		sscr0 |= SSCR0_DSIZE(data_size);

	/* setting TFT and RFT */
	switch (ssp->params.sample_valid_bits) {
	case 16:
			/* use 2 bytes for each slot */
			sample_width = 2;
			break;
	case 24:
	case 32:
			/* use 4 bytes for each slot */
			sample_width = 4;
			break;
	default:
			LOG_ERR("ssp_set_config(): sample_valid_bits %d",
				ssp->params.sample_valid_bits);
			ret = -EINVAL;
			goto out;
	}

	tft = MIN(SSP_FIFO_DEPTH - SSP_FIFO_WATERMARK,
		  sample_width * active_tx_slots);
	rft = MIN(SSP_FIFO_DEPTH - SSP_FIFO_WATERMARK,
		  sample_width * active_rx_slots);

	sscr3 |= SSCR3_TX(tft) | SSCR3_RX(rft);

	ssp_write(dai, SSCR0, sscr0);
	ssp_write(dai, SSCR1, sscr1);
	ssp_write(dai, SSCR2, sscr2);
	ssp_write(dai, SSCR3, sscr3);
	ssp_write(dai, SSPSP, sspsp);
	ssp_write(dai, SSPSP2, sspsp2);
	ssp_write(dai, SSIOC, ssioc);
	ssp_write(dai, SSTO, ssto);
	ssp_write(dai, SSTSA, sstsa);
	ssp_write(dai, SSRSA, ssrsa);

	LOG_INF("ssp_set_config(), sscr0 = 0x%08x, sscr1 = 0x%08x, ssto = 0x%08x, sspsp = 0x%0x",
		 sscr0, sscr1, ssto, sspsp);
	LOG_INF("ssp_set_config(), sscr2 = 0x%08x, sspsp2 = 0x%08x, sscr3 = 0x%08x, ssioc = 0x%08x",
		 sscr2, sspsp2, sscr3, ssioc);
	LOG_INF("ssp_set_config(), ssrsa = 0x%08x, sstsa = 0x%08x",
		 ssrsa, sstsa);

	ssp->state[DAI_DIR_PLAYBACK] = DAI_STATE_PRE_RUNNING;
	ssp->state[DAI_DIR_CAPTURE] = DAI_STATE_PRE_RUNNING;

clk:
	switch (config->options & SOF_DAI_CONFIG_FLAGS_CMD_MASK) {
	case SOF_DAI_CONFIG_FLAGS_HW_PARAMS:
		if (ssp->params.clks_control & SOF_DAI_INTEL_SSP_CLKCTRL_MCLK_ES) {
			ret = ssp_mclk_prepare_enable(dai);
			if (ret < 0)
				goto out;

			ssp->clk_active |= SSP_CLK_MCLK_ES_REQ;

			LOG_INF("ssp_set_config(): hw_params stage: enabled MCLK clocks for SSP%d...",
				 dai->index);
		}

		if (ssp->params.clks_control & SOF_DAI_INTEL_SSP_CLKCTRL_BCLK_ES) {
			bool enable_sse = false;

			if (!(ssp->clk_active & SSP_CLK_BCLK_ACTIVE))
				enable_sse = true;

			ret = ssp_bclk_prepare_enable(dai);
			if (ret < 0)
				goto out;

			ssp->clk_active |= SSP_CLK_BCLK_ES_REQ;

			if (enable_sse) {

				/* enable TRSE/RSRE before SSE */
				ssp_update_bits(dai, SSCR1,
						SSCR1_TSRE | SSCR1_RSRE,
						SSCR1_TSRE | SSCR1_RSRE);

				/* enable port */
				ssp_update_bits(dai, SSCR0, SSCR0_SSE, SSCR0_SSE);

				LOG_INF("ssp_set_config(): SSE set for SSP%d", dai->index);
			}

			LOG_INF("ssp_set_config(): hw_params stage: enabled BCLK clocks for SSP%d...",
				 dai->index);
		}
		break;
	case SOF_DAI_CONFIG_FLAGS_HW_FREE:
		/* disable SSP port if no users */
		if (ssp->state[SOF_IPC_STREAM_CAPTURE] != DAI_STATE_PRE_RUNNING ||
		    ssp->state[SOF_IPC_STREAM_PLAYBACK] != DAI_STATE_PRE_RUNNING) {
			LOG_INF("ssp_set_config(): hw_free stage: ignore since SSP%d still in use",
				 dai->index);
			break;
		}

		if (ssp->params.clks_control & SOF_DAI_INTEL_SSP_CLKCTRL_BCLK_ES) {
			LOG_INF("ssp_set_config(): hw_free stage: releasing BCLK clocks for SSP%d...",
				 dai->index);
			if (ssp->clk_active & SSP_CLK_BCLK_ACTIVE) {
				/* clear TRSE/RSRE before SSE */
				ssp_update_bits(dai, SSCR1,
						SSCR1_TSRE | SSCR1_RSRE,
						0);

				ssp_update_bits(dai, SSCR0, SSCR0_SSE, 0);
				LOG_INF("ssp_set_config(): SSE clear for SSP%d", dai->index);
			}
			ssp_bclk_disable_unprepare(dai);
			ssp->clk_active &= ~SSP_CLK_BCLK_ES_REQ;
		}
		if (ssp->params.clks_control & SOF_DAI_INTEL_SSP_CLKCTRL_MCLK_ES) {
			LOG_INF("ssp_set_config: hw_free stage: releasing MCLK clocks for SSP%d...",
				 dai->index);
			ssp_mclk_disable_unprepare(dai);
			ssp->clk_active &= ~SSP_CLK_MCLK_ES_REQ;
		}
		break;
	default:
		break;
	}
out:

	k_spin_unlock(&dai->lock, key);

	return ret;
}

static int ssp_set_config_blob(struct dai *dai, const void *spec_config)
{
	const struct ipc4_ssp_configuration_blob *blob = spec_config;
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	uint32_t ssc0, sstsa, ssrsa;

	/* set config only once for playback or capture */
	if (dai->sref > 1)
		return 0;

	ssc0 = blob->i2s_driver_config.i2s_config.ssc0;
	sstsa = blob->i2s_driver_config.i2s_config.sstsa;
	ssrsa = blob->i2s_driver_config.i2s_config.ssrsa;

	ssp_write(dai, SSCR0, ssc0);
	ssp_write(dai, SSCR1, blob->i2s_driver_config.i2s_config.ssc1);
	ssp_write(dai, SSCR2, blob->i2s_driver_config.i2s_config.ssc2);
	ssp_write(dai, SSCR3, blob->i2s_driver_config.i2s_config.ssc3);
	ssp_write(dai, SSPSP, blob->i2s_driver_config.i2s_config.sspsp);
	ssp_write(dai, SSPSP2, blob->i2s_driver_config.i2s_config.sspsp2);
	ssp_write(dai, SSIOC, blob->i2s_driver_config.i2s_config.ssioc);
	ssp_write(dai, SSTO, blob->i2s_driver_config.i2s_config.sscto);
	ssp_write(dai, SSTSA, sstsa);
	ssp_write(dai, SSRSA, ssrsa);

	LOG_INF("ssp_set_config(), sscr0 = 0x%08x, sscr1 = 0x%08x, ssto = 0x%08x, sspsp = 0x%0x",
		 ssc0, blob->i2s_driver_config.i2s_config.ssc1,
		 blob->i2s_driver_config.i2s_config.sscto,
		 blob->i2s_driver_config.i2s_config.sspsp);
	LOG_INF("ssp_set_config(), sscr2 = 0x%08x, sspsp2 = 0x%08x, sscr3 = 0x%08x",
		 blob->i2s_driver_config.i2s_config.ssc2, blob->i2s_driver_config.i2s_config.sspsp2,
		 blob->i2s_driver_config.i2s_config.ssc3);
	LOG_ERR("ssp_set_config(), ssioc = 0x%08x, ssrsa = 0x%08x, sstsa = 0x%08x",
		blob->i2s_driver_config.i2s_config.ssioc, ssrsa, sstsa);

	ssp->params.sample_valid_bits = SSCR0_DSIZE_GET(ssc0);
	if (ssc0 & SSCR0_EDSS)
		ssp->params.sample_valid_bits += 16;

	ssp->params.tdm_slots = SSCR0_FRDC_GET(ssc0);
	ssp->params.tx_slots = SSTSA_GET(sstsa);
	ssp->params.rx_slots = SSRSA_GET(ssrsa);
	ssp->params.fsync_rate = 48000;

	ssp->state[DAI_DIR_PLAYBACK] = DAI_STATE_PRE_RUNNING;
	ssp->state[DAI_DIR_CAPTURE] = DAI_STATE_PRE_RUNNING;

	/* ssp blob is set by pcm_hw_params for ipc4 stream, so enable
	 * mclk and bclk at this time.
	 */
	mn_set_mclk_blob(dai, blob->i2s_driver_config.mclk_config.mdivc,
			 blob->i2s_driver_config.mclk_config.mdivr);
	ssp->clk_active |= SSP_CLK_MCLK_ES_REQ;

	/* enable TRSE/RSRE before SSE */
	ssp_update_bits(dai, SSCR1, SSCR1_TSRE | SSCR1_RSRE, SSCR1_TSRE | SSCR1_RSRE);

	/* enable port */
	ssp_update_bits(dai, SSCR0, SSCR0_SSE, SSCR0_SSE);
	ssp->clk_active |= SSP_CLK_BCLK_ES_REQ;

	return 0;
}

/*
 * Portion of the SSP configuration should be applied just before the
 * SSP dai is activated, for either power saving or params runtime
 * configurable flexibility.
 */
static int ssp_pre_start(struct dai *dai)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	int ret = 0;

	LOG_INF("ssp_pre_start()");

	/*
	 * We will test if mclk/bclk is configured in
	 * ssp_mclk/bclk_prepare_enable/disable functions
	 */
	if (!(ssp->clk_active & SSP_CLK_MCLK_ES_REQ)) {
		/* MCLK config */
		ret = ssp_mclk_prepare_enable(dai);
		if (ret < 0)
			return ret;
	}

	if (!(ssp->clk_active & SSP_CLK_BCLK_ES_REQ))
		ret = ssp_bclk_prepare_enable(dai);

	return ret;
}

/*
 * For power saving, we should do kinds of power release when the SSP
 * dai is changed to inactive, though the runtime param configuration
 * don't have to be reset.
 */
static void ssp_post_stop(struct dai *dai)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);

	/* release clocks if SSP is inactive */
	if (ssp->state[SOF_IPC_STREAM_PLAYBACK] != DAI_STATE_RUNNING &&
	    ssp->state[SOF_IPC_STREAM_CAPTURE] != DAI_STATE_RUNNING) {
		if (!(ssp->clk_active & SSP_CLK_BCLK_ES_REQ)) {
			LOG_INF("ssp_post_stop releasing BCLK clocks for SSP%d...",
				 dai->index);
			ssp_bclk_disable_unprepare(dai);
		}
		if (!(ssp->clk_active & SSP_CLK_MCLK_ES_REQ)) {
			LOG_INF("ssp_post_stop releasing MCLK clocks for SSP%d...",
				 dai->index);
			ssp_mclk_disable_unprepare(dai);
		}
	}
}

/* get SSP hw params zephyr way */
static struct dai_config * ssp_get_hw_params(const struct device *dev, int dir)
{
	struct dai_config *params = (struct dai_config *)dev->config;
	struct dai *dai = (struct dai *)dev->data;
	struct ssp_pdata *ssp = dai_get_drvdata(dai);

	params->rate = ssp->params.fsync_rate;

	if (dir == SOF_IPC_STREAM_PLAYBACK)
		params->channels = popcount(ssp->params.tx_slots);
	else
		params->channels = popcount(ssp->params.rx_slots);

	switch (ssp->params.sample_valid_bits) {
	case 16:
		params->format = SOF_IPC_FRAME_S16_LE;
		break;
	case 24:
		params->format = SOF_IPC_FRAME_S24_4LE;
		break;
	case 32:
		params->format = SOF_IPC_FRAME_S32_LE;
		break;
	default:
		LOG_ERR("ssp_get_hw_params(): not supported format");
		return NULL;
	}

	return params;
}

static void ssp_early_start(struct dai *dai, int direction)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	k_spinlock_key_t key;

	key = k_spin_lock(&dai->lock);

	/* request mclk/bclk */
	ssp_pre_start(dai);

	if (!(ssp->clk_active & SSP_CLK_BCLK_ES_REQ)) {
		/* enable TRSE/RSRE before SSE */
		ssp_update_bits(dai, SSCR1,
				SSCR1_TSRE | SSCR1_RSRE,
				SSCR1_TSRE | SSCR1_RSRE);

		/* enable port */
		ssp_update_bits(dai, SSCR0, SSCR0_SSE, SSCR0_SSE);
		LOG_INF("ssp_early_start(): SSE set for SSP%d", dai->index);
	}


	k_spin_unlock(&dai->lock, key);
}

/* start the SSP for either playback or capture */
static void ssp_start(struct dai *dai, int direction)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	k_spinlock_key_t key;

	key = k_spin_lock(&dai->lock);

	LOG_INF("ssp_start()");

	/* enable DMA */
	if (direction == DAI_DIR_PLAYBACK)
		ssp_update_bits(dai, SSTSA, SSTSA_TXEN, SSTSA_TXEN);
	else
		ssp_update_bits(dai, SSRSA, SSRSA_RXEN, SSRSA_RXEN);

	ssp->state[direction] = DAI_STATE_RUNNING;

	/*
	 * Wait to get valid fifo status in clock consumer mode. TODO it's
	 * uncertain which SSP clock consumer modes need the delay atm, but
	 * these can be added here when confirmed.
	 */
	switch (ssp->config.format & SOF_DAI_FMT_CLOCK_PROVIDER_MASK) {
	case SOF_DAI_FMT_CBC_CFC:
		break;
	default:
		/* delay for all SSP consumed clocks atm - see above */
		ssp_wait_delay(PLATFORM_SSP_DELAY);
		break;
	}

	LOG_INF("ssp_start() 2");
	k_spin_unlock(&dai->lock, key);
}

/* stop the SSP for either playback or capture */
static void ssp_stop(struct dai *dai, int direction)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	k_spinlock_key_t key;

	key = k_spin_lock(&dai->lock);

	/*
	 * Wait to get valid fifo status in clock consumer mode. TODO it's
	 * uncertain which SSP clock consumer modes need the delay atm, but
	 * these can be added here when confirmed.
	 */
	switch (ssp->config.format & SOF_DAI_FMT_CLOCK_PROVIDER_MASK) {
	case SOF_DAI_FMT_CBC_CFC:
		break;
	default:
		/* delay for all SSP consumed clocks atm - see above */
		ssp_wait_delay(PLATFORM_SSP_DELAY);
		break;
	}

	/* stop Rx if neeed */
	if (direction == DAI_DIR_CAPTURE &&
	    ssp->state[SOF_IPC_STREAM_CAPTURE] != DAI_STATE_PRE_RUNNING) {
		ssp_update_bits(dai, SSRSA, SSRSA_RXEN, 0);
		ssp_empty_rx_fifo(dai);
		ssp->state[SOF_IPC_STREAM_CAPTURE] = DAI_STATE_PRE_RUNNING;
		LOG_INF("ssp_stop(), RX stop");
	}

	/* stop Tx if needed */
	if (direction == DAI_DIR_PLAYBACK &&
	    ssp->state[SOF_IPC_STREAM_PLAYBACK] != DAI_STATE_PRE_RUNNING) {
		ssp_empty_tx_fifo(dai);
		ssp_update_bits(dai, SSTSA, SSTSA_TXEN, 0);
		ssp->state[SOF_IPC_STREAM_PLAYBACK] = DAI_STATE_PRE_RUNNING;
		LOG_INF("ssp_stop(), TX stop");
	}

	/* disable SSP port if no users */
	if (ssp->state[SOF_IPC_STREAM_CAPTURE] == DAI_STATE_PRE_RUNNING &&
	    ssp->state[SOF_IPC_STREAM_PLAYBACK] == DAI_STATE_PRE_RUNNING) {
		if (!(ssp->clk_active & SSP_CLK_BCLK_ES_REQ)) {
			/* clear TRSE/RSRE before SSE */
			ssp_update_bits(dai, SSCR1,
					SSCR1_TSRE | SSCR1_RSRE,
					0);

			ssp_update_bits(dai, SSCR0, SSCR0_SSE, 0);
			LOG_INF("ssp_stop(): SSE clear SSP%d", dai->index);
		}
	}

	ssp_post_stop(dai);

	k_spin_unlock(&dai->lock, key);
}

static void ssp_pause(struct dai *dai, int direction)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);

	if (direction == SOF_IPC_STREAM_CAPTURE)
		LOG_INF("ssp_pause(), RX");
	else
		LOG_INF("ssp_pause(), TX");

	ssp->state[direction] = DAI_STATE_PAUSED;
}

static int ssp_trigger(struct dai *dai, int cmd, int direction)
{
	struct ssp_pdata *ssp = dai_get_drvdata(dai);
	int array_index = 0;

	LOG_INF("ssp_trigger() cmd %d", cmd);

	switch (direction) {
	case DAI_DIR_RX:
		array_index = 1;
		break;
	case DAI_DIR_TX:
		array_index = 0;
		break;
	default:
		LOG_INF("ssp_trigger() only supports dir RX or TX");
		break;
	}

	switch (cmd) {
	case DAI_TRIGGER_START:
		if (ssp->state[array_index] == DAI_STATE_PAUSED ||
		    ssp->state[array_index] == DAI_STATE_PRE_RUNNING)
			ssp_start(dai, array_index);
		break;
	case DAI_TRIGGER_STOP:
		ssp_stop(dai, array_index);
		break;
	case DAI_TRIGGER_PAUSE:
		ssp_pause(dai, array_index);
		break;
	case DAI_TRIGGER_PRE_START:
		ssp_early_start(dai, array_index);
		break;
	}

	return 0;
}

static int ssp_probe(struct dai *dai)
{
	struct ssp_pdata *ssp;

	if (dai_get_drvdata(dai))
		return -EEXIST; /* already created */

	/* allocate private data */
	ssp = k_calloc(1, sizeof(*ssp));
	if (!ssp) {
		LOG_ERR("ssp_probe(): alloc failed");
		return -ENOMEM;
	}
	dai_set_drvdata(dai, ssp);

	ssp->state[DAI_DIR_PLAYBACK] = DAI_STATE_READY;
	ssp->state[DAI_DIR_CAPTURE] = DAI_STATE_READY;

#if CONFIG_INTEL_MN
	/* Reset M/N, power-gating functions need it */
	mn_reset_bclk_divider(dai, dai->index);
#endif

	/* Enable SSP power */
	pm_runtime_en_ssp_power(dai->index);

	/* Disable dynamic clock gating before touching any register */
	pm_runtime_dis_ssp_clk_gating(dai->index);

	ssp_empty_rx_fifo(dai);

	return 0;
}

static int ssp_remove(struct dai *dai)
{
	LOG_INF("really removing ssp ");

	pm_runtime_en_ssp_clk_gating(dai->index);

	ssp_mclk_disable_unprepare(dai);
	ssp_bclk_disable_unprepare(dai);

	/* Disable SSP power */
	pm_runtime_dis_ssp_power(dai->index);

	k_free(dai_get_drvdata(dai));
	dai_set_drvdata(dai, NULL);

	return 0;
}

static int dai_ssp_probe(const struct device *dev)
{
	struct dai *d = (struct dai *)dev->data;
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&d->lock);

	LOG_INF("1 dai_ssp_probe() %d", d->sref);
	if (d->sref == 0)
		ret = ssp_probe(d);

	if (!ret)
		d->sref++;

	LOG_INF("2 dai_ssp_probe() %d", d->sref);
	k_spin_unlock(&d->lock, key);

	return ret;
}

static int dai_ssp_trigger(const struct device *dev, enum dai_dir dir,
			   enum dai_trigger_cmd cmd)
{
	return ssp_trigger((struct dai *)dev->data, cmd, dir);
}

static int dai_ssp_config_set(const struct device *dev, const struct dai_config *cfg,
			      const void *bespoke_cfg)
{
	struct dai *d = (struct dai *)dev->data;

	if (cfg->type == DAI_INTEL_SSP)
		return ssp_set_config_tplg(d, cfg, bespoke_cfg);
	else
		return ssp_set_config_blob(d, bespoke_cfg);
}

static const struct dai_config *dai_ssp_config_get(const struct device *dev, enum dai_dir dir)
{
	return ssp_get_hw_params(dev, dir);
}

static int dai_ssp_remove(const struct device *dev)
{
	struct dai *d = (struct dai *)dev->data;
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&d->lock);

	LOG_INF("1 dai_ssp_remove() %d", d->sref);
        if (--d->sref == 0)
		ret = ssp_remove(d);

	LOG_INF("2 dai_ssp_remove() %d", d->sref);

	k_spin_unlock(&d->lock, key);

	return ret;
}

static const struct dai_properties * dai_ssp_get_properties(const struct device *dev, enum dai_dir
							    dir, int stream_id)
{
	struct dai *d = (struct dai *)dev->data;
	struct ssp_pdata *ssp = dai_get_drvdata(d);
	struct dai_properties *prop = &ssp->props;
	int array_index = 0;

	switch (dir) {
	case DAI_DIR_RX:
		array_index = 1;
		break;
	case DAI_DIR_TX:
		array_index = 0;
		break;
	default:
		LOG_INF("ssp_get_properties() only supports dir RX or TX");
		break;
	}

	prop->fifo_address = d->plat_data.fifo[array_index].offset;
	prop->dma_hs_id = d->plat_data.fifo[array_index].handshake;

	if (ssp->clk_active & SSP_CLK_BCLK_ACTIVE)
		prop->reg_init_delay = 0;
	else
		prop->reg_init_delay = ssp->params.bclk_delay;

	LOG_INF("ssp_get_properties() dai_index %u", d->index);
	LOG_INF("ssp_get_properties() fifo %u", prop->fifo_address);
	LOG_INF("ssp_get_properties() handshake %u", prop->dma_hs_id);
	LOG_INF("ssp_get_init_delay() init delay %u", prop->reg_init_delay);

	return prop;
}

static int ssp_init(const struct device *dev)
{
	return 0;
}

static struct dai_driver_api sof_ssp_api_funcs = {
	.probe			= dai_ssp_probe,
	.remove			= dai_ssp_remove,
	.config_set		= dai_ssp_config_set,
	.config_get		= dai_ssp_config_get,
	.trigger		= dai_ssp_trigger,
	.get_properties		= dai_ssp_get_properties,
};

#define CLK_AUDIO DT_PROP(DT_NODELABEL(audioclk), clock_frequency)
#define CLK_SYSTEM DT_PROP(DT_NODELABEL(sysclk), clock_frequency)
#define CLK_PLL DT_PROP(DT_NODELABEL(pllclk), clock_frequency)

static struct freq_table sof_ssp_freq_table[] = {
	{ CLK_AUDIO, CLK_AUDIO / 1000},
	{ CLK_SYSTEM, CLK_SYSTEM / 1000},
	{ CLK_PLL, CLK_PLL / 1000},
};

static uint32_t sof_ssp_freq_sources[] = {
	SSP_CLOCK_AUDIO_CARDINAL,
	SSP_CLOCK_XTAL_OSCILLATOR,
	SSP_CLOCK_PLL_FIXED,
};


static struct mn sof_ssp_mn_divider = {
	.base = DT_REG_ADDR_BY_IDX(DT_NODELABEL(ssp0), 1),
};

const char irq_name_level5_z[] = "level5";

#define DAI_SOF_SSP_DEVICE_INIT(n)						\
	static struct dai_config sof_ssp_config_##n;				\
	static struct dai dai_sof_ssp_data_##n = {				\
		.index = n,							\
		.plat_data = {							\
			.base =	DT_INST_REG_ADDR_BY_IDX(n, 0),			\
                        .irq = n,						\
                        .irq_name = irq_name_level5_z,				\
                        .fifo[SOF_IPC_STREAM_PLAYBACK].offset =			\
				DT_INST_REG_ADDR_BY_IDX(n, 0) + SSDR,		\
                        .fifo[SOF_IPC_STREAM_PLAYBACK].handshake =		\
				DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),	\
                        .fifo[SOF_IPC_STREAM_CAPTURE].offset =			\
				DT_INST_REG_ADDR_BY_IDX(n, 0) + SSDR,		\
			.fifo[SOF_IPC_STREAM_CAPTURE].handshake =		\
				DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),	\
			.mn_inst = &sof_ssp_mn_divider,				\
			.ftable = sof_ssp_freq_table,				\
			.fsources = sof_ssp_freq_sources,			\
		},								\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			ssp_init, NULL,					\
			&dai_sof_ssp_data_##n,				\
			&sof_ssp_config_##n,				\
			POST_KERNEL, 32,				\
			&sof_ssp_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(DAI_SOF_SSP_DEVICE_INIT)
