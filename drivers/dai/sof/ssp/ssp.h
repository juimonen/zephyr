/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOF_DAI_DRIVER_SSP_H__
#define __SOF_DAI_DRIVER_SSP_H__

#include <stdint.h>
#include <drivers/dai.h>

#define MASK(b_hi, b_lo)	\
	(((1ULL << ((b_hi) - (b_lo) + 1ULL)) - 1ULL) << (b_lo))
#define SET_BIT(b, x)		(((x) & 1) << (b))
#define SET_BITS(b_hi, b_lo, x)	\
	(((x) & ((1ULL << ((b_hi) - (b_lo) + 1ULL)) - 1ULL)) << (b_lo))
#define GET_BIT(b, x) \
	(((x) & (1ULL << (b))) >> (b))
#define GET_BITS(b_hi, b_lo, x) \
	(((x) & MASK(b_hi, b_lo)) >> (b_lo))

/* ssp_freq array constants */
#define NUM_SSP_FREQ	3
#define MAX_SSP_FREQ_INDEX	(NUM_SSP_FREQ - 1)
#define SSP_DEFAULT_IDX	1

/* the SSP port fifo depth */
#define SSP_FIFO_DEPTH		16

/* the watermark for the SSP fifo depth setting */
#define SSP_FIFO_WATERMARK	8

/* minimal SSP port delay in cycles */
#define PLATFORM_SSP_DELAY	1600

#define PLATFORM_DEFAULT_DELAY 12
#define DEFAULT_TRY_TIMES 8


#define DAI_DIR_PLAYBACK	0
#define DAI_DIR_CAPTURE		1

#define DAI_IPC_MAX_CHANNELS			8

#define SOF_DAI_FMT_I2S		1 /**< I2S mode */
#define SOF_DAI_FMT_RIGHT_J	2 /**< Right Justified mode */
#define SOF_DAI_FMT_LEFT_J	3 /**< Left Justified mode */
#define SOF_DAI_FMT_DSP_A	4 /**< L data MSB after FRM LRC */
#define SOF_DAI_FMT_DSP_B	5 /**< L data MSB during FRM LRC */
#define SOF_DAI_FMT_PDM		6 /**< Pulse density modulation */

#define SOF_DAI_FMT_CONT	(1 << 4) /**< continuous clock */
#define SOF_DAI_FMT_GATED	(0 << 4) /**< clock is gated */

#define SOF_DAI_FMT_NB_NF	(0 << 8) /**< normal bit clock + frame */
#define SOF_DAI_FMT_NB_IF	(2 << 8) /**< normal BCLK + inv FRM */
#define SOF_DAI_FMT_IB_NF	(3 << 8) /**< invert BCLK + nor FRM */
#define SOF_DAI_FMT_IB_IF	(4 << 8) /**< invert BCLK + FRM */

#define SOF_DAI_FMT_CBP_CFP	(0 << 12) /**< codec bclk provider & frame provider */
#define SOF_DAI_FMT_CBC_CFP	(2 << 12) /**< codec bclk consumer & frame provider */
#define SOF_DAI_FMT_CBP_CFC	(3 << 12) /**< codec bclk provider & frame consumer */
#define SOF_DAI_FMT_CBC_CFC	(4 << 12) /**< codec bclk consumer & frame consumer */

#define SOF_DAI_FMT_FORMAT_MASK		0x000f
#define SOF_DAI_FMT_CLOCK_MASK		0x00f0
#define SOF_DAI_FMT_INV_MASK		0x0f00
#define SOF_DAI_FMT_CLOCK_PROVIDER_MASK	0xf000

/*
 * DAI_CONFIG flags. The 4 LSB bits are used for the commands, HW_PARAMS, HW_FREE and PAUSE
 * representing when the IPC is sent. The 4 MSB bits are used to add quirks along with the above
 * commands.
 */
#define SOF_DAI_CONFIG_FLAGS_CMD_MASK	0xF
#define SOF_DAI_CONFIG_FLAGS_NONE	0 /**< DAI_CONFIG sent without stage information */
#define SOF_DAI_CONFIG_FLAGS_HW_PARAMS	BIT(0) /**< DAI_CONFIG sent during hw_params stage */
#define SOF_DAI_CONFIG_FLAGS_HW_FREE	BIT(1) /**< DAI_CONFIG sent during hw_free stage */

/**< DAI_CONFIG sent during pause trigger. Only available ABI 3.20 onwards */
#define SOF_DAI_CONFIG_FLAGS_PAUSE	BIT(2)
#define SOF_DAI_CONFIG_FLAGS_QUIRK_SHIFT 4
#define SOF_DAI_CONFIG_FLAGS_QUIRK_MASK  (0xF << SOF_DAI_CONFIG_FLAGS_QUIRK_SHIFT)
/*
 * This should be used along with the SOF_DAI_CONFIG_FLAGS_HW_PARAMS to indicate that pipeline
 * stop/pause and DAI DMA stop/pause should happen in two steps. This change is only available
 * ABI 3.20 onwards.
 */
#define SOF_DAI_CONFIG_FLAGS_2_STEP_STOP BIT(0)

#define SOF_DAI_QUIRK_IS_SET(flags, quirk) \
	(((flags & SOF_DAI_CONFIG_FLAGS_QUIRK_MASK) >> SOF_DAI_CONFIG_FLAGS_QUIRK_SHIFT) & quirk)

/** \brief Number of 'base' SSP ports available */
#define DAI_NUM_SSP_BASE        6

/** \brief Number of 'extended' SSP ports available */
#define DAI_NUM_SSP_EXT         0

/** \brief Number of SSP MCLKs available */
#define DAI_NUM_SSP_MCLK        2

#define SSP_CLOCK_XTAL_OSCILLATOR	0x0
#define SSP_CLOCK_AUDIO_CARDINAL	0x1
#define SSP_CLOCK_PLL_FIXED		0x2

/* SSP register offsets */
#define SSCR0		0x00
#define SSCR1		0x04
#define SSSR		0x08
#define SSITR		0x0C
#define SSDR		0x10
#define SSTO		0x28
#define SSPSP		0x2C
#define SSTSA		0x30
#define SSRSA		0x34
#define SSTSS		0x38
#define SSCR2		0x40

/* extern const struct dai_driver ssp_driver; */

/* SSCR0 bits */
#define SSCR0_DSIZE(x)	SET_BITS(3, 0, (x) - 1)
#define SSCR0_DSIZE_GET(x)	(((x) & MASK(3, 0)) + 1)
#define SSCR0_FRF	MASK(5, 4)
#define SSCR0_MOT	SET_BITS(5, 4, 0)
#define SSCR0_TI	SET_BITS(5, 4, 1)
#define SSCR0_NAT	SET_BITS(5, 4, 2)
#define SSCR0_PSP	SET_BITS(5, 4, 3)
#define SSCR0_ECS	BIT(6)
#define SSCR0_SSE	BIT(7)
#define SSCR0_SCR_MASK	MASK(19, 8)
#define SSCR0_SCR(x)	SET_BITS(19, 8, x)
#define SSCR0_EDSS	BIT(20)
#define SSCR0_NCS	BIT(21)
#define SSCR0_RIM	BIT(22)
#define SSCR0_TIM	BIT(23)
#define SSCR0_FRDC(x)	SET_BITS(26, 24, (x) - 1)
#define SSCR0_FRDC_GET(x) ((((x) & MASK(26, 24)) >> 24) + 1)
#define SSCR0_ACS	BIT(30)
#define SSCR0_MOD	BIT(31)

/* SSCR1 bits */
#define SSCR1_RIE	BIT(0)
#define SSCR1_TIE	BIT(1)
#define SSCR1_LBM	BIT(2)
#define SSCR1_SPO	BIT(3)
#define SSCR1_SPH	BIT(4)
#define SSCR1_MWDS	BIT(5)
#define SSCR1_TFT_MASK	MASK(9, 6)
#define SSCR1_TFT(x)	SET_BITS(9, 6, (x) - 1)
#define SSCR1_RFT_MASK	MASK(13, 10)
#define SSCR1_RFT(x)	SET_BITS(13, 10, (x) - 1)
#define SSCR1_EFWR	BIT(14)
#define SSCR1_STRF	BIT(15)
#define SSCR1_IFS	BIT(16)
#define SSCR1_PINTE	BIT(18)
#define SSCR1_TINTE	BIT(19)
#define SSCR1_RSRE	BIT(20)
#define SSCR1_TSRE	BIT(21)
#define SSCR1_TRAIL	BIT(22)
#define SSCR1_RWOT	BIT(23)
#define SSCR1_SFRMDIR	BIT(24)
#define SSCR1_SCLKDIR	BIT(25)
#define SSCR1_ECRB	BIT(26)
#define SSCR1_ECRA	BIT(27)
#define SSCR1_SCFR	BIT(28)
#define SSCR1_EBCEI	BIT(29)
#define SSCR1_TTE	BIT(30)
#define SSCR1_TTELP	BIT(31)

#define SSCR2_TURM1		BIT(1)
#define SSCR2_PSPSRWFDFD	BIT(3)
#define SSCR2_PSPSTWFDFD	BIT(4)
#define SSCR2_SDFD		BIT(14)
#define SSCR2_SDPM		BIT(16)
#define SSCR2_LJDFD		BIT(17)
#define SSCR2_MMRATF		BIT(18)
#define SSCR2_SMTATF		BIT(19)

/* SSR bits */
#define SSSR_TNF	BIT(2)
#define SSSR_RNE	BIT(3)
#define SSSR_BSY	BIT(4)
#define SSSR_TFS	BIT(5)
#define SSSR_RFS	BIT(6)
#define SSSR_ROR	BIT(7)
#define SSSR_TUR	BIT(21)

/* SSPSP bits */
#define SSPSP_SCMODE(x)		SET_BITS(1, 0, x)
#define SSPSP_SFRMP(x)		SET_BIT(2, x)
#define SSPSP_ETDS		BIT(3)
#define SSPSP_STRTDLY(x)	SET_BITS(6, 4, x)
#define SSPSP_DMYSTRT(x)	SET_BITS(8, 7, x)
#define SSPSP_SFRMDLY(x)	SET_BITS(15, 9, x)
#define SSPSP_SFRMWDTH(x)	SET_BITS(21, 16, x)
#define SSPSP_DMYSTOP(x)	SET_BITS(24, 23, x)
#define SSPSP_DMYSTOP_BITS	2
#define SSPSP_DMYSTOP_MASK	MASK(SSPSP_DMYSTOP_BITS - 1, 0)
#define SSPSP_FSRT		BIT(25)
#define SSPSP_EDMYSTOP(x)	SET_BITS(28, 26, x)

#define SSPSP2			0x44
#define SSPSP2_FEP_MASK		0xff

#define SSCR3		0x48
#define SSIOC		0x4C
#define SSP_REG_MAX	SSIOC

/* SSTSA bits */
#define SSTSA_SSTSA(x)		SET_BITS(7, 0, x)
#define SSTSA_GET(x)		((x) & MASK(7, 0))
#define SSTSA_TXEN		BIT(8)

/* SSRSA bits */
#define SSRSA_SSRSA(x)		SET_BITS(7, 0, x)
#define SSRSA_GET(x)		((x) & MASK(7, 0))
#define SSRSA_RXEN		BIT(8)

/* SSCR3 bits */
#define SSCR3_FRM_MST_EN	BIT(0)
#define SSCR3_I2S_MODE_EN	BIT(1)
#define SSCR3_I2S_FRM_POL(x)	SET_BIT(2, x)
#define SSCR3_I2S_TX_SS_FIX_EN	BIT(3)
#define SSCR3_I2S_RX_SS_FIX_EN	BIT(4)
#define SSCR3_I2S_TX_EN		BIT(9)
#define SSCR3_I2S_RX_EN		BIT(10)
#define SSCR3_CLK_EDGE_SEL	BIT(12)
#define SSCR3_STRETCH_TX	BIT(14)
#define SSCR3_STRETCH_RX	BIT(15)
#define SSCR3_MST_CLK_EN	BIT(16)
#define SSCR3_SYN_FIX_EN	BIT(17)

/* SSCR4 bits */
#define SSCR4_TOT_FRM_PRD(x)	((x) << 7)

/* SSCR5 bits */
#define SSCR5_FRM_ASRT_CLOCKS(x)	(((x) - 1) << 1)
#define SSCR5_FRM_POLARITY(x)	SET_BIT(0, x)

/* SFIFOTT bits */
#define SFIFOTT_TX(x)		((x) - 1)
#define SFIFOTT_RX(x)		(((x) - 1) << 16)

/* SFIFOL bits */
#define SFIFOL_TFL(x)		((x) & 0xFFFF)
#define SFIFOL_RFL(x)		((x) >> 16)

#define SSTSA_TSEN			BIT(8)
#define SSRSA_RSEN			BIT(8)

#define SSCR3_TFL_MASK	MASK(5, 0)
#define SSCR3_RFL_MASK	MASK(13, 8)
#define SSCR3_TFL_VAL(scr3_val)	(((scr3_val) >> 0) & MASK(5, 0))
#define SSCR3_RFL_VAL(scr3_val)	(((scr3_val) >> 8) & MASK(5, 0))
#define SSCR3_TX(x)	SET_BITS(21, 16, (x) - 1)
#define SSCR3_RX(x)	SET_BITS(29, 24, (x) - 1)

#define SSIOC_TXDPDEB	BIT(1)
#define SSIOC_SFCR	BIT(4)
#define SSIOC_SCOE	BIT(5)

/* For 8000 Hz rate one sample is transmitted within 125us */
#define SSP_MAX_SEND_TIME_PER_SAMPLE 125

/* SSP flush retry counts maximum */
#define SSP_RX_FLUSH_RETRY_MAX	16

#define SSP_CLK_MCLK_ES_REQ	BIT(0)
#define SSP_CLK_MCLK_ACTIVE	BIT(1)
#define SSP_CLK_BCLK_ES_REQ	BIT(2)
#define SSP_CLK_BCLK_ACTIVE	BIT(3)

#define I2SLCTL			0x71C04
#define I2SLCTL_SPA(x)		BIT(0 + x)
#define I2SLCTL_CPA(x)		BIT(8 + x)

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

#if CONFIG_INTEL_MN
/** \brief BCLKs can be driven by multiple sources - M/N or XTAL directly.
 *	   Even in the case of M/N, the actual clock source can be XTAL,
 *	   Audio cardinal clock (24.576) or 96 MHz PLL.
 *	   The MN block is not really the source of clocks, but rather
 *	   an intermediate component.
 *	   Input for source is shared by all outputs coming from that source
 *	   and once it's in use, it can be adjusted only with dividers.
 *	   In order to change input, the source should not be in use, that's why
 *	   it's necessary to keep track of BCLKs sources to know when it's safe
 *	   to change shared input clock.
 */
enum bclk_source {
	MN_BCLK_SOURCE_NONE = 0, /**< port is not using any clock */
	MN_BCLK_SOURCE_MN, /**< port is using clock driven by M/N */
	MN_BCLK_SOURCE_XTAL, /**< port is using XTAL directly */
};
#endif

struct mn {
	uint32_t base;
	/**< keep track of which MCLKs are in use to know when it's safe to
	 * change shared clock
	 */
	int mclk_sources_ref[DAI_NUM_SSP_MCLK];
	int mclk_rate[DAI_NUM_SSP_MCLK];
	int mclk_source_clock;

#if CONFIG_INTEL_MN
	enum bclk_source bclk_sources[(DAI_NUM_SSP_BASE + DAI_NUM_SSP_EXT)];
	int bclk_source_mn_clock;
#endif

	struct k_spinlock lock; /**< lock mechanism */
};

struct freq_table {
        uint32_t freq;
        uint32_t ticks_per_msec;
};

struct dai_plat_fifo_data {
	uint32_t offset;
	uint32_t width;
	uint32_t depth;
	uint32_t watermark;
	uint32_t handshake;
};

struct dai_plat_data {
	uint32_t base;
	int irq;
	const char *irq_name;
	uint32_t flags;
	struct dai_plat_fifo_data fifo[2];
	struct mn *mn_inst;
	struct freq_table *ftable;
	uint32_t *fsources;
};

struct dai {
	uint32_t index;		/**< index */
	struct k_spinlock lock;	/**< locking mechanism */
	int sref;		/**< simple ref counter, guarded by lock */
	struct dai_plat_data plat_data;
	void* priv_data;
};

extern const struct freq_table *ssp_freq_z;
extern const uint32_t *ssp_freq_sources_z;

#endif
