/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <zephyr.h>

#include <toolchain.h>
#include <sys/printk.h>
#include <sys/util.h>

#include <device.h>
#include <drivers/dai.h>

#include <soc.h>

static const struct device *dev_dai_ssp;

/* sof ssp bespoke data */
struct sof_dai_ssp_params {
	uint32_t reserved0;
	uint16_t reserved1;
	uint16_t mclk_id;

	uint32_t mclk_rate;
	uint32_t fsync_rate;
	uint32_t bclk_rate;

	uint32_t tdm_slots;
	uint32_t rx_slots;
	uint32_t tx_slots;

	uint32_t sample_valid_bits;
	uint16_t tdm_slot_width;
	uint16_t reserved2;

	uint32_t mclk_direction;

	uint16_t frame_pulse_width;
	uint16_t tdm_per_slot_padding_flag;
	uint32_t clks_control;
	uint32_t quirks;
	uint32_t bclk_delay;
} __attribute__((packed, aligned(4)));


void test_adsp_ssp_config_set(void)
{
	int ret;

	/* generic config */
	struct dai_config config = {
		.type = DAI_INTEL_SSP,
		.dai_index = 0,
		.channels = 2,
		.rate = 48000,
		.format = 16385,
		.options = 0,
		.word_size = 0,
		.block_size = 0,
	};

	/* bespoke config */
	struct sof_dai_ssp_params ssp_config = {
		.mclk_id = 0,
		.mclk_rate = 24576000,
		.fsync_rate = 48000,
		.bclk_rate = 3072000,
		.tdm_slots = 2,
		.rx_slots = 3,
		.tx_slots = 3,
		.sample_valid_bits = 32,
		.tdm_slot_width = 32,
		.mclk_direction = 0,
		.frame_pulse_width = 0,
		.tdm_per_slot_padding_flag = 0,
		.clks_control = 0,
		.quirks = 64,
		.bclk_delay = 0,
	};

	ret = dai_config_set(dev_dai_ssp, &config, &ssp_config);

	zassert_equal(ret, TC_PASS, NULL);
}

void test_adsp_ssp_probe(void)
{
	int ret;
	
	ret = dai_probe(dev_dai_ssp);

	zassert_equal(ret, TC_PASS, NULL);
}

void test_main(void)
{
  	dev_dai_ssp = device_get_binding("SSP_0");

	if (dev_dai_ssp != NULL) {
		k_object_access_grant(dev_dai_ssp, k_current_get());
	}

	zassert_not_null(dev_dai_ssp, "device SSP_0 not found");

	ztest_test_suite(adsp_ssp,
			 ztest_unit_test(test_adsp_ssp_probe),
			 ztest_unit_test(test_adsp_ssp_config_set));

	ztest_run_test_suite(adsp_ssp);

	printk("humppa");
}
