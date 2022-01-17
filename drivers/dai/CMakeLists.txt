# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_DAI_SOF_SSP		mn.c ssp.c)
zephyr_library_compile_options(-std=gnu99)
