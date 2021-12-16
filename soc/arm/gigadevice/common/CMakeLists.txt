# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_linker_sources_ifdef(CONFIG_GD32_CCM SECTIONS ccm.ld)
