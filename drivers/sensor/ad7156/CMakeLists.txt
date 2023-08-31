# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_AD7156 ad7156.c)
zephyr_library_sources_ifdef(CONFIG_AD7156_TRIGGER ad7156_trigger.c)
