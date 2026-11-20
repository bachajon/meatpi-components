/**
 * @file isotp_test_kconfig.h
 * @brief The Kconfig values src/isotp_config.h reads, for the host test:
 *        this project does not load the component's Kconfig. They are the
 *        values of the WiCAN PRO firmware's tracked sdkconfig. The block
 *        size and the separation time are defaults no session of
 *        can_isotp_esp uses (it always sets its own, and so does every
 *        link of the test); frame padding is off in the library (the
 *        component pads at run time, in esp_isotp.c).
 */
#pragma once

#define CONFIG_ISO_TP_DEFAULT_BLOCK_SIZE          8
#define CONFIG_ISO_TP_DEFAULT_ST_MIN_US           1000
#define CONFIG_ISO_TP_MAX_WFT_NUMBER              1
#define CONFIG_ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US 1000000
