/*
 * Goldeneye-only helpers for the Quectel EG800Q modem driver.
 *
 * Copyright (c) 2026 Voi Technology AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_CELLULAR_CV_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_CELLULAR_CV_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Change the APN of PDP context 2 (CV over USB ECM) at runtime.
 *
 * Redefines context 2 and re-activates it without restarting the modem or
 * the PPP link on context 1. The new address is read back into the
 * CELLULAR_MODEM_INFO_PDP_ADDR_2 info value.
 *
 * Only valid while the modem is connected (carrier on).
 *
 * Blocks for up to about 75 s (40 s for the busy lock, up to 5 s of retries and
 * 30 s for the script). Do not call it from the system workqueue or an ISR.
 *
 * A failed switch keeps the last APN that worked as the recovery target, so the
 * periodic check re-activates context 2 with it.
 *
 * @param dev Modem device.
 * @param apn Zero-terminated APN string (max 31 characters).
 *
 * @retval 0 on success.
 * @retval -EINVAL if the APN is empty or too long.
 * @retval -EALREADY if the APN equals the one in use and context 2 is up.
 * @retval -ENOTCONN if the modem is not connected.
 * @retval -ETIMEDOUT if the AT script did not finish in time.
 * @retval -EIO if an AT command in the script failed.
 */
int modem_cellular_goldeneye_set_cv_apn(const struct device *dev, const char *apn);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_MODEM_CELLULAR_CV_H_ */
