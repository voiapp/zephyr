/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GSM_PPP_H_
#define GSM_PPP_H_

#define GSM_MODEM_DEVICE_NAME "modem_gsm"

struct device;
void gsm_ppp_start(const struct device *device);
void gsm_ppp_stop(const struct device *device);

#endif /* GSM_PPP_H_ */
