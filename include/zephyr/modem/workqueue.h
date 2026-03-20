/*
 * Copyright (c) 2025 Embeint Pty Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Modem workqueue API
 *
 * Re-exports the modem workqueue API for drivers that need to schedule
 * work on the dedicated modem workqueue (when CONFIG_MODEM_DEDICATED_WORKQUEUE).
 */

#include "../../../subsys/modem/modem_workqueue.h"
