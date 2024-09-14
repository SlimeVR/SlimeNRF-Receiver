/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RETAINED_H_
#define RETAINED_H_

#include <inttypes.h>

int ram_range_retain(const void *ptr, size_t len, bool enable);

#endif /* RETAINED_H_ */
