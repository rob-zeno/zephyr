/*
 * Copyright 2023, 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NXP_S32_S32K_SOC_H_
#define _NXP_S32_S32K_SOC_H_

#if CONFIG_SOC_S32K344
#include <S32K344.h>
#endif

#if CONFIG_SOC_S32K311
#include <S32K311.h>
#endif

#if CONFIG_SOC_S32K312
#include <S32K312.h>
#endif

#include <core_cm7.h>

#if defined(CONFIG_CMSIS_RTOS_V2)
#include <cmsis_rtos_v2_adapt.h>
#include <soc_common.h>
#endif

#endif /* _NXP_S32_S32K_SOC_H_ */
