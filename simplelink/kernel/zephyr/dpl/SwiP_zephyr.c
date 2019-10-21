/*
 * Copyright (c) 2019, Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <ti/drivers/dpl/SwiP.h>

#include "stubs.h"

/*
 *  ======== SwiP_disable ========
 */
uintptr_t SwiP_disable(void)
{
    k_sched_lock();

    return 0;
}

/*
 *  ======== SwiP_restore ========
 */
void SwiP_restore(uintptr_t key)
{
    ARG_UNUSED(key);
    k_sched_unlock();
}

SwiP_Handle SwiP_construct(SwiP_Struct *swiP, SwiP_Fxn swiFxn, SwiP_Params *params) {
    ARG_UNUSED(swiP);
    ARG_UNUSED(swiFxn);
    ARG_UNUSED(params);
    STUB("");
    return NULL;
}
uint32_t SwiP_getTrigger(void) {
    STUB("");
    return 0;
}
void SwiP_or(SwiP_Handle handle, uint32_t mask) {
    ARG_UNUSED(handle);
    ARG_UNUSED(mask);
    STUB("");
}
void SwiP_Params_init(SwiP_Params *params) {
    ARG_UNUSED(params);
    STUB("");
}

