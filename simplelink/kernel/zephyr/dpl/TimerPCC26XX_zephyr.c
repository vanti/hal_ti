/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== TimerPCC26XX_zephyr.c ========
 */

#include <zephyr.h>
#include <kernel.h>
#include <timeout_q.h>

#include <stdlib.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aon_rtc.h)

#include <ti/drivers/dpl/HwiP.h>
#include "TimerP.h"

#define MAX_SKIP  (0x7E9000000000)    /* 32400 seconds (9 hours) */
#define TIMER_FREQ 65536 /* 2x timer freq since rtc counter increments by 2 */

/* NOTE: Ensure TimerP_STRUCT_SIZE TimerP.h matches sizeof(TimerP_Obj) */
typedef struct _TimerP_Obj {
    struct _timeout         timeout;
    uint32_t                period;  /* must match period of RTC */
    uintptr_t               arg;
    HwiP_Fxn                tickFxn;
    uint64_t                period64;
    uint64_t                prevThreshold;
    uint64_t                nextThreshold;
} TimerP_Obj;

static TimerP_Params TimerP_defaultParams = {
    .startMode = TimerP_StartMode_AUTO,
    .arg = 0,
    .period = 10,
};


/*
 *  ======== TimerP_Params_init ========
 */
void TimerP_Params_init(TimerP_Params *params)
{
    /* structure copy */
    *params = TimerP_defaultParams;
}

/*
 *  ======== TimerP_construct ========
 */
TimerP_Handle TimerP_construct(TimerP_Struct *handle, TimerP_Fxn timerFxn, TimerP_Params *params)
{
    TimerP_Obj *obj = (TimerP_Obj *)handle;

    if (handle != NULL) {
        if (params == NULL) {
            params = (TimerP_Params *)&TimerP_defaultParams;
        }

        obj->arg = params->arg;
        obj->tickFxn = timerFxn;
        obj->period = (0x100000000UL * params->period) / 1000000U;
        obj->period64 = obj->period;   /* period in timer counts */
        obj->prevThreshold = obj->period;
        obj->nextThreshold = obj->period;
        z_init_timeout(&obj->timeout);
    }

    return ((TimerP_Handle)handle);
}

/*
 *  ======== TimerP_create ========
 */
TimerP_Handle TimerP_create(TimerP_Fxn timerFxn, TimerP_Params *params)
{
    TimerP_Handle handle;

    handle = (TimerP_Handle)malloc(sizeof(TimerP_Obj));

    /* TimerP_construct will check handle for NULL, no need here */
    handle = TimerP_construct((TimerP_Struct *)handle, timerFxn, params);

    return (handle);
}

/*
 *  ======== TimerP_getMaxTicks ========
 */
uint32_t TimerP_getMaxTicks(TimerP_Handle handle)
{
    TimerP_Obj *obj = (TimerP_Obj *)handle;
    uint32_t ticks;
    uint64_t temp;

    temp = (uint64_t)(MAX_SKIP) / obj->period64;

    /* clip value to half of Clock tick count limit of signed 32-bits
     * to ensure z_add_timeout() can handle it when adding elapsed time
     * since the last tick.
     */
    if (temp > (0x7FFFFFFF / 2)) {
        ticks = 0x7FFFFFFF / 2;
    }
    else {
        ticks = (uint32_t)temp;
    }

    return (ticks);
}

static void timer_expiration_handler(struct _timeout *t)
{
    TimerP_Obj * obj = CONTAINER_OF(t, TimerP_Obj, timeout);
    obj->tickFxn(obj->arg);
}

/*
 *  ======== TimerP_setNextTick ========
 */
void TimerP_setNextTick(TimerP_Handle handle, uint32_t ticks)
{
    TimerP_Obj *obj = (TimerP_Obj *)handle;
    
    /*
     * Use Zephyr internal timeout API to add a timeout for the
     * next tick on this TimerP instance. Ideally we should use
     * the public API k_timer, but unfortunately it only has
     * millisecond-level accuracy.
     */
    z_abort_timeout(&obj->timeout);
    z_add_timeout(&obj->timeout, timer_expiration_handler, ticks);
}

/*
 *  ======== TimerP_setPeriod ========
 *
 * 1. Stop timer
 * 2. Set period value in timer obj
 *
 */
void TimerP_setPeriod(TimerP_Handle handle, uint32_t period)
{
    TimerP_Obj *obj = (TimerP_Obj *)handle;
    obj->period = period;
}

/*
 *  ======== TimerP_getCount64 ========
 */
uint64_t TimerP_getCount64(TimerP_Handle handle)
{
    return(AONRTCCurrent64BitValueGet());
}

/*
 *  ======== TimerP_getFreq ========
 */
void TimerP_getFreq(TimerP_Handle handle, TimerP_FreqHz *freq)
{
    freq->lo = TIMER_FREQ;
    freq->hi = 0;
}

/*
 *  ======== TimerP_getCurrentTick ========
 *  Used by the Clock module for TickMode_DYNAMIC to query the corresponding
 *  Clock tick, as derived from the current timer count.
 */
uint32_t TimerP_getCurrentTick(TimerP_Handle handle, bool saveFlag)
{
    return (z_tick_get_32());
}
