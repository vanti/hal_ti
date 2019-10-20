#include <stdbool.h>

#include <zephyr.h>

#include "ti/devices/cc13x2_cc26x2/driverlib/interrupt.h"

#include "ti/drivers/dpl/ClockP.h"
#include "ti/drivers/dpl/HwiP.h"
#include "ti/drivers/dpl/SemaphoreP.h"
#include "ti/drivers/dpl/SwiP.h"

#include "ti/drivers/power/PowerCC26X2.h"
#define DeviceFamily_CC13X2
#include "ti/drivers/rf/RF.h"

#define STUB(fmt, args...) printk("%s: %s(): %d: STUB: " fmt "\n", __FILE__, __func__, __LINE__, ##args)

uint32_t ClockP_getTimeout(ClockP_Handle handle) {
	STUB("");
	return 0;
}
bool ClockP_isActive(ClockP_Handle handle) {
	STUB("");
	return false;
}
uint32_t ClockP_tickPeriod = -1;
void HwiP_post(int interruptNum) {
	STUB("");
}
void HwiP_setFunc(HwiP_Handle hwiP, HwiP_Fxn fxn, uintptr_t arg) {
	STUB("");
}
const PowerCC26X2_Config PowerCC26X2_config;
const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = INT_PRI_LEVEL7,  // Lowest HWI priority:  INT_PRI_LEVEL7
                                           // Highest HWI priority: INT_PRI_LEVEL1

    .swiPriority        = 0,               // Lowest SWI priority:  0
                                           // Highest SWI priority: Swi.numPriorities - 1

    .xoscHfAlwaysNeeded = true             // Power driver always starts XOSC-HF:       true
                                           // RF driver will request XOSC-HF if needed: false
};

extern SemaphoreP_Handle SemaphoreP_constructBinary(SemaphoreP_Struct *handle, unsigned int count) {
	STUB("");
	return NULL;
}
void SemaphoreP_post(SemaphoreP_Handle handle) {
	STUB("");
}
SwiP_Handle SwiP_construct(SwiP_Struct *swiP, SwiP_Fxn swiFxn, SwiP_Params *params) {
	STUB("");
	return NULL;
}
uint32_t SwiP_getTrigger(void) {
	STUB("");
}
void SwiP_or(SwiP_Handle handle, uint32_t mask) {
	STUB("");
}
void SwiP_Params_init(SwiP_Params *params) {
	STUB("");
}

