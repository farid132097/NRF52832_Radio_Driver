

#ifndef  _CONFIG_H_
#define  _CONFIG_H_
#include "nrf.h"

/*
 * Device address for this node (TX)
 * Device address must not be    (0x0000000000000000ULL) or (0xFFFFFFFFFFFFFFFFULL)
 */
#define  OWN_DEV_ADDRESS_QWORD   (0x0000000000000010ULL)




/*
 * Capacitor Low voltage indicator in mV
 * Activated below this threshold
 */
#define  CAP_LOW_VOLTAGE_IND_THRESHOLD (5600UL)

#endif

