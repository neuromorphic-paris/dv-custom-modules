// Copyright 2020 iniVation AG
#ifndef AEDAT4_CONVERT_H
#define AEDAT4_CONVERT_H

#include "dv-sdk/module.h"

#include <libcaer/events/common.h>

#ifdef __cplusplus
extern "C" {
#endif

void dvConvertToAedat4(caerEventPacketHeaderConst oldPacket, dvModuleData moduleData);

#ifdef __cplusplus
}
#endif

#endif // AEDAT4_CONVERT_H
