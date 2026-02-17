#ifndef BBB_SHARED_MEM_H_
#define BBB_SHARED_MEM_H_

#include <stdint.h>
#include "rpmsg_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BBB_SHARED_MEM_VERSION 1u

typedef struct __attribute__((packed)) {
    uint32_t version;
    uint32_t heartbeat;
    bbb_enc_snapshot_t encoder_snapshot;
    int16_t commanded_left;
    int16_t commanded_right;
    uint8_t estop_asserted;
    uint8_t armed;
    uint16_t reserved;
} bbb_shared_mem_t;

#ifdef __cplusplus
}
#endif

#endif
