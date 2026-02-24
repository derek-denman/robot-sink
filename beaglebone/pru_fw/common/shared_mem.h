#ifndef BBB_SHARED_MEM_H_
#define BBB_SHARED_MEM_H_

#include <stdint.h>

#include "uio_shared_layout.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BBB_SHARED_MEM_VERSION BBB_UIO_LAYOUT_VERSION
typedef bbb_uio_shared_layout_t bbb_shared_mem_t;

#ifdef __cplusplus
}
#endif

#endif
