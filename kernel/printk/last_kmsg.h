#ifndef __LAST_KMSG_HEAD__
#define __LAST_KMSG_HEAD__

#define LAST_VER "4.0"

#include "last_shared.h"

#define MAX(a, b) ((a) >= (b) ? (a) : (b))
void* get_mem_last_info(void);
uint32 calc_crc32(const uint8 *data_in, const uint32 nbytes_in);
#endif
