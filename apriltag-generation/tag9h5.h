#ifndef _TAG9H5
#define _TAG9H5

#include "apriltag.h"

#ifdef __cplusplus
extern "C" {
#endif

apriltag_family_t *tag9h5_create();
void tag9h5_destroy(apriltag_family_t *tf);

#ifdef __cplusplus
}
#endif

#endif
