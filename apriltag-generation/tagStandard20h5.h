#ifndef _TAGStandard20H5
#define _TAGStandard20H5

#include "apriltag.h"

#ifdef __cplusplus
extern "C" {
#endif

apriltag_family_t *tagStandard20h5_create();
void tagStandard20h5_destroy(apriltag_family_t *tf);

#ifdef __cplusplus
}
#endif

#endif
