#include <stdlib.h>
#include "tag16h5.h"

static uint64_t codedata[25] = {
   0x0000000000009765UL,
   0x0000000000009d2aUL,
   0x000000000000a2efUL,
   0x000000000000a8b4UL,
   0x000000000000ba03UL,
   0x000000000000bfc8UL,
   0x000000000000f3b5UL,
   0x0000000000002d67UL,
   0x000000000000447bUL,
   0x00000000000055caUL,
   0x0000000000003c92UL,
   0x00000000000098e2UL,
   0x000000000000ed49UL,
   0x000000000000384aUL,
   0x00000000000071fcUL,
   0x000000000000b691UL,
   0x00000000000091cfUL,
   0x000000000000aa60UL,
   0x000000000000435cUL,
   0x000000000000c768UL,
   0x000000000000af35UL,
   0x000000000000e6c3UL,
   0x00000000000064f0UL,
   0x000000000000e5b9UL,
   0x000000000000dfacUL,
};
apriltag_family_t *tag16h5_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tag16h5");
   tf->h = 5;
   tf->ncodes = 25;
   tf->codes = codedata;
   tf->nbits = 16;
   tf->bit_x = calloc(16, sizeof(uint32_t));
   tf->bit_y = calloc(16, sizeof(uint32_t));
   tf->bit_x[0] = 1;
   tf->bit_y[0] = 1;
   tf->bit_x[1] = 2;
   tf->bit_y[1] = 1;
   tf->bit_x[2] = 3;
   tf->bit_y[2] = 1;
   tf->bit_x[3] = 2;
   tf->bit_y[3] = 2;
   tf->bit_x[4] = 4;
   tf->bit_y[4] = 1;
   tf->bit_x[5] = 4;
   tf->bit_y[5] = 2;
   tf->bit_x[6] = 4;
   tf->bit_y[6] = 3;
   tf->bit_x[7] = 3;
   tf->bit_y[7] = 2;
   tf->bit_x[8] = 4;
   tf->bit_y[8] = 4;
   tf->bit_x[9] = 3;
   tf->bit_y[9] = 4;
   tf->bit_x[10] = 2;
   tf->bit_y[10] = 4;
   tf->bit_x[11] = 3;
   tf->bit_y[11] = 3;
   tf->bit_x[12] = 1;
   tf->bit_y[12] = 4;
   tf->bit_x[13] = 1;
   tf->bit_y[13] = 3;
   tf->bit_x[14] = 1;
   tf->bit_y[14] = 2;
   tf->bit_x[15] = 2;
   tf->bit_y[15] = 3;
   tf->width_at_border = 6;
   tf->total_width = 8;
   tf->reversed_border = true;
   return tf;
}

void tag16h5_destroy(apriltag_family_t *tf)
{
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
