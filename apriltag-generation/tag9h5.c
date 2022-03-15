#include <stdlib.h>
#include "tag9h5.h"

static uint64_t codedata[0] = {
};
apriltag_family_t *tag9h5_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tag9h5");
   tf->h = 5;
   tf->ncodes = 0;
   tf->codes = codedata;
   tf->nbits = 9;
   tf->bit_x = calloc(9, sizeof(uint32_t));
   tf->bit_y = calloc(9, sizeof(uint32_t));
   tf->bit_x[0] = 1;
   tf->bit_y[0] = 1;
   tf->bit_x[1] = 2;
   tf->bit_y[1] = 1;
   tf->bit_x[2] = 3;
   tf->bit_y[2] = 1;
   tf->bit_x[3] = 3;
   tf->bit_y[3] = 2;
   tf->bit_x[4] = 3;
   tf->bit_y[4] = 3;
   tf->bit_x[5] = 2;
   tf->bit_y[5] = 3;
   tf->bit_x[6] = 1;
   tf->bit_y[6] = 3;
   tf->bit_x[7] = 1;
   tf->bit_y[7] = 2;
   tf->bit_x[8] = 2;
   tf->bit_y[8] = 2;
   tf->width_at_border = 5;
   tf->total_width = 7;
   tf->reversed_border = false;
   return tf;
}

void tag9h5_destroy(apriltag_family_t *tf)
{
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
