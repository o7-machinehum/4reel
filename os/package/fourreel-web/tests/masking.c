#include "../depth.h"
#include "../rve_sad.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define PIXELS (DEPTH_W * DEPTH_H)
static uint8_t left[PIXELS], right[PIXELS], valid[2][PIXELS], visual[PIXELS];
static uint16_t mm[PIXELS];

int main(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        unsigned random = 0x51eab39;
        for (int i = 0; i < PIXELS; ++i) {
            random ^= random << 13; random ^= random >> 17; random ^= random << 5;
            left[i] = (uint8_t)random;
            random ^= random << 13; random ^= random >> 17; random ^= random << 5;
            right[i] = (uint8_t)random;
        }
        int vertical = direction * 2;
        /* Keep the census centre near mid-grey so unrelated random windows
         * are distinguishable; a nearly extreme centre is ambiguous. */
        left[41 * DEPTH_W + 81] = 128;
        /* Two equally good image matches. One is in an invalid right window.
         * Rejecting only the winning output after matching cannot recover
         * the valid candidate: the invalid cost also destroys uniqueness. */
        for (int disparity = 8; disparity <= 24; disparity += 16)
            for (int y = 36; y < 48; ++y)
                for (int x = 76; x < 88; ++x)
                    right[(y + vertical) * DEPTH_W + x - direction * disparity] =
                        left[y * DEPTH_W + x];
        struct Calibration cal = {.scale_px_mm = direction * 24000,
                                   .vertical_px = vertical, .valid = 1};
        unsigned target = 41 * DEPTH_W + 81;
        memset(valid, 255, sizeof(valid));
        depth_compute_masked(left, right, &cal, valid[0], valid[1], visual, mm);
        assert(!visual[target] && !mm[target]);
        valid[1][(41 + vertical) * DEPTH_W + 81 - direction * 8] = 0;
        depth_compute_masked(left, right, &cal, valid[0], valid[1], visual, mm);
        assert(visual[target] && mm[target] == 1000);
        valid[0][target] = 0;
        depth_compute_masked(left, right, &cal, valid[0], valid[1], visual, mm);
        assert(!visual[target] && !mm[target]);
        memset(valid[1], 0, PIXELS);
        depth_compute_masked(left, right, &cal, valid[0], valid[1], visual, mm);
        for (unsigned i = 0; i < PIXELS; ++i)
            assert(!visual[i] && !mm[i]);
    }
    rve_sad_close();
    puts("PASS: invalid shifted windows cannot win or affect uniqueness, signed disparity and full-window masking");
    return 0;
}
