/* Host-only SDK replacement: exercise the real RVE matcher using CPU SAD. */
#include "../rve_abi.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

struct Block { uint32_t size; uint8_t *data; };

int __wrap_access(const char *path, int mode)
{
    (void)mode;
    assert(!strcmp(path, "/dev/rve"));
    return 0;
}

int32_t RK_MPI_IVE_Init(void) { return 0; }
int32_t RK_MPI_IVE_Deinit(void) { return 0; }
int32_t RK_MPI_MMZ_Alloc(void **block, uint32_t size, uint32_t flags)
{
    (void)flags;
    struct Block *b = malloc(sizeof(*b));
    assert(b);
    b->size = size;
    b->data = calloc(1, size);
    assert(b->data);
    *block = b;
    return 0;
}

int32_t RK_MPI_MMZ_Free(void *block)
{
    struct Block *b = block;
    free(b->data);
    free(b);
    return 0;
}

uint64_t RK_MPI_MB_Handle2PhysAddr(void *block)
{
    return (uintptr_t)((struct Block *)block)->data;
}
void *RK_MPI_MB_Handle2VirAddr(void *block) { return ((struct Block *)block)->data; }
uint64_t RK_MPI_MMZ_GetSize(void *block) { return ((struct Block *)block)->size; }
int32_t RK_MPI_MMZ_FlushCacheVaddrStart(void *address, uint32_t size, uint32_t flags)
{
    (void)address; (void)size; (void)flags;
    return 0;
}
int32_t RK_MPI_MMZ_FlushCacheVaddrEnd(void *address, uint32_t size, uint32_t flags)
{
    return RK_MPI_MMZ_FlushCacheVaddrStart(address, size, flags);
}

int32_t RK_MPI_IVE_SAD(int32_t *handle, RveImage *left, RveImage *right,
                     RveImage *sad, RveImage *threshold,
                     RveSadControl *control, bool instant)
{
    (void)handle;
    assert(!threshold && instant);
    assert(control->mode == 0 && control->output_mode == 0 && control->output_bits == 1);
    const uint8_t *a = (void *)(uintptr_t)left->virtual_address[0];
    const uint8_t *b = (void *)(uintptr_t)right->virtual_address[0];
    uint16_t *out = (void *)(uintptr_t)sad->virtual_address[0];
    for (unsigned y = 0; y < sad->height; ++y)
        for (unsigned x = 0; x < sad->width; ++x) {
            unsigned cost = 0;
            for (unsigned dy = 0; dy < 4; ++dy)
                for (unsigned dx = 0; dx < 4; ++dx)
                    cost += abs(a[(y * 4 + dy) * left->stride[0] + x * 4 + dx] -
                                b[(y * 4 + dy) * right->stride[0] + x * 4 + dx]);
            out[y * sad->stride[0] + x] = cost;
        }
    return 0;
}
