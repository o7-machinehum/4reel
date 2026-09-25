/* SPDX-License-Identifier: Apache-2.0
 * SAD/MMZ ABI subset from Rockchip's rk_comm_ive.h, rk_ive.h,
 * rk_mpi_ive.h and rk_mpi_mmz.h, Copyright 2020-2021 Rockchip Electronics.
 * SDK revision: 824b817f889c2cbff1d48fcdb18ab494a68f69d1 (LuckfoxTECH/luckfox-pico).
 */
#ifndef FOURREEL_RVE_ABI_H
#define FOURREEL_RVE_ABI_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint64_t physical[3];
    uint64_t virtual_address[3];
    uint32_t stride[3];
    uint32_t width, height;
    int32_t type, reserved;
} RveImage;

typedef struct {
    int32_t mode, output_mode, output_bits;
    uint16_t threshold;
    uint8_t minimum, maximum;
} RveSadControl;

_Static_assert(sizeof(RveImage) == 80, "RVE image ABI");
_Static_assert(offsetof(RveImage, width) == 60, "RVE image ABI offset");
_Static_assert(sizeof(RveSadControl) == 16, "RVE SAD ABI");

int32_t RK_MPI_IVE_Init(void);
int32_t RK_MPI_IVE_Deinit(void);
int32_t RK_MPI_IVE_SAD(int32_t *handle, RveImage *left, RveImage *right,
                     RveImage *sad, RveImage *threshold,
                     RveSadControl *control, bool instant);

int32_t RK_MPI_MMZ_Alloc(void **block, uint32_t size, uint32_t flags);
int32_t RK_MPI_MMZ_Free(void *block);
uint64_t RK_MPI_MB_Handle2PhysAddr(void *block);
void *RK_MPI_MB_Handle2VirAddr(void *block);
void *RK_MPI_MB_VirAddr2Handle(void *address);
uint64_t RK_MPI_MMZ_GetSize(void *block);
int32_t RK_MPI_MMZ_Handle2Fd(void *block);
int32_t RK_MPI_MMZ_FlushCacheVaddrStart(void *address, uint32_t size, uint32_t flags);
int32_t RK_MPI_MMZ_FlushCacheVaddrEnd(void *address, uint32_t size, uint32_t flags);
#endif
