/* Minimal Rockchip MMZ adapter using CMA DMA buffers on the musl rootfs. */
#define _GNU_SOURCE
#include "rve_abi.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

/* Vendor kernel extension, absent from the toolchain's upstream headers. */
struct RveHeapPhysical { uint64_t address; uint32_t fd; };
#define RVE_HEAP_GET_PHYS _IOWR('H', 1, struct RveHeapPhysical)

struct Block {
    struct Block *next;
    void *address;
    uint64_t physical;
    size_t size;
    int fd;
};
static struct Block *blocks;

int32_t RK_MPI_MMZ_Alloc(void **result, uint32_t size, uint32_t flags)
{
    (void)flags; /* All allocations are contiguous, cached, explicitly synced. */
    if (!result || !size || size > 8 * 1024 * 1024) {
        errno = EINVAL;
        return -1;
    }
    *result = NULL;
    struct Block *b = calloc(1, sizeof(*b));
    if (!b)
        return -1;
    b->fd = -1;
    DIR *directory = opendir("/dev/dma_heap");
    if (!directory) {
        free(b);
        return -1;
    }
    struct dirent *entry;
    while ((entry = readdir(directory))) {
        if (entry->d_name[0] == '.')
            continue;
        int heap = openat(dirfd(directory), entry->d_name, O_RDWR | O_CLOEXEC);
        if (heap < 0)
            continue;
        struct dma_heap_allocation_data allocation = {
            .len = (size + 4095U) & ~4095U,
            .fd_flags = O_RDWR | O_CLOEXEC,
        };
        if (ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &allocation) == 0) {
            struct RveHeapPhysical physical = {.fd = allocation.fd};
            if (ioctl(heap, RVE_HEAP_GET_PHYS, &physical) == 0 &&
                physical.address && physical.address <= UINT32_MAX - allocation.len) {
                b->fd = allocation.fd;
                b->physical = physical.address;
                b->size = allocation.len;
            } else {
                close(allocation.fd);
            }
        }
        close(heap);
        if (b->fd >= 0)
            break;
    }
    closedir(directory);
    if (b->fd < 0) {
        free(b);
        errno = ENODEV;
        return -1;
    }
    b->address = mmap(NULL, b->size, PROT_READ | PROT_WRITE, MAP_SHARED, b->fd, 0);
    if (b->address == MAP_FAILED) {
        int saved = errno;
        close(b->fd);
        free(b);
        errno = saved;
        return -1;
    }
    b->next = blocks;
    blocks = b;
    *result = b;
    return 0;
}

int32_t RK_MPI_MMZ_Free(void *block)
{
    struct Block **cursor = &blocks;
    while (*cursor && *cursor != block)
        cursor = &(*cursor)->next;
    if (!*cursor) {
        errno = EINVAL;
        return -1;
    }
    struct Block *b = *cursor;
    *cursor = b->next;
    munmap(b->address, b->size);
    close(b->fd);
    free(b);
    return 0;
}

void *RK_MPI_MB_VirAddr2Handle(void *address)
{
    uintptr_t p = (uintptr_t)address;
    for (struct Block *b = blocks; b; b = b->next)
        if (p >= (uintptr_t)b->address && p - (uintptr_t)b->address < b->size)
            return b;
    return NULL;
}

uint64_t RK_MPI_MB_Handle2PhysAddr(void *block)
{ return block ? ((struct Block *)block)->physical : 0; }
void *RK_MPI_MB_Handle2VirAddr(void *block)
{ return block ? ((struct Block *)block)->address : NULL; }
uint64_t RK_MPI_MMZ_GetSize(void *block)
{ return block ? ((struct Block *)block)->size : 0; }
int32_t RK_MPI_MMZ_Handle2Fd(void *block)
{ return block ? ((struct Block *)block)->fd : -1; }

static int sync_memory(void *address, uint32_t size, uint32_t flags, int end)
{
    struct Block *b = RK_MPI_MB_VirAddr2Handle(address);
    if (!b || size > b->size - ((uintptr_t)address - (uintptr_t)b->address)) {
        errno = EINVAL;
        return -1;
    }
    struct dma_buf_sync sync = {
        .flags = (end ? DMA_BUF_SYNC_END : DMA_BUF_SYNC_START) |
            (flags == 0 ? DMA_BUF_SYNC_READ :
             flags == 1 ? DMA_BUF_SYNC_WRITE : DMA_BUF_SYNC_RW),
    };
    int ret;
    do { ret = ioctl(b->fd, DMA_BUF_IOCTL_SYNC, &sync); }
    while (ret < 0 && errno == EINTR);
    return ret;
}
int32_t RK_MPI_MMZ_FlushCacheVaddrStart(void *a, uint32_t n, uint32_t f)
{ return sync_memory(a, n, f, 0); }
int32_t RK_MPI_MMZ_FlushCacheVaddrEnd(void *a, uint32_t n, uint32_t f)
{ return sync_memory(a, n, f, 1); }

/* The pinned archive uses DRM for internal command buffers even when MMZ is
 * supplied by the application. Route that entire allocator interface through
 * the same CMA heap; this board has no display/DRM device. These functions are
 * linked with --wrap, including allocation, release and cache synchronization.
 */
int __wrap_rve_drm_init(void) { return 0; }
int __wrap_rve_drm_deinit(void) { return 0; }

int __wrap_RVE_DRM_MmzAlloc(uint64_t *physical, void **address, uint32_t size)
{
    void *block;
    if (!physical || !address || RK_MPI_MMZ_Alloc(&block, size, 1))
        return -1;
    *physical = RK_MPI_MB_Handle2PhysAddr(block);
    *address = RK_MPI_MB_Handle2VirAddr(block);
    return 0;
}

int __wrap_RVE_DRM_MmzFree(void *address)
{
    struct Block *b = RK_MPI_MB_VirAddr2Handle(address);
    if (!b || b->address != address) {
        errno = EINVAL;
        return -1;
    }
    return RK_MPI_MMZ_Free(b);
}

static int drm_sync(void *address, int fd, int end)
{
    if (address) {
        struct Block *b = RK_MPI_MB_VirAddr2Handle(address);
        if (!b) {
            errno = EINVAL;
            return -1;
        }
        return sync_memory(b->address, b->size, 2, end);
    }
    struct dma_buf_sync sync = {
        .flags = DMA_BUF_SYNC_RW | (end ? DMA_BUF_SYNC_END : DMA_BUF_SYNC_START),
    };
    int ret;
    do { ret = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync); }
    while (ret < 0 && errno == EINTR);
    return ret;
}
int __wrap_RVE_DRM_MmzFlush_Start(void *address, int fd)
{ return drm_sync(address, fd, 0); }
int __wrap_RVE_DRM_MmzFlush_End(void *address, int fd)
{ return drm_sync(address, fd, 1); }

/* The vendor static archive uses uClibc's assertion symbol. */
_Noreturn void __assert(const char *expression, const char *file, unsigned line,
                      const char *function)
{
    fprintf(stderr, "RVE assertion: %s (%s:%u %s)\n", expression, file, line,
            function ? function : "");
    abort();
}
