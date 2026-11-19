#define _POSIX_C_SOURCE 200809L
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <sys/mman.h>
#include <unistd.h>

/* The vendor's ARM runtime uses uClibc's C-locale table ABI. Its masks are
 * native-endian and its lowercase entries are signed 16-bit values, unlike
 * glibc/musl's compatibility tables. Keep the translation local to RKNN. */
static uint16_t classes[384];
static int16_t lowercase[384];
const uint16_t *__ctype_b = classes + 128;
const int16_t *__ctype_tolower = lowercase + 128;

void rknn_musl_init(void)
{
    for (int c = -128; c < 256; ++c) {
        lowercase[c + 128] = (int16_t)(c < 0 ? c : tolower(c));
        if (c < 0)
            continue;
        classes[c + 128] = (!!isupper(c) << 0) | (!!islower(c) << 1) |
            (!!isalpha(c) << 2) | (!!isdigit(c) << 3) | (!!isxdigit(c) << 4) |
            (!!isspace(c) << 5) | (!!isprint(c) << 6) | (!!isgraph(c) << 7) |
            (!!isblank(c) << 8) | (!!iscntrl(c) << 9) | (!!ispunct(c) << 10) |
            (!!isalnum(c) << 11);
    }
}

int __isinf(double value) { return __builtin_isinf_sign(value); }
int __isnan(double value) { return __builtin_isnan(value); }

/* This vendor archive was built with 32-bit off_t. ARM EABI passes musl's
 * 64-bit off_t at a different stack/register location. Only the archive's
 * references are renamed to these adapters; native musl callers keep mmap,
 * lseek and ftruncate and their normal 64-bit ABI. */
void *rknn_uclibc_mmap(void *address, size_t length, int protection, int flags,
                       int fd, int32_t offset)
{
    return mmap(address, length, protection, flags, fd, (off_t)offset);
}

int32_t rknn_uclibc_lseek(int fd, int32_t offset, int whence)
{
    off_t result = lseek(fd, (off_t)offset, whence);
    if (result > INT32_MAX) {
        errno = EOVERFLOW;
        return -1;
    }
    return (int32_t)result;
}

int rknn_uclibc_ftruncate(int fd, int32_t length)
{
    return ftruncate(fd, (off_t)length);
}
