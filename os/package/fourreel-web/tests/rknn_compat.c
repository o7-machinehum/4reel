#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

extern const uint16_t *__ctype_b;
extern const int16_t *__ctype_tolower;
void rknn_musl_init(void);
int __isinf(double);
int __isnan(double);
void *rknn_uclibc_mmap(void *, size_t, int, int, int, int32_t);
int32_t rknn_uclibc_lseek(int, int32_t, int);
int rknn_uclibc_ftruncate(int, int32_t);

int main(void)
{
    rknn_musl_init();
    assert(__ctype_b['A'] == ((1<<0)|(1<<2)|(1<<4)|(1<<6)|(1<<7)|(1<<11)));
    assert(__ctype_b['z'] == ((1<<1)|(1<<2)|(1<<6)|(1<<7)|(1<<11)));
    assert(__ctype_b['7'] == ((1<<3)|(1<<4)|(1<<6)|(1<<7)|(1<<11)));
    assert(__ctype_b[' '] == ((1<<5)|(1<<6)|(1<<8)));
    assert(__ctype_b['\n'] == ((1<<5)|(1<<9)));
    assert(!__ctype_b[-1] && !__ctype_b[-128]);
    assert(__ctype_tolower['A'] == 'a' && __ctype_tolower['z'] == 'z');
    assert(__ctype_tolower[-1] == -1 && __ctype_tolower[-128] == -128);
    assert(__isinf(INFINITY) == 1 && __isinf(-INFINITY) == -1);
    assert(!__isinf(1) && !__isnan(1) && __isnan(NAN));
    char name[] = "/tmp/4reel-rknn-compat-XXXXXX";
    int fd = mkstemp(name);
    assert(fd >= 0 && !unlink(name));
    assert(!rknn_uclibc_ftruncate(fd, 8192));
    const char pattern[] = "RKNN offset ABI";
    assert(pwrite(fd, pattern, sizeof(pattern), 4096) == sizeof(pattern));
    assert(rknn_uclibc_lseek(fd, 4096, SEEK_SET) == 4096);
    char buffer[sizeof(pattern)];
    assert(read(fd, buffer, sizeof(buffer)) == sizeof(buffer));
    assert(!memcmp(buffer, pattern, sizeof(pattern)));
    void *mapped = rknn_uclibc_mmap(NULL, 4096, PROT_READ, MAP_SHARED, fd, 4096);
    assert(mapped != MAP_FAILED && !memcmp(mapped, pattern, sizeof(pattern)));
    assert(!munmap(mapped, 4096) && !close(fd));
    puts("PASS: RKNN uClibc character tables, math and 32-bit file-offset ABI on musl");
    return 0;
}
