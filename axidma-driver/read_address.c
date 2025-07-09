#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
#define READ_SIZE 44  // number of bytes to read

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <physical address in hex>\n", argv[0]);
        return 1;
    }

    off_t target_addr = strtoull(argv[1], NULL, 16);

    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    void *map_base = mmap(NULL, MAP_SIZE, PROT_READ, MAP_SHARED, fd, target_addr & ~MAP_MASK);
    if (map_base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return 1;
    }

    uint8_t *virt_addr = (uint8_t *)map_base + (target_addr & MAP_MASK);

    printf("Reading 44 bytes from physical address 0x%lx:\n", (unsigned long)target_addr);
    for (int i = 0; i < READ_SIZE; ++i) {
        printf("%02x ", virt_addr[i]);
        if ((i + 1) % 4 == 0) printf(" ");  // extra spacing every 4 bytes
    }
    printf("\n");

    munmap(map_base, MAP_SIZE);
    close(fd);
    return 0;
}
