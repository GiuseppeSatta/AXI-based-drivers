#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>

#define DEVICE "/dev/axidma"
#define MAP_SIZE 0x10000  // 64KB
#define TEST_PATTERN 0xAB

int main() {
    int fd = open(DEVICE, O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("Failed to open device");
        return EXIT_FAILURE;
    }

    // Allocate buffer to write
    unsigned char write_buf[16];
    memset(write_buf, TEST_PATTERN, sizeof(write_buf));

    // Write to device
    if (write(fd, write_buf, sizeof(write_buf)) < 0) {
        perror("Write failed");
        close(fd);
        return EXIT_FAILURE;
    }

    // Read back from device
    lseek(fd, 0, SEEK_SET);
    unsigned char read_buf[16];
    if (read(fd, read_buf, sizeof(read_buf)) < 0) {
        perror("Read failed");
        close(fd);
        return EXIT_FAILURE;
    }

    printf("Read values:\n");
    for (int i = 0; i < sizeof(read_buf); ++i)
        printf("0x%02x ", read_buf[i]);
    printf("\n");

    // mmap test
    void *mapped = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mapped == MAP_FAILED) {
        perror("mmap failed");
        close(fd);
        return EXIT_FAILURE;
    }

    printf("First 8 bytes via mmap: ");
    for (int i = 0; i < 8; ++i)
        printf("0x%02x ", ((unsigned char *)mapped)[i]);
    printf("\n");

    // Clean up
    munmap(mapped, MAP_SIZE);
    close(fd);
    return EXIT_SUCCESS;
}
