#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#define DEVICE_PATH "/dev/inverter"

int main() {
    int fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return EXIT_FAILURE;
    }

    uint32_t write_val = 0xA5A5A5A5;  // example value to write
    ssize_t ret;

    // Write the 32-bit value to the first register (offset 0)
    ret = pwrite(fd, &write_val, sizeof(write_val), 0);
    if (ret != sizeof(write_val)) {
        perror("Failed to write");
        close(fd);
        return EXIT_FAILURE;
    }
    printf("Wrote 0x%08X to register 0\n", write_val);

    // Read the 32-bit inverted value from the second register (offset 4)
    uint32_t read_val = 0;
    ret = pread(fd, &read_val, sizeof(read_val), 4);
    if (ret != sizeof(read_val)) {
        perror("Failed to read");
        close(fd);
        return EXIT_FAILURE;
    }
    printf("Read 0x%08X from register 1 (inverted)\n", read_val);

    close(fd);
    return EXIT_SUCCESS;
}

