#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#define DEVICE "/dev/axidma"
#define BUFFER_SIZE 4096

int main() {
    int fd = open(DEVICE, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    // Prepare some test data to write (MM2S)
    char write_buf[BUFFER_SIZE];
    for (int i = 0; i < BUFFER_SIZE; i++) {
        write_buf[i] = (char)(i & 0xFF);
    }

    // Write to device (starts MM2S DMA transfer)
    ssize_t ret = write(fd, write_buf, BUFFER_SIZE);
    if (ret < 0) {
        perror("Write failed");
        close(fd);
        return 1;
    }
    printf("Wrote %zd bytes to device\n", ret);

    // Clear read buffer
    char read_buf[BUFFER_SIZE];
    memset(read_buf, 0, BUFFER_SIZE);

    // Read from device (starts S2MM DMA transfer)
    ret = read(fd, read_buf, BUFFER_SIZE);
    if (ret < 0) {
        perror("Read failed");
        close(fd);
        return 1;
    }
    printf("Read %zd bytes from device\n", ret);

    // Print first 64 bytes of read data as hex
    printf("First 64 bytes of read data:\n");
    for (int i = 0; i < 64 && i < ret; i++) {
        printf("%02x ", (unsigned char)read_buf[i]);
        if ((i + 1) % 16 == 0)
            printf("\n");
    }
    printf("\n");

    close(fd);
    return 0;
}
