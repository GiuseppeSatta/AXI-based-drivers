#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#define DEVICE_PATH "/dev/my_dma_chrdev"
#define IMAGE_WIDTH 88
#define IMAGE_HEIGHT 142
#define IMAGE_SIZE (IMAGE_HEIGHT * IMAGE_WIDTH)
#define IMAGE_DATA_SIZE (IMAGE_SIZE * 4)
#define RESULT_SIZE 4

int hexchar_to_int(char c) {
    if ('0' <= c && c <= '9') return c - '0';
    if ('A' <= c && c <= 'F') return c - 'A' + 10;
    if ('a' <= c && c <= 'f') return c - 'a' + 10;
    return -1;
}

uint8_t hex_to_byte(const char *hex) {
    int high = hexchar_to_int(hex[0]);
    int low  = hexchar_to_int(hex[1]);
    if (high == -1 || low == -1) {
        fprintf(stderr, "Invalid hex digit: %c%c\n", hex[0], hex[1]);
        exit(EXIT_FAILURE);
    }
    return (high << 4) | low;
}

int main() {
    int fd;
    ssize_t bytes_written, bytes_read;
    uint32_t result;
    uint32_t *image_data;

    FILE *fp = fopen("input32.txt", "r");
    if (!fp) {
        perror("Failed to open file");
        return 1;
    }

    size_t capacity = 256;
    size_t count = 0;
    uint32_t *buffer = malloc(capacity * sizeof(uint32_t));
    if (!buffer) {
        perror("malloc failed");
        fclose(fp);
        return 1;
    }

    char line[64];
    while (fgets(line, sizeof(line), fp)) {
        // Strip newline and whitespace
        line[strcspn(line, "\r\n")] = 0;
        int character=0;
        while (line[character] && isspace(line[character]) && character < 64) ++character;

        if (character == 64){
            printf("\ncharacter reached 64\n");
            return 0;
        } 

        if (strlen(line) != 8) {
            fprintf(stderr, "Skipping invalid line: '%s'\n", line);
            continue;
        }

        // Resize if needed
        if (count >= capacity) {
            capacity *= 2;
            uint32_t *new_buf = realloc(buffer, capacity * sizeof(uint32_t));
            if (!new_buf) {
                perror("realloc failed");
                free(buffer);
                fclose(fp);
                return 1;
            }
            buffer = new_buf;
        }

        // Safely parse 8-character hex string
        uint32_t value = 0;
        for (int i = 0; i < 4; ++i) {
            value = (value << 8) | hex_to_byte(&line[i * 2]); // Big endian
        }

        buffer[count++] = value;
    }

    fclose(fp);




    uint32_t *new_buf = realloc(buffer, IMAGE_SIZE * sizeof(uint32_t));
    if (!new_buf) {
        perror("realloc failed");
        free(buffer);
        fclose(fp);
        return 1;
    }

    buffer = new_buf;
    
    image_data = buffer;

    // Open device
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
         
        return EXIT_FAILURE;
    }

    // Write image data to device
    bytes_written = write(fd, image_data, IMAGE_SIZE);
    if (bytes_written < 0) {
        perror("Failed to write to device");
        close(fd);
         
        return EXIT_FAILURE;
    } else if (bytes_written != IMAGE_SIZE) {
        fprintf(stderr, "Incomplete write: %zd bytes\n", bytes_written);
    }

    // Read result from device
    bytes_read = read(fd, &result, RESULT_SIZE);
    if (bytes_read < 0) {
        perror("Failed to read from device");
        close(fd);
         
        return EXIT_FAILURE;
    } else if (bytes_read != RESULT_SIZE) {
        fprintf(stderr, "Incomplete read: %zd bytes\n", bytes_read);
    }

    printf("Received result: 0x%08X (%d)\n", result, result);

    // Cleanup
    close(fd);
     

    return EXIT_SUCCESS;
}
