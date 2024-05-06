#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

int main() {

    char chardriver_node[256];
    strcpy(chardriver_node, "/dev/chardriver0");

    int fd = open(chardriver_node, O_RDONLY);
    if (fd < 0) {
        perror("fd did not open");
        exit(-1);
    }

    int fd1 = open(chardriver_node, O_RDONLY);
    if (fd1 < 0) {
        perror("fd1 failed to open");
        exit(-1);
    }

    int fd2 = open(chardriver_node, O_RDONLY);
    if (fd1 < 0) {
        perror("fd2 failed to open");
        exit(-1);
    }
   

    int fd3 = open(chardriver_node, O_RDONLY);
    if (fd1 < 0) {
        perror("fd3 failed to open");
        exit(-1);
    }

    close(fd);
    close(fd1);
    close(fd2);
}
