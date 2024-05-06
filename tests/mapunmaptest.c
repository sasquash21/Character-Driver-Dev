#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "../chardriver_ioctl.h"   //needs to be installed usr/include/sys

#define BUFFER_SIZE   4096


int main() {

    char chardriver_node[256];
    strcpy(chardriver_node, "/dev/chardriver0");
    int i, ret;
    stats_t driverstats;
    stats_t *sp;
    sp = &driverstats;


    int fd = open(chardriver_node, O_RDWR);

    if (fd < 0) {
        perror("fd did not open");
        exit(-1);
    }
    unsigned char *mappedbp;
    mappedbp = mmap(NULL, BUFFER_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    printf("mappedbp : %p\n", mappedbp);
    printf("*mappedbp : %x\n", *mappedbp);
    if(mappedbp == NULL){
        perror("mappedbp");
        exit(-1);
    }

    ret = munmap(mappedbp, BUFFER_SIZE);
    if(ret < 0){
        perror("did not complete munmap\n");
        exit(-1);
    }

    close(fd);
}
