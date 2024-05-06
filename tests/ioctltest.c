#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include "../chardriver_ioctl.h"   //needs to be installed usr/include/sys

#define BUFFER_SIZE   4096

void print_buffer(unsigned int* buffer_pointer, int size){

    int i;
    for(i = 0;i<size;i++){
        printf("%x ", buffer_pointer[i]);
	if(i%4 == 3){
	    printf("\n");
	}
    }
}

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

    ret = ioctl(fd, CHARDRIVER_IOCTL_RETURN_STATS, &driverstats);
    if (ret<0) {
        perror("ioctl failed");
	exit(-1);
    }
    printf("opencount: %ld\n", driverstats.opencount);
    printf("opencount: %ld\n", sp->opencount);
    printf("closecount: %ld\n", driverstats.closecount);
    printf("readcount: %ld\n", driverstats.readcount);
    printf("writecount: %ld\n", driverstats.writecount);
    printf("mmapcount: %ld\n", driverstats.mmapcount);
    printf("munmapcount: %ld\n", driverstats.munmapcount);
    printf("openfail: %ld\n", driverstats.openfail);
    printf("closefail: %ld\n", driverstats.closefail);
    printf("readfail: %ld\n", driverstats.readfail);
    printf("writefail: %ld\n", driverstats.writefail);
    printf("mmapfail: %ld\n", driverstats.mmapfail);

    /*ret = ioctl(fd, CHARDRIVER_IOCTL_GET_BUFFERSIZE, 0);

    printf("chardriver hwptr size = %d\n", ret);

    ret = ioctl(fd, CHARDRIVER_IOCTL_CHANGE_BUFFERSIZE, 5500);
    if (ret<0) {
        perror("fd could not change buffer size");
	exit(-1);
    }*/

    close(fd);
}
