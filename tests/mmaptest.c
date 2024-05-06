#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include "../chardriver_ioctl.h"   //needs to be installed usr/include/sys

#define BUFFER_SIZE   4096

struct timespec diff(struct timespec start, struct timespec end)
{
    struct timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

void print_buffer(unsigned int* buffer_pointer, int size){

    int i;
    for(i = 0;i<size;i++){
        printf("%x ", buffer_pointer[i]);
	if(i%4 == 3){
	    printf("\n");
	}
    }
}

int comp_buffer(unsigned char* a, unsigned char* b, int size){
    
    int success = 0;
    unsigned long* along_ptr = (unsigned long*)a;
    unsigned long* blong_ptr = (unsigned long*)b;
    for(int i = 0; i<(size/sizeof(unsigned long)) ; i++){
	if(along_ptr[i] != blong_ptr[i]){
	    success = -1;
	    printf("%s:  a[i] = 0x%lx, b[i] = 0x%lx, at offset = %d\n", __func__, along_ptr[i], blong_ptr[i], i);
	}
    }
    
    return success;
}


int main() {

    struct timespec start, end;
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

    //print_buffer((unsigned int*)mappedbp, BUFFER_SIZE/sizeof(unsigned int));
    clock_gettime(CLOCK_REALTIME, &start);
    int counter = 0;
    while(counter < 500){
        for(i = 0;i<BUFFER_SIZE;i++){
            mappedbp[i] = 0xBB;
        }
	counter++;
    }
    clock_gettime(CLOCK_REALTIME, &end);

    struct timespec d = diff(start, end);
    printf("write time: %lx seconds %lx ns\n", d.tv_sec, d.tv_nsec);

/*
    unsigned char pattern = 0x00;
    int icount = 0;
    while(icount < 5){
	for(i = 0;i<BUFFER_SIZE;i++){
            mappedbp[i] = pattern;
	}
	pattern++;
    

        unsigned char* readbuff = malloc(BUFFER_SIZE);
        if(readbuff == NULL) {
	    perror("readbuff failed to allocate");
	    exit(-1);
        }

        ret = read(fd, readbuff, BUFFER_SIZE);
        if(ret < 0){
	    perror("read failed");
	    exit(-1);
        }

        if(comp_buffer(readbuff, mappedbp, BUFFER_SIZE) < 0){
	    perror("read buff and mappedbp are no the same");
	    exit(-1);
        }
	icount++;
	printf("%d iterative counter in while loop\n", icount);
	sleep(1);
    }*/

    ret = munmap(mappedbp, BUFFER_SIZE);
    if(ret < 0){
	perror("did not complete munmap\n");
	exit(-1);
    }

    /*unsigned char* buffer = malloc(BUFFER_SIZE);
    if(buffer == NULL){
	perror("could not malloc");
	exit(-1);
    }

    for(i = 0;i<BUFFER_SIZE;i++){
        buffer[i] = 0xAA;
    }

    ret = write(fd, buffer, BUFFER_SIZE);
    if(ret != BUFFER_SIZE){
	perror("Write");
	exit(-1);
    }*/

    close(fd);
}
