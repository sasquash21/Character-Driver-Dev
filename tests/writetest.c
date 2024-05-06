#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

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

int main() {

    struct timespec start, end;
    char chardriver_node[256];
    strcpy(chardriver_node, "/dev/chardriver0");
    int i;

    unsigned char* buffer = malloc(BUFFER_SIZE);

    if(buffer == NULL){
        perror("buffer was not allocated");
	exit(-1);
    }
    
    unsigned char* readbuffer = malloc(BUFFER_SIZE);

    if(readbuffer == NULL){
	perror("buffer was not allocated");
	exit(-1);
    }
    
    unsigned int* buffp = (unsigned int*)buffer;
    unsigned char* bufp = (unsigned char*)buffer;

    for(i = 0; i<(BUFFER_SIZE/sizeof(unsigned int));i++){
        buffp[i] = 0xDEADBEEF;
    }

    print_buffer(buffp, (BUFFER_SIZE/sizeof(unsigned int)));

    int fd = open(chardriver_node, O_RDWR);

    if (fd < 0) {
        perror("fd did not open");
        exit(-1);
    }

    printf("buffp. buffer: %p, %p\n", buffp, buffer);
    clock_gettime(CLOCK_REALTIME, &start);
    int counter = 0;
    while(counter < 500){
        for(int i = 0; i < BUFFER_SIZE ;i++){ 

	    ssize_t write_ret;
	    //printf("bufp[i] = %x\n", bufp[i]);
            write_ret = write(fd, &bufp[i], 1);

            if(write_ret < 0){
                perror("write failed");
	        exit(-1);
	    }
	}
	off_t offset = 0;
        lseek(fd, offset, SEEK_SET);

        counter++;
    }
    clock_gettime(CLOCK_REALTIME, &end);

    struct timespec d = diff(start, end);

    printf("write time: %lx seconds %lx ns\n", d.tv_sec, d.tv_nsec);

    off_t offset = 0;
    lseek(fd, offset, SEEK_SET);


    ssize_t bytes_read;
    bytes_read = read(fd, readbuffer, BUFFER_SIZE);

    if(bytes_read < 0){
	perror("read failed");
	exit(-1);
    }
    printf("printing the read bytes: bytes read: %ld\n", bytes_read);
    print_buffer((unsigned int*)readbuffer, bytes_read/sizeof(unsigned int));

    close(fd);
}
