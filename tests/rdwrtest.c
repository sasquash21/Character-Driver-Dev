#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

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
    ssize_t write_ret;
    write_ret = write(fd, buffp, BUFFER_SIZE);

    if(write_ret < 0){
        perror("write failed");
	exit(-1);
    }

    printf("write returns: %ld\n", write_ret);

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
