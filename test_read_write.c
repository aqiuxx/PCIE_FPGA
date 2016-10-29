#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>

#define DEV_NAME "/dev/pcie-lat/00:05.0"

int main(int argc, char *argv[])
{
	int i;
	int fd=0;
	char buff[64];
	fd = open (DEV_NAME, O_RDWR);
	if(fd<0){
		perror("Open "DEV_NAME" Failed!\n");
		exit(1);
	}



	printf("read orig data from device\n");
	i = read(fd, &buff, 64);

	if (!i) {
		perror("read "DEV_NAME" Failed!\n");
		exit(1);
	}

	for (i=0; i<64; i++ ) {
		printf("0x%02x ", buff[i]);
	}



	printf("\n");
	printf("write data into device\n");

	for (i=0; i<64; i++ ) {
		buff[i] = 63 - i;
	}

	i = write(fd, &buff, 64);
	if (!i) {
		perror("write "DEV_NAME" Failed!\n");
		exit(1); 
	}



	printf("read new data from device\n"); 
	i = read(fd, &buff, 64);

	if (!i) {
		perror("read "DEV_NAME" Failed!\n");
		exit(1); 
	}

	for (i=0; i<64; i++ ) { 
		printf("0x%02x ", buff[i]);
	}
	printf("\n");

	close(fd);
	return 0;
}
