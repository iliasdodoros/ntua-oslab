#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <unistd.h>

/*Usage eg: ./challenge12 /tmp/riddle-w3mNVp T*/
int main(int argc, char **argv){

	int fd;
	char letter;

	fd = open(argv[1], O_RDWR);
	letter = *argv[2];

	lseek(fd, 0x6f, SEEK_SET);
	write(fd, &letter, sizeof(letter));
}
