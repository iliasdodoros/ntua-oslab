#include <stdlib.h> 
#include <stdio.h> 
#include <fcntl.h> 
#include <sys/mman.h> 
#include <sys/stat.h> 
#include <unistd.h> 

int main (int argc, char* argv[]) 

{
	int fd;   	
   	fd = open(".hello_there",O_RDWR);	
   	if (!fd){
		perror("opening .hello_there\n");
	  	exit (-1);
   	}
   	ftruncate(fd,32768);
   	
	return 0; 
} 
