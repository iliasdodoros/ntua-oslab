#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

int main (){

	int fd=open("secret_number",O_RDWR|O_CREAT,0600);
	ssize_t count=0;

	char buff;
	while(!count){

		count=read(fd,&buff,sizeof(char));
	}

	while(count){
		printf("%c",buff);
		count=read(fd,&buff,sizeof(char));

	}
}
