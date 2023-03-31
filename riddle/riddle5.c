
#include <fcntl.h>
#include <unistd.h>

int main (){
	dup2(1,99);
	char * const*  environ={NULL};
	char * const*  arg={NULL};
	execve("riddle",arg,environ);
}

