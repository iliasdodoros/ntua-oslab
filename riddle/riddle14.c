#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char *argv[])
{
	int fd, pid;
	char buf[32];	
	fd = open("/proc/sys/kernel/ns_last_pid", O_RDWR | O_CREAT, 0644);
	
	if (flock(fd, LOCK_EX)) {
		close(fd);
		return 1;
	}
	
	pid = atoi(argv[1]);
	snprintf(buf, sizeof(buf), "%d", pid - 1);

	if (write(fd, buf, strlen(buf)) != strlen(buf)) {
		return 1;
	}
	
	int pidnew;
	pidnew = fork();
	if (pidnew == 0) {
		char executable[] = "riddle";
		char *newargv[] = {executable, NULL};
		char *newenviron[] = {NULL};
		execve(executable, newargv, newenviron);

		exit(0);
	}

	if (flock(fd, LOCK_UN)) {
		printf("Can't unlock");
	}

	close(fd);
	return 0;
}
