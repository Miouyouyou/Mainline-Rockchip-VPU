#include <stdio.h>
#include <sys/mman.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>

#include <stdlib.h>

int main(void)
{
	int fd;
	void *address_from_mmap;

	int len = 100 * getpagesize();

	if ((fd=open("/dev/vpu-service", O_RDWR|O_SYNC)) < 0) {
		perror("open");
		exit(-1);
	}
	fprintf(stderr, "mmap_alloc: open OK\n");

	address_from_mmap = mmap(0, len,
		PROT_READ|PROT_WRITE,
		MAP_SHARED|MAP_LOCKED,
		fd, 0);

	if (address_from_mmap == MAP_FAILED)	{
		perror("mmap");
		exit(-1);
	}
	
	fprintf(stderr,
		"mmap_alloc: mmap OK - Address : %p\n",
		 address_from_mmap);

	fprintf(stderr, "closing...\n");

	close(fd);
}
