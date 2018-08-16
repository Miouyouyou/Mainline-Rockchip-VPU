#include <stdint.h>

#include <stdio.h>
#include <sys/mman.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>

#include <stdlib.h>

#include <sys/ioctl.h>
#include "myy_ioctl.h"

int main(void)
{
	int fd;
	uint8_t *address_from_mmap;

	uint32_t len = 1920 * 1080 * 4;

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

	int const out_fd =
		open("output_frame", O_RDWR|O_CREAT, 00664);
	if (out_fd < 0)
		perror("Could not create the output file\n");
	else {
		write(out_fd, address_from_mmap, len);
		close(out_fd);
	}

	ioctl(fd, MYY_IOCTL_DUMP_OUTPUT_FIRST_BYTES, NULL);

	fprintf(stderr,
		"mmap_alloc: mmap OK - Address : %p\n",
		 address_from_mmap);

	fprintf(stderr, "closing...\n");

	close(fd);
}
