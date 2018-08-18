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

	printf("Write the registers into the VPU HW...\n");
	ioctl(fd, MYY_IOCTL_VPU_WRITE_REGISTERS);
	sleep(1);

	printf("Launch a decode pass...\n");
	ioctl(fd, MYY_IOCTL_VPU_LAUNCH_DECODING);
	sleep(1);

	printf("Get the registers...\n");
	uint32_t regs[60];
	ioctl(fd, MYY_IOCTL_VPU_COPY_REGISTERS, regs);

	printf("uint32_t regs[60] = {\n\t");
	for (uint_fast8_t i = 0; i < 60; i++) {
		printf("0x%08x, ", regs[i]);
		if ((i & 3) == 3) printf("\n\t");
	}
	printf("\n};\n");

	int const out_fd =
		open("output_frame", O_RDWR|O_CREAT, 00664);
	if (out_fd < 0)
		perror("Could not create the output file\n");
	else {
		write(out_fd, address_from_mmap, len);
		close(out_fd);
	}

	fprintf(stderr,
		"mmap_alloc: mmap OK - Address : %p\n",
		 address_from_mmap);

	fprintf(stderr, "closing...\n");

	close(fd);
}
