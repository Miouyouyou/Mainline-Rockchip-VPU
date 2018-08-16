#ifndef MYY_VPU_INCLUDE_MYY_IOCTL_H
#define MYY_VPU_INCLUDE_MYY_IOCTL_H 1

// -- IOCTLS BEGIN

#include <linux/ioctl.h>
#include <linux/types.h>

// magic number actually unused by other drivers
#define MYY_IOCTL_BASE 0xB6

#define MYY_IO(nr)         _IO(MYY_IOCTL_BASE, nr)
#define MYY_IOR(nr, type) _IOR(MYY_IOCTL_BASE, nr, type)

#define MYY_IOCTL_VPU_COPY_REGISTERS      MYY_IO(0x1)
#define MYY_IOCTL_VPU_LAUNCH_DECODING     MYY_IO(0x2)
#define MYY_IOCTL_CHECK_OUTPUT_WRITE      MYY_IO(0x3)
#define MYY_IOCTL_DUMP_OUTPUT_FIRST_BYTES MYY_IO(0x4)

// -- IOCTLS END

#endif
