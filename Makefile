MYY_KERNEL_DIR ?= ../RockMyy/linux
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

ccflags-y += -DMYY_TESTS

#myy-vpu-objs := myy-vpu.c
#test-dma-to-from-user := test-dma-to-from-user.c
test-devicetree-dma-to-from-user := test-devicetree-dma-to-from-user.c

# Careful, obj-y is for elements built into the kernel !
# obj-m is for elements built as modules
# We're building a module, so obj-m is required !
#obj-m += myy-vpu.o
#obj-m += test-dma-to-from-user.o
obj-m += test-devicetree-dma-to-from-user.o

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) -C $(MYY_KERNEL_DIR) modules

clean:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) -C $(MYY_KERNEL_DIR) clean

install: all
	scp test-devicetree-dma-to-from-user.ko tinkerboard:/tmp
	scp -r user-mode/ tinkerboard:/tmp

