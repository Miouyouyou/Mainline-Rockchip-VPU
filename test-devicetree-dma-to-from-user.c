
/* If you got a dumb editor that does not support parsing defines from
 * a .h file, but support defining special macro values, then define
 * DUMBY_THE_EDITOR to 1 */
#ifdef DUMBY_THE_EDITOR
#include <generated/autoconf.h>
#endif

// THIS_MODULE and others things
#include <linux/module.h>

// cdev_init and help us generate the /dev entries
#include <linux/cdev.h>

// struct inode
#include <linux/fs.h>

// dma_alloc_coherent
#include <linux/dma-mapping.h>

// clock functions
#include <linux/clk.h>


// -- OF

// platform_driver, platform_device, module_platform_driver
// Also include mod_devicetable.h
// of_device_id
#include <linux/platform_device.h>

// of_match_ptr
#include <linux/of.h>

// of_find_device_by_node
#include <linux/of_platform.h>

// Adds test_regs and test_encoded_frame
#include "test_static_data.h"


// TODO : Copy the frame
//        Pass the registers to the VPU
//        Start the whole thing
//        -> Should be done through a simple memcpy ?
//           Or a memcpy + write ?
//           What about the IRQ handlers ?
//        Get the result


// Structures
#define N_PAGES_FOR_4Kx4Kx4_BYTES_PER_COLOR 2025

struct myy_dma {
	dma_addr_t handle;
	void *cpu_address;
};

struct myy_driver_data {
	struct myy_dma myy_dma;
	dev_t device_id;
	struct class * __restrict cls;
	struct device * __restrict sub_dev;
	/* Not a pointer in order to use the container_of macro hack to get
	 * back this allocated data structure.
	 * 
	 * Basically, when opening the /dev/node, the kernel will provide us
	 * the inode of the opened node. From this inode, we'll get back the
	 * address of the cdev structure that generated this /dev/node.
	 * 
	 * Since :
	 * - the cdev structure used in our code is actually embedded in
	 *   a myy_driver_data structure;
	 * - the C language provide facilities to determine the offset of the
	 *   cdev member inside myy_driver_data;
	 * - the cdev structure address passed in "open" is actually the
	 *   address of struct myy_driver_data + its cdev member offset.
	 * 
	 * `container_of` will be able to use that cdev address to get the
	 * address where `struct myy_driver_data` actually start and pass it
	 * back to us.
	 * 
	 * This will NEVER work if we allocate a memory region for cdev, using
	 * `cdev_alloc` and only store its address in `myy_driver_data`,
	 * since the address of `myy_driver_data` and the address of `cdev`
	 * would be then uncorrelated !
	 * 
	 * Still, this is the kind of hack that only pass in kernel code.
	 */
	struct cdev cdev;

};

// Functions

/// Helpers

static inline const char * __restrict myy_bool_str
(bool value)
{
	return value ? "true" : "false";
}


static void print_resources
(struct resource * resources, uint const n_resources)
{
	uint r;
	uint r_max = n_resources - 1;
	for (r = 0; r < n_resources; r++) {
		struct resource * current_res =	&resources[r];
		printk("[%d/%d] Resource\n", r, r_max);
		printk("  Address : %p\n", current_res);
		printk("  Start : %pa\n", &current_res->start);
		printk("  End   : %pa\n", &current_res->end);
		printk("  Name  : %s\n",  current_res->name);
		printk("  Flags : %lx\n", current_res->flags);
		printk("  Desc  : %lx\n", current_res->desc);
	}
}

static void print_platform_device
(struct platform_device const * __restrict const pdev)
{
	printk(KERN_INFO
		"Name               : %s\n"
		"ID                 : %d\n"
		"ID auto            : %s\n"
		"Device (name)      : %s\n"
		"Num resources      : %u\n"
		"Resources address  : %p\n"
		"Platform device ID : %s\n",
		pdev->name,
		pdev->id,
		myy_bool_str(pdev->id_auto),
		dev_name(&pdev->dev),
		pdev->num_resources,
		pdev->resource,
		pdev->id_entry->name
	);
	printk(KERN_INFO "Printing resources of device");
	print_resources(pdev->resource, pdev->num_resources);
}

/// Open/IOCTL/Close/MMAP

static int test_user_dma_open(struct inode *inode, struct file *filp) {
	struct myy_driver_data * __restrict const myy_driver_data =
		container_of(inode->i_cdev, struct myy_driver_data, cdev);
	printk(KERN_INFO "Open !");
	/* This backflip bullshit always got me */
	filp->private_data = myy_driver_data;
	return 0;
};
static long test_user_dma_ioctl(struct file * const filp,
	unsigned int const cmd, unsigned long const arg)
{
	printk(KERN_INFO "IOCTL !");
	return 0;
};
static int test_user_dma_mmap(struct file *filp,
	struct vm_area_struct *vma)
{
	struct myy_driver_data const * __restrict const myy_driver_data =
		(struct myy_driver_data const *)
		filp->private_data;
	int ret;
	printk(KERN_INFO "MMAP !");

	ret = dma_common_mmap(NULL, vma,
		myy_driver_data->myy_dma.cpu_address,
		myy_driver_data->myy_dma.handle,
		vma->vm_end - vma->vm_start);

	if (ret)
		printk(KERN_INFO "MMAP failed :C : -%d\n", ret);

	return ret;
};

static int test_user_dma_release(struct inode * inode,
	struct file * filp)
{
	printk(KERN_INFO "Close !");
	return 0;
};


static struct file_operations test_user_dma_fops = {
	.owner          = THIS_MODULE,
	.open           = test_user_dma_open,
	.unlocked_ioctl = test_user_dma_ioctl,
	.mmap           = test_user_dma_mmap,
	.release        = test_user_dma_release
};


/* Should return 0 on success and a negative errno on failure. */
static int myy_vpu_probe(struct platform_device * pdev)
{
	/* The device associated with the platform_device. Identifier
	 * used for convenience. (Dereferencing pdev every time is useless) */
	struct device * __restrict const vpu_dev = &pdev->dev;

	/* The data structure that will be used as driver's private data. */
	struct myy_driver_data * __restrict const driver_data =
		devm_kzalloc(&pdev->dev, sizeof(*driver_data), GFP_KERNEL);

	/* Will be used to create /dev entries */
	struct cdev * __restrict const cdev = &driver_data->cdev;
	const char * __restrict const name = pdev->dev.of_node->name;

	/* Used to check various return codes for errors */
	int ret;

	/*print_platform_device(pdev);*/

	/* Allocate the DMA buffer */
	dev_info(vpu_dev, "devm_ioremap_resource");
	devm_ioremap_resource(vpu_dev, platform_get_resource(pdev, IORESOURCE_MEM, 0));
	
	dev_info(vpu_dev, "clk_prepare_enable aclk");
	clk_prepare_enable(devm_clk_get(vpu_dev, "aclk"));
	
	dev_info(vpu_dev, "clk_prepare_enable iface");
	clk_prepare_enable(devm_clk_get(vpu_dev, "iface"));

	/* Setup the private data storage for this device.
	 * 
	 * The data will be available through the device structure until
	 * the driver module gets unloaded.
	 * 
	 * platform_set_drvdata is equivalent to dev_set_drvdata, except that
	 * - dev_set_drvdata takes a device pointer
	 * - platform_set_drvdata takes a platform_device pointer
	 * 
	 * The main difference between kzalloc and devm_kzalloc is that
	 * allocated memory with devm_kzalloc gets automagically freed when
	 * releasing the driver.
	 */
	platform_set_drvdata(pdev, driver_data);

	dev_info(vpu_dev, "Private data address : %p\n", driver_data);

	/* Initialize our /dev entries
	 * 
	 * It seems that creating a sub-device is necessary to get the
	 * - /dev/#{name} entries to appear... Weird
	 * 
	 * alloc_chrdev_region will call MKDEV
	 */
	dev_info(vpu_dev, "alloc_chrdev_region");
	ret = alloc_chrdev_region(&driver_data->device_id, 0, 1, name);
	if (ret)
	{
		dev_err(vpu_dev, "alloc_chrdev_region returned %d\n", ret);
		goto chrdev_alloc_failed;
	}

	dev_info(vpu_dev, "cdev_init");
	cdev_init(cdev, &test_user_dma_fops);
	cdev->owner = THIS_MODULE;

	dev_info(vpu_dev, "cdev_add");
	ret = cdev_add(cdev, driver_data->device_id, 1);

	if (ret)
	{
		dev_err(vpu_dev, "cdev_add failed !?");
		goto cdev_add_failed;
	}

	dev_info(vpu_dev, "class_create");
	driver_data->cls = class_create(THIS_MODULE, name);

	if (IS_ERR(driver_data->cls))
	{
		ret = PTR_ERR(driver_data->cls);
		dev_err(vpu_dev, "class_create err:%d\n", ret);
		goto class_create_failed;
	}

	dev_info(vpu_dev, "device_create");
	driver_data->sub_dev = device_create(
		driver_data->cls, vpu_dev,
		driver_data->device_id, NULL,
		"%s", name
	);

	if (IS_ERR(driver_data->sub_dev))
	{
		dev_err(vpu_dev,
			"device_create returned : %ld\n",
			PTR_ERR(driver_data->sub_dev));
		goto device_create_failed;
	}

	driver_data->myy_dma.cpu_address = dma_alloc_coherent(vpu_dev,
		N_PAGES_FOR_4Kx4Kx4_BYTES_PER_COLOR * PAGE_SIZE,
		&driver_data->myy_dma.handle, GFP_KERNEL);

	if (!driver_data->myy_dma.cpu_address)
	{
		dev_err(vpu_dev,
			"DMA Alloc coherent could not allocate\n"
			"Calling the cops right now !\n");
		ret = -ENOMEM;
		goto dma_alloc_failed;
	}

	return ret;

device_create_failed:
	class_destroy(driver_data->cls);
class_create_failed:
	cdev_del(cdev);
cdev_add_failed:
	unregister_chrdev_region(driver_data->device_id, 1);
chrdev_alloc_failed:
	dma_free_coherent(vpu_dev,
		N_PAGES_FOR_4Kx4Kx4_BYTES_PER_COLOR * PAGE_SIZE,
		&driver_data->myy_dma.cpu_address, GFP_KERNEL);
dma_alloc_failed:
	return ret;
}

/* Should return 0 on success and a negative errno on failure. */
static int myy_vpu_remove(struct platform_device * pdev)
{
	/* The device associated with the platform_device. Identifier
	 * used for convenience. (Dereferencing pdev every time is useless) */
	struct device * __restrict const vpu_dev = &pdev->dev;

	struct myy_driver_data * __restrict const driver_data =
		(struct myy_driver_data * __restrict)
		platform_get_drvdata(pdev);

	/* Remove the subdevice and /dev entries */
	device_destroy(driver_data->cls, driver_data->device_id);
	dev_info(vpu_dev, "Device destroyed\n");
	class_destroy(driver_data->cls);
	dev_info(vpu_dev, "Class destroyed\n");
	cdev_del(&driver_data->cdev);
	dev_info(vpu_dev, "Character device destroyed\n");
	unregister_chrdev_region(driver_data->device_id, 1);
	dev_info(vpu_dev, "chrdev_region unregistered\n");

	dma_free_coherent(&pdev->dev,
		N_PAGES_FOR_4Kx4Kx4_BYTES_PER_COLOR * PAGE_SIZE,
		driver_data->myy_dma.cpu_address, GFP_KERNEL);
	dev_info(vpu_dev, "DMA buffer Freed\n");

	printk(KERN_INFO "Okay, I'll go, I'll go... !\n");

	/* driver_data was allocated through devm_kzalloc and will be
	 * deallocated automatically.
	 */

	return 0;
}

static void myy_vpu_shutdown(struct platform_device *pdev)
{
	printk(KERN_INFO "Shutting down...\n");
}


static struct of_device_id const myy_vpu_dt_ids[] = {
	{
		.compatible = "rockchip,vpu_service",
	},
	{}
};


static struct platform_driver myy_vpu_driver = {
	.probe = myy_vpu_probe,
	.remove = myy_vpu_remove,
	.shutdown = myy_vpu_shutdown,
	.driver = {
		.name = "myy-vpu",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(myy_vpu_dt_ids),
	},
};

module_platform_driver(myy_vpu_driver);
MODULE_DESCRIPTION("Test the allocation of DMA'ble memory and sharing with the user");
MODULE_AUTHOR("Myy");
MODULE_LICENSE("GPL");
