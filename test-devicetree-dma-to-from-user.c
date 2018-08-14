
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

// Adds cabac_table
#include "test_static_tables.h"

// Provide H264dVdpu1Regs_t
#include "rkmpp_h264_vpu_regs.h"

// irqreturn_t, IRQ_HANDLED, devm_request_threaded_irq
#include <linux/interrupt.h>
#include <linux/of_irq.h>

#include <linux/delay.h>
// TODO : Copy the frame
//        Pass the registers to the VPU
//        Start the whole thing
//        -> Should be done through a simple memcpy ?
//           Or a memcpy + write ?
//           What about the IRQ handlers ?
//        Get the result


// Structures
#define N_PAGES_FOR_1080P_RGBA32 (1000 * PAGE_SIZE)
#define VPU_DEC_REGS_OFFSET 0x400

struct myy_dma {
	dma_addr_t iommu_address;
	void *mmu_address;
};

struct myy_vpu_io {
	void __iomem * base;
	u32 __iomem * dec_regs;
};

struct myy_vpu_clocks {
	struct clk * aclk;
	struct clk * iface;
};

struct myy_driver_data {
	struct myy_dma output_frame;
	struct myy_dma cabac_table;
	// Our test only uses 1 frame, with no dependencies
	struct myy_dma encoded_frame; 
	dev_t device_id;
	struct class * __restrict cls;
	struct device * __restrict sub_dev;
	struct myy_vpu_io vpu_io;
	struct myy_vpu_clocks vpu_clocks;
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
	 * would be then unrelated !
	 * 
	 * Still, this is the kind of hack that only pass in kernel code.
	 */
	struct cdev cdev;

};

// Functions

/// Interrupt Request handlers

static irqreturn_t vpu_is_done_or_borked(
	int irq, void *dev_id)
{
	struct myy_driver_data const * __restrict const driver_data =
		(struct myy_driver_data const * __restrict) dev_id;

	/* Currently, we don't care about interruptions.
	 * It seems they're only called if the VPU has finished
	 * a job or if some issue happened (bad setup, ...).
	 */
	printk(KERN_INFO "IRQ : %d\n", irq);

	writel_relaxed(0, driver_data->vpu_io.dec_regs + 1);
	return IRQ_HANDLED;
}

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

static int test_user_dma_open(
	struct inode *inode,
	struct file *filp)
{
	struct myy_driver_data * __restrict const myy_driver_data =
		container_of(inode->i_cdev, struct myy_driver_data, cdev);
	printk(KERN_INFO "Open !");
	/* This backflip bullshit always got me */
	filp->private_data = myy_driver_data;
	return 0;
}

// TODO : Factoriser
static inline void * allocate_dma_space(
	struct device * __restrict const dev,
	u32 const size_in_octets,
	dma_addr_t * __restrict const returned_dma_handle)
{
	return dma_alloc_coherent(dev,
		ALIGN(size_in_octets, PAGE_SIZE),
		returned_dma_handle, GFP_KERNEL);
}

static inline void free_all_dma_space(
	struct device * __restrict const dev,
	u32 const size_in_octets,
	void * __restrict const mmu_address)
{
	dma_free_coherent(dev,
		ALIGN(size_in_octets, PAGE_SIZE),
		mmu_address, GFP_KERNEL);
}

static void * allocate_dma_space_and_copy(
	struct device * __restrict const dev,
	u32 const size_in_octets,
	dma_addr_t * __restrict const returned_dma_handle,
	u8 const * __restrict const data_to_copy,
	u32 const data_size_in_octets)
{
	void * cpu_mmu_address = allocate_dma_space(
		dev, ALIGN(size_in_octets, PAGE_SIZE),
		returned_dma_handle);

	if (cpu_mmu_address)
		memcpy(cpu_mmu_address, data_to_copy, data_size_in_octets);

	return cpu_mmu_address;
}

static void copy_registers_and_launch_decoding(
	struct device * __restrict const vpu,
	struct myy_driver_data const * __restrict const driver_data,
	u32 const * __restrict const regs)
{
	u32 __iomem * const hw_regs = driver_data->vpu_io.dec_regs;
	u32 i;
	/* According to Rockchip "vpu_service" code, only the registers
	 * 0, [2,59] should be copied.
	 * Well, reg[1] contains the "Let's do this !" bit which will
	 * just launch the VPU.
	 * If you copy blindly, the VPU will just run before being set
	 * up correctly, leading to all sorts of problems.
	 */

	// I don't know what the base is for...
	writel_relaxed(regs[0], hw_regs);

	// Prepare the VPU
	for (i = 2; i < 59; i++)
	{
		writel_relaxed(regs[i], hw_regs + i);
	}

	// Let's do this !
	writel_relaxed(1, hw_regs + 1);
}

static long test_user_dma_ioctl(
	struct file * const filp,
	unsigned int const cmd,
	unsigned long const arg)
{


	printk("IOCTL");

	//vdpu_write(vpu, VDPU_REG_INTERRUPT_DEC_E, VDPU_REG_INTERRUPT);
	return 0;
	
}

static int test_user_dma_mmap(
	struct file *filp,
	struct vm_area_struct *vma)
{
	struct myy_driver_data const * __restrict const myy_driver_data =
		(struct myy_driver_data const *)
		filp->private_data;
	int ret;
	printk(KERN_INFO "MMAP !");

	ret = dma_common_mmap(NULL, vma,
		myy_driver_data->output_frame.mmu_address,
		myy_driver_data->output_frame.iommu_address,
		vma->vm_end - vma->vm_start);

	if (ret)
		printk(KERN_INFO "MMAP failed :C : -%d\n", ret);

	return ret;
}

static int test_user_dma_release(
	struct inode * inode,
	struct file * filp)
{
	printk(KERN_INFO "Close !");
	return 0;
}


static struct file_operations test_user_dma_fops = {
	.owner          = THIS_MODULE,
	.open           = test_user_dma_open,
	.unlocked_ioctl = test_user_dma_ioctl,
	.mmap           = test_user_dma_mmap,
	.release        = test_user_dma_release
};

static void print_regs(
	struct device const * __restrict const device,
	u32 const * __restrict const regs)
{
	unsigned int i;

	dev_info(device, "[Current VPU registers state]\n");
	dev_info(device, "u32 vpu_regs[101] = {\n");
	for (i = 0; i < 100; i += 4)
	{
		dev_err(device,
			"\t0x%08x, 0x%08x, 0x%08x, 0x%08x,\n",
			regs[i], regs[i+1], regs[i+2], regs[i+3]);
	}

	dev_info(device, "\t0x%08x\n", regs[100]);
	dev_info(device, "};\n");
}

static void print_dma_addresses(
	struct device const * __restrict const device,
	struct myy_driver_data const * __restrict const driver_data)
{
	dev_info(device, "[DMA Addresses]\n");
	dev_info(device, "NAME   MMU      IOMMU\n");
	dev_info(device, "---------------------\n");
	dev_info(device, "%6s %p 0x%08x\n",
			"Input",
			driver_data->encoded_frame.mmu_address, 
			driver_data->encoded_frame.iommu_address);
	dev_info(device, "%6s %p 0x%08x\n",
			"Output",
			driver_data->output_frame.mmu_address,
			driver_data->output_frame.iommu_address);
	dev_info(device, "%6s %p 0x%08x\n",
			"CABAC",
			driver_data->cabac_table.mmu_address,
			driver_data->cabac_table.iommu_address);
}

static void prepare_the_registers(
	struct myy_driver_data const * __restrict const driver_data,
	u32 * __restrict const regs)
{
	unsigned int i;
	unsigned int offset;

	/* Input address */
	regs[12] = driver_data->encoded_frame.iommu_address;

	/* Output and reference frames addresses */
	for (i = 13; i < 30; i++)
		regs[i] = driver_data->output_frame.iommu_address;

	regs[40] = driver_data->cabac_table.iommu_address;

	/* TODO This frame also contains an... offset ?
	 *
	 * It seems to be some kind of hack due to design limits
	 * of the VPU.
	 * 
	 * The offset is computed like this on RKMPP
	 *
	 * {
		RK_U32 dirMvOffset = 0;
		RK_U32 picSizeInMbs = 0;

		picSizeInMbs = p_hal->pp->wFrameWidthInMbsMinus1 + 1;
		picSizeInMbs = picSizeInMbs * (2 - pp->frame_mbs_only_flag)
			* (pp->wFrameHeightInMbsMinus1 + 1);
		dirMvOffset = picSizeInMbs
			* ((p_hal->pp->chroma_format_idc == 0) ? 256 : 384);
		dirMvOffset += (pp->field_pic_flag && pp->CurrPic.AssociatedFlag)
			? (picSizeInMbs * 32) : 0;
		p_regs->SwReg41.dmmv_st_adr = (mpp_buffer_get_fd(frame_buf) | (dirMvOffset << 6));
	  * }
	  */
	/* ... Used as an offset when the IDC profile is superior
	 * to 66... which is completely broken because, on this
	 * example since (0x0bf40000) >> 10 gives 195840
	 * The frame in itself is only 6128 bytes long...
	 * That doesn't make any sense to me but all the others
	 * drivers does the same shit, so let's do the same thing.
	 * Maybe this register is documented somewhere...
	 */
	offset = 0; // regs[41] >> 10;
	regs[41] = driver_data->output_frame.iommu_address +
		offset;

}

static void disable_clocks(
	struct device * __restrict const vpu_dev,
	struct myy_driver_data const * __restrict const driver_data)
{
	clk_disable_unprepare(driver_data->vpu_clocks.aclk);
	devm_clk_put(vpu_dev, driver_data->vpu_clocks.aclk);

	clk_disable_unprepare(driver_data->vpu_clocks.iface);
	devm_clk_put(vpu_dev, driver_data->vpu_clocks.iface);
}

static int enable_and_prepare_clocks(
	struct device * __restrict const vpu_dev,
	struct myy_driver_data * __restrict const driver_data)
{

	int ret = 0;
	struct clk * __restrict aclk;
	struct clk * __restrict iface;

	dev_info(vpu_dev, "Clock : Preparing aclk");
	aclk = devm_clk_get(vpu_dev, "aclk");

	if (IS_ERR(aclk)) {
		ret = (int) aclk;
		dev_err(vpu_dev, "Failed to get aclk : %d", ret);
		goto get_aclk_failed;
	}

	ret = clk_prepare_enable(aclk);
	if (ret) {
		dev_err(vpu_dev, "Failed to prepare aclk : %d", ret);
		goto prepare_aclk_failed;
	}

	dev_info(vpu_dev, "Clock : aclk prepared !");
	dev_info(vpu_dev, "Clock : Preparing iface !");

	iface = devm_clk_get(vpu_dev, "iface");
	if (IS_ERR(iface)) {
		ret = (int) iface;
		dev_err(vpu_dev, "Failed to get iface : %d", ret);
		goto get_iface_failed;
	}

	ret = clk_prepare_enable(iface);
	if (ret) {
		dev_err(vpu_dev, "Failed to prepare iface : %d", ret);
		goto prepare_iface_failed;
	}

	dev_info(vpu_dev, "Clock : iface prepared !");

	driver_data->vpu_clocks.aclk  = aclk;
	driver_data->vpu_clocks.iface = iface;
	return ret;

// clk_disable_unprepare(iface);
prepare_iface_failed:
	devm_clk_put(vpu_dev, iface);
get_iface_failed:
	clk_disable_unprepare(aclk);
prepare_aclk_failed:
	devm_clk_put(vpu_dev, aclk);
get_aclk_failed:
	return ret;
}

/* Should return 0 on success and a negative errno on failure. */
static int myy_vpu_probe(struct platform_device * pdev)
{
	/* TODO Fragment this stupidly long function into several
	 *      parts.
	 */
	/* The device associated with the platform_device. Identifier
	 * used for convenience. (Dereferencing pdev every time is useless) */
	struct device * __restrict const vpu_dev = &pdev->dev;

	/* The data structure that will be used as driver's private data. */
	struct myy_driver_data * __restrict const driver_data =
		devm_kzalloc(&pdev->dev, sizeof(*driver_data), GFP_KERNEL);

	/* Will be used to create /dev entries */
	struct cdev * __restrict const cdev = &driver_data->cdev;
	const char * __restrict const name = pdev->dev.of_node->name;
	void * vpu_base_address;

	/* The "I'm done" or "The setup was invalid" IRQ number */
	int irq_dec;

	/* Used to check various return codes for errors */
	int ret;


	/*print_platform_device(pdev);*/

	/* Allocate the DMA buffer */
	dev_info(vpu_dev, "devm_ioremap_resource");
	vpu_base_address = devm_ioremap_resource(
		vpu_dev, platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if(IS_ERR(vpu_base_address)) {
		dev_err(vpu_dev, "Could not remap the VPU resources...");
		dev_err(vpu_dev, "So we can't address it, basically...");
		ret = (int) vpu_base_address;
		goto ioremap_failed;
	}

	driver_data->vpu_io.base     = vpu_base_address;
	driver_data->vpu_io.dec_regs = 
		(u32 __iomem *)
		(((u8 __iomem *) vpu_base_address) + VPU_DEC_REGS_OFFSET);

	enable_and_prepare_clocks(vpu_dev, driver_data);


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

	driver_data->output_frame.mmu_address = 
		allocate_dma_space(
			vpu_dev,
			1920*1080*4, // RGBA8888 : 4 bytes per pixel
			&driver_data->output_frame.iommu_address);

	if (!driver_data->output_frame.mmu_address)
	{
		dev_err(vpu_dev,
			"DMA Alloc coherent could not allocate\n"
			"Calling the cops right now !\n");
		ret = -ENOMEM;
		goto output_frame_dma_alloc_failed;
	}

	driver_data->encoded_frame.mmu_address =
		allocate_dma_space_and_copy(
			vpu_dev,
			sizeof(test_encoded_frame),
			&driver_data->encoded_frame.iommu_address,
			test_encoded_frame,
			sizeof(test_encoded_frame));

	if (!driver_data->encoded_frame.mmu_address)
	{
		dev_err(vpu_dev,
			"No memory for the encoded frame :C\n");
		ret = -ENOMEM;
		goto encoded_frame_dma_alloc_failed;
	}

	driver_data->cabac_table.mmu_address =
		allocate_dma_space_and_copy(vpu_dev,
			sizeof(h264_cabac_table),
			&driver_data->cabac_table.iommu_address,
			(u8 const *) h264_cabac_table,
			sizeof(h264_cabac_table));

	if (!driver_data->cabac_table.mmu_address)
	{
		dev_err(vpu_dev,
			"No memory for the cabac table :C\n");
		ret = -ENOMEM;
		goto cabac_table_dma_alloc_failed;
	}

	/* Setup the IRQ */
	irq_dec = platform_get_irq_byname(pdev, "irq_dec");
	if (irq_dec <= 0) {
		dev_err(vpu_dev,
			"Could not get the VPU Decoding IRQ number...\n");
		ret = -ENXIO;
		goto get_vdpu_irq_failed;
	}

	/* TODO Why Threaded ? Why not any context ? */
	ret = devm_request_threaded_irq(
		vpu_dev, irq_dec,
		vpu_is_done_or_borked, NULL,
		IRQF_ONESHOT, dev_name(vpu_dev), driver_data);

	if (ret) {
		dev_err(vpu_dev,
			"Could not setup the IRQ handler : %d\n", ret);
		goto get_vdpu_irq_failed;
	}

	print_dma_addresses(vpu_dev, driver_data);
	print_regs(vpu_dev, test_regs);
	prepare_the_registers(driver_data, test_regs);
	print_regs(vpu_dev, test_regs);

	copy_registers_and_launch_decoding(
		vpu_dev, driver_data, test_regs);
	return ret;

get_vdpu_irq_failed:
	free_all_dma_space(vpu_dev,
		sizeof(test_encoded_frame),
		driver_data->cabac_table.mmu_address);
cabac_table_dma_alloc_failed:
	free_all_dma_space(vpu_dev,
		sizeof(test_encoded_frame),
		driver_data->encoded_frame.mmu_address);
encoded_frame_dma_alloc_failed:
	free_all_dma_space(vpu_dev,
		1920*1080*4,
		driver_data->output_frame.mmu_address, GFP_KERNEL);
output_frame_dma_alloc_failed:
device_create_failed:
	class_destroy(driver_data->cls);
class_create_failed:
	cdev_del(cdev);
cdev_add_failed:
	unregister_chrdev_region(driver_data->device_id, 1);
chrdev_alloc_failed:
	disable_clocks(vpu_dev, driver_data);
	devm_iounmap(vpu_dev, driver_data->vpu_io.base);
ioremap_failed:
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

	free_all_dma_space(vpu_dev,
		1920*1080*4,
		driver_data->output_frame.mmu_address, GFP_KERNEL);

	free_all_dma_space(vpu_dev,
		sizeof(test_encoded_frame),
		driver_data->encoded_frame.mmu_address);

	free_all_dma_space(vpu_dev,
		sizeof(h264_cabac_table),
		driver_data->cabac_table.mmu_address);

	dev_info(vpu_dev, "DMA buffers Freed\n");

	/*disable_clocks(vpu_dev, driver_data);
	devm_ioremap_resource(vpu_dev, driver_data->vpu_io.base);*/

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
