
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

static int test_user_dma_open(struct inode *inode, struct file *filp) {
	printk(KERN_INFO "Open !");
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
	printk(KERN_INFO "MMAP !");
	return 0;
};
static int test_user_dma_release(struct inode * inode,
	struct file * filp)
{
	printk(KERN_INFO "Close !");
	return 0;
};

static dev_t test_user_dma_dev;
static struct cdev test_user_dma_cdev;

dma_addr_t myy_dma_handle;

static struct file_operations test_user_dma_fops = {
	.owner          = THIS_MODULE,
	.open           = test_user_dma_open,
	.unlocked_ioctl = test_user_dma_ioctl,
	.mmap           = test_user_dma_mmap,
	.release        = test_user_dma_release
};

#define N_PAGES_FOR_4Kx4Kx4_BYTES_PER_COLOR 4000
void *myy_dma_coherent_va_ptr;

static int __init test_user_dma_init(void) {
	int ret = 0;

	printk(KERN_INFO "Use dma_alloc_coherent\n");

	/* Some examples imply a +2 here... but I have no idea
	 * why you should allocate enough pages + 2 extra pages
	 * ... ?
	 */
	/* Let's try to allocate enough space to map a
	 * 4K*4K*4 bytes per pixel buffer */
	myy_dma_coherent_va_ptr = dma_alloc_coherent(NULL,
		N_PAGES_FOR_4Kx4Kx4_BYTES_PER_COLOR * PAGE_SIZE,
		&myy_dma_handle, GFP_KERNEL);

	if (!myy_dma_coherent_va_ptr)
	{
		printk(KERN_INFO
			"DMA Alloc coherent could not allocate\n"
			"Calling the cops right now !\n");
		ret = -ENOMEM;
		goto failed_dma_alloc;
	}

	if ((ret = alloc_chrdev_region(
	     &test_user_dma_dev, 0, 1, "myy_dma")) < 0)
	{
		printk(KERN_ERR
			"chrdev_region failed for unknown reasons !\n");
		goto failed_chrdev_alloc;
	}

	cdev_init(&test_user_dma_cdev, &test_user_dma_fops);
	if ((ret = cdev_add(&test_user_dma_cdev, test_user_dma_dev, 1))
	     < 0)
	{
		printk(KERN_ERR "cdev_add failed ! Fuck cdev_add !");
		goto failed_cdev_add;
	}

	return ret;

failed_cdev_add:
	unregister_chrdev_region(test_user_dma_dev, 1);
failed_chrdev_alloc:
	dma_free_coherent(NULL, N_PAGES_FOR_4Kx4Kx4_BYTES_PER_COLOR,
		&myy_dma_handle, GFP_KERNEL);
		
failed_dma_alloc:
	return ret;
}

/* module unload */
static void __exit test_user_dma_exit(void)
{
	/* remove the character deivce */
	cdev_del(&test_user_dma_cdev);
	/* Free the space allocated for the character device */
	unregister_chrdev_region(test_user_dma_dev, 1);

	/* free the DMA mapped areas */
	dma_free_coherent(NULL, N_PAGES_FOR_4Kx4Kx4_BYTES_PER_COLOR,
		&myy_dma_handle, GFP_KERNEL);
}

module_init(test_user_dma_init);
module_exit(test_user_dma_exit);
MODULE_DESCRIPTION("Test the allocation of DMA'ble memory and sharing with the user");
MODULE_AUTHOR("Myy");
MODULE_LICENSE("GPL");
