#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/mm.h>

#include "offsets.h"

#define DEVICE_NAME "axidma"
#define CLASS_NAME  "axidmaclass"

//Commented code is for device tree integration.

//int reg_dma[2];
//int reg_fifo[2];

// #define DMA_PHYS_ADDRESS reg_dma[0]
// #define DMA_SIZE reg_dma[1]
// #define FIFO__PHYS_ADDRESS reg_fifo[0]
// #define FIFO_SIZE reg_fifo[1]

static int major;
static struct class *dmachar_class;
static void __iomem *mapped_mem;

// static struct of_device_id axidma_ids[] = {
// 	{
// 		.compatible = "xlnx,axidma_1.0",
// 	}, { /* sentinel */ }
// };
// MODULE_DEVICE_TABLE(of, axidma_ids);

// static int dt_probe(struct platform_device *);
// static int dt_remove(struct platform_device *);

// static struct platform_driver axidma = {
// 	.probe = dt_probe,
// 	.remove = dt_remove,
// 	.driver = {
// 		.name = "axidma",
// 		.of_match_table = axidma_ids,
// 	},
// };

// static int dt_probe(struct platform_device *pdev) {
// 	struct device *dev = &pdev->dev;
// 	int ret;

// 	printk("dt_probe - Now I am in the probe function!\n");

// 	/* Check for device properties */
// 	if(!device_property_present(dev, "reg")) {
// 		printk("dt_probe - Error! Device property 'reg' not found!\n");
// 		return -1;
// 	}

// 	/* Read device properties */
// 	ret = device_property_read_u32_array(dev, "reg", reg, 2);
// 	if(ret) {
// 		printk("dt_probe - Error! Could not read 'reg'\n");
// 		return -1;
// 	}
// 	printk("dt_probe - reg: %x, %x\n", reg[0], reg[1]);

// 	return 0;
// }

// static int dt_remove(struct platform_device *pdev) {
// 	printk("dt_probe - Now I am in the remove function\n");
// 	return 0;
// }

static int axidma_open(struct inode *inode, struct file *file) {
    pr_info("axidma: device opened\n");
    return 0;
    //If we wanted to only allocate memory when the device was used we can put the ioremap function here 
    //and the iounmap function in axidma_release, but that could be risky
    //if the user doesn't close the file properly every time and it wastes time remapping the memory multiple times.
    //This function can be used for logging for example, or other session specific operations. 
    //An option could be for implementing process exclusivity.
}

static int axidma_release(struct inode *inode, struct file *file) {
    pr_info("axidma: device closed\n");
    return 0;
}

static ssize_t axidma_read(struct file *file, char __user *buf, size_t len, loff_t *offset) {
    if (*offset >= DMA_SIZE)
        return 0;

    if (*offset + len > DMA_SIZE)
        len = DMA_SIZE - *offset;

    if (copy_to_user(buf, (u8 __force *)mapped_mem + *offset, len))
        return -EFAULT;

    *offset += len;
    return len;
}

static ssize_t axidma_write(struct file *file, const char __user *buf, size_t len, loff_t *offset) {
    if (*offset >= DMA_SIZE)
        return -EINVAL;

    if (*offset + len > DMA_SIZE)
        len = DMA_SIZE - *offset;

    if (copy_from_user((u8 __force *)mapped_mem + *offset, buf, len))
        return -EFAULT;

    *offset += len;
    return len;
}

static int axidma_mmap(struct file *file, struct vm_area_struct *vma) { //returns pointer to userspace,
    unsigned long phys_start = DMA_PHYS_ADDRESS;                               //useful for testing
    unsigned long vma_size = vma->vm_end - vma->vm_start;

    if (vma_size > DMA_SIZE)
        return -EINVAL;

    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

    if (remap_pfn_range(vma,
                        vma->vm_start,
                        phys_start >> PAGE_SHIFT,
                        vma_size,
                        vma->vm_page_prot)) {
        pr_err("axidma: remap_pfn_range failed\n");
        return -EAGAIN;
    }

    pr_info("axidma: mmap successful (userspace 0x%lx -> phys 0x%lx)\n",
            vma->vm_start, phys_start);

    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = axidma_open,
    .release = axidma_release,
    .read = axidma_read,
    .write = axidma_write,
    .mmap = axidma_mmap
};

static int __init axidma_init(void) {
    struct device *dev;


    // if(platform_driver_register(&axidma)) {
	// 	printk("dt_probe - Error! Could not load driver\n");
	// 	return -1;
	// }

    // Allocate major number
    major = register_chrdev(0, DEVICE_NAME, &fops);
    if (major < 0) {
        pr_err("axidma: failed to register char device\n");
        return major;
    }

    // Create device class
    dmachar_class = class_create(CLASS_NAME);
    if (IS_ERR(dmachar_class)) {
        unregister_chrdev(major, DEVICE_NAME);
        return PTR_ERR(dmachar_class);
    }

    // Create device node
    dev = device_create(dmachar_class, NULL, MKDEV(major, 0), NULL, DEVICE_NAME);
    if (IS_ERR(dev)) {
        class_destroy(dmachar_class);
        unregister_chrdev(major, DEVICE_NAME);
        return PTR_ERR(dev);
    }

    // Map physical memory to kernel virtual address space
    mapped_mem = ioremap(DMA_PHYS_ADDRESS, DMA_SIZE);
    if (!mapped_mem) {
        device_destroy(dmachar_class, MKDEV(major, 0));
        class_destroy(dmachar_class);
        unregister_chrdev(major, DEVICE_NAME);
        return -EIO;
    }

    pr_info("axidma: module loaded, mapped 0x%x bytes at phys 0x%x, virt %p\n", DMA_SIZE, DMA_PHYS_ADDRESS, mapped_mem);

    return 0;
}

static void __exit axidma_exit(void) {
    iounmap(mapped_mem);
    device_destroy(dmachar_class, MKDEV(major, 0));
    class_destroy(dmachar_class);
    unregister_chrdev(major, DEVICE_NAME);
    // platform_driver_unregister(&axidma);
    pr_info("dmachar: module unloaded\n");
}

module_init(axidma_init);
module_exit(axidma_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Giuseppe Satta");
MODULE_DESCRIPTION("axidma driver with ioremap, character device and device tree probing attempt");

