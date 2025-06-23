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

#define DEVICE_NAME "inverter"
#define CLASS_NAME  "invclass"
u32 reg[2];
#define PHYS_ADDR reg[0]    // Replace with physical address, to be automated
#define MAP_SIZE reg[1]     // Replace with range, to be automated

static int major;
static struct class *invchar_class;
static void __iomem *mapped_mem;
static struct of_device_id inverter_ids[] = {
	{
		.compatible = "xlnx,inverter_1.0",
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, inverter_ids);

static int dt_probe(struct platform_device *);
static int dt_remove(struct platform_device *);

static struct platform_driver inverter = {
	.probe = dt_probe,
	.remove = dt_remove,
	.driver = {
		.name = "inverter",
		.of_match_table = inverter_ids,
	},
};

static int dt_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	int ret;

	printk("dt_probe - Now I am in the probe function!\n");

	/* Check for device properties */
	if(!device_property_present(dev, "reg")) {
		printk("dt_probe - Error! Device property 'reg' not found!\n");
		return -1;
	}

	/* Read device properties */
	ret = device_property_read_u32_array(dev, "reg", reg, 2);
	if(ret) {
		printk("dt_probe - Error! Could not read 'reg'\n");
		return -1;
	}
	printk("dt_probe - reg: %x, %x\n", reg[0], reg[1]);

	return 0;
}

static int dt_remove(struct platform_device *pdev) {
	printk("dt_probe - Now I am in the remove function\n");
	return 0;
}

static int inverter_open(struct inode *inode, struct file *file) {
    pr_info("inverter: device opened\n");
    return 0;
    //If we wanted to only allocate memory when the device was used we can put the ioremap function here 
    //and the iounmap function in inverter_release, but that could be risky
    //if the user doesn't close the file properly every time and it wastes time remapping the memory multiple times.
    //This function can be used for logging for example, or other session specific operations. 
    //An option could be for implementing process exclusivity.
}

static int inverter_release(struct inode *inode, struct file *file) {
    pr_info("inverter: device closed\n");
    return 0;
}

static ssize_t inverter_read(struct file *file, char __user *buf, size_t len, loff_t *offset) {
    if (*offset >= MAP_SIZE)
        return 0;

    if (*offset + len > MAP_SIZE)
        len = MAP_SIZE - *offset;

    if (copy_to_user(buf, (u8 __force *)mapped_mem + *offset, len))
        return -EFAULT;

    *offset += len;
    return len;
}

static ssize_t inverter_write(struct file *file, const char __user *buf, size_t len, loff_t *offset) {
    if (*offset >= MAP_SIZE)
        return -EINVAL;

    if (*offset + len > MAP_SIZE)
        len = MAP_SIZE - *offset;

    if (copy_from_user((u8 __force *)mapped_mem + *offset, buf, len))
        return -EFAULT;

    *offset += len;
    return len;
}


static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = inverter_open,
    .release = inverter_release,
    .read = inverter_read,
    .write = inverter_write
};

static int __init inverter_init(void) {
    struct device *dev;


    if(platform_driver_register(&inverter)) {
		printk("dt_probe - Error! Could not load driver\n");
		return -1;
	}

    // Allocate major number
    major = register_chrdev(0, DEVICE_NAME, &fops);
    if (major < 0) {
        pr_err("inverter: failed to register char device\n");
        return major;
    }

    // Create device class
    invchar_class = class_create(CLASS_NAME);
    if (IS_ERR(invchar_class)) {
        unregister_chrdev(major, DEVICE_NAME);
        return PTR_ERR(invchar_class);
    }

    // Create device node
    dev = device_create(invchar_class, NULL, MKDEV(major, 0), NULL, DEVICE_NAME);
    if (IS_ERR(dev)) {
        class_destroy(invchar_class);
        unregister_chrdev(major, DEVICE_NAME);
        return PTR_ERR(dev);
    }

    // Map physical memory to kernel virtual address space
    mapped_mem = ioremap(PHYS_ADDR, MAP_SIZE);
    if (!mapped_mem) {
        device_destroy(invchar_class, MKDEV(major, 0));
        class_destroy(invchar_class);
        unregister_chrdev(major, DEVICE_NAME);
        return -EIO;
    }

    pr_info("inverter: module loaded, mapped 0x%x bytes at phys 0x%x, virt %p\n", MAP_SIZE, PHYS_ADDR, mapped_mem);

    return 0;
}

static void __exit inverter_exit(void) {
    iounmap(mapped_mem);
    device_destroy(invchar_class, MKDEV(major, 0));
    class_destroy(invchar_class);
    unregister_chrdev(major, DEVICE_NAME);
    platform_driver_unregister(&inverter);
    pr_info("invchar: module unloaded\n");
}

module_init(inverter_init);
module_exit(inverter_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Giuseppe Satta");
MODULE_DESCRIPTION("Inverter driver with ioremap, character device and device tree probing attempt");

