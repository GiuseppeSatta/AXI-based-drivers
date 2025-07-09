#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>           
#include <linux/err.h>           

#define IMAGE_WIDTH 88
#define IMAGE_HEIGHT 142
#define IMAGE_SIZE (IMAGE_HEIGHT*IMAGE_WIDTH)
#define IMAGE_DATA_SIZE (IMAGE_SIZE*4)

#define DRIVER_NAME "my_dma_driver"
#define DEVICE_NAME "my_dma_chrdev"

#define DMA_REG_SIZE  0x10000
#define DMA_BUF_SIZE  IMAGE_SIZE

#define S2MM_DMACR   0x30
#define S2MM_DMASR   0x34
#define S2MM_SA      0x48
#define S2MM_LENGTH  0x58

#define MM2S_DMACR   0x00
#define MM2S_DMASR   0x04
#define MM2S_SA      0x18
#define MM2S_LENGTH  0x28

#define RESET_TIMEOUT_US 10000
#define DMA_RESET_BIT 0x4

#define ACC_BASE_ADDR  0xB0000000
#define ACC_SIZE 0x10000
#define ACC_CTR_REG_ADDR_OFFSET 0x00
#define ACC_GIE_REG_ADDR_OFFSET 0x04 //Global Interrupt Enable Register
#define ACC_IER_REG_ADDR_OFFSET 0x08 //IP Interrupt Enable Register
#define ACC_OUT_DATA_REG_ADDR_OFFSET 0x10
#define ACC_OUT_CTRL_REG_ADDR_OFFSET 0x14
#define ACC_OUT_DATA_SIZE 0x4

#define AP_READY_MASK 0x08
#define AP_START_MASK 0x01
#define DMA_HALTED_MASK 0x01
#define DMA_IDLE_MASK 0x2

#define EXPECTED_RESULT 0x28

struct my_device {
    struct device *dev;
    void *dma_virt_tx;
    void *dma_virt_rx;
    dma_addr_t dma_phys_tx;
    dma_addr_t dma_phys_rx;
    size_t dma_buffer_size;
    void *dma_ctr;

    struct cdev cdev;
    dev_t devt;
    struct class *class;
};

static void *vpa_ip_ctr;

static struct my_device *global_mydev;

static int dma_reset_wait(void __iomem *base, u32 offset) {
    u32 val;
    int timeout = RESET_TIMEOUT_US;

    iowrite32(DMA_RESET_BIT, base + offset);
    do {
        val = ioread32(base + offset);
        if (!(val & DMA_RESET_BIT)){
            printk("dma reset\n");
            return 0;
        }
        udelay(1);
    } while (--timeout);

    dev_err(global_mydev->dev, "axidma: DMA reset timeout\n");
    return -ETIMEDOUT;
}


/* Character device file ops */
static ssize_t chrdev_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    u32 res;

    if (len != ACC_OUT_DATA_SIZE)
        len = ACC_OUT_DATA_SIZE;

    
    res = (u32)ioread32(vpa_ip_ctr + ACC_OUT_DATA_REG_ADDR_OFFSET);

    if (copy_to_user(buf, &res, len))
        return -EFAULT;

    return len;
}

static ssize_t chrdev_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
    u32 res;
    if (len != IMAGE_DATA_SIZE)
        len = IMAGE_DATA_SIZE;

    int timeout=0;
    iowrite32(AP_START_MASK,vpa_ip_ctr + ACC_CTR_REG_ADDR_OFFSET);

    res=ioread32(vpa_ip_ctr + ACC_CTR_REG_ADDR_OFFSET);

    printk("ACC_CTR_REG = %u",res);

    iowrite32(0x1,vpa_ip_ctr + ACC_GIE_REG_ADDR_OFFSET);
    
    res=ioread32(vpa_ip_ctr + ACC_GIE_REG_ADDR_OFFSET);
    printk("ACC_GIE_REG = %u",res);

    iowrite32(0x3,vpa_ip_ctr + ACC_IER_REG_ADDR_OFFSET);
    res=ioread32(vpa_ip_ctr + ACC_IER_REG_ADDR_OFFSET);
    printk("ACC_IER_REG = %u\n",res);

    printk("%p",global_mydev->dma_virt_tx);
    if(!global_mydev->dma_virt_rx){
        printk("end of write function");
        return -1;
    }

    if (!access_ok(buf, len)) {
        pr_err("AXIDMA: Invalid user pointer\n");
        return -EFAULT;
    }

    if (copy_from_user(global_mydev->dma_virt_tx, buf, len/4))
        return -EFAULT;


    res=ioread32(global_mydev->dma_ctr + MM2S_DMASR);

    printk("status register: %u\n",res);


    iowrite32(DMA_RESET_BIT, global_mydev->dma_ctr + MM2S_DMACR);
    if (dma_reset_wait(global_mydev->dma_ctr, MM2S_DMACR))
        return -EIO;


    res=ioread32(global_mydev->dma_ctr + MM2S_DMASR);

    printk("status register: %u\n",res);

    iowrite32(0x1, global_mydev->dma_ctr + MM2S_DMACR);

    res=ioread32(global_mydev->dma_ctr + MM2S_DMACR);

    printk("control register after start: %u\n",res);

    iowrite32(global_mydev->dma_phys_tx, global_mydev->dma_ctr + MM2S_SA);

    res=ioread32(global_mydev->dma_ctr + MM2S_SA);

    printk("source address register: %u\n",res);

    printk("number of bytes to transfer: %lu\n",len);

    iowrite32(len, global_mydev->dma_ctr + MM2S_LENGTH);

    res=ioread32(global_mydev->dma_ctr + MM2S_LENGTH);

    printk("transfer length register: %u\n",res);

    timeout=0;

    res=ioread32(global_mydev->dma_ctr + MM2S_DMASR);

    printk("status register: %u\n",res);

    while ((!((res & DMA_IDLE_MASK) || (res & DMA_HALTED_MASK))) && timeout < 100000){
        udelay(10);
        timeout++;
        res=ioread32(global_mydev->dma_ctr + MM2S_DMASR);
    }
    
    if(timeout >= 100000) {
        printk("write dma_idle timeout\n");
        res=ioread32(global_mydev->dma_ctr + MM2S_DMASR);

        printk("status register: %u\n",res);
        return 0;
    }

    return len;
}

static int chrdev_open(struct inode *inode, struct file *filp)
{
    filp->private_data = global_mydev;
    return 0;
}

static struct file_operations chrdev_fops = {
    .owner = THIS_MODULE,
    .read = chrdev_read,
    .write = chrdev_write,
    .open = chrdev_open
};

/* Probe */
static int dt_probe(struct platform_device *pdev)
{
    printk("dt_probe - Now I am in the probe function!\n");
    struct my_device *mydev;
    int ret;
    struct resource * res;
    void __iomem *regs;
    dev_info(&pdev->dev, "Probing %s\n", DRIVER_NAME);

    mydev = devm_kzalloc(&pdev->dev, sizeof(*mydev), GFP_KERNEL);
    if (!mydev)
        return -ENOMEM;

    mydev->dev = &pdev->dev;


    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "No IORESOURCE_MEM\n");
        return -ENODEV;
    }

    
    struct clk *clk;

    clk = devm_clk_get(&pdev->dev, "s_axi_lite_aclk");

    if (IS_ERR(clk)) {
        dev_err(&pdev->dev, "Failed to get clock\n");
        return PTR_ERR(clk);
    }

    ret = clk_prepare_enable(clk);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable clock\n");
        return ret;
    }
    struct clk *clk2 = devm_clk_get(&pdev->dev, "m_axi_mm2s_aclk");

    if (!IS_ERR(clk2)) {
        ret=clk_prepare_enable(clk2);
        if (ret) {
            dev_err(&pdev->dev, "Failed to enable clock\n");
            return ret;
        }
    }

    unsigned long clk_rate = clk_get_rate(clk);
    dev_info(&pdev->dev, "Clock s_axi_lite_aclk rate: %lu Hz\n", clk_rate);
    clk_get_rate(clk2);
    dev_info(&pdev->dev, "Clock s_axi_lite_aclk rate: %lu Hz\n", clk_rate);

    regs = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(regs)) {
        dev_err(&pdev->dev, "Failed to map registers\n");
        return PTR_ERR(regs);
    }
    
    printk("DMA controller base: 0x%llx, size: 0x%lx, virt: %p\n",(unsigned long long)res->start,(unsigned long)resource_size(res), regs);


    vpa_ip_ctr = ioremap(ACC_BASE_ADDR,ACC_SIZE);

    if (!vpa_ip_ctr) {
        printk("vpa remap error"); 
        return 0;
    }
    
    printk("vpa mapped\n");
    iowrite32(0x4,regs);
    printk("test read from dma ctrl reg: 0x%08x\n", ioread32(regs + 0x0));
    iowrite32(0x1,regs);
    printk("test read from dma ctrl reg after enable: 0x%08x\n", ioread32(regs + 0x0));
    printk("test read from dma status reg: 0x%x\n", *(unsigned int *)(regs+4));

    

    global_mydev = mydev;

    ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
    if (ret) return ret;
    ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
    if (ret) return ret;

    mydev->dma_buffer_size = DMA_BUF_SIZE;
    mydev->dma_virt_tx = dma_alloc_coherent(&pdev->dev, mydev->dma_buffer_size,
                                         &mydev->dma_phys_tx, GFP_KERNEL);
    if (!mydev->dma_virt_tx)
        return -ENOMEM;

    mydev->dma_virt_rx = dma_alloc_coherent(&pdev->dev, mydev->dma_buffer_size,
                                         &mydev->dma_phys_rx, GFP_KERNEL);
    if (!mydev->dma_virt_rx)
        return -ENOMEM;

    printk("coherent alloc success - tx: %p - rx: %p", mydev->dma_virt_tx, mydev->dma_virt_rx);

    /* Allocate char device number */
    ret = alloc_chrdev_region(&mydev->devt, 0, 1, DEVICE_NAME);
    if (ret) goto err_dma;

    /* Create device class */
    mydev->class = class_create(DEVICE_NAME);
    if (IS_ERR(mydev->class)) {
        ret = PTR_ERR(mydev->class);
        goto err_chrdev;
    }

    /* Initialize char device */
    cdev_init(&mydev->cdev, &chrdev_fops);
    mydev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&mydev->cdev, mydev->devt, 1);
    if (ret) goto err_class;

    /* Create device node */
    device_create(mydev->class, NULL, mydev->devt, NULL, DEVICE_NAME);

    dev_info(&pdev->dev, "Char device registered: /dev/%s\n", DEVICE_NAME);

    platform_set_drvdata(pdev, mydev);

    return 0;

err_class:
    class_destroy(mydev->class);
err_chrdev:
    unregister_chrdev_region(mydev->devt, 1);
err_dma:
    dma_free_coherent(&pdev->dev, mydev->dma_buffer_size, mydev->dma_virt_tx, mydev->dma_phys_tx);
    dma_free_coherent(&pdev->dev, mydev->dma_buffer_size, mydev->dma_virt_rx, mydev->dma_phys_rx);
    return ret;
}

/* Remove */
static int dt_remove(struct platform_device *pdev)
{
    struct my_device *mydev = platform_get_drvdata(pdev);

    device_destroy(mydev->class, mydev->devt);
    cdev_del(&mydev->cdev);
    class_destroy(mydev->class);
    unregister_chrdev_region(mydev->devt, 1);

    dma_free_coherent(&pdev->dev, mydev->dma_buffer_size, mydev->dma_virt_tx, mydev->dma_phys_tx);
    dma_free_coherent(&pdev->dev, mydev->dma_buffer_size, mydev->dma_virt_rx, mydev->dma_phys_rx);

    dev_info(&pdev->dev, "Device removed\n");
    return 0;
}

static const struct of_device_id dt_of_match[] = {
    { .compatible = "xlnx,axi-dma", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dt_of_match);

static struct platform_driver dt_driver = {
    .probe = dt_probe,
    .remove = dt_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = dt_of_match,
    },
};

module_platform_driver(dt_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Giuseppe Satta");
MODULE_DESCRIPTION("Platform driver with DMA and character device");