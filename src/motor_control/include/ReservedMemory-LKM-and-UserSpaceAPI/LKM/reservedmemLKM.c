#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
// #include <linux/types.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
// #include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <asm/io.h>

MODULE_AUTHOR("Thor Kamp Opstrup");
MODULE_DESCRIPTION("Linux Kernel Module, to acces reserved memory from user space");

#define DRIVER_NAME "reservedmemLKM"
#define DRIVER_CLASS "reservedmemLKMClass"

#define P_OFFSET 0x70000000 //!UPDATE TO YOUR PHYSICAL MEMORY OFFSET
#define P_LENGTH 0x01000000 //!UPDATE TO YOUR PHYSICAL MEMORY LENGTH

static struct class *class;
static struct device *device;
static int major;

static void *pmem = NULL;

static DEFINE_MUTEX(reservedmemLKM_action_mutex); //! update to action mutex

/*  executed once the device is closed or releaseed by userspace
 *  @param inodep: pointer to struct inode
 *  @param filep: pointer to struct file
 */
static int f_close(struct inode *inodep, struct file *filep)
{
    pr_info("reservedmemLKM: Device successfully closed\n");
    return 0;
}

/* executed once the device is opened.
 *
 */
static int f_open(struct inode *inodep, struct file *filep)
{
    int ret = 0;
    pr_info("reserved_mem: Device opened\n");
    return ret;
}

/*  executed when the user calls read to file //!SOMEHOW memcpy_fromio IS REALLY SLOW
 * the information in the buffer should be uint32_t[_offset, length, u_buffer_low, u_buffer_high]
 * Once called the data with "length(Bytes)" (from above) parameter is read from pmem (starting at p_offset) to
 * user buffer
 */
static ssize_t f_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
    int ret;
    int i;
    uint32_t p_offset = 0;
    uint32_t read_length = 0;
    uintptr_t u_buffer_ptr = 0;

    if (!mutex_trylock(&reservedmemLKM_action_mutex))
    {
        pr_alert("reservedmemLKM: device busy!\n");
        ret = -EBUSY;
        goto out;
    }

    p_offset = (buffer[0] |
                (buffer[1] << 8) |
                (buffer[2] << 16) |
                (buffer[3] << 24));
    read_length = (buffer[4] |
                   (buffer[5] << 8) |
                   (buffer[6] << 16) |
                   (buffer[7] << 24));
    for (i = 8; i < 15 + 1; i++)
    {
        u_buffer_ptr |= ((uintptr_t)buffer[i] << ((i - 8) * 8));
    }

    if ((p_offset * sizeof(uint32_t) + read_length) > P_LENGTH)
    {
        pr_err("reservedmemLKM: read fault requested read dst is out of bound\n");
        ret = -EFAULT;
        goto out;
    }

    // printk("start read!\n");
    memcpy_fromio((uint32_t *)u_buffer_ptr, (uint32_t *)pmem + p_offset, read_length);
    // printk("done!\n");
    ret = 1; //! THIS IS NOT CONVENTION

out:
    // pr_info("data read\n");
    mutex_unlock(&reservedmemLKM_action_mutex);
    return ret;
}


/*  executed when the user calls write to file
 * The information written to the file should be a uint32_t[[p_offset, length, u_buffer_low, u_buffer_high]]
 * Once called the data from user buffer with "length(Bytes)"(from above) parameter is written from user buffer to
 * physical memory at the specified start(P_OFFSET) and "p_offset"(from above)
 * Returns the number of Bytes written
 */
static ssize_t f_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
    int ret;
    int i;
    uint32_t p_offset = 0;
    uint32_t write_length = 0;
    uintptr_t u_buffer_ptr = 0;

    if (!mutex_trylock(&reservedmemLKM_action_mutex))
    {
        pr_alert("reservedmemLKM: device busy!\n");
        ret = -EBUSY;
        goto out;
    }
    
    p_offset = (buffer[0] |
                (buffer[1] << 8) |
                (buffer[2] << 16) |
                (buffer[3] << 24));
    write_length = (buffer[4] |
                    (buffer[5] << 8) |
                    (buffer[6] << 16) |
                    (buffer[7] << 24));
    for (i = 8; i < 15 + 1; i++)
    {
        u_buffer_ptr |= ((uintptr_t)buffer[i] << ((i - 8) * 8));
    }
    // pr_info("u_buffer: 0x%lx\n", u_buffer_ptr);
    printk("p_offset: %i ", p_offset);
    printk("write_length: %i ", write_length);
    printk("P_LENGTH: %i\n", P_LENGTH);
    pr_info("u_buffer: 0x%lx\n", u_buffer_ptr);

    if ((p_offset * sizeof(uint32_t) + write_length) > P_LENGTH)
    {
        pr_err("reservedmemLKM: write fault requested writing dst is out of bound\n");

        ret = -EFAULT;
        goto out;
    }

    // printk("start write!\n");
    memcpy_toio((uint32_t *)pmem + p_offset, (uint32_t *)u_buffer_ptr, write_length);
    // printk("done!\n");
    ret = write_length * sizeof(uint32_t); //! THIS IS NOT CONVENTION

    pr_info("data written\n");
    mutex_unlock(&reservedmemLKM_action_mutex);
out:
    return ret;
}

static const struct file_operations reservedmemLKM_fops = {
    .open = f_open,
    .read = f_read,
    .write = f_write,
    .release = f_close,
    //set the owner of the file operations to all users (0666)
};

// static char *LMK_devnode(struct device *dev, umode_t *mode)
// {
//         if (!mode)
//                 return NULL;
//         if (dev->devt == MKDEV(TTYAUX_MAJOR, 0) ||
//             dev->devt == MKDEV(TTYAUX_MAJOR, 2))
//                 *mode = 0666;
//         return NULL;
// }


static int __init reservedmemLKM_init(void)
{
    int ret = 0;

    major = register_chrdev(0, DRIVER_NAME, &reservedmemLKM_fops);
    if (major < 0)
    {
        pr_info("reservedmemLKM: fail to register major number!\n");
        ret = major;
        goto out;
    }

    class = class_create(THIS_MODULE, DRIVER_CLASS);
    if (IS_ERR(class))
    {
        unregister_chrdev(major, DRIVER_NAME);
        pr_info("reservedmemLKM: failed to register device class");
        ret = PTR_ERR(class);
        goto out;
    }


    device = device_create(class, NULL, MKDEV(major, 0), NULL, DRIVER_NAME);
    if (IS_ERR(device))
    {
        printk("error creating file");
        class_destroy(class);
        unregister_chrdev(major, DRIVER_NAME);
        ret = PTR_ERR(device);
        goto out;
    }

    pmem = ioremap_nocache(P_OFFSET, P_LENGTH); //! UPDATE length
    if (pmem == NULL)
    {
        printk("could not ioremap_nocache/(mem == NULL)");
        ret = -ENOMEM;
        goto out;
    }

    mutex_init(&reservedmemLKM_action_mutex);

    printk("reservedmemLKM inserted!\n");
out:
    return ret;
}

static void __exit reservedmemLKM_exit(void)
{
    device_destroy(class, MKDEV(major, 0));
    class_unregister(class);
    class_destroy(class);
    iounmap(pmem);
    unregister_chrdev(major, DRIVER_NAME);

    mutex_destroy(&reservedmemLKM_action_mutex);
    printk("reservedmemLKM unregistered!\n");
}

module_init(reservedmemLKM_init);
module_exit(reservedmemLKM_exit);
MODULE_LICENSE("GPL");