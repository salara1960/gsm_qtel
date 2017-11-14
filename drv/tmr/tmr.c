//---------------------------------------------------------------------------------------------
//   Linux kernel device driver for internal timer with period 1ms for HZ=1000 (g20 gateway)
//---------------------------------------------------------------------------------------------

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <asm/atomic.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/spinlock_types.h>
//-----------------------------------------------------------------

#undef KERNEL_OLD	//2.6.26
#define KERNEL_NEW	//3.1.6

//-----------------------------------------------------------------

#ifdef KERNEL_NEW	
    #define CLASS_DEV_CREATE(class, devt, device, name) device_create(class, device, devt, NULL, "%s", name)
    #define CLASS_DEV_DESTROY(class, devt) device_destroy(class, devt)
#else
    #define CLASS_DEV_CREATE(_class, _devt, _device, _name) device_create(_class, _device, _devt, _name)
    #define CLASS_DEV_DESTROY(_class, _devt) device_destroy(_class, _devt)
#endif
static struct class *tmr_class = NULL;

#define	UNIT(file) MINOR(file->f_dentry->d_inode->i_rdev)

//--------------------------------------------------------------------------------------

#define DevName "tmr" 

#define Buf_Size 128

static unsigned char *ibuff=NULL;

static unsigned int my_msec;
static struct timer_list my_timer; 
static unsigned long long varta1;
static atomic_t varta10;

static int Major=0;
static int Device_Open=0;
static int tim10=10;
static spinlock_t tmr_lock;// = __ARCH_SPIN_LOCK_UNLOCKED;

//************************************************************
//
//************************************************************

struct tmr_sio {

  struct cdev cdev;

};
//************************************************************
//                    таймер 1 mcek
//************************************************************
void MyTimer(unsigned long d)
{

spin_lock(&tmr_lock);
    varta1++;
spin_unlock(&tmr_lock);

    tim10--;
    if (!tim10) {
	tim10=10;
	atomic_inc(&varta10);
    }
    my_timer.expires = jiffies + HZ/1000;	// 1 mсек.
    add_timer(&my_timer);

    return;

}
//***********************************************************
//               открытие устройства
//************************************************************
static int tmr_open(struct inode *inode, struct file *filp) 
{
struct tmr_sio *sio;

    if (Device_Open) return -EBUSY;

    Device_Open++;

    sio = container_of(inode->i_cdev, struct tmr_sio, cdev);

    filp->private_data = sio; //для других методов

    return 0;

}
//***********************************************************
//               закрытие устройства
//************************************************************
static int tmr_release(struct inode *inode, struct file *filp)
{

    if (Device_Open>0) Device_Open--;

    return 0;

}
//***********************************************************
//                     чтение
//************************************************************

static ssize_t tmr_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
ssize_t ret;
unsigned int tim, done=0;
unsigned long long one_ms;

    if (count==0) return (atomic_read(&varta10));

    ret=0;	memset(ibuff,0,8);

    switch (count) {//анализ заданной команды чтения
	case 4 :		// чтение 10-ти милисекундного таймера (32 разряда)
	    ret=4;
	    tim = atomic_read(&varta10);
	    memcpy(&ibuff[0],&tim,ret);
	    done=1;	
	break;
	case 8 :		// чтение милисекундного таймера (64 разряда)
	    spin_lock_bh(&tmr_lock);
		one_ms = varta1;
	    spin_unlock_bh(&tmr_lock);
	    memcpy(&ibuff[0],&one_ms,8);
	    done=1;	ret=8;
	break;
    }

    if (done)
	if(copy_to_user(buff,ibuff,ret)) {
	    printk(KERN_ALERT "\n%s: Kernel tmr_read ERROR (copy_to_user) : count=%u\n", DevName, ret);
	    return -EFAULT;
	}

    return ret;

}
//***********************************************************
//		запись
//************************************************************

static ssize_t tmr_write(struct file *filp, const char __user *buff,  size_t count, loff_t *offp)
{
ssize_t ret=0;
unsigned char cmd;
unsigned int zero;


    ret=count; if (ret>Buf_Size) return -EFAULT;

    if (copy_from_user(ibuff,buff,ret)) {
	printk(KERN_ALERT "\n%s: Kernel tmr_write ERROR (copy_from_user) : count=%u\n", DevName, ret);
	return -EFAULT;
    }

    cmd = *(ibuff);
    switch (cmd) { //анализ принятой команды
	case 2:
	    if (count != 1) return -EFAULT;  	// длинна должна быть 1 байт !!!
	    zero=0;	atomic_set(&varta10, zero);//10ms clear
	    spin_lock_bh(&tmr_lock);
		varta1 = 0;
	    spin_unlock_bh(&tmr_lock);
	    ret=1;
	    printk(KERN_ALERT "\n%s: clear all timers (10ms & 1ms) from user\n", DevName);
	break;

	case 4://сбросить 10-ти милесекундный таймер
	    if (count != 1) return -EFAULT;  	// длинна должна быть 1 байт !!!
	    zero=0;	atomic_set(&varta10, zero);
	    ret=1;
	    printk(KERN_ALERT "%s: clear timer (10ms) from user\n", DevName);
	break;
	case 8://сбросить милесекундный таймер
	    if (count != 1) return -EFAULT;  	// длинна должна быть 1 байт !!!
	    zero=0;
	    spin_lock_bh(&tmr_lock);
		varta1 = 0;
	    spin_unlock_bh(&tmr_lock);
	    ret=1;
	    printk(KERN_ALERT "%s: clear timer (1ms) from user\n", DevName);
	break;
    }

    return ret;

}
//*************************************************************
static struct file_operations tmr_fops = {

  .owner   = THIS_MODULE,
  .open    = tmr_open,
  .release = tmr_release,
  .read    = tmr_read,
  .write   = tmr_write,

};
//************************************************************
//
//************************************************************
static void init_sio(struct tmr_sio *sio) {

  dev_t dev = MKDEV(Major,0);

  cdev_init(&sio->cdev, &tmr_fops);

  cdev_add(&sio->cdev, dev, 1);

}
//************************************************************
//
//************************************************************

static void deinit_sio(struct tmr_sio *sio) {

  cdev_del(&sio->cdev);

}

static struct tmr_sio chan_sio;

// ************************************************************
//		инициализация модуля
// ************************************************************
static int __init tmr_init(void)
{
dev_t dev;

    if (!Major) {
	if ((alloc_chrdev_region(&dev, 0, 1, DevName)) < 0){
	    printk(KERN_ALERT "%s: Allocation device failed\n", DevName);
	    return 1;
	}
	Major = MAJOR(dev);
	printk(KERN_ALERT "%s: Device allocated with major number %d\n", DevName, Major);
    } else {
	if (register_chrdev_region(MKDEV(Major,0), 1, DevName) < 0) {
	    printk(KERN_ALERT "%s: Registration failed\n", DevName);
	    return 1;
	}
	printk(KERN_ALERT "%s: Device registered with major number %d\n", DevName, Major);
    }

    init_sio(&chan_sio);

    tmr_class = class_create(THIS_MODULE, "timer");

    CLASS_DEV_CREATE(tmr_class, MKDEV(Major, 0), NULL, DevName);

//--------------------------------------------------------------------

    ibuff = kmalloc(Buf_Size,GFP_KERNEL);
    if (ibuff == NULL){
	printk(KERN_ALERT "%s: VM for reading buffer allocation failed\n", DevName);
	goto err_out;
    }

    spin_lock_init(&tmr_lock); 

    Device_Open=0;
    my_msec=0;	tim10=10;    varta1 =0;
    atomic_set(&varta10, my_msec);
    init_timer(&my_timer);
    my_timer.function = MyTimer;
    my_timer.expires = jiffies + 10;//1 //HZ/1000;	// 1 msec
    add_timer(&my_timer);

    printk(KERN_ALERT "%s: Start timer (1ms)\n", DevName);

    return 0;

err_out:

    CLASS_DEV_DESTROY(tmr_class, MKDEV(Major, 0));
    class_destroy(tmr_class);

    if (ibuff!=NULL) kfree(ibuff);

    return -1;
}
//************************************************************
//                   выгрузка модуля
//************************************************************
static void __exit tmr_exit(void)
{

    del_timer(&my_timer);

    if (ibuff!=NULL) kfree(ibuff);

    unregister_chrdev_region(MKDEV(Major, 0), 1);

    deinit_sio(&chan_sio);

    CLASS_DEV_DESTROY(tmr_class, MKDEV(Major, 0));
    class_destroy(tmr_class);

    printk(KERN_ALERT "%s: Unregistered timer, release memory buffer\n", DevName);

    return;

}

module_init(tmr_init);
module_exit(tmr_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<salara@ltd.com.ua>");
