#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/nvme.h>
#include <linux/dmi.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/pid_namespace.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/mm.h>
#include <asm/io.h>
#include "chardriver_ioctl.h"
#define CHARDRIVER_MAJOR       311
#define CHARDRIVER_MAX_MINORS  1   //number of device nodes to manage
#define CHARDRIVER_BUFFER_SIZE   4096
#define MAX_OPEN_ALLOWED     3
#define OPEN_RDONLY    0X0
#define OPEN_WRITEONLY   0X1
#define OPEN_RDWR     0X2
#define OPEN_MODE_MASK   0x3
#define OPEN_EXCL     0X4
#define INVALID_PID   -1
//debug macros
//#define CHARDRIVER_DEBUG   1
#ifdef CHARDRIVER_DEBUG
#define CD_DEBUG(a,b...) printk(KERN_NOTICE a,##b)
#else
#define CD_DEBUG(a,b...)
#endif

struct chardriver_data {
    struct cdev cdev;
    struct mutex cd_state_mutex;
    struct lock_class_key cd_state_key;
    dev_t device;
    unsigned char* hwptr;
    int hwptr_size;
    int excl_use;
    struct open_state {
        int open_mode;
        pid_t owner;
        struct file *filep;
    } dev_open_state[MAX_OPEN_ALLOWED];
    int open_counter;
    int minor;
    /* my data starts here */
    stats_t devicestats;
};
