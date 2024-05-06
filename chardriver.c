/*  In this character driver, I demonstrate how to manipulate a hardware buffer by implementing chardriver_init, _open, _read, _write, 
 *  _ioctl, and mmap functions. Through this process, I also learned how to create test cases for each of these functions and learned 
 *  how to debug Kernel level code using CD_DEBUG.  This driver's major number is 311 and the driver can support up to 3 devices of the
 *  major driver type.
*/

#include "chardriver.h"


struct chardriver_data devs[CHARDRIVER_MAX_MINORS];

/* User can access a hardware buffer with Open exlcusive, write, read, read/write. In this function I check for whether there is an
 * open exclusive device, in which another minor device cannot be used or open.  If another device is open and a open exlusive
 * is issued then we will deny the open exclusive and wait for the other minor devices to free up. When issuing an open we need
 * a mutex in order to make sure that the hardware buffer is not corrupted when the other minor devices are accessing the buffer.
*/
int chardriver_open(struct inode *inp, struct file *p){

    bool found_slot = false;
    struct chardriver_data *cp;
    int minor = MINOR(inp->i_rdev);
    int open_state_index;
    int slot;
    struct open_state *osp;
    stats_t *sp;
    cp = &devs[minor];
    sp = &cp->devicestats;

    mutex_lock(&cp->cd_state_mutex);
    open_state_index = cp->open_counter;

    if(open_state_index >= MAX_OPEN_ALLOWED){
    	mutex_unlock(&cp->cd_state_mutex);
	pr_err("Chardriver: Open: # of opens are over allowed minor: %d", minor);
	return -EMFILE;
    }

    CD_DEBUG("open chardriver\n");
    CD_DEBUG("file pointer: %p\n", p);
    CD_DEBUG("fmode is: 0x%x\n", p->f_mode);
    CD_DEBUG("fflags is: 0x%x\n", p->f_flags);
    
    //if exclusive open exists return EBUSY
    if(cp->excl_use == OPEN_EXCL){
	BUG_ON(open_state_index != 1);
	sp->openfail++;
	mutex_unlock(&cp->cd_state_mutex);
	return -EBUSY;
    }

    //open another device (minor number) if it is not open already
    for(int i = 0;i < MAX_OPEN_ALLOWED;i++){
        osp = &(cp->dev_open_state[i]);
	if(osp->owner == INVALID_PID){  //Check for open minor slot
	     found_slot = true;
	     slot = i;
	     break;
	}
    }	

    BUG_ON(!found_slot);

    if((p->f_flags & O_EXCL) == O_EXCL){
        if(open_state_index > 0){
	    sp->openfail++;
	    mutex_unlock(&cp->cd_state_mutex);
	    return -EBUSY;
	}
	osp->open_mode |= OPEN_EXCL;
        cp->excl_use |= OPEN_EXCL; 
    }

    osp->open_mode |= (p->f_flags & 0x3);
    osp->owner = pid_vnr(p->f_owner.pid);
    osp->filep = p;
    p->private_data = cp;
    cp->open_counter++;
    sp->opencount++;

    CD_DEBUG("Open: cp value: %p\n", cp);

    CD_DEBUG("%s: open mode: 0x%x, owner: %d; open_counter: %d, slot: %d\n",
             __func__, osp->open_mode, osp->owner, cp->open_counter, slot);

    CD_DEBUG("Open: hw ptr: %p, minor: %d\n", cp->hwptr, cp->minor);
    mutex_unlock(&cp->cd_state_mutex);
    
    
    return 0;

}

/* This function releases a file stucture. I find the right file by looking at the filep pointer stored in the chardriver_data structure.  
After we find the right device file to close, we set the device file's open state to close. We also set the owner to INVALID_PID and the 
file pointer to NULL. The open state is what determines whether a new open can be issued. 
*/
int chardriver_release(struct inode *inp, struct file *p){
    
    struct chardriver_data *cp;
    int minor = MINOR(inp->i_rdev);
    int open_state_index;
    int slot;
    bool found_slot = false;
    struct open_state *osp;
    stats_t *sp;
    cp = &devs[minor];
    sp = &cp->devicestats;

    CD_DEBUG("Release file pointer: %p\n", p); 
    
    mutex_lock(&cp->cd_state_mutex);
    open_state_index = devs[minor].open_counter;
    BUG_ON(open_state_index < 1);
    for(int i=0;i<MAX_OPEN_ALLOWED;i++){
	    osp = &(cp->dev_open_state[i]);
	    if(osp->filep == p){
	    found_slot = true;
            slot = i;
	    break;
	    }
    }

    BUG_ON(!found_slot);

    if(cp->excl_use == OPEN_EXCL){
       BUG_ON(!((osp->open_mode & OPEN_EXCL) == OPEN_EXCL)); 
       cp->excl_use &= ~(OPEN_EXCL);
    }
    //set open state to close
    osp->open_mode &= ~(p->f_flags & 0x3);
    osp->owner = INVALID_PID;
    osp->filep = NULL;
    cp->open_counter--;
    sp->closecount++;

    CD_DEBUG("%s: open mode: 0x%x, owner: %d; open_counter: %d, slot: %d\n",
            __func__, osp->open_mode, osp->owner, cp->open_counter, slot);

    mutex_unlock(&cp->cd_state_mutex);

    pr_info("release chardriver\n");
    return 0;

}
/*The llseek function is used to change the current read/write position of the file. Here I read the offset bytes in accordance to the whence
condition.
1. If whence is SEEK_SET, the file position will be set to offset byte from the beginning.  I make sure that if the offset is greater than
the buffer size, I set the file position to the end.
2. If whence is SEEK_CUR, the file position will be set to the offset byte from the current file position.  I check if the file position is
greater than the buffer size, the file position is set to chardriver buffer size
3. If whence is SEEK_ENDm the file position is set to chardriver buffer size
Return the file position
*/
loff_t chardriver_llseek(struct file *p, loff_t offset, int whence){
    if(whence == SEEK_SET){
        p->f_pos = offset;

	if(p->f_pos > CHARDRIVER_BUFFER_SIZE){
	    p->f_pos = CHARDRIVER_BUFFER_SIZE;
	}
    }
    if(whence == SEEK_CUR){
	p->f_pos = p->f_pos + offset;

	if(p->f_pos > CHARDRIVER_BUFFER_SIZE){
	    p->f_pos = CHARDRIVER_BUFFER_SIZE;
	}
	if(p->f_pos < 0){
	    p->f_pos = 0;
	}
    }
    if(whence == SEEK_END){
	p->f_pos = CHARDRIVER_BUFFER_SIZE;
    }
    return p->f_pos;
}


/*This function is used to retrieve data from the hardware buffer. I first find the device file that needs to be read from. Then we put the
contents of the hardware buffer into the user buffer buffp with the copy_to_user function, passing along with it the offset bytes for the hardware
buffer and the bytes to be read. I also check for whether the open mode is in Write only in which the driver does not allow a read and returns
an error. I check if the n_bytes and offset to be read are also larger than the hardware buffer size in which I also return an error. Lastly,
I also check if the user provided buffer buffp is a NULL. If it is NULL we return an error.  
*/
ssize_t chardriver_read(struct file *p, char *buffp, size_t n_byte, loff_t *offset){
    
    struct chardriver_data *cp;
    struct open_state *osp = NULL;
    stats_t *sp;
    int slot;
    int bytes_to_read;
    int cpyout_ret;
    bool found_slot = false;
    CD_DEBUG("read chardriver input file: %p, buffer %p, n_byte %ld, offset %lld\n", 
	     p, buffp, n_byte, *offset);

    cp = (struct chardriver_data *)p->private_data;
    sp = &cp->devicestats;
    mutex_lock(&cp->cd_state_mutex);
    for(int i=0;i<MAX_OPEN_ALLOWED;i++){
	osp = &(cp->dev_open_state[i]);
        if(osp->filep == p){
            found_slot = true;
	    slot = i;
            break;
        }
    }
    
    BUG_ON(!found_slot);

    if((osp->open_mode & OPEN_WRITEONLY) == OPEN_WRITEONLY){
	sp->readfail++;
	mutex_unlock(&cp->cd_state_mutex);
	pr_err("Chardriver: Read: device is not in read mode; minor : %d\n", cp->minor);
	return -EBADF;
    }

    if(n_byte > CHARDRIVER_BUFFER_SIZE){	
	sp->readfail++;
	mutex_unlock(&cp->cd_state_mutex);
	pr_err("Chardriver: Read: number of bytes not valid; minor : %d\n",
	     cp->minor);
	return -EINVAL;
    }

    if(*offset > CHARDRIVER_BUFFER_SIZE){
	sp->readfail++;
	mutex_unlock(&cp->cd_state_mutex);
	pr_err("Chardriver: Read: number of offset bytes not valid: minor : %d\n",
	     cp->minor);
	return -EINVAL;
    }

    if(buffp == NULL){
	sp->readfail++;
	mutex_unlock(&cp->cd_state_mutex);
        pr_err("Chardriver: Read: user provided buffer is not valid: minor : %d\n",
             cp->minor);
	return -EINVAL;
    }

    BUG_ON(cp->hwptr == NULL);

    if((*offset + n_byte) > CHARDRIVER_BUFFER_SIZE){
	bytes_to_read = CHARDRIVER_BUFFER_SIZE - *offset;
    }else{
	bytes_to_read = n_byte;
    }

    cpyout_ret = copy_to_user((void*) buffp, (cp->hwptr + *offset), bytes_to_read);
    
    CD_DEBUG("%x %x %x %x %x \
		    %x %x %x\n", cp->hwptr[0],
		    cp->hwptr[1],
		    cp->hwptr[2],
		    cp->hwptr[3],
		    cp->hwptr[4],
		    cp->hwptr[5],
		    cp->hwptr[6],
		    cp->hwptr[7]);
    if(cpyout_ret < 0){
	sp->readfail++;
	mutex_unlock(&cp->cd_state_mutex);
        pr_err("Chardriver: Read: read failed: minor : %d\n",
             cp->minor);
	return -EIO;
    }

    sp->readcount++;
    *offset += n_byte;//adjust file position by offset
    mutex_unlock(&cp->cd_state_mutex);

    return bytes_to_read;
}

/* The write sends data to the device. This function is like read however the function used is copy_from_user.  Our error cases are to return
an error if the open state of the file is OPEN_RDONLY.  I also return an error if the offset and n_bytes thats written to is larger than the
hardware buffer size.  An error is returned if buffp is null as well.
*/
ssize_t chardriver_write(struct file *p, const char *buffp, size_t n_byte, loff_t *offset){

    struct chardriver_data *cp;
    struct open_state *osp = NULL;
    stats_t *sp;
    int slot;
    int bytes_to_write;
    int cpyin_ret;
    bool found_slot = false;
    CD_DEBUG("write chardriver input file: %p, buffer %p, n_byte %ld, offset %lld\n", 
	     p, buffp, n_byte, *offset);
    cp = (struct chardriver_data *)p->private_data;
    sp = &cp->devicestats;
    CD_DEBUG("Write: cp value: %p\n", cp);

    mutex_lock(&cp->cd_state_mutex);
    for(int i=0;i<MAX_OPEN_ALLOWED;i++){
	osp = &(cp->dev_open_state[i]);
        if(osp->filep == p){
            found_slot = true;
	    slot = i;
            break;
        }
    }
    
    BUG_ON(!found_slot);

    CD_DEBUG("Write: hw ptr: %p, minor: %d\n", cp->hwptr, cp->minor);

    if((osp->open_mode & OPEN_MODE_MASK) == OPEN_RDONLY){
	sp->writefail++;
	mutex_unlock(&cp->cd_state_mutex);
	pr_err("Chardriver: Write: device is not in write mode; minor : %d\n", cp->minor);
	return -EBADF;
    }

    if(n_byte > CHARDRIVER_BUFFER_SIZE){
	sp->writefail++;
	mutex_unlock(&cp->cd_state_mutex);
	pr_err("Chardriver: Write: number of bytes not valid; minor : %d\n",
	     cp->minor);
	return -EINVAL;
    }

    if(*offset > CHARDRIVER_BUFFER_SIZE){
	sp->writefail++;
	mutex_unlock(&cp->cd_state_mutex);
	pr_err("Chardriver: Write: number of offset bytes not valid: minor : %d\n",
	     cp->minor);
	return -EINVAL;
    }

    if(buffp == NULL){
	sp->writefail++;
	mutex_unlock(&cp->cd_state_mutex);
        pr_err("Chardriver: Write: user provided buffer is not valid: minor : %d\n",
             cp->minor);
	return -EINVAL;
    }

    BUG_ON(cp->hwptr == NULL);

    if((*offset + n_byte) > CHARDRIVER_BUFFER_SIZE){
	bytes_to_write = CHARDRIVER_BUFFER_SIZE - *offset;
    }else{
	bytes_to_write = n_byte;
    }

    cpyin_ret = copy_from_user((cp->hwptr + *offset), (void*)buffp, bytes_to_write);
    
    if(cpyin_ret < 0){
	sp->writefail++;
	mutex_unlock(&cp->cd_state_mutex);
        pr_err("Chardriver: Write: write failed: minor : %d\n",
             cp->minor);
	return -EIO;
    }
    sp->writecount++;
    *offset += n_byte;//adjust file position by offset
    mutex_unlock(&cp->cd_state_mutex);

    CD_DEBUG("0x%x 0x%x 0x%x 0x%x\n", cp->hwptr[0], cp->hwptr[1], cp->hwptr[2], cp->hwptr[3]);
    return bytes_to_write;
}

/*Status handler returns device stats of device to user (&cp->devicestats). These device stats show:
     opencount;
     closecount;
     readcount;
     writecount;
     mmapcount;
     munmapcount;
     openfail;
     closefail;
     readfail;
     writefail;
     mmapfail;
*/
long statsreturn_handler(struct chardriver_data *cp, unsigned long param){
    
    int cpyout_ret;
    CD_DEBUG("in stats handler %p, %ld\n", cp, param);
    mutex_lock(&cp->cd_state_mutex);
    cpyout_ret = copy_to_user((void*) param, &cp->devicestats, sizeof(stats_t));
    if(cpyout_ret < 0){
	mutex_unlock(&cp->cd_state_mutex);
	pr_err("Chardriver: IOCTL: unable to return stats: minor : %d\n",
             cp->minor);
        return -EINVAL;
    }
    
    mutex_unlock(&cp->cd_state_mutex);

    return 0;
}

/*Change buffersize allows user to change the buffer size to any mutliple of 4096 bytes. I check for whether the the device is in Open exclusive.
If it is not in open exclusive I return an error.  I check if the new buffer size is same as the old buffer size, I return an error.  When
allocating a new size, we kfree the existing hardware buffer and kmalloc a new buffer that is a multiple of 4096 bytes.
*/
long changebuffersize_handler(struct chardriver_data *cp, unsigned long param){

    CD_DEBUG("in changebuffer size %p, %ld\n", cp, param);
    mutex_lock(&cp->cd_state_mutex);
    //Check whether it is exclusive opened if not error return ENOTTY
    if(cp->excl_use != OPEN_EXCL){
	mutex_unlock(&cp->cd_state_mutex);
	CD_DEBUG("Not in Exclusive Open; Cannot change buffer size");
	return -ENOTTY;
    }
    
    //validate whether given param value (new buffer size) is the same as what is existing
    if(param == cp->hwptr_size){
	mutex_unlock(&cp->cd_state_mutex);
	CD_DEBUG("New buffer size is the same as existing buffer size");
	return -EINVAL;
    }

    //validate whether new buffersize is a multiple of 4096 and greater than 0
    if((param == 0) || (param%4096 != 0)){
	mutex_unlock(&cp->cd_state_mutex);
        pr_err("Chardriver IOCTL cannot change buffer size since it is 0 or not a multiple of 4096");
        return -EINVAL;	
    }

    //if it is different use kfree and allocate new size
    kfree(cp->hwptr);
    cp->hwptr = kmalloc(param, GFP_KERNEL);
    if(cp->hwptr == NULL){
        mutex_unlock(&cp->cd_state_mutex);
	pr_err("Chardriver IOCTL: hwptr was not successful in allocating buffer; minor: %d\n", cp->minor);
	return -ENOMEM;
    }
    cp->hwptr_size = (int)param;
    CD_DEBUG("New buffersize allocated: %d", cp->hwptr_size);
    mutex_unlock(&cp->cd_state_mutex);
    
    return 0;
}

/*Ioctl functions covered above
Returns drivers stats
Changes buffer size
Gets buffer size
returns these values to user based off of what is called
*/
long chardriver_ioctl(struct file *p, unsigned int cmd, unsigned long param){

    long retval;
    struct chardriver_data *cp;
    //struct open_state *osp = NULL;
    cp = (struct chardriver_data *)p->private_data;

    switch(cmd) {
    case CHARDRIVER_IOCTL_RETURN_STATS: 
	retval = statsreturn_handler(cp, param);
        break;
  
    case CHARDRIVER_IOCTL_CHANGE_BUFFERSIZE:
	retval = changebuffersize_handler(cp, param);
	break;

    case CHARDRIVER_IOCTL_GET_BUFFERSIZE:
	return cp->hwptr_size;

    default : return -ENOTTY;
    }
    return retval;
}
//This function is used to take count of the amount of mmap called
void vma_open(struct vm_area_struct *vma){

    struct file *p;
    struct chardriver_data *cp;
    stats_t *sp;
    
    p = vma->vm_file;
    cp = (struct chardriver_data *)p->private_data;
    sp = &cp->devicestats;

    sp->mmapcount++;

    CD_DEBUG("mmap was called vma is open: %p", vma);

}
//This function is used to take count of the amount of munmaps called
void vma_close(struct vm_area_struct *vma){

    struct file *p;
    struct chardriver_data *cp;
    stats_t *sp;
    
    p = vma->vm_file;
    cp = (struct chardriver_data *)p->private_data;
    sp = &cp->devicestats;

    sp->munmapcount++;

    CD_DEBUG("munmap was called vma is closed: %p", vma);
}

static struct vm_operations_struct chardriver_vm_ops = {
    open: vma_open,
    close: vma_close,
};
/*This function is used to request mapping to the hardware buffer. This function had me go deeper into vm_area_struct on elixir bootlin.
When mapping we have to make sure that VM_SYNC is enabled in the vma->vm_flags.  By calling virt_to_phys I get the physical address of the
hwptr. I then derive the page frame number to which the virtual address should be mapped.  Then the remap_pfn_range function maps a physical
address space into the virtual space using the vm_area_struct.
*/
int chardriver_mmap(struct file *p, struct vm_area_struct *vma){
    
    unsigned long pfn;
    int ret;
    phys_addr_t physhwptr;
    stats_t *sp;
    struct chardriver_data *cp;
    cp = (struct chardriver_data *)p->private_data;
    sp = &cp->devicestats;


    CD_DEBUG("VMA start: 0x%lx", vma->vm_start);
    CD_DEBUG("VMA end: 0x%lx", vma->vm_end);
    CD_DEBUG("VMA page protect: 0x%lx", vma->vm_page_prot.pgprot);
    CD_DEBUG("VMA flags: 0x%lx", vma->vm_flags);

    vma->vm_flags |= VM_SYNC;
    CD_DEBUG("VM _flags : 0x%lx", vma->vm_flags);
    physhwptr = virt_to_phys(cp->hwptr);
    CD_DEBUG("Physical addr: %p, Kernel Virutal addr: %p", (void *)physhwptr, cp->hwptr);
    pfn = physhwptr>>PAGE_SHIFT;

    ret = remap_pfn_range(vma, vma->vm_start, pfn, (vma->vm_end - vma->vm_start), vma->vm_page_prot);
    if (ret < 0){
	sp->mmapfail++;
        return -EAGAIN;
    }
    vma->vm_ops = &chardriver_vm_ops;
    vma_open(vma);

    return 0;
}
//These are the file operations used in the driver. We use and implement open, read, write, release, ioctl, mmap, and llseek.
const struct file_operations chardriver_fops = {
    .owner = THIS_MODULE,
    .open = chardriver_open,
    .read = chardriver_read,
    .write = chardriver_write,
    .release = chardriver_release,
    .unlocked_ioctl = chardriver_ioctl,
    .mmap = chardriver_mmap,
    .llseek = chardriver_llseek,
};

/*After creating the node and assigning a Major number, the Chardriver _init function is where I register the character driver.  
 This function is called when we insmod the driver. I initialize the chardriverdata structure for the minor devices here to either
 0 or NULL.
*/
static int __init chardriver_init(void)
{
    int i, j, err;
    
    pr_info("first driver\n");

    err = register_chrdev_region(MKDEV(CHARDRIVER_MAJOR, 0), CHARDRIVER_MAX_MINORS,
                                 "chardriver");
    if (err != 0) {
        /* report error */
        return err;
    }

    for(i = 0; i < CHARDRIVER_MAX_MINORS; i++) {
        /* initialize devs[i] fields */
        struct open_state *osp;
	stats_t *sp;
	cdev_init(&devs[i].cdev, &chardriver_fops);
	devs[i].device = MKDEV(CHARDRIVER_MAJOR, i);
        cdev_add(&devs[i].cdev, devs[i].device, 1);
	devs[i].hwptr = kmalloc(CHARDRIVER_BUFFER_SIZE, GFP_KERNEL);
	if(devs[i].hwptr == NULL){
	   pr_err("Chardriver: Init: hwptr was not successful in allocating buffer; minor: %d\n", i);
	   goto initexit;
	}
	devs[i].hwptr_size = CHARDRIVER_BUFFER_SIZE;
	CD_DEBUG("hw ptr: %p, minor: %d\n", devs[i].hwptr, i);
	devs[i].open_counter = 0;
	devs[i].excl_use = 0;
	devs[i].minor = i;
	__mutex_init(&devs[i].cd_state_mutex, "chardriver mutex", &devs[i].cd_state_key);
	//osp = &(devs[i].dev_open_state[0]);
	osp = (devs[i].dev_open_state);
	for(j = 0;j < MAX_OPEN_ALLOWED;j++){
	   //osp = &(devs[i].dev_open_state[j]);
	   osp->open_mode = 0;
	   osp->owner = INVALID_PID;
	   osp++;
	}
        sp = &devs[i].devicestats;
	sp->opencount = 0;
	sp->closecount = 0;
	sp->readcount = 0;
	sp->writecount = 0;
	sp->mmapcount = 0;
	sp->munmapcount = 0;
	sp->openfail = 0;
	sp->closefail = 0;
	sp->readfail = 0;
	sp->writefail = 0;
	sp->mmapfail = 0;
	
    }
    return 0;
initexit:
   //write clean up code for exit
   return -1;
}

//Exit chardriver frees the hardware structure and unregisters the driver. This is done by calling rmmod.
static void __exit chardriver_exit(void)
{
    pr_info("first driver exit\n");
    
    for(int i=0;i<CHARDRIVER_MAX_MINORS; i++){
        if(devs[i].hwptr != NULL){
	    CD_DEBUG("hw ptr: %p, minor: %d\n", devs[i].hwptr, i);
	    kfree(devs[i].hwptr);
	    devs[i].hwptr = NULL;
	    devs[i].hwptr_size = 0;
	}
        cdev_del(&devs[i].cdev);

    }

    unregister_chrdev_region(MKDEV(CHARDRIVER_MAJOR, 0), CHARDRIVER_MAX_MINORS);
    
}


MODULE_AUTHOR("Sashank Rao");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
module_init(chardriver_init);
module_exit(chardriver_exit);
