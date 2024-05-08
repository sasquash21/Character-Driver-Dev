/*  This character driver demonstrates  _init, _open, _read, _write,  _ioctl, and mmap functions. 
*  This driver manipulates a kmalloc'ed buffer of size 4K as HW. The chardriver is used as a vehicle to
*  learn the kernel driver interfaces available in general purpose linux. This driver implements all the driver entry points
* mentioned above and have been tested for the functionality and semantics implemented using individual test cases.
*  As part of debugging, macros like CD_DEBUG were used for
* different problems seen during development. This driver controls three different instances of HW buffer with distinct minor numbers. 
*  A distinct driver state structure is maintained for managing different instances of the Pseudo HW buffer.
*  The unused major number is allocated for this chardriver using the rules laid out in linux for allocating major numbers.
*/


#include "chardriver.h"


struct chardriver_data devs[CHARDRIVER_MAX_MINORS];

/*Users can open an HW (pseudo  hardware buffer)using the chardriver node created using mknode - with Open using different flags, O_EXCL - exclusive, O_WRONLY - write, 
* O_RDONLY - read, O_RDWR- read/write.  Each such open from a user level process is maintained in the driver state structure.
* Only one user can hold exclusive access, if already a user has opened the  device
* and another user is requesting  exclusive access, such an open request will be failed with error code. 
* This driver allows only for the MAX_ALLOWED_OPEN number of opens, beyond which it will reject opening the device. 
* Throughout  the driver to maintain synchronization of the pseudo HW buffer in multi-threaded or Multi-process use cases, mutex is 
* used to provide such synchronization on each instance of the HW buffer. This also maintains statistics pertaining to this entry point, both
* the open success and failure counts.
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

/* This  entry point will be called whenever the application closes the character device node.  This entry point identifies the appropriate
* open state maintained in the driver state structure and marks the state of the open corresponding to this process ID to be closed. Once such
* open state is marked closed, we also mark such open state slots to be available for managing other opens that might come from other applications.
* This also maintains stats for successful close accomplished by this routine. 
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
/*The llseek function is used to change the current read/write position of the device node file maintaining the Psuedo HW buffer. 
* This entry points returns the file offset  in accordance to the whence condition.
*  1.  If whence is SEEK_SET, the file position will be set to offset byte from the beginning.  This entry point validates  if the offset is greater than
*       the Pseudo HW buffer size managed, it will set the file position to the end.
*  2.  If whence is SEEK_CUR, the file position will be set to the offset byte from the current file position.  This entry point validates if the file position is
*       greater than the pseudo HW  buffer size, the file position is set to chardriver buffer size. If a negative offer is provide by the user, this entry point validates
*       the file position does not go below zero as well.
*  3.  If whence is SEEK_END the file position is set to end of pseudo HW Buffer size
*  lseek stats to be implemented yet.
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


/*This entry point is called whenever the application/user code issues a read system call to read the contents of the pseudo HW buffer.  
*  This routine validates whether the application has opened the device node in appropriate mode to read the data from a pseudo HW buffer.
*  if the requested data > greater than the HW Buffer size, the read request is failed with EINVAL as error code.
* This routine  returns the requested amount of data from the current file pointer  if the amount of data requested is available in the pseudo HW
* buffer. If the requested size exceeds  the amount available in the HW buffer from the current file position, this driver satisfies the read with partial data
* available until the end of the pseudo HW buffer and returns appropriately to reflect the amount of data returned. Once the data is returned, the offset is adjusted
* to make sure the file pointer reflects the position required to read the next byte from HW Buffer.
* The contents of the HW buffer is moved to the user virtual address using the Kernel API copy_to_user.  This entry point validates the destination
* buffer to make sure it is a valid buffer before initiating a copy to the user virtual address. This entry point maintains stats on both successful and failed reads.
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

/* This entry point is called whenever the application/user code issues a write system call to write the contents of the pseudo HW buffer.  
*  This routine validates whether the application has opened the device node in appropriate mode to write  data into the pseudo HW buffer.
*  if the requested data > greater than the HW Buffer size, the write request fails with EINVAL as error code.
* This routine  returns the amount of data written into the pseudo HW buffer.   If the requested write size exceeds  the amount of space available in 
 * the HW buffer from the current file position, this driver satisfies the write by writing  partial data
* into the pseudo HW buffer and returns appropriately to reflect the amount of data written. Once the data is written, the offset is adjusted
* to make sure the file pointer reflects the position required to read/write the next byte from/into the HW Buffer.
* The contents of the data provided by the user application is copied into the pseudo HW buffer using the kernel API copy_from_user.  This entry point validates the source
* buffer to make sure it is a valid buffer before initiating a copy_from_user routine.  This entry point maintains stats on both successful and failed writes.
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

/*Status handler returns device stats of device to user (&cp->devicestats).The device stats structure maintains the different happy
 * path and failure path stats for each entrypoint. This is an helper routine to  implement the GET_DEVICE_STATS ioctl. These device stats show:
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

/* This is the  helper routine to implement the change pseudo HW Buffer size ioctl. By default the driver allocates 4K buffer using kmalloc for
* use as a pseudo HW Buffer. This ioctl provides a means to change that pseudo HW Buffer to be of a different bigger size buffer(only multiples of 4K allowed). 
* This routine expects the application which is issuing this ioctl to hold exclusive access to the device node, so that pseudo HW buffer size can be changed
* without affecting the functionality of other applications which might have opened and using this pseudo HW buffer.
* the current HW Buffer size is maintained in the driver state structure. 
* TBD: still the error checks in various entry points are using the default HW buffer size (#define), but should validate against the buffer size maintained in state structure.
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

/* Implements the ioctl entry points.
* support the following IOCTLs.
* 1. CHARDRIVER_IOCTL_RETURN_STATS:
* 2. CHARDRIVER_IOCTL_CHANGE_BUFFERSIZE
* 3.  CHARDRIVER_IOCTL_CHANGE_BUFFERSIZ
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
//This function implements the VM area operations entry point open.
//This function is done to reliably count the stats on the number of mmaps handled by this driver.
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
//This function implements the VM area operations entry point close.
//This entry point is the only place where one could count the number of unmaps handled by this driver instance.
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
/*This entry point implements a simple mmap operation of the pseudo HW buffer.  
* This function had me go deeper into vm_area_struct on elixir bootlin. When mapping, make sure that VM_SYNC is enabled in the vma->vm_flags, so that the 
* write is reflected immediately to other processes and synchronization happens as soon as the write to mmaped buffer is complete from one process. 
* By calling virt_to_phys I get the physical address of the hwptr. The entry point derives the page frame number to which the virtual address should be mapped. 
* Then call the remap_pfn_range function to  map a physical address of the pseudo HW buffer into the virtual space using the vm_area_struct.
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
//These are the file operations/entry points implemented by this driver. The chardriver sets up entry points for open, read, write, release, ioctl, mmap, and llseek.
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

/*This driver is written as a loadable module. This entry point is set up to be called using module_init macro. during the time a load of this driver is requested by 
* insmod  driver load application.
*
* The Chardriver _init function used to register the character driver with the required/unused major number and also declare how many
* instances of such device nodes can be managed by this driver. This also allocates as many driver state structure for each such instance to
* be managed and initializes the structure to a clean state. This also allocates the default size pseudo HW buffer required for each instance and 
* initializes the buffer point in the driver state structure for that instance.
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

/*This entry point is set up to be called using module_exit macro, during the time a unload of this driver is requested by 
* rmmod  driver unload application.
* The chardriver_exit entry point releases all the resources allocated during _init entry point and make sure the driver tears down
* all the allocated resources in an orderly manner.
*/
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
