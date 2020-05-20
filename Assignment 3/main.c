#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/blk-mq.h>
#include <linux/hdreg.h>
#include <linux/types.h>  /* size_t */
#include <linux/sched.h>
#include <linux/bio.h>
#include <linux/workqueue.h>
#include<linux/spinlock.h>
#include <linux/highmem.h>

#define VID1  0x0781   
#define PID1  0x5151

//#define VID1  0x03f0    
//#define PID1  0xf440
#define be_to_int32(buf) (((buf)[0]<<24)|((buf)[1]<<16)|((buf)[2]<<8)|(buf)[3])

#ifndef SECTOR_SIZE
#define SECTOR_SIZE 512
#endif

#ifndef NR_OF_SECTORS
#define NR_OF_SECTORS 7858175
#endif

#define bio_iovec_idx(bio, idx) (&((bio)->bi_io_vec[(idx)]))
#define __bio_kmap_atomic(bio, idx, kmtype)             \
    (kmap_atomic(bio_iovec_idx((bio), (idx))->bv_page) +    \
        bio_iovec_idx((bio), (idx))->bv_offset)


#define __bio_kunmap_atomic(addr, kmtype) kunmap_atomic(addr)

int dev_major=0;
static unsigned int nr_bytes=0;
uint8_t ep_in , ep_out ;
struct usb_device *device;

/* Work structure */
struct my_work{
    struct work_struct work;
    struct request *req;
}my_work;

static struct workqueue_struct *myqueue = NULL;

/* Just internal representation of the our block device
 * can hold any useful data */
struct block_dev {
    sector_t capacity;
    int size;                       /* Device size in sectors */
    u8 *data;                       /* Data buffer to emulate real storage device */
    short media_change;             /* Flag a media change? */
    struct blk_mq_tag_set tag_set;   /*tags used in v5 kenel not in kernel v4*/
    struct request_queue *queue;    /* Device request queue*/
    struct gendisk *gdisk;          /* gendisk structure*/
    spinlock_t lock;                /* For mutual exclusion */
    short users;                    /*Count number of users*/
};

/* Device instance */
static struct block_dev *blk_device = NULL;
struct request *req;

/* Command Block Wrapper (CBW) */
struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

/* Command Status Wrapper (CSW) */
struct command_status_wrapper {
	uint8_t dCSWSignature[4];
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
};

static uint8_t cdb_length[256] = {
//	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  0
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  1
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  2
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  3
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  4
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  5
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  6
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  7
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  8
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  9
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  A
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  B
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  C
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  D
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  E
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  F
};
 /*Stores VID PID */
static struct usb_device_id usbdev_table [] = {
	{USB_DEVICE(VID1 , PID1)},
	{} /*terminating entry*/	
};

static int get_mass_storage_status(struct usb_device *device, uint8_t endpoint, uint32_t expected_tag)
{	
	int size;
    int r;
	struct command_status_wrapper *csw;

    /*Allocating memory to the CSW structure*/
	csw=(struct command_status_wrapper *)kmalloc(sizeof(struct command_status_wrapper),GFP_KERNEL);

    /*Receiving the status info from Bulk message*/
	r=usb_bulk_msg(device,usb_rcvbulkpipe(device,endpoint),(void*)csw,13, &size, 1000);
    if(r<0)
        printk("error in status\n");
    
    if (csw->dCSWTag != expected_tag) {
        printk("   get_mass_storage_status: mismatched tags (expected %08X, received %08X)\n",
            expected_tag, csw->dCSWTag);
        return -1;
    }
    if (size != 13) {
        printk("   get_mass_storage_status: received %d bytes (expected 13)\n", size);
        return -1;
    }   

    printk(KERN_INFO "Mass Storage Status: %02X (%s)\n", csw->bCSWStatus, csw->bCSWStatus?"FAILED":"Success");
	return 0;
}

static int send_mass_storage_command(struct usb_device *device, uint8_t endpoint,
	uint8_t *cdb, uint8_t direction, int data_length, uint32_t *ret_tag)
{
	static uint32_t tag = 1;
	uint8_t cdb_len;
	int size;
	struct command_block_wrapper *cbw;
	
    /*Allocating memory to the CBW structure*/
    cbw=(struct command_block_wrapper *)kmalloc(sizeof(struct command_block_wrapper),GFP_KERNEL);
    
    if (cdb == NULL) {
        return -1;
    }

    if (endpoint & USB_DIR_IN) {
        printk("send_mass_storage_command: cannot send command on IN endpoint\n");
        return -1;
    }   

    cdb_len = cdb_length[cdb[0]];

    if ((cdb_len == 0) || (cdb_len > sizeof(cbw->CBWCB))) {
        printk("Invalid command\n");
        return -1;
    }   

	memset(cbw, 0, sizeof(*cbw));
	cbw->dCBWSignature[0] = 'U';
	cbw->dCBWSignature[1] = 'S';
	cbw->dCBWSignature[2] = 'B';
	cbw->dCBWSignature[3] = 'C';
	*ret_tag = tag;
	cbw->dCBWTag = tag++;
	cbw->dCBWDataTransferLength = data_length;
	cbw->bmCBWFlags = direction;
	cbw->bCBWLUN = 0;

	cbw->bCBWCBLength = cdb_len;
	memcpy(cbw->CBWCB, cdb, cdb_len);

	// The transfer length must always be exactly 31 bytes.
	usb_bulk_msg(device, usb_sndbulkpipe(device,endpoint), (void*)cbw, 31, &size, 1000);


	return 0;
}
/////////////////// **************READ(10)***************** //////////////////////////

static int myread(sector_t sector,sector_t nr_sector,char *page_address)
{
    int result;
    unsigned int size;
    uint8_t cdb[16];    // SCSI Command Descriptor block
    uint32_t expected_tag;
    size=0;
    memset(cdb,0,sizeof(cdb));
    cdb[0] = 0x28;  // Read(10)
    cdb[2]=(sector>>24) & 0xFF;
    cdb[3]=(sector>>16) & 0xFF;
    cdb[4]=(sector>>8) & 0xFF;
    cdb[5]=(sector>>0) & 0xFF;
    cdb[7]=(nr_sector>>8) & 0xFF;
    cdb[8]=(nr_sector>>0) & 0xFF; // 1 block
    printk(KERN_INFO "READ (10)\n");

    /*Sending Request*/
    send_mass_storage_command(device,ep_out,cdb,USB_DIR_IN,(nr_sector*512),&expected_tag);

    /*Receiving the data*/
    usb_bulk_msg(device,usb_rcvbulkpipe(device,ep_in),(void*)(page_address),(nr_sector*512),&size, 5000);
    
    printk("size of data sent: %d\n",size);
    get_mass_storage_status(device, ep_in, expected_tag);  
    return 0;
}
/*Write(10) SCSI function*/
 static int mywrite(sector_t sector,sector_t nr_sector,char *page_address)
{   //int i;
    int result;
    unsigned int size;
    uint8_t cdb[16];    // SCSI Command Descriptor Block
    uint32_t expected_tag;
    printk(KERN_INFO "WRITE(10)\n");
    memset(cdb,0,sizeof(cdb));
    cdb[0]=0x2A;
    cdb[2]=(sector>>24)&0xFF;
    cdb[3]=(sector>>16)&0xFF;
    cdb[4]=(sector>>8)&0xFF;
    cdb[5]=(sector>>0)&0xFF;
    cdb[7]=(nr_sector>>8)&0xFF;
    cdb[8]=(nr_sector>>0)&0xFF;   // 1 block

    /*Sending Request*/
    send_mass_storage_command(device,ep_out,cdb,USB_DIR_OUT,(nr_sector*512),&expected_tag);

    /*Transferring data*/
    usb_bulk_msg(device,usb_sndbulkpipe(device,ep_out),(void*)page_address,(nr_sector*512),&size, 1000);

    printk("WRITE: sent %d bytes\n", size);
    get_mass_storage_status(device, ep_in, expected_tag); 
    return 0;

}
///////////////// Serve requests ///////////////////////

static int do_request(struct request *req)
{
    struct bio_vec bvec;
    struct req_iterator iter;
    sector_t sector = req->bio->bi_iter.bi_sector;
    int i;
    
    loff_t pos = blk_rq_pos(req) << 9; /*pos= Current Sector*512*/
    loff_t dev_size = (loff_t)(blk_device->capacity << 9); /*device size = Nr of bytes*512 */
    
    
    /* Iterate over all requests segments */
    rq_for_each_segment(bvec, req, iter)
    {
        unsigned long b_len = bvec.bv_len;
        sector_t nr_sector = (b_len/SECTOR_SIZE);

        /* Get pointer to the data */
        char* b_buf = __bio_kmap_atomic(req->bio, i, KM_USER0);
    
        /* Simple check that we are not out of the memory bounds */
        if ((pos + b_len) > dev_size) {
            printk (KERN_NOTICE "Out of Memory Bounds\n");
            b_len = (unsigned long)(dev_size - pos);
        }

        if (rq_data_dir(req) == WRITE) {
            mywrite(sector, nr_sector, b_buf);
            /* Copy data to the buffer in to required position */
        } else {
            myread(sector, nr_sector, b_buf);
            /* Read data from the buffer's position */
        }
        __bio_kunmap_atomic(req->bio, KM_USER0);
    }
    return 0;

}

/* Deferred Work */
static void deferred_work(struct work_struct *work) 
{
    struct my_work *mywork=container_of(work, struct my_work, work);
    
    do_request(mywork->req);  // Serving Requests
    __blk_end_request_cur(mywork->req,0); // End Current Request
    kfree(mywork);     
    return;

}

/* queue callback function */
void req_queue(struct request_queue *queue)
{
    
	struct request *req;
    struct my_work *mywork=NULL;
    

    while((req=blk_fetch_request(queue)) != NULL)
    {
        if(req == NULL && !blk_rq_is_passthrough(req)) // check if file sys req, for DRiver to handle 
        {
            printk(KERN_INFO "non FS request\n");
            __blk_end_request_all(req, -EIO);
            continue;
        }
        mywork=(struct my_work *)kmalloc(sizeof(struct my_work),GFP_ATOMIC);
        if(mywork==NULL){

            printk("Memory Allocation to deferred work failed\n");
            __blk_end_request_all(req, 0);
            continue;
        }

        mywork->req=req;
        INIT_WORK(&mywork->work,deferred_work); // Initialize Deferred Work
        queue_work(myqueue,&mywork->work);      // Initializing Work Queue
        
    }   
}
/*Opening the device and lock*/
static int blockdev_open(struct block_device *dev, fmode_t mode)
{
    struct block_dev *blk=dev->bd_disk->private_data;
    spin_lock(&blk->lock);
    if (! blk->users) 
        check_disk_change(dev);  /* Check for Media Change*/   
    blk->users++;       /*Increment Users*/
    spin_unlock(&blk->lock);

    printk(">>> blockdev_open\n");

    return 0;
}

/*Releasing device and unlock*/
static void blockdev_release(struct gendisk *gdisk, fmode_t mode)
{
    struct block_dev *dev = gdisk->private_data;
    spin_lock(&dev->lock);
    dev->users--;         
    spin_unlock(&dev->lock);
    printk(">>> blockdev_release\n");
}
/* Check for media change*/
int blockdev_media_changed(struct gendisk *gdisk)
{
    struct block_dev *dev=gdisk->private_data;
    printk(KERN_INFO "Media Test\n");
    return dev->media_change;
}
/*Verify wether media is changed or not*/
int blockdev_revalidate(struct gendisk *gdisk)
{
    struct block_dev *dev=gdisk->private_data;
    if(dev->media_change)
    {
        dev->media_change = 0;
        printk(KERN_INFO "Media Revalidiate\n");
        memset (dev->data, 0, dev->size);
    }
    return 0;
}
/* Set block device file I/O */
static struct block_device_operations blockdev_ops = {
    .owner = THIS_MODULE,
    .open = blockdev_open,
    .release = blockdev_release,
    .media_changed   = blockdev_media_changed,
    .revalidate_disk = blockdev_revalidate,
  
};
/* Function for getting the capacity of the device*/
static int test_mass_storage (struct usb_device *device , uint8_t endpoint_in, uint8_t endpoint_out)
{
    int size;
    uint8_t *lun=(uint8_t *)kmalloc(sizeof(uint8_t),GFP_KERNEL);
    uint32_t expected_tag;
    uint32_t max_lba, block_size;
    long device_size;
    uint8_t cdb[16];    
    uint8_t *buffer=(uint8_t *)kmalloc(64*sizeof(uint8_t),GFP_KERNEL);


    /*Sending control Messages*/
    usb_control_msg(device,usb_sndctrlpipe(device,0),0xFF,0x21,0,0,NULL,0,1000);

    usb_control_msg(device,usb_sndctrlpipe(device,0), 0xFE ,0xA1,
       0, 0, (void*)lun, 1, 1000);

    // Read capacity
    printk(KERN_INFO "Reading Capacity:\n");
    memset(buffer, 0, sizeof(buffer));
    memset(cdb, 0, sizeof(cdb));
    cdb[0] = 0x25;  // Read Capacity

    send_mass_storage_command(device, endpoint_out, cdb, USB_DIR_IN, 0x08, &expected_tag);
    usb_bulk_msg(device, usb_rcvbulkpipe(device,endpoint_in), (void*)buffer, 64, &size, 10000);

    max_lba = be_to_int32(&buffer[0]);
    block_size = be_to_int32(&buffer[4]);
    device_size = ((long)(max_lba+1))*block_size/(1024*1024*1024);

    printk("Device Capacity: %ld GB \n",device_size);

    get_mass_storage_status(device, endpoint_in, expected_tag);

    return 0;

}

/* This function will work after inserting USB Drive */
static int usbdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{   
    struct usb_endpoint_descriptor *ep_desc; //End point descriptor structure
	unsigned char epAddr, epAttr;            

	int i,type;

    if(id->idProduct == PID1 && id->idVendor==VID1)   //Matching VID and PID with the given VID PID  
	{
		printk(KERN_INFO "Known USB Drive detected\n");
	}
    device = interface_to_usbdev(interface);
    /*Printing VID, PID, Device Class, Device Subclass, Device Protocol and Number of Endpoinds of the usb drive*/
	printk(KERN_INFO "Vendor ID VID: %04X\n",id->idVendor);    
	printk(KERN_INFO "Product ID PID: %04X\n",id->idProduct); 
	printk(KERN_INFO "USB DEVICE CLASS : %x\n", interface->cur_altsetting->desc.bInterfaceClass);
	printk(KERN_INFO "USB DEVICE SUB CLASS : %x\n", interface->cur_altsetting->desc.bInterfaceSubClass);
	printk(KERN_INFO "USB DEVICE Protocol : %d\n", interface->cur_altsetting->desc.bInterfaceProtocol);
    printk(KERN_INFO "No. Of Endpoints = %d\n",interface->cur_altsetting->desc.bNumEndpoints);

	for (i=0; i<interface->cur_altsetting->desc.bNumEndpoints; i++) 
			{
				ep_desc = &interface->cur_altsetting->endpoint[i].desc;


			 	epAddr = ep_desc->bEndpointAddress; // End point Address
	        	epAttr = ep_desc->bmAttributes;       //End point Attributes
                type=usb_endpoint_type(ep_desc);       //getting type

        if(type==2){
            if(epAddr & 0x80)
            {       
                printk(KERN_INFO "EP %d is Bulk IN\n", i);
                ep_in=ep_desc->bEndpointAddress;
                printk("endpoint_in : %x",ep_in);
            
            }
            else
            {   
                ep_out=ep_desc->bEndpointAddress;
                printk(KERN_INFO "EP %d is Bulk OUT\n", i); 
                printk("endpoint_out : %x",ep_out);
            }
        }
        if(type==3)
            {
            if(epAddr && 0x80)
                printk(KERN_INFO "EP %d is Interrupt IN\n", i);
            else
                printk(KERN_INFO "EP %d is Interrupt OUT\n", i);
            }
            
		  } 

        if ( (interface->cur_altsetting->desc.bInterfaceClass == 0x08) && (interface->cur_altsetting->desc.bInterfaceSubClass == 0x06) 
			&& (interface->cur_altsetting->desc.bInterfaceProtocol == 0x50) ) 
			{   
				printk(KERN_INFO "Valid SCSI Device\n");

			}
            else
            {
                printk(KERN_INFO "Device is not valid SCSI\n");
            }

	test_mass_storage(device, ep_in, ep_out);  // Function for getting Capacity of the device

	/* Register new block device and get device major number */
    
    dev_major = register_blkdev(0, "testblk");
    if(dev_major<=0)
    {
        printk(KERN_WARNING "testblk:Unable to get major number\n");
    }

    blk_device = kmalloc(sizeof (struct block_dev), GFP_KERNEL);
   
    memset(blk_device ,0 ,sizeof(struct block_dev));
    if (blk_device == NULL) {
        printk("Failed to allocate struct block_dev\n");
        unregister_blkdev(dev_major, "testblk");

        return -ENOMEM;
    }

// Set capacity of the device 
   blk_device->capacity = NR_OF_SECTORS;  /* nsectors * SECTOR_SIZE; */

    spin_lock_init(&blk_device->lock); /* Spin lock*/
    printk("Initializing queue\n");

    blk_device->queue = blk_init_queue(req_queue, &blk_device->lock); //Queue

    if (blk_device->queue == NULL) {
        printk("Failed to allocate device queue\n");

        unregister_blkdev(dev_major, "testblk");
        kfree(blk_device);

        return -ENOMEM;
    }
    /* Allocate new disk */
    blk_device->gdisk = alloc_disk(2);
    
    if(!blk_device->gdisk)
    {
        printk(KERN_NOTICE "alloc_disk_failure\n");
        kfree(blk_device);
        return 0;
    }

    /* Set all required flags and data */
    //block_device->gdisk->flags = GENHD_FL_NO_PART_SCAN;
    blk_device->gdisk->major = dev_major;
    blk_device->gdisk->first_minor = 0;

    blk_device->gdisk->fops = &blockdev_ops;
    blk_device->gdisk->queue = blk_device->queue;
    blk_device->gdisk->private_data = blk_device;

    /* Set device name as it will be represented in /dev */
    strcpy(blk_device->gdisk->disk_name, "blockdev");

    printk("Adding disk %s\n", blk_device->gdisk->disk_name);

    /* Set device capacity */
    set_capacity(blk_device->gdisk, NR_OF_SECTORS);

    /* Notify kernel about new disk device */
    add_disk(blk_device->gdisk);            

	return 0;
}

static void usbdev_disconnect(struct usb_interface *interface)
{
	printk(KERN_INFO "USBDEV Device Removed\n");
       /* Deregister Block Driver */
        if (blk_device->gdisk) {
        del_gendisk(blk_device->gdisk);
        printk(KERN_INFO "Delete Gendisk\n");
        put_disk(blk_device->gdisk);
    }

    /*Queue Cleanup*/
    if (blk_device->queue) {

        blk_cleanup_queue(blk_device->queue);
        blk_device->queue=NULL;
        printk(KERN_INFO "Queue Cleanup\n");
    }
   
    unregister_blkdev(dev_major, "testblk");
    kfree(blk_device);
	return;
}


/*Operations structure*/
static struct usb_driver skel_driver = {
        .name        = "usbdev",
        .probe       = usbdev_probe,
        .disconnect  = usbdev_disconnect,
        .id_table    = usbdev_table,
};

static int __init usb_skel_init(void)
{
        int result;

        /* register this driver with the USB subsystem */
        result=usb_register(&skel_driver);
        if(result<0)
        {
        	printk(KERN_INFO "Unable to register USB Driver\n");

        }
        else
        {
        	printk(KERN_INFO "UAS READ Capacity Driver Inserted \n");

        }
        myqueue=create_workqueue("myqueue");  // Deferred work thread name


        return 0;
}

static void __exit usb_skel_exit(void)
{
        /* deregister this driver with the USB subsystem */
        usb_deregister(&skel_driver);
        printk(KERN_INFO "Exit\n");
        flush_workqueue(myqueue);  // delete workqueue on exit
        destroy_workqueue(myqueue);

    printk(KERN_INFO "Device Unloaded\n");
}

module_init(usb_skel_init);
module_exit(usb_skel_exit);
MODULE_LICENSE("GPL"); 