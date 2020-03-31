#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>

#define VID1  0x03f0
#define PID1  0xf440
#define be_to_int32(buf) (((buf)[0]<<24)|((buf)[1]<<16)|((buf)[2]<<8)|(buf)[3])



// Section 5.1: Command Block Wrapper (CBW)
struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

// Section 5.2: Command Status Wrapper (CSW)
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

static struct usb_device_id usbdev_table [] = {
	{USB_DEVICE(VID1 , PID1)},
	{} /*terminating entry*/	
};

static int get_mass_storage_status(struct usb_device *device, uint8_t endpoint, uint32_t expected_tag)
{	
	int size;
	struct command_status_wrapper *csw;
	csw=(struct command_status_wrapper *)kmalloc(sizeof(struct command_status_wrapper),GFP_KERNEL);

	usb_bulk_msg(device,usb_rcvbulkpipe(device,endpoint),(void*)csw,13, &size, 1000);
	return 0;
}

static int send_mass_storage_command(struct usb_device *device, uint8_t endpoint, uint8_t lun,
	uint8_t *cdb, uint8_t direction, int data_length, uint32_t *ret_tag)
{
	static uint32_t tag = 1;
	uint8_t cdb_len;
	int size;
	struct command_block_wrapper *cbw;
	cbw=(struct command_block_wrapper *)kmalloc(sizeof(struct command_block_wrapper),GFP_KERNEL);
	cdb_len = cdb_length[cdb[0]];

	memset(cbw, 0, sizeof(*cbw));
	cbw->dCBWSignature[0] = 'U';
	cbw->dCBWSignature[1] = 'S';
	cbw->dCBWSignature[2] = 'B';
	cbw->dCBWSignature[3] = 'C';
	*ret_tag = tag;
	cbw->dCBWTag = tag++;
	cbw->dCBWDataTransferLength = data_length;
	cbw->bmCBWFlags = direction;
	cbw->bCBWLUN = lun;
	// Subclass is 1 or 6 => cdb_len
	cbw->bCBWCBLength = cdb_len;
	memcpy(cbw->CBWCB, cdb, cdb_len);

	// The transfer length must always be exactly 31 bytes.
	usb_bulk_msg(device, usb_sndbulkpipe(device,endpoint), (void*)cbw, 31, &size, 1000);


	return 0;
}

static int test_mass_storage (struct usb_device *device , uint8_t endpoint_in, uint8_t endpoint_out)
{
	int size;
	uint8_t *lun=(uint8_t *)kmalloc(sizeof(uint8_t),GFP_KERNEL);
	uint32_t expected_tag;
	uint32_t max_lba, block_size;
	long device_size;
	uint8_t cdb[16];	
	uint8_t *buffer=(uint8_t *)kmalloc(64*sizeof(uint8_t),GFP_KERNEL);



	usb_control_msg(device,usb_sndctrlpipe(device,0),0xFF,0x21,0,0,NULL,0,1000);

	usb_control_msg(device,usb_sndctrlpipe(device,0), 0xFE ,0xA1,
	   0, 0, (void*)lun, 1, 1000);

	// Read capacity
	printk(KERN_INFO "Reading Capacity:\n");
	memset(buffer, 0, sizeof(buffer));
	memset(cdb, 0, sizeof(cdb));
	cdb[0] = 0x25;	// Read Capacity

	send_mass_storage_command(device, endpoint_out, *lun, cdb, USB_DIR_IN, 0x08, &expected_tag);
	usb_bulk_msg(device, usb_rcvbulkpipe(device,endpoint_in), (void*)buffer, 64, &size, 10000);

	max_lba = be_to_int32(&buffer[0]);
	block_size = be_to_int32(&buffer[4]);
	device_size = ((long)(max_lba+1))*block_size/(1024*1024*1024);

	printk("Device Capacity: %ld GB \n",device_size);

	get_mass_storage_status(device, endpoint_in, expected_tag);

	return 0;

}

static int usbdev_probe(struct usb_interface *interface, const struct usb_device_id *id)
{   struct usb_device *device;
	device = interface_to_usbdev(interface);

	struct usb_endpoint_descriptor *ep_desc;
	unsigned char epAddr, epAttr;

	int i;


	uint8_t ep_in = 0, ep_out = 0;	


    if(id->idProduct == PID1 && id->idVendor==VID1)
	{
		printk(KERN_INFO "Known USB Drive detected\n");
	}

	printk(KERN_INFO "Vendor ID VID: %04X\n",id->idVendor);
	printk(KERN_INFO "Product ID PID: %04X\n",id->idProduct);
	printk(KERN_INFO "USB DEVICE CLASS : %x\n", interface->cur_altsetting->desc.bInterfaceClass);
	printk(KERN_INFO "USB DEVICE SUB CLASS : %x", interface->cur_altsetting->desc.bInterfaceSubClass);
	printk(KERN_INFO "USB DEVICE Protocol : %d", interface->cur_altsetting->desc.bInterfaceProtocol);
    printk(KERN_INFO "No. Of Endpoints = %d\n",interface->cur_altsetting->desc.bNumEndpoints);

	for (i=0; i<interface->cur_altsetting->desc.bNumEndpoints; i++) 
			{
				ep_desc = &interface->cur_altsetting->endpoint[i].desc;


			 	epAddr = ep_desc->bEndpointAddress;
	        	epAttr = ep_desc->bmAttributes;

		        if((epAttr & USB_ENDPOINT_XFERTYPE_MASK)==USB_ENDPOINT_XFER_BULK)
		        {
			       if(epAddr & 0x80)
				     {   
				     	 ep_in= ep_desc->bEndpointAddress;
				         printk(KERN_INFO "EP %d is Bulk IN\n", i);
				     }
			       else
				      {   
				      	  ep_out= ep_desc->bEndpointAddress;
    				      printk(KERN_INFO "EP %d is Bulk OUT\n", i);
				      }

		        }
		        } 



			if ( (interface->cur_altsetting->desc.bInterfaceClass == 0x08) && (interface->cur_altsetting->desc.bInterfaceSubClass == 0x06) 
			&& (interface->cur_altsetting->desc.bInterfaceProtocol == 0x50) ) 
			{   
				printk(KERN_INFO "Valid SCSI Device");

			}

	            test_mass_storage(device, ep_in, ep_out);

	return 0;
}

static void usbdev_disconnect(struct usb_interface *interface)
{
	printk(KERN_INFO "USBDEV Device Removed\n");
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
        	printk(KERN_INFO "Error");

        }
        else
        {
        	printk(KERN_INFO "UAS READ Capacity Driver Inserted \n");

        }

        return 0;
}

static void __exit usb_skel_exit(void)
{
        /* deregister this driver with the USB subsystem */
        usb_deregister(&skel_driver);
        printk(KERN_INFO "Exit\n");
}
module_init(usb_skel_init);
module_exit(usb_skel_exit);
MODULE_LICENSE("GPL"); 
