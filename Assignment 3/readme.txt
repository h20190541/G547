#########################################################################

CONTENTS OF THE FILE
====================

1. Details of the Project
2. System Requirements
3. Compilation
4. Running the Code
5. Mounting
6. Testing
7. Unmounting
8. Exiting


#########################################################################

1. DETAILS OF THE PROJECT
=========================

This README file contains information for the project based on implementing a Block Device Driver for the USB drive.

The block device driver includes the following features:

 - Implements the BIO request processing function.

 - Implements the READ(10) and WRITE(10) SCSI command functions in order to read and write sectors on the USB disk.

 - Allocates the USB drive as a disk of its size.

 - Reads the bio and defers the read/write work to the bottom half using workqueue.

 - Process the request in the bottom half by calling appropriate functions (READ/WRITE).


#########################################################################

2. SYSTEM REQUIREMENTS
======================

The code must be run on a LINUX operating system with Kernel Version < 5.0.

 - This code won't work on systems with kernel version > 5.0 as newer versions have completely migrated to multi-queue block layer (blk-mq) by default.

 - Set the values of VID1 (Vendor ID), PID1 (Product ID) and NR_OF_SECTORS (Number of Sectors) in the main.c file.


#########################################################################

3. COMPILATION
==============

To compile successfully, both the files (makefile and main.c) must be in same directory.

Follow the steps for compilation:

 - Open the terminal and move to the directory containing these files.

 - Run the command: 

    		make all


#########################################################################

4. RUNNING THE CODE
===================

After compilation, a kernel object file (main.ko) will be created in the same directory.

To run the code, follow the steps below:

 - Remove the default usb driver modules from kernel by running the following commands:

    		sudo rmmod uas

     		sudo rmmod usb_storage

 - Insert the module into the kernel by typing the following command:

     		sudo insmod main.ko

 - To view driver related messages of kernel, open a new terminal and type the following command:

     		dmesg -wH

Insert the pendrive

 - To view whether the driver is registered and the disk is partitioned, type:

 			sudo fdisk -l

 - The disk partition will appear as 'blockdev1'.

 - Again, remove the default usb driver modules from kernel by running the following commands:

    		sudo rmmod uas

     		sudo rmmod usb_storage


#########################################################################
 
5. MOUNTING
===========

To mount the drive in the system, run the following command in the terminal:

 - Create a mount directory:

 			sudo mkdir /media/mount_dir

 - Mount the drive with fat32 filesystem:

  			sudo mount -t vfat /dev/blockdev1 /media/mount_dir


#########################################################################

6. TESTING
==========

After buliding the module, test it to see the BIO request processing of the driver.

 - To gain root access, run the command:

 			sudo -i

 - Change the directory to the mount point:

 			cd /media/mount_dir/

 - To test the write10 function, run the following command:

 			echo final final > controltry.txt

 - To test the read10 function, run the following command:

 			cat controltry.txt

 - Exit from the root access:

 			exit


#########################################################################

7. UNMOUNTING
=============

Unmount the filesystem before removing the pendrive:

			sudo umount /media/mount_dir


#########################################################################

8. EXITING
==========

After unmounting, remove the module by following these steps:

 - Remove the Pendrive

 - Remove the module from Kernel:

 			sudo rmmod main.ko