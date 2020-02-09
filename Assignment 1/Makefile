CONFIG_MODULE_SIG=n

obj-m := main.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	rm -rf *.o *.mod.c *.symvers *.order *.mod
	
