KDIR= /lib/modules/$(shell uname -r)/build

obj-m := driver.o



all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean

modules_install:
	make -C $(KDIR) M=$(PWD) modules_install
