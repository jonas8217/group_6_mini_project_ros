obj-m += reservedmemLKM.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

install:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules && sudo insmod reservedmemLKM.ko && sudo chmod 666 /dev/reservedmemLKM

reload:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules && sudo rmmod reservedmemLKM && sudo insmod reservedmemLKM.ko && sudo chmod 666 /dev/reservedmemLKM

uninstall:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean && sudo rmmod reservedmemLKM

cleanTrash:
	rm *.o *.mod .*.ko.cmd .*.mod.cmd .*.mod.o.cmd* *.mod.c .*mod.c .*.o.cmd *.symvers *.order

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean