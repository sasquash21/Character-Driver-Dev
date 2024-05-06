KVERSION = $(shell uname -r)
KERNELDIR = /usr/src/linux-headers-6.2.0-32-generic

CFLAGS = -D__KERNEL__ -DMODULE -I$(KERNELDIR)/include -O -Wall

obj-m = chardriver.o
all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
