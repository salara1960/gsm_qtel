EXTRA_CFLAGS += -Wall
export ${EXTRA_CFLAGS}
GRP=apache

obj-m := k16_mod.o
k16_mod-objs:= k16.o

MODULE_NAME = k16_mod
KVER := `uname -r`

all:
	make -C /lib/modules/$(KVER)/build M=${PWD} modules
clean:
	make -C /lib/modules/$(KVER)/build M=${PWD} clean
install:
	chown root:$(GRP) $(MODULE_NAME).ko
	chmod g+rw $(MODULE_NAME).ko
	install -p -m 644 $(MODULE_NAME).ko /lib/modules/$(KVER)/kernel/drivers/
	/sbin/depmod -a
