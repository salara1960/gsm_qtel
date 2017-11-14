EXTRA_CFLAGS += -Wall
export ${EXTRA_CFLAGS}
GRP=apache

obj-m := tmr_mod.o
tmr_mod-objs:= tmr.o

MODULE_NAME = tmr_mod
KVER := `uname -r`

all:
	make -C /lib/modules/$(KVER)/build M=${PWD} modules
clean:
	make -C /lib/modules/$(KVER)/build M=${PWD} clean
install:
	chown :$(GRP) $(MODULE_NAME).ko
	chmod g+rw $(MODULE_NAME).ko
	install -p -m 644 $(MODULE_NAME).ko /lib/modules/$(KVER)/kernel/drivers/
	/sbin/depmod -a
