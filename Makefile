
#	Compiler
CC = gcc

#	compiler flags:
#	-Wall turn on compiler warnings
CFLAGS  = -Wall

# the build target executable:
TARGET = assignment4

#	Option to attach POSIX Thread Library
OPTION	= -lpthread -lrt

obj-m:= hcsr_drv.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	$(CC) $(CFLAGS) $(TARGET).c -o $(TARGET) $(OPTION)

clean:
	rm -f *.ko
	rm -f *.o
	rm -f Module.symvers
	rm -f modules.order
	rm -f *.mod
	rm -rf .tmp_versions
	rm -f *.mod.c
	rm -f *.mod.o
	rm -f \.*.cmd
	rm -f Module.markers
	$(RM) $(TARGET)
