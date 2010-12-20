# Use hardware serial implementation if available.
#
ifneq ($(BOARD_SERIAL_LIBRARIES),)
  LOCAL_CFLAGS           += -DHAVE_SERIAL_HARDWARE
  LOCAL_SHARED_LIBRARIES += $(BOARD_SERIAL_LIBRARIES)
endif

# Use serial hardware implementation if QEMU_HARDWARE is set.
#
USE_QEMU_SERIAL_HARDWARE := $(QEMU_HARDWARE)

ifeq ($(USE_QEMU_SERIAL_HARDWARE),true)
    LOCAL_CFLAGS    += -DHAVE_QEMU_SERIAL_HARDWARE
    LOCAL_SRC_FILES += serial/serial_qemu.c
endif

LOCAL_SRC_FILES += serial/serial.cpp

