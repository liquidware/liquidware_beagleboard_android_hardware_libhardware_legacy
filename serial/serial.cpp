#include <hardware_legacy/serial.h>
#include <cutils/properties.h>

#define LOG_TAG "serial_hardware"
#include <utils/Log.h>
#include "qemu.h"

static const SerialInterface*  sSerialInterface = NULL;

static void
serial_find_hardware( void )
{
#ifdef HAVE_QEMU_SERIAL_HARDWARE
    LOGD("serial: checking for qemu");
    if (qemu_check()) {
        LOGD("serial: qemu found, getting qemu_interface");
        sSerialInterface = serial_get_qemu_interface();
        if (sSerialInterface) {
            LOGD("using QEMU SERIAL Hardware emulation\n");
            return;
        } else {
            LOGD("serial: gemu SERIAL interface not found");
        }
    } else {
        LOGD("serial: qemu not found");
    }
#endif

#ifdef HAVE_SERIAL_HARDWARE
    sSerialInterface = serial_get_hardware_interface();
#endif
    if (!sSerialInterface)
        LOGD("no SERIAL hardware on this device\n");
}

const SerialInterface*
serial_get_interface()
{
    if (sSerialInterface == NULL)
         serial_find_hardware();

    return sSerialInterface;
}
