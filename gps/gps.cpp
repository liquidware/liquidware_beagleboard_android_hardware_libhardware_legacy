#include <hardware_legacy/gps.h>
#include <cutils/properties.h>

#define LOG_TAG "libhardware_legacy"
#include <utils/Log.h>
#include "qemu.h"

static const GpsInterface*  sGpsInterface = NULL;

static void
gps_find_hardware( void )
{
#ifdef HAVE_QEMU_GPS_HARDWARE
    LOGD("gps: checking for qemu");
    if (qemu_check()) {
        LOGD("gps: qemu found, getting qemu_interface");
        sGpsInterface = gps_get_qemu_interface();
        if (sGpsInterface) {
            LOGD("using QEMU GPS Hardware emulation\n");
            return;
        } else {
            LOGD("gps: gemu GPS interface not found");
        }
    } else {
        LOGD("gps: qemu not found");
    }
#endif

#ifdef HAVE_GPS_HARDWARE
    sGpsInterface = gps_get_hardware_interface();
#endif
    if (!sGpsInterface)
        LOGD("no GPS hardware on this device\n");
}

const GpsInterface*
gps_get_interface()
{
    if (sGpsInterface == NULL)
         gps_find_hardware();

    return sGpsInterface;
}
