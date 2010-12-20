/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _HARDWARE_SERIAL_H
#define _HARDWARE_SERIAL_H

#include <stdint.h>

#if __cplusplus
extern "C" {
#endif

/** GPS status event values. */
typedef uint16_t SerialStatusValue;

// IMPORTANT: Note that the following values must match
// constants in SerialManager.java.
/** SERIAL status unknown. */
#define SERIAL_STATUS_NONE             0
/** SERIAL has powered on but is not transmitting. */
#define SERIAL_STATUS_ENGINE_ON        1
/** SERIAL is powered off. */
#define SERIAL_STATUS_ENGINE_OFF       2

/** Represents a serial message. */
typedef struct {
	uint8_t flags;
	char * data;
} SerialMsg;

/** Represents the status. */
typedef struct {
    SerialStatusValue status;
} SerialStatus;

/** Callback with status information. */
typedef void (* serial_status_callback)(SerialStatus* status);

/** Callback for reporting Serial messages. */
typedef void (* serial_receive_callback)(SerialMsg* msg);

/** SERIAL callback structure. */
typedef struct {
        serial_status_callback status_cb;
        serial_receive_callback receive_cb;
} SerialCallbacks;

/** Represents the standard GPS interface. */
typedef struct {
    /**
     * Opens the interface and provides the callback routines
     * to the implemenation of this interface.
     */
    int   (*init)( SerialCallbacks* callbacks );

    /** Starts serialing. */
    int   (*start)( const char* device, int baud );

    /** Stops serialing. */
    int   (*stop)( void );

    /** Closes the interface. */
    void  (*cleanup)( void );

    void (*print)(const char* msg);

    /** Get a pointer to extension information. */
    const void* (*get_extension)(const char* name);
} SerialInterface;

/** Returns the hardware SERIAL interface. */
const SerialInterface* serial_get_hardware_interface();

/**
 * Returns the qemu emulated SERIAL interface.
 */
const SerialInterface* serial_get_qemu_interface();

/**
 * Returns the default SERIAL interface.
 */
const SerialInterface* serial_get_interface();

#if __cplusplus
}  // extern "C"
#endif

#endif  // _HARDWARE_SERIAL_H
