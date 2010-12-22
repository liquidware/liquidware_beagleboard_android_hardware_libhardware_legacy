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

/* this file contains various functions used by all libhardware modules
 * that support QEMU emulation
 */
#include "qemu.h"
#define  LOG_TAG  "hardware-qemu"
#include <cutils/log.h>
#include <cutils/properties.h>
#include <cutils/sockets.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdarg.h>

#define  QEMU_DEBUG  1

#if QEMU_DEBUG
#  define  D(...)   LOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif


int
qemu_check(void)
{
    static int  in_qemu = -1;

    if (__builtin_expect(in_qemu < 0,0)) {
        char  propBuf[PROPERTY_VALUE_MAX];
        property_get("ro.kernel.qemu", propBuf, "");
        in_qemu = (propBuf[0] == '1');
    }
    return in_qemu;
}

int
qemu_fd_write( int  fd, const char*  cmd, int  len )
{
    int  len2;
    do {
        len2 = write(fd, cmd, len);
    } while (len2 < 0 && errno == EINTR);
    fsync(fd);
    return len2;
}

int
qemu_fd_read( int  fd, char*  buff, int  len )
{
    int  len2;
    do {
        len2 = read(fd, buff, len);
    } while (len2 < 0 && errno == EINTR);
    return len2;
}


static int
qemu_channel_open_qemud( QemuChannel*  channel,
                         const char*   name )
{
    int   fd, ret, namelen = strlen(name);
    char  answer[2];

    fd = socket_local_client( "qemud",
                              ANDROID_SOCKET_NAMESPACE_RESERVED,
                              SOCK_STREAM );
    if (fd < 0) {
        D("no qemud control socket: %s", strerror(errno));
        return -1;
    }

    /* send service name to connect */
    if (qemu_fd_write(fd, name, namelen) != namelen) {
        D("can't send service name to qemud: %s",
           strerror(errno));
        close(fd);
        return -1;
    }

    /* read answer from daemon */
    if (qemu_fd_read(fd, answer, 2) != 2 ||
        answer[0] != 'O' || answer[1] != 'K') {
        D("cant' connect to %s service through qemud", name);
        close(fd);
        return -1;
    }

    channel->is_qemud = 1;
    channel->fd       = fd;
    return 0;
}


static int
qemu_channel_open_qemud_old( QemuChannel*  channel,
                             const char*   name )
{
    int  fd;

    snprintf(channel->device, sizeof channel->device,
                "qemud_%s", name);

    fd = socket_local_client( channel->device,
                              ANDROID_SOCKET_NAMESPACE_RESERVED,
                              SOCK_STREAM );
    if (fd < 0) {
        D("no '%s' control socket available: %s",
            channel->device, strerror(errno));
        return -1;
    }

    close(fd);
    channel->is_qemud_old = 1;
    return 0;
}

static int
qemu_channel_get_device_tty_initrc(QemuChannel*  channel,
        					   	   const char*   name,
        					   	   int           mode ) {
    char   key[PROPERTY_KEY_MAX];
    char   prop[PROPERTY_VALUE_MAX];
    int    ret = -1;

    ret = snprintf(key, sizeof key, "ro.kernel.android.%s", name);
	if (ret >= (int)sizeof key)
		return -1;

	if (property_get(key, prop, "") == 0) {
		LOGE("qemu_channel_get_device_tty_initrc: no kernel-provided %s device name", name);
		return -1;
	}

	ret = snprintf(channel->device, sizeof channel->device,
					"/dev/%s", prop);

	D("Using property ro.kernel.android.%s=%s, device=%s", name, prop, channel->device);
	if (ret >= (int)sizeof channel->device) {
		LOGE("qemu_channel_open_tty: %s device name too long: '%s'", name, prop);
		return -1;
	}

	return ret;
}

static int
qemu_channel_open_tty( QemuChannel*  channel,
                       const char*   name,
                       int           mode )
{
	int ret;
	D("qemu_channel_open_tty: Begin");
    
    if (!channel->getDeviceFromApp) {
    	//Disabled
    	//ret = qemu_channel_get_device_tty_initrc(channel,
    	//										 name,
    	//										 mode);
    	if (ret < 0) {
    		LOGE("Error: couldn't get tty device from init.rc file");
    		return -1;
    	}
    }
    
    D("qemu_channel_open_tty: device=%s getDeviceFromApp=%d", channel->device, channel->getDeviceFromApp);
    channel->is_tty = !memcmp("/dev/tty", channel->device, 8);
    
    if (channel->is_tty) {
    	D("qemu_channel_open_tty: Using tty device %s", channel->device);
    } else {
    	LOGE("Error: channel is not a tty");
    	return -1;
    }

    return 0;
}

void
qemu_channel_close( QemuChannel*  channel,
                    int fd)
{
	D("Closing %s", channel->device);
	channel->is_inited = 0;
	close(fd);
}

int
qemu_channel_get_baud(int baud) {
	int bmask;

	switch (baud) {
	case 4800:
		bmask = B4800;
		break;
	case 9600:
		bmask = B9600;
		break;
	case 19200:
		bmask = B19200;
		break;
	case 38400:
		bmask = B38400;
		break;
	case 57600:
		bmask = B57600;
		break;
	case 115200:
		bmask = B115200;
		break;
	case 230400:
		bmask = B230400;
		break;
	case 1000000:
		bmask = B1000000;
		break;
	case 2000000:
		bmask = B2000000;
		break;
	case 3000000:
		bmask = B3000000;
		break;
	case 4000000:
		bmask = B4000000;
		break;
	default:
		bmask = B9600;
	}

	D("Serial baud %d, mask: %d", baud, bmask);
	return bmask;
}

int
qemu_channel_open( QemuChannel*  channel,
                   const char*   name,
                   int           mode )
{
    int  fd = -1;
    D("qemu_channel_open: device=%s getDeviceFromApp=%d", channel->device, channel->getDeviceFromApp);

    /* initialize the channel is needed */
    if (!channel->is_inited)
    {
        channel->is_inited = 1;

        do {
        	// disabled
            //if (qemu_channel_open_qemud(channel, name) == 0)
            //    break;

            //if (qemu_channel_open_qemud_old(channel, name) == 0)
            //    break;

            if (qemu_channel_open_tty(channel, name, mode) == 0)
                break;

            channel->is_available = 0;
            return -1;
        } while (0);

        channel->is_available = 1;
    } else {
    	D("qemu_channel_open: channel already init");
    }

    /* try to open the file */
    if (!channel->is_available) {
    	D("qemu_channel_open: file not available");
        errno = ENOENT;
        return -1;
    }

    if (channel->is_qemud) {
        return dup(channel->fd);
    }

    if (channel->is_qemud_old) {
        do {
            fd = socket_local_client( channel->device,
                                      ANDROID_SOCKET_NAMESPACE_RESERVED,
                                      SOCK_STREAM );
        } while (fd < 0 && errno == EINTR);
    }
    else /* /dev/ttySn ? */
    {
        D("Opening device '%s'", channel->device);
        do {
        	fd = open(channel->device, O_RDWR | O_NONBLOCK, 0);
            D("Waiting for device %s", channel->device);
        } while (fd < 0 && errno == EINTR);

        if (fd < 0) {
        	D("Error opening device, error=%s\n", strerror(errno));
        	return -1;
        } else {
        	D("Device opened successfully\n");
        }

        /* disable ECHO on serial lines */
        if (fd >= 0 && channel->is_tty) {
            struct termios  ios;
            int ret, flags;
            int baud;

            baud = qemu_channel_get_baud(channel->baud);

            /* Set flags */
            tcgetattr( fd, &ios );
            ios.c_cflag =  baud | CS8 | CLOCAL | CREAD;
            ios.c_cflag &= ~HUPCL; //disable hang-up on close to avoid reset
            ios.c_lflag &= ~(ECHO | ICANON);
            tcsetattr( fd, TCSANOW, &ios );
            D("Serial flags: c_iflag=%d,c_oflag=%d,c_cflag=%d,c_lflag=%d", ios.c_iflag, ios.c_oflag, ios.c_cflag, ios.c_lflag);
            D("Successfully set flags for %s", channel->device);
        } else {
        	LOGE("qemu_channel_open: error opening device %s, fd=%d, is_tty=%d, errno=%d", channel->device, fd, channel->is_tty, errno);
        }
    }
    return fd;
}


static int
qemu_command_vformat( char*        buffer, 
                      int          buffer_size,
                      const char*  format,
                      va_list      args )
{
    char     header[5];
    int      len;

    if (buffer_size < 6)
        return -1;

    len = vsnprintf(buffer+4, buffer_size-4, format, args);
    if (len >= buffer_size-4)
        return -1;

    snprintf(header, sizeof header, "%04x", len);
    memcpy(buffer, header, 4);
    return len + 4;
}

extern int
qemu_command_format( char*        buffer, 
                     int          buffer_size,
                     const char*  format,
                     ... )
{
    va_list  args;
    int      ret;

    va_start(args, format);
    ret = qemu_command_format(buffer, buffer_size, format, args);
    va_end(args);
    return ret;
}


static int
qemu_control_fd(void)
{
    static QemuChannel  channel[1];
    int                 fd;
    char				dev[] = {"/dev/ttyUSB0"};

    fd = qemu_channel_open( channel, dev, O_RDWR );
    if (fd < 0) {
        D("%s: could not open control channel: %s, error: %s", __FUNCTION__, dev,
          strerror(errno));
    }
    return fd;
}

int
qemu_control_send(int fd, const char*  cmd, int  len)
{
    //int  fd, len2;
	int len2;

    if (len < 0) {
        errno = EINVAL;
        return -1;
    }

    //fd = qemu_control_fd();
    if (fd < 0)
        return -1;

    len2 = qemu_fd_write(fd, cmd, len);
    //close(fd);
    if (len2 != len) {
        D("%s: could not send everything %d < %d",
          __FUNCTION__, len2, len);
        return -1;
    }
    return 0;
}

int
qemu_control_command( const char*  fmt, ... )
{
    va_list  args;
    char     command[256];
    int      len, fd;

    va_start(args, fmt);
    len = qemu_command_vformat( command, sizeof command, fmt, args );
    va_end(args);

    if (len < 0 || len >= (int)sizeof command) {
        if (len < 0) {
            D("%s: could not send: %s", __FUNCTION__, strerror(errno));
        } else {
            D("%s: too large %d > %d", __FUNCTION__, len, (int)(sizeof command));
        }
        errno = EINVAL;
        return -1;
    }

    //return qemu_control_send( command, len );
    return 0;
}

extern int  qemu_control_query( const char*  question, int  questionlen,
                                char*        answer,   int  answersize )
{
    int   ret, fd, len, result = -1;
    char  header[5], *end;

    if (questionlen <= 0) {
        errno = EINVAL;
        return -1;
    }

    fd = qemu_control_fd();
    if (fd < 0)
        return -1;

    ret = qemu_fd_write( fd, question, questionlen );
    if (ret != questionlen) {
        D("%s: could not write all: %d < %d", __FUNCTION__,
          ret, questionlen);
        goto Exit;
    }

    /* read a 4-byte header giving the length of the following content */
    ret = qemu_fd_read( fd, header, 4 );
    if (ret != 4) {
        D("%s: could not read header (%d != 4)",
          __FUNCTION__, ret);
        goto Exit;
    }

    header[4] = 0;
    len = strtol( header, &end,  16 );
    if ( len < 0 || end == NULL || end != header+4 || len > answersize ) {
        D("%s: could not parse header: '%s'",
          __FUNCTION__, header);
        goto Exit;
    }

    /* read the answer */
    ret = qemu_fd_read( fd, answer, len );
    if (ret != len) {
        D("%s: could not read all of answer %d < %d",
          __FUNCTION__, ret, len);
        goto Exit;
    }

    result = len;

Exit:
    close(fd);
    return result;
}
