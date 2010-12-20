#include <errno.h>
#include <pthread.h>
#include "qemu.h"
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#define  LOG_TAG  "serial_qemu"
#include <cutils/log.h>
#include <cutils/sockets.h>
#include <hardware_legacy/serial.h>

/* the name of the qemud-controlled socket */
#define  QEMU_CHANNEL_NAME  "serial"

#define  SERIAL_DEBUG  1

#if SERIAL_DEBUG
#  define  D(...)   LOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

#define SERIAL_MSG_MAX_SIZE 1024

typedef struct {
	int pos;
	int overflow;
	SerialMsg receive;
	serial_receive_callback callback;
	char in [SERIAL_MSG_MAX_SIZE + 1];
} SerialReader;

typedef struct {
	int pos;
	int overflow;
	SerialMsg transmit;
	char out [SERIAL_MSG_MAX_SIZE + 1];
} SerialWriter;

SerialWriter serial_writer[1];

static void
serial_reader_init( SerialReader*  r )
{
    memset( r, 0, sizeof(*r) );

    r->pos      = 0;
    r->overflow = 0;
    r->callback = NULL;
}

static void
serial_reader_set_callback( SerialReader*  r, serial_receive_callback  cb )
{
	D("Setting callback in %s", __FUNCTION__);
    r->callback = cb;
    if (cb != NULL && r->receive.flags != 0) {
        D("%s: sending latest receive message to new callback", __FUNCTION__);
        r->callback( &r->receive );
        r->receive.flags = 0;
    }
}

static void
serial_reader_parse( SerialReader*  r )
{
   /* we received a complete sentence, now parse it to generate
    * a new serial message...
    */

	/**
	 * Error check
	 */
    D("Received: '%.*s'", r->pos, r->in);
    if (r->pos < 1) {
        D("Too short. discarded.");
        return;
    }


    /*
     * Parse the message
     */
	r->receive.data = &r->in[0];		// Store a pointer to our message
	r->receive.flags = 1;				// Message received

#if 0
    char * pch;
    int msg_id = 0;
    char * msg_txt = NULL;

    pch = strtok(r->in, ",");
    if (pch != NULL) {
    	msg_id = atoi(pch);
    	D("Serial Message ID=%d", msg_id);
    	pch = strtok(NULL, ",");
    	msg_txt = pch;
    	if (msg_txt != NULL) {
    		/* Store a pointer to the data */
    		r->receive.data = msg_txt;
    		r->receive.flags = 1;				//message received
    	} else {
    		D("No message");
    	}
    } else {
    	D("Invalid message");
    }
#endif

    /**
     * Handle the message
     */
	if (r->receive.flags != 0) {
	    if (r->callback) {
	    	D("%s sending '%s' to callback", __FUNCTION__, r->receive.data);
	        r->callback(&r->receive);
	        r->receive.flags = 0;
	    }
	    else {
	        D("No callback, keeping data until needed!");
	    }
	}
}

static void
serial_reader_addc( SerialReader*  r, int  c )
{
    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == '\n') {
    	r->in[r->pos] = 0; //terminate the string
        serial_reader_parse(r);
        r->pos = 0;
    }
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/* commands sent to the serial thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2
};


/* this is the state of our connection to the qemu_seriald daemon */
typedef struct {
    int                     init;
    int                     fd;
    SerialCallbacks            callbacks;
    pthread_t               thread;
    int                     control[2];
    QemuChannel             channel;
} SerialState;

static SerialState  _serial_state[1];

static void
serial_state_done( SerialState*  s )
{
    // tell the thread to quit, and wait for it
	D("serial_state_done: Telling Serial thread to quit, waiting for it");

    char   cmd = CMD_QUIT;
    void*  dummy;
    write( s->control[0], &cmd, 1 );
    pthread_join(s->thread, &dummy);

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

	/* Close Serial*/
    D("serial_state_done: Closing qemu channel");
    qemu_channel_close(&s->channel, s->fd);
    s->fd = -1;

    s->init = 0;
    s->channel.baud = 0;
    s->channel.getDeviceFromApp = 0;
    memset(s->channel.device, 0, sizeof(s->channel.device));
    D("Serial closed.");
}


static int
serial_state_start( SerialState*  s )
{
    char  cmd = CMD_START;
    int   ret;

    D("serial_state_start");

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1) {
        LOGE("%s: could not send CMD_START command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
        return -1;
    }

    return 0;
}


static int
serial_state_stop( SerialState*  s )
{
    char  cmd = CMD_STOP;
    int   ret;

    D("serial_state_stop");

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1) {
        LOGE("%s: could not send CMD_STOP command: ret=%d: %s",
          __FUNCTION__, ret, strerror(errno));
        return -1;
    }

    return 0;
}


static int
epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int                 ret, flags;

    D("epoll_register");

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN | EPOLLOUT;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}


static int
epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;
    D("epoll_deregister");

    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

/* this is the main thread, it waits for commands from serial_state_start/stop and,
 * when started, messages from the QEMU SERIAL daemon. these are simple NMEA sentences
 * that must be parsed to be converted into SERIAL fixes sent to the framework
 */
static void*
serial_state_thread( void*  arg )
{
    SerialState*   state = (SerialState*) arg;
    SerialReader serial_reader[1];
    int         epoll_fd   = epoll_create(2);
    int         started    = 0;
    int         serial_fd     = state->fd;
    int         control_fd = state->control[1];

    serial_reader_init( serial_reader );

    // register control file descriptors for polling
    epoll_register( epoll_fd, control_fd );
    epoll_register( epoll_fd, serial_fd );

    D("serial thread running");

    // now loop
    for (;;) {
        struct epoll_event   events[2];
        int                  ne, nevents, ret, nn;

        nevents = epoll_wait( epoll_fd, events, 2, 100 );
        if (nevents < 0) {
            if (errno != EINTR)
                LOGE("epoll_wait() unexpected error: %s", strerror(errno));
            continue;
        }
        //D("serial thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) {
        	//D("events=%d, fd=%d, control_fd=%d, serial_fd=%d", events[ne].events,
        	//												   events[ne].data.fd,
        	//												   control_fd,
        	//												   serial_fd);

            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                LOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                goto Exit;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd)
                {
                    char  cmd = 255;
                    int   ret;
                    D("serial control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        D("serial thread quitting on demand");
                        goto Exit;
                    }
                    else if (cmd == CMD_START) {
                        if (!started) {
                            D("serial thread starting  location_cb=%p", state->callbacks.receive_cb);
                            started = 1;
                            serial_reader_set_callback( serial_reader, state->callbacks.receive_cb );
                        }
                    }
                    else if (cmd == CMD_STOP) {
                        if (started) {
                            D("serial thread stopping");
                            started = 0;
                            serial_reader_set_callback( serial_reader, NULL );
                        }
                    }
                }
                else if (fd == serial_fd)
                {
                    char  buff[32];
                    D("serial fd event");
                    for (;;) {
                        int  nn, ret;

                        ret = read( fd, buff, sizeof(buff) );
                        if (ret < 0) {
                            if (errno == EINTR)
                                continue;
                            if (errno != EWOULDBLOCK)
                                LOGE("error while reading from serial daemon socket: %s:", strerror(errno));
                            break;
                        }
                        for (nn = 0; nn < ret; nn++)
                        	serial_reader_addc(serial_reader, buff[nn]);
                    }
                    D("serial fd event end");
                }
                else
                {
                    LOGE("epoll_wait() returned unkown fd %d ?", fd);
                }
            }
        }
    }
Exit:
	// deregister control file descriptors
 	 epoll_deregister( epoll_fd, control_fd );
 	 epoll_deregister( epoll_fd, serial_fd );
    return NULL;
}


static void
serial_state_init( SerialState*  state )
{
    state->init       = 1;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;

    D("serial_state_init: about to open");
    state->fd = qemu_channel_open( &state->channel,
                                   QEMU_CHANNEL_NAME,
                                   O_RDWR);

    if (state->fd < 0) {
        D("no serial emulation detected");
        return;
    }

    D("serial emulation will read from '%s' qemud channel", QEMU_CHANNEL_NAME );

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        LOGE("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }

    if ( pthread_create( &state->thread, NULL, serial_state_thread, state ) != 0 ) {
        LOGE("could not create serial thread: %s", strerror(errno));
        goto Fail;
    }

    D("serial state initialized");
    return;

Fail:
    serial_state_done( state );
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/


static int
qemu_serial_init(SerialCallbacks* callbacks)
{
    SerialState*  s = _serial_state;

    D("qemu_serial_init");

    s->callbacks = *callbacks;

    return 0;
}

static void
qemu_serial_cleanup(void)
{
    SerialState*  s = _serial_state;

    D("qemu_serial_cleanup");

    if (s->init)
        serial_state_done(s);
}

static int
qemu_serial_stop()
{
    SerialState*  s = _serial_state;

    D("qemu_serial_stop");

    if (!s->init) {
        D("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    D("%s: called", __FUNCTION__);
    serial_state_stop(s);

    return 0;

}

static int
qemu_serial_start(const char* device, int baud)
{

	SerialState*  s = _serial_state;
	QemuChannel* channel = &s->channel;

	D("qemu_serial_start:");

	/* Stop the thread and cleanup files */
	qemu_serial_cleanup();

    /* Copy application parameters */
    snprintf(channel->device, sizeof channel->device,
        						"/dev/%s", device);
    channel->baud = baud;
    channel->getDeviceFromApp = 1;

    /* Initialize the serial device */
    D("qemu_serial_start: Trying device='%s' at %d baud", channel->device, baud);
    if (!s->init) {
    	serial_state_init(s);
    }

    /* Error checking */
    if (s->fd < 0) {
    	LOGE("Error: could not init serial");
    	return -1;
    }

    if (!s->init) {
        LOGE("%s: uninitialized state !!", __FUNCTION__);
        return -1;
    }

    if (serial_state_start(s) < 0) {
    	LOGE("Error: cannot send start command to thread");
    	return -1;
    }

    return 0;
}




static void
qemu_serial_print(const char* msg)
{
	SerialState*  s = _serial_state;

	D("Printing '%s' from %s", msg, __FUNCTION__);
	qemu_control_send(s->fd, msg, strlen(msg));
	D("%s Done Printing", __FUNCTION__);
}

static const void*
qemu_serial_get_extension(const char* name)
{
    return NULL;
}

static const SerialInterface  qemuSerialInterface = {
    qemu_serial_init,
    qemu_serial_start,
    qemu_serial_stop,
    qemu_serial_cleanup,
    qemu_serial_print,
    qemu_serial_get_extension,
};

const SerialInterface* serial_get_qemu_interface()
{
    return &qemuSerialInterface;
}
