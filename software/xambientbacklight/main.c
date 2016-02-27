/*
    xambientbacklight - utility to scan Xorg's screen a report to ambient
      backlight controller

    Copyright (C) 2016  Dmitry Yu Okunev <dyokunev@ut.mephi.ru> 0x8E30679C

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <X11/extensions/Xdamage.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <strings.h>
#include <stdint.h>

#define FPS 30
#define TILE_SIZE 64
#define TILES_X_COUNT 5
#define TILES_Y_COUNT 3
#define X_STEP 4
#define Y_STEP 4

#define TTY_PATH "/dev/ttyACM0"

struct __attribute__ ( ( __packed__ ) ) out {
	uint8_t preamble;
	uint8_t edge_id;
	uint8_t led_id;
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

void get_pixel ( int x, int y, Display *d, XImage *image, XColor *c )
{
	//fprintf ( stderr, "Info: get_pixel(): image == %p; x == %i; y == %i\n", image, x, y );
	unsigned long rgb = XGetPixel ( image, x, y );
	c->red   =   rgb >> 16;
	c->green = ( rgb >>  8 ) & 0xff;
	c->blue  =   rgb         & 0xff;
}

void get_tile ( int remote, char edge_id, char led_id, Display *d, XImage *image, int center_x, int center_y )
{
	float r = 0, g = 0, b = 0;
	int x, y, pixel_count = 0;
	XColor c;
	x = center_x - TILE_SIZE / 2;
	y = center_y - TILE_SIZE / 2;

	while ( x < center_x + TILE_SIZE / 2 ) {
		get_pixel ( x, y, d, image, &c );
		r += ( c.red   );
		g += ( c.green );
		b += ( c.blue  );
		pixel_count ++;
		x += X_STEP;
	}

	while ( y < center_y + TILE_SIZE / 2 ) {
		get_pixel ( x, y, d, image, &c );
		r += ( c.red   );
		g += ( c.green );
		b += ( c.blue  );
		pixel_count ++;
		y += Y_STEP;
	}

	while ( x > center_x - TILE_SIZE / 2 ) {
		get_pixel ( x, y, d, image, &c );
		r += ( c.red   );
		g += ( c.green );
		b += ( c.blue  );
		pixel_count ++;
		x -= X_STEP;
	}

	while ( y > center_y - TILE_SIZE / 2 ) {
		get_pixel ( x, y, d, image, &c );
		r += ( c.red   );
		g += ( c.green );
		b += ( c.blue  );
		pixel_count ++;
		y -= Y_STEP;
	}

	r /= pixel_count;
	g /= pixel_count;
	b /= pixel_count;
	//printf ( "%i\t%i\t%i\t%i\t%i\t%i\t%i\n", edge_id, led_id, x, y, (int)r, (int)g, (int)b );
	struct out s;

	if ( r > 254 ) r = 254;

	if ( g > 254 ) g = 254;

	if ( b > 254 ) b = 254;

	s.preamble = 0;
	s.edge_id  = edge_id + 1;
	s.led_id   = led_id + 1;
	s.r	   = r+1;
	s.g	   = g+1;
	s.b	   = b+1;
	assert ( write ( remote, &s, sizeof ( s ) ) == sizeof ( s ) );
	return;
}


int open_remote ()
{
	struct termios tty;
	const int fd = open ( TTY_PATH, O_RDWR | O_NONBLOCK );
	assert ( fd /* of the /dev/tty* device */ != -1 );
	assert ( tcgetattr ( fd, &tty ) >= 0 );
	cfsetospeed ( &tty, ( speed_t ) B115200 );
	cfsetispeed ( &tty, ( speed_t ) B115200 );
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |=  CS8;
	tty.c_cflag |= ( CLOCAL | CREAD );
	tty.c_cflag &= ~ ( PARENB | PARODD );
	tty.c_cflag &= ~ ( CRTSCTS );
	tty.c_cflag &= ~ ( CSTOPB );
	tty.c_iflag |= IGNBRK;
	tty.c_iflag &= ~ ( IXON | IXOFF | IXANY );
	tty.c_lflag  = 0;
	tty.c_oflag  = 0;
	tcflush ( fd, TCIFLUSH );
	assert ( !tcsetattr ( fd, TCSANOW, &tty ) );
	//int flags = fcntl(fd, F_GETFL, 0);
	//assert ( fcntl(fd, F_SETFL, flags | O_NONBLOCK) != -1 );
	return fd;
}

int main ( int argc, char *argv[] )
{
	XShmSegmentInfo shminfo;
	Display *display = XOpenDisplay ( NULL );

	if ( display == NULL ) {
		fprintf ( stderr, "Trying display :0\n" );
		display = XOpenDisplay ( ":0" );
	}

	assert ( display != NULL );
	// Initializing SHM
	int dummy;
	int remote;
	assert ( XQueryExtension ( display, "MIT-SHM", &dummy, &dummy, &dummy ) );
	int screen    = DefaultScreen ( display );
	int root      = RootWindow ( display, screen );
	int width     = DisplayWidth  ( display, screen );
	int height    = DisplayHeight ( display, screen );
	int running   = 1;
	XImage *image = XShmCreateImage ( display, DefaultVisual ( display, screen ),
	                                  DefaultDepth ( display, screen ),
	                                  ZPixmap, NULL, &shminfo,
	                                  width, height );
	//fprintf ( stderr, "Info: image == %p; w == %i; h == %i; image->bytes_per_line == %i\n", image, width, height, image->bytes_per_line );
	assert ( image != NULL );
	assert ( image->height > 0 );
	assert ( image->bytes_per_line > 0 );
	shminfo.shmid = shmget ( IPC_PRIVATE, image->bytes_per_line * image->height,
	                         IPC_CREAT | 0777 );

	if ( shminfo.shmid < 0 ) {
		fprintf ( stderr, "Error: \"shminfo.shmid < 0\": %s\n", strerror ( errno ) );
		return errno;
	}

	shminfo.shmaddr = image->data = ( char * ) shmat ( shminfo.shmid, 0, 0 );
	assert ( shminfo.shmaddr != ( char * ) - 1 );
	assert ( shminfo.shmaddr != NULL );
	shminfo.readOnly = False;
	assert ( XShmAttach ( display, &shminfo ) );
	// Initializing Damage
	int damage_event, damage_error;
	XDamageQueryExtension ( display, &damage_event, &damage_error );
	Damage damage = XDamageCreate ( display, root, XDamageReportNonEmpty );
	XEvent ev;
	// The main loop
	remote = open_remote();
	fprintf ( stderr, "Running\n" );

	while ( running ) {
		int i;
		XNextEvent ( display, &ev );

		if ( ev.type == damage_event + XDamageNotify ) {
			XDamageSubtract ( display, damage, None, None );
			XShmGetImage ( display, root, image, 0, 0, AllPlanes );
			//printf("test\n");continue;
			// TOP and BOTTOM
			i = 0;

			while ( i < TILES_X_COUNT ) {
				get_tile ( remote, 0, i, display, image, TILE_SIZE / 1 + ( i * ( width - TILE_SIZE ) ) / TILES_X_COUNT, TILE_SIZE / 2 );
				get_tile ( remote, 1, i, display, image, TILE_SIZE / 2 + ( i * ( width - TILE_SIZE ) ) / TILES_X_COUNT, height - TILE_SIZE / 2 - 1 );
				i++;
			}

			// LEFT and RIGHT
			i = 0;

			while ( i < TILES_Y_COUNT ) {
				get_tile ( remote, 2, i, display, image,         TILE_SIZE / 2    , TILE_SIZE / 2 + ( i * ( height - TILE_SIZE ) ) / TILES_Y_COUNT );
				get_tile ( remote, 3, i, display, image, width - TILE_SIZE / 2 - 1, TILE_SIZE / 2 + ( i * ( height - TILE_SIZE ) ) / TILES_Y_COUNT );
				i++;
			}

			{
				char buf[BUFSIZ];
				int r = read ( remote, buf, BUFSIZ );
				fprintf ( stderr, "read -> %i\n", r );
				if ( r > 0 )
					assert ( write ( 2, buf, r ) > 0 );
			}

			usleep ( 1000000 / FPS );
		}
	}

	return 0;
}
