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

#define FPS 30
#define TILE_SIZE 64
#define TILES_X_COUNT 5
#define TILES_Y_COUNT 3
#define X_STEP 4
#define Y_STEP 4

struct out {
	char edge_id;
	char led_id;
	char r;
	char g;
	char b;
};

void get_pixel ( int x, int y, Display *d, XImage *image, XColor *c )
{
	unsigned long rgb = XGetPixel ( image, x, y );
	c->red   =  rgb >> 16;
	c->green = (rgb >>  8) & 0xff;
	c->blue  =  rgb        & 0xff;
}

void get_tile ( char edge_id, char led_id, Display *d, XImage *image, int center_x, int center_y )
{
	float r=0, g=0, b=0;
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

	s.edge_id = edge_id;
	s.led_id  = led_id;
	s.r       = r;
	s.g       = g;
	s.b       = b;

	assert (write(1, &s, sizeof(s)) == sizeof(s));
	return;
}

int main ( int argc, char *argv[] )
{
	XShmSegmentInfo shminfo;
	Display *display = XOpenDisplay ( NULL );
	assert ( display != NULL );
	// Initializing SHM
	int dummy;
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
	assert ( image != NULL );
	assert ( image->height > 0 );
	assert ( image->bytes_per_line > 0 );
	shminfo.shmid = shmget ( IPC_PRIVATE, image->bytes_per_line * image->height,
	                         IPC_CREAT | 0777 );

	if ( shminfo.shmid < 0 ) {
		fprintf ( stderr, "Error: shminfo.shmid < 0: %s\n", strerror ( errno ) );
		return errno;
	}

	shminfo.shmaddr = image->data = ( char * ) shmat ( shminfo.shmid, 0, 0 );
	assert ( shminfo.shmaddr != ( char * ) - 1 );
	shminfo.readOnly = False;
	assert ( XShmAttach ( display, &shminfo ) );
	// Initializing Damage
	int damage_event, damage_error;
	XDamageQueryExtension ( display, &damage_event, &damage_error );
	Damage damage = XDamageCreate ( display, root, XDamageReportNonEmpty );
	XEvent ev;

	// The main loop

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
				get_tile ( 0, i, display, image, TILE_SIZE / 1 + ( i * ( width - TILE_SIZE ) ) / TILES_X_COUNT, TILE_SIZE / 2 );
				get_tile ( 1, i, display, image, TILE_SIZE / 2 + ( i * ( width - TILE_SIZE ) ) / TILES_X_COUNT, height - TILE_SIZE / 2 );
				i++;
			}

			// LEFT and RIGHT
			i = 0;

			while ( i < TILES_Y_COUNT ) {
				get_tile ( 2, i, display, image,         TILE_SIZE / 2, TILE_SIZE / 2 + ( i * ( height - TILE_SIZE ) ) / TILES_Y_COUNT );
				get_tile ( 3, i, display, image, width - TILE_SIZE / 2, TILE_SIZE / 2 + ( i * ( height - TILE_SIZE ) ) / TILES_Y_COUNT );
				i++;
			}

			usleep ( 1000000 / FPS );
		}
	}

	return 0;
}
