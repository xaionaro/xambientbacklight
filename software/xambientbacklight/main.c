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


void get_tile ( int edge_id, int led_id, Display *d, XImage *image, int x, int y )
{
	XColor c;
	c.pixel = XGetPixel ( image, x, y );
	XQueryColor ( d, DefaultColormap ( d, DefaultScreen ( d ) ), &c );
	printf ( "%i\t%i\t%i\t%i\t%i\t%i\t%i\n", edge_id, led_id, x, y, c.red/256, c.green/256, c.blue/256 );
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
