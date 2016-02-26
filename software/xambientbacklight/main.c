#include <stdio.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

int main ( int argc, char *argv[] )
{
	XColor c;
	Display *d = XOpenDisplay ( ( char * ) NULL );
	int x = 0; // Pixel x
	int y = 0; // Pixel y
	XImage *image;
	image = XGetImage ( d, RootWindow ( d, DefaultScreen ( d ) ), x, y, 3, 3, AllPlanes, XYPixmap );
	c.pixel = XGetPixel ( image, 2, 2 );
	XFree ( image );
	XQueryColor ( d, DefaultColormap ( d, DefaultScreen ( d ) ), &c );
	printf("%i %i %i\n", c.red/255, c.green/255, c.blue/255);
	return 0;
}
