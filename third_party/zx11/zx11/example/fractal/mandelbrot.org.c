/*
 * Mandelbrot Series
 * (C)Copyright, Zhidao since 2000.
 *
 * 2001. 7.21. Created.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <zx11/zxutil.h>

#define WIDTH_X 100
#define WIDTH_Y 100

double ratio_x=1.0/WIDTH_X, ratio_y=1.0/WIDTH_Y;
int ox=WIDTH_X*2, oy=WIDTH_Y*1.5;
int wx=WIDTH_X*3, wy=WIDTH_Y*3;

zxWindow c;

char *color[] = {
  "green", "red", "magenta", "yellow", "blue",
  "cyan", "green", "blue", "yellow", "red", "magenta", "black"
};

#define COLOR_NUM 10

int MandelbrotPixel(int x, int y)
{
  double cx, cy, sx, sy, zx=0, zy=0;
  register int i;

  cx = ( x - ox ) * ratio_x;
  cy = ( y - oy ) * ratio_y;
  for( i=0; i<=COLOR_NUM; i++ ){
    sx = zx*zx;
    sy = zy*zy;
    if( ( sx + sy ) >= 4 ){
      zxSetColor( &c, color[i] );
      zxDrawPoint( &c, x, y );
      break;
    }
    zy = 2*zx*zy + cy;
    zx = sx - sy + cx;
  }
  return i;
}

void Mandelbrot(void)
{
  register int i, j;

  for( i=0; i<=wx; i++ )
    for( j=0; j<=wy; j++ )
      MandelbrotPixel( i, j );
}

int main(int argc, char *argv[])
{
  zxInit();
  zxWindowCreateAndOpen( &c, 100, 100, wx, wy );
  zxWindowSetTitle( &c, "Mandelbrot Series" );
  zxFlush();

  printf( "*** Mandelbrot Series on X11 ***\n" );
  zxWindowClear( &c );

  Mandelbrot();
  zxFlush();
  getchar();

  zxClose();
  return 1;
}
