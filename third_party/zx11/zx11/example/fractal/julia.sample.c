/*
 * Julia series
 * (C)Copyright, Zhidao since 2000.
 *
 * 2000.10.27. Created.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <zx11/zxutil.h>

#define K 180
#define A 0.11031
#define B -0.670371
#define W 500

int main(int argc, char *argv[])
{
  zxWindow c;
  register int i, j;
  int rd;
  double x, y, zx, zy, zz, r, t, u, v;

  zxInit();
  zxWindowCreateAndOpen( &c, 100, 100, 700, 500 );
  zxWindowSetTitle( &c, "Julia series" );

  printf( "*** Julia series on X11 ***\n" );
  zxWindowClear( &c );

  for( i=1; i<=W; i++ ){
    for( j=1; j<=W; j++ ){
      rd = rand() % 100;
      zx = x - A;
      zy = y - B;
      zz = zy / zx;
      t = zx > 0 ? atan(zz) : ( zx < 0 ? 3.14159+atan(zz) : 1.57079 );
      t *= 0.5;
      r = sqrt( sqrt( zx*zx + zy*zy ) );
      if( rd > 50 ) r = -r;
      x = zx = r*cos(t);
      y = zy = r*sin(t);
      u = ( x + 4 ) * K - 400;
      v = ( 2 - y ) * K - 150;
      zxDrawPoint( &c, u, v );
    }
  }

  zxFlush();
  getchar();

  zxClose();
  return 1;
}
