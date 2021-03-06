#include <unistd.h>
#include <sys/param.h>
#include <zximage.h>
#include <zxpm.h>

#define zxSleep(t) usleep( (t) * 1000000 / HZ )

void zxPixmapDrawWipeBoxIn(zxContents *xc, zxPM *pm, Pixmap bmap, int x, int y, int step)
{
  register int dw, dh, x1, x2, y1, y2, dx, dy;

  dx = step;
  dy = step * pm->height / pm->width;
  for( dw=pm->width, dh=pm->height,
       x1=0, x2=pm->width-dx, y1=0, y2=pm->height-dy;
       dw > 0 || dh > 0;
       x1+=dx, x2-=dx, y1+=dy, y2-=dy,
       dw-=2*dx, dh-=2*dy ){
    if( dw > 0 ){
      zxPMDrawCromachy( xc, pm, x1, y1, bmap, x1, y1, dw, dy, x+x1, y+y1 );
      zxPMDrawCromachy( xc, pm, x1, y2, bmap, x1, y2, dw, dy, x+x1, y+y2 );
    }
    if( dh > 0 ){
      zxPMDrawCromachy( xc, pm, x2, y1, bmap, x2, y1, dx, dh, x+x2, y+y1 );
      zxPMDrawCromachy( xc, pm, x1, y1, bmap, x1, y1, dx, dh, x+x1, y+y1 );
    }
    zxSleep( 1 );
  }
}

void zxPixmapDrawWipeBoxOut(zxContents *xc, zxPM *pm, Pixmap bmap, int x, int y, int step)
{
  register int dw, dh, x1, x2, x3, y1, y2, y3, dx, dy;

  dx = step;
  dy = step * pm->height / pm->width;
  for( dw=dx, dh=dy, x2=pm->width/2, x1=x2-dw, x3=x2,
       y2=pm->height/2, y1=y2-dh, y3=y2;
       x1 > 0 && y1 > 0;
       x1-=dx, x2+=dx, x3-=dx, y1-=dy, y2+=dy, y3-=dy,
       dw=x2-x1, dh=y2-y1 ){
    if( dw > 0 ){
      zxPMDrawCromachy( xc, pm, x1, y1, bmap, x1, y1, dw, dy, x+x1, y+y1 );
      zxPMDrawCromachy( xc, pm, x3, y2, bmap, x3, y2, dw, dy, x+x3, y+y2 );
    }
    if( dh > 0 ){
      zxPMDrawCromachy( xc, pm, x2, y1, bmap, x2, y1, dx, dh, x+x2, y+y1 );
      zxPMDrawCromachy( xc, pm, x1, y3, bmap, x1, y3, dx, dh, x+x1, y+y3 );
    }
    zxSleep( 1 );
  }
  zxPMDrawCromachyAll( xc, pm, bmap, 0, 0, x, y );
}

void zxPixmapDrawWipeBoxCWIn(zxContents *xc, zxPM *pm, Pixmap bmap, int x, int y, int step)
{
  register int dx, dy, x1, x2, y1, y2, sx, sy;
  ubyte flag = 0;

  x1=0, x2=pm->width-step, y1=0, y2=pm->height-step;
  dx=step, dy=0, sx=x1, sy=y1;
  while( 1 ){
    zxPMDrawCromachy( xc, pm, sx, sy, bmap, sx, sy, step, step, x+sx, y+sy );
    if( flag == 0xf ) break;
    sx += dx;
    sy += dy;
    if( sx > x2 ){
      sx = x2; dx = 0; dy = step;
      if( y1 == y2 ) flag |= 0x1;
      else if( ( y1 += step ) > y2 ) y1 = y2;
    } else
    if( sx < x1 ){
      sx = x1; dx = 0; dy =-step;
      if( y2 == y1 ) flag |= 0x2;
      else if( ( y2 -= step ) < y1 ) y2 = y1;
    } else
    if( sy > y2 ){
      sy = y2; dx =-step; dy = 0;
      if( x2 == x1 ) flag |= 0x4;
      else if( ( x2 -= step ) < x1 ) x2 = x1;
    } else
    if( sy < y1 ){
      sy = y1; dx = step; dy = 0;
      if( x1 == x2 ) flag |= 0x8;
      else if( ( x1 += step ) > x2 ) x1 = x2;
    }
    zxSleep( 1 );
  }
}

void zxPixmapDrawWipeBoxCCWIn(zxContents *xc, zxPM *pm, Pixmap bmap, int x, int y, int step)
{
  register int dx, dy, x1, x2, y1, y2, sx, sy;
  ubyte flag = 0;

  x1=0, x2=pm->width-step, y1=0, y2=pm->height-step;
  dx=-step, dy=0, sx=x2, sy=y1;
  while( 1 ){
    zxPMDrawCromachy( xc, pm, sx, sy, bmap, sx, sy, step, step, x+sx, y+sy );
    if( flag == 0xf ) break;
    sx += dx;
    sy += dy;
    if( sx > x2 ){
      sx = x2; dx = 0; dy =-step;
      if( y2 == y1 ) flag |= 0x1;
      else if( ( y2 -= step ) < y1 ) y2 = y1;
    } else
    if( sx < x1 ){
      sx = x1; dx = 0; dy = step;
      if( y1 == y2 ) flag |= 0x2;
      else if( ( y1 += step ) > y2 ) y1 = y2;
    } else
    if( sy > y2 ){
      sy = y2; dx = step; dy = 0;
      if( x1 == x2 ) flag |= 0x4;
      else if( ( x1 += step ) > x2 ) x1 = x2;
    } else
    if( sy < y1 ){
      sy = y1; dx =-step; dy = 0;
      if( x2 == x1 ) flag |= 0x8;
      else if( ( x2 -= step ) < x1 ) x2 = x1;
    }
    zxSleep( 1 );
  }
}

void zxPixmapDrawWipeBoxCWOut(zxContents *xc, zxPM *pm, Pixmap bmap, int x, int y, int step)
{
  register int dx, dy, x1, x2, y1, y2, sx, sy;
  int x_max, y_max;
  ubyte flag = 0;

  x2=pm->width/2, x1=x2-step, y2=pm->height/2, y1=y2-step;
  dx=step, dy=0, sx=x1, sy=y1;
  x_max = pm->width - step;
  y_max = pm->height - step;
  while( 1 ){
    zxPMDrawCromachy( xc, pm, sx, sy, bmap, sx, sy, step, step, x+sx, y+sy );
    if( flag == 0xf ) break;
    sx += dx;
    sy += dy;
    if( sx > x2 ){
      sx = x2; dx = 0; dy = step;
      if( y1 == 0 ) flag |= 0x1;
      if( ( y1 -= step ) < 0 ) y1 = 0;
    } else
    if( sx < x1 ){
      sx = x1; dx = 0; dy =-step;
      if( y2 == y_max ) flag |= 0x2;
      if( ( y2 += step ) > y_max ) y2 = y_max;
    } else
    if( sy > y2 ){
      sy = y2; dx =-step; dy = 0;
      if( x2 == x_max ) flag |= 0x4;
      if( ( x2 += step ) > x_max ) x2 = x_max;
    } else
    if( sy < y1 ){
      sy = y1; dx = step; dy = 0;
      if( x1 == 0 ) flag |= 0x8;
      if( ( x1 -= step ) < 0 ) x1 = 0;
    }
    zxSleep( 1 );
  }
}

void zxPixmapDrawWipeBoxCCWOut(zxContents *xc, zxPM *pm, Pixmap bmap, int x, int y, int step)
{
  register int dx, dy, x1, x2, y1, y2, sx, sy;
  int x_max, y_max;
  ubyte flag = 0;

  x2=pm->width/2, x1=x2-step, y2=pm->height/2, y1=y2-step;
  dx=-step, dy=0, sx=x2, sy=y1;
  x_max = pm->width - step;
  y_max = pm->height - step;
  while( 1 ){
    zxPMDrawCromachy( xc, pm, sx, sy, bmap, sx, sy, step, step, x+sx, y+sy );
    if( flag == 0xf ) break;
    sx += dx;
    sy += dy;
    if( sx > x2 ){
      sx = x2; dx = 0; dy =-step;
      if( y2 == y_max ) flag |= 0x1;
      if( ( y2 += step ) > y_max ) y2 = y_max;
    } else
    if( sx < x1 ){
      sx = x1; dx = 0; dy = step;
      if( y1 == 0 ) flag |= 0x2;
      if( ( y1 -= step ) < 0 ) y1 = 0;
    } else
    if( sy > y2 ){
      sy = y2; dx = step; dy = 0;
      if( x1 == 0 ) flag |= 0x4;
      if( ( x1 -= step ) < 0 ) x1 = 0;
    } else
    if( sy < y1 ){
      sy = y1; dx =-step; dy = 0;
      if( x2 == x_max ) flag |= 0x8;
      if( ( x2 += step ) > x_max ) x2 = x_max;
    }
    zxSleep( 1 );
  }
}

void back(zxContents *xc, Pixmap bmap, int w, int h)
{
  zxWindowClear( xc );
  if( bmap != None )
    zxPixmapDraw( xc, bmap, 0, 0, w, h, 10, 10 );
  zxDrawRect( xc, 9, 9, w+2, h+2 );
}

int main(int argc, char *argv[])
{
  zxContents xc;
  zxImage img, bg;
  zxPM pm;
  Pixmap bmap = None;

  zxInit();
  if( zxImageReadFile( &img, argv[1] ) == 0 ) exit( 1 );
  zxWindowCreateAndOpen( &xc, 0, 0, img.width+20, img.height+20 );
  zxImageToPM( &xc, &img, &pm );
  zxImageDestroy( &img );
  if( argc > 2 ){
    if( zxImageReadFile( &bg, argv[2] ) == 0 ) exit( 1 );
    bmap = zxImageCreatePixmap( &xc, &bg );
    zxImageDestroy( &bg );
  }
  zxSetColor( &xc, "red" );

  /* box wipe in */
  back( &xc, bmap, pm.width, pm.height );
  zxPixmapDrawWipeBoxIn( &xc, &pm, bmap, 10, 10, 18 );
  getchar();

  /* box wipe out */
  back( &xc, bmap, pm.width, pm.height );
  zxPixmapDrawWipeBoxOut( &xc, &pm, bmap, 10, 10, 18 );
  getchar();

  /* box wipe clockwise in */
  back( &xc, bmap, pm.width, pm.height );
  zxPixmapDrawWipeBoxCWIn( &xc, &pm, bmap, 10, 10, 42 );
  getchar();

  /* box wipe counterclockwise in */
  back( &xc, bmap, pm.width, pm.height );
  zxPixmapDrawWipeBoxCCWIn( &xc, &pm, bmap, 10, 10, 42 );
  getchar();

  /* box wipe clockwise out */
  back( &xc, bmap, pm.width, pm.height );
  zxPixmapDrawWipeBoxCWOut( &xc, &pm, bmap, 10, 10, 42 );
  getchar();

  /* box wipe counterclockwise out */
  back( &xc, bmap, pm.width, pm.height );
  zxPixmapDrawWipeBoxCCWOut( &xc, &pm, bmap, 10, 10, 42 );
  getchar();

  zxPixmapDestroy( bmap );
  zxPMDestroy( &pm );
  zxClose();

  return 0;
} 
