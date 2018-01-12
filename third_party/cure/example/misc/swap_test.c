#include <cure/cure_misc.h>

int main(void)
{
  char ac = 'a', bc = 'b';
  int ai = 1, bi = 2;
  double ad = 1.0, bd = 2.0;
  double *ap, *bp;

  ap = &ad;
  bp = &bd;
  printf( ">>before swap<<\n" );
  printf( "ac:%c, bc:%c\n", ac, bc );
  printf( "ai:%d, bi:%d\n", ai, bi );
  printf( "ad:%g, bd:%g\n", ad, bd );
  printf( "ap:%p, bp:%p\n", ap, bp );

  zSwap( char, ac, bc );
  zSwap( int, ai, bi );
  zSwap( double, ad, bd );
  zSwap( double*, ap, bp );

  printf( ">>after swap<<\n" );
  printf( "ac:%c, bc:%c\n", ac, bc );
  printf( "ai:%d, bi:%d\n", ai, bi );
  printf( "ad:%g, bd:%g\n", ad, bd );
  printf( "ap:%p, bp:%p\n", ap, bp );
  return 0;
}
