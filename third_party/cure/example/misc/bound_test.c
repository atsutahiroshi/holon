#include <cure/cure_misc.h>
#include <cure/cure_rand.h>

int main(void)
{
  double x, b1, b2;

  zRandInit();
  x = zRandF( -10, 10 );
  b1 = zRandF( -10, 10 );
  b2 = zRandF( -10, 10 );
  printf( "%f vs [%f, %f] -> %f\n", x, b1, b2, zBound(x,b1,b2) );
  return 0;
}
