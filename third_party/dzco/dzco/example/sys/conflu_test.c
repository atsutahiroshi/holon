#include <dzco/dz_sys.h>

int main(void)
{
  dzSys adder, subtr;
  double v1, v2, v3;

  zRandInit();

  dzSysCreateAdder( &adder, 3 );
  dzSysInputPtr(&adder,0) = &v1;
  dzSysInputPtr(&adder,1) = &v2;
  dzSysInputPtr(&adder,2) = &v3;

  dzSysCreateSubtr( &subtr, 3 );
  dzSysInputPtr(&subtr,0) = &v1;
  dzSysInputPtr(&subtr,1) = &v2;
  dzSysInputPtr(&subtr,2) = &v3;

  v1 = zRandF(-10,10);
  v2 = zRandF(-10,10);
  v3 = zRandF(-10,10);

  printf( "sum  = %f (%f)\n", zVecElem(dzSysUpdate(&adder,1),0), v1+v2+v3 );
  printf( "diff = %f (%f)\n", zVecElem(dzSysUpdate(&subtr,1),0), v1-v2-v3 );

  dzSysDestroy( &adder );
  dzSysDestroy( &subtr );
  return 0;
}
