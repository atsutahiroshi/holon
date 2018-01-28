#include <dzco/dz_sys.h>

#define N 100

int main(void)
{
  double v;
  dzSys sys;
  int i;

  zRandInit();
  dzSysCreateLimit( &sys, 1, -1 );
  dzSysInputPtr(&sys,0) = &v;

  for( i=0; i<N; i++ ){
    v = zRandF(-2,2);
    printf( "%f %f\n", v, zVecElem(dzSysUpdate(&sys,1),0) );
  }

  dzSysDestroy( &sys );
  return 0;
}
