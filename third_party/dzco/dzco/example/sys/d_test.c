#include <dzco/dz_sys.h>

#define DT     0.01
#define STEP 1000
#define OMEGA 2.0

int main(int argc, char *argv[])
{
  register int i;
  double ref;
  dzSys d1, d2, d3;

  dzSysCreateD( &d1, 1.0, 0.0 ); dzSysInputPtr(&d1,0) = &ref;
  dzSysCreateD( &d2, 1.0, 0.1 ); dzSysInputPtr(&d2,0) = &ref;
  dzSysCreateD( &d3, 1.0, 0.5 ); dzSysInputPtr(&d3,0) = &ref;

  for( i=0; i<=STEP; i++ ){
    ref = sin( OMEGA*DT*i );
    printf( "%f %f %f %f\n", ref, zVecElem(dzSysUpdate(&d1,DT),0), zVecElem(dzSysUpdate(&d2,DT),0), zVecElem(dzSysUpdate(&d3,DT),0) );
  }
  dzSysDestroy( &d1 );
  dzSysDestroy( &d2 );
  dzSysDestroy( &d3 );
  return 0;
}
