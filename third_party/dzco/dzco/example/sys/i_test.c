#include <dzco/dz_sys.h>

#define DT       0.01
#define STEP  1000
#define OMEGA    2.0

int main(int argc, char *argv[])
{
  double ref;
  dzSys s1, s2, s3;
  register int i;

  dzSysCreateI( &s1, 1.0, 0.0 ); dzSysInputPtr(&s1,0) = &ref;
  dzSysCreateI( &s2, 1.0, 0.1 ); dzSysInputPtr(&s2,0) = &ref;
  dzSysCreateI( &s3, 1.0, 0.5 ); dzSysInputPtr(&s3,0) = &ref;

  for( i=0; i<=STEP; i++ ){
    ref = sin( OMEGA*DT*i );
    dzSysUpdate( &s1, DT );
    dzSysUpdate( &s2, DT );
    dzSysUpdate( &s3, DT );
    printf( "%f %f %f %f\n", ref, dzSysOutputVal(&s1,0), dzSysOutputVal(&s2,0), dzSysOutputVal(&s3,0) );
  }
  dzSysDestroy( &s1 );
  dzSysDestroy( &s2 );
  dzSysDestroy( &s3 );
  return 0;
}
