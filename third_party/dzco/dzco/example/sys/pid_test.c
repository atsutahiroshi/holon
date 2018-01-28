#include <dzco/dz_sys.h>

#define DT     0.01
#define STEP 200

#define OMEGA ( 4*zPI/(STEP*DT) )

int main(int argc, char *argv[])
{
  double omega, u, yp, yi, yd, y;
  dzSys psys, isys, dsys, pidsys;
  int i, step;

  omega = argc > 1 ? atof( argv[1] ) : OMEGA;
  step = argc > 2 ? atoi( argv[2] ) : STEP;
  dzSysCreateP( &psys, 2 );
  dzSysInputPtr(&psys,0) = &u;
  dzSysCreateI( &isys, 2, 0 );
  dzSysInputPtr(&isys,0) = &u;
  dzSysCreateD( &dsys, 2, 0.0 );
  dzSysInputPtr(&dsys,0) = &u;
  dzSysCreatePID( &pidsys, 2, 2, 2, 0.0, 0.0 );
  dzSysInputPtr(&pidsys,0) = &u;
  for( i=0; i<=step; i++ ){
    u = sin( omega * DT * i );
    yp = zVecElem(dzSysUpdate(&psys,DT),0);
    yi = zVecElem(dzSysUpdate(&isys,DT),0);
    yd = zVecElem(dzSysUpdate(&dsys,DT),0);
    y  = zVecElem(dzSysUpdate(&pidsys,DT),0);
    printf( "%f %f %f %f %f %f\n", u, yp, yi, yd, y, yp+yi+yd );
  }
  dzSysDestroy( &psys );
  dzSysDestroy( &isys );
  dzSysDestroy( &dsys );
  dzSysDestroy( &pidsys );
  return 0;
}
