#include <zeo/zeo_bv.h>

#define N 1000
zVec3D v[N];
int main(void)
{
  register int i;
  zVec3DList ch;
  zVec3DListCell *vc;
  FILE *fp;

  zRandInit();
  fp = fopen( "src", "w" );
  for( i=0; i<N; i++ ){
#if 0
    zVec3DCreate( &v[i], zRandF(-10,10), zRandF(-10,10), 0 );
#else
    zVec3DCreatePolar( &v[i], zRandF(-10,10), 0.5*zPI, zRandF(-zPI,zPI) );
#endif
    zVec3DDataNLFWrite( fp, &v[i] );
  }
  fclose( fp );

  zCH2D( &ch, v, N );

  fp = fopen( "dest", "w" );
  zListForEach( &ch, vc )
    zVec3DDataNLFWrite( fp, vc->data );
  zVec3DDataNLFWrite( fp, zListTail(&ch)->data );
  fclose( fp );
  zVec3DListDestroy( &ch, false );
  return 0;
}
