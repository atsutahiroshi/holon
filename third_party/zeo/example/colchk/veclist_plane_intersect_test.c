#include <zeo/zeo_col.h>
#include <zeo/zeo_bv.h>

#define N 100

int main(void)
{
  int i, n;
  zVec3DList vlist, ch;
  zVec3DListCell *vp;
  zVec3D v, ip[N], po;
  zPlane3D pl;
  FILE *fp;

  zRandInit();
  zListInit( &vlist );
  fp = fopen( "src", "w" );
  for( i=0; i<N; i++ ){
    zVec3DCreate( &v, zRandF(-10,10), zRandF(-10,10), 0 );
    zVec3DListInsert( &vlist, &v, true );
    zVec3DDataNLFWrite( fp, &v );
  }
  fclose( fp );

  zVec3DCreate( &po, -1, 0, 0 );
  zPlane3DCreate( &pl, &po, Z_UNITXVEC3D );

  n = zIntersectVecListPlane3D( &vlist, &pl, ip );
  fp = fopen( "ip", "w" );
  for( i=0; i<n; i++ )
    zVec3DDataNLFWrite( fp, &ip[i] );
  fclose( fp );

  zCH2DPL( &ch, &vlist );
  fp = fopen( "ch", "w" );
  zListForEach( &ch, vp )
    zVec3DDataNLFWrite( fp, vp->data );
  zVec3DDataNLFWrite( fp, zListTail(&ch)->data );
  fclose( fp );

  n = zIntersectVecListPlane3D( &ch, &pl, ip );
  fp = fopen( "ipch", "w" );
  for( i=0; i<n; i++ )
    zVec3DDataNLFWrite( fp, &ip[i] );
  fclose( fp );

  zVec3DListDestroy( &ch, false );
  zVec3DListDestroy( &vlist, true );
  return 0;
}
