#include <zeo/zeo.h>

#define N 100
zVec3D v[N];

void veclist_create_rand(zVec3DList *pl)
{
  register int i;
  FILE *fp;

  zListInit( pl );
  fp = fopen( "src", "w" );
  for( i=0; i<N; i++ ){
    zVec3DCreate( &v[i], zRandF(-5,5), zRandF(-5,5), zRandF(-5,5) );
    zVec3DListInsert( pl, &v[i], false );
    zVec3DDataNLFWrite( fp, &v[i] );
  }
  fclose( fp );
}

void verify(zAABox3D *bb, zVec3DListCell **vp)
{
  register int i;
  bool ret;
  zBox3D box;
  zVec3D vert;
  FILE *fp;

  eprintf( "++verify+++\n" );
  zAABox3DToBox3D( bb, &box );
  for( i=0; i<N; i++ ){
    if( ( ret = zBox3DPointIsInside(&box,&v[i],true) ) == false ){
      zVec3DDataNLFWrite( stderr, &v[i] );
      eprintf( "D=%.15g\n", zBox3DPointDist(&box,&v[i]) );
    }
  }
  /* points on the walls */
  fp = fopen( "bd", "w" );
  for( i=0; i<6; i++ )
    zVec3DDataNLFWrite( fp, vp[i]->data );
  fclose( fp );
  /* AABB */
  fp = fopen( "aabb", "w" );
  for( i=0; i<4; i++ )
    zVec3DDataNLFWrite( fp, zBox3DVert(&box,i,&vert) );
  zVec3DDataNLFWrite( fp, zBox3DVert(&box,0,&vert) );
  fprintf( fp, "\n" );
  for( i=4; i<8; i++ )
    zVec3DDataNLFWrite( fp, zBox3DVert(&box,i,&vert) );
  zVec3DDataNLFWrite( fp, zBox3DVert(&box,4,&vert) );
  fprintf( fp, "\n" );
  for( i=0; i<4; i++ ){
    zVec3DDataNLFWrite( fp, zBox3DVert(&box,i,&vert) );
    zVec3DDataNLFWrite( fp, zBox3DVert(&box,i+4,&vert) );
    fprintf( fp, "\n" );
  }
  fclose( fp );
}

int main(void)
{
  zAABox3D bb;
  zVec3DListCell *vp[6];
  zVec3DList pl;

  zRandInit();
  veclist_create_rand( &pl );
  zAABBPL( &bb, &pl, vp );
  verify( &bb, vp );
  zVec3DListDestroy( &pl, false );
  return 0;
}
