#include <zeo/zeo_vec3d.h>

#define N 20
zVec3D v[N];

void test_vert(zVec3DList *vl)
{
  register int i;

  zRandInit();
  for( i=0; i<N; i++ )
    zVec3DCreate( &v[i], zRandF(-1,1), zRandF(-1,1), zRandF(-1,1) );
  zVec3DListCreate( vl, v, N, false );
}

void destroy(zVec3DList *vl)
{
  zVec3DListDestroy( vl, false );
}

int cmp(void *a, void *b, void *dummy)
{
  zVec3D *v1, *v2;

  v1 = ((zVec3DListCell*)a)->data;
  v2 = ((zVec3DListCell*)b)->data;
  if( v1->e[zX] > v2->e[zX] ) return 1;
  if( v1->e[zX] < v2->e[zX] ) return -1;
  if( v1->e[zY] > v2->e[zY] ) return 1;
  if( v1->e[zY] < v2->e[zY] ) return -1;
  if( v1->e[zZ] > v2->e[zZ] ) return 1;
  if( v1->e[zZ] < v2->e[zZ] ) return -1;
  return 0;
}

int main(void)
{
  zVec3DList vl;

  test_vert( &vl );
  zVec3DListWrite( &vl );
  zVec3DListQuickSort( &vl, cmp, NULL );
  zVec3DListWrite( &vl );
  destroy( &vl );
  return 0;
}
