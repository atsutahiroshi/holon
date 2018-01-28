#include <zeo/zeo_vec3d.h>

zVec3D v[] = {
  { { 0, 0, 0 } },
  { { 1,-1, 2 } },
  { { 2,-2, 4 } },
  { { 3,-3, 6 } },
  { { 4,-4, 8 } },
  { { 5,-5,10 } },
};

void test(bool flag)
{
  register int n;
  zVec3DList list;

  n = sizeof(v) / sizeof(zVec3D);
  zVec3DListCreate( &list, v, n, flag );
  zVec3DListWrite( &list );
  zVec3DListDestroy( &list, flag );
}

void test_bad(void)
{
  register int n;
  zVec3DList list;

  n = sizeof(v) / sizeof(zVec3D);
  zVec3DListCreate( &list, v, n, false );
  zVec3DListWrite( &list );
  zVec3DListDestroy( &list, true );
}

int main(void)
{
  test( true );
  test( false );
  printf( "(The next test causes memory error.)\n" );
  test_bad();
  return 0;
}
