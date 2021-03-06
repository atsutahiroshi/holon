#include <zeo/zeo.h>

#if 1
#define MODEL "../model/scc.z3d"
#else
#define MODEL "../model/cyl.z3d"
#endif

int main(void)
{
  zMShape3D m;

  zMShape3DReadFile( &m, MODEL );
  zMShape3DWrite( &m );
  zMShape3DDestroy( &m );
  return 0;
}
