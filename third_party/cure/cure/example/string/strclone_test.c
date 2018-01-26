#if 1
#define _GNU_SOURCE
#endif

#include <cure/cure_string.h>

int main(void)
{
  char *cp[2];

  cp[0] = zStrClone( "foo" );
  cp[1] = zStrClone( "baa" );

  printf( "%p:%s\n", cp[0], cp[0] );
  printf( "%p:%s\n", cp[1], cp[1] );

  free( cp[0] );
  free( cp[1] );

  cp[0] = zStrClone( NULL );
  printf( "%p:%s\n", cp[0], cp[0] );
  zFree( cp[0] );

  return 0;
}
