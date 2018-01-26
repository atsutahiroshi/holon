#include <cure/cure_string.h>

int main(void)
{
  char org[BUFSIZ], new[BUFSIZ];

  strcpy( org, "C:\\user\\foo\\baa.d\\woo\\yeah.sfx" );
  zGetBasename( org, new, BUFSIZ );
  printf( "org->%s\n", org );
  printf( "new->%s\n", new );

  strcpy( org, "C:\\user\\foo\\baa.d\\woo\\yeah" );
  zGetBasename( org, new, BUFSIZ );
  printf( "org->%s\n", org );
  printf( "new->%s\n", new );

  strcpy( org, "/home/user/foo/baa.d/woo/yeah.sfx.sfx" );
  zGetBasename( org, new, BUFSIZ );
  printf( "org->%s\n", org );
  printf( "new->%s\n", new );

  zGetBasename( org, org, BUFSIZ );
  printf( "ovw->%s\n", org );
  return 0;
}
