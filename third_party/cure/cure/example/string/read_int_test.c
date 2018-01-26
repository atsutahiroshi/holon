#include <cure/cure_string.h>

int main(void)
{
  char buf[BUFSIZ];

  printf( "enter a string: " );
  fgets( buf, BUFSIZ, stdin );
  while( buf[0] )
    printf( "%d\n", zSInt( buf ) );

  printf( "enter 3 numbers: " );
  printf( "val[1]=%d, val[2]=%d, val[3]=%d\n", zFInt(stdin), zFInt(stdin), zFInt(stdin) );
  printf( "(probably the result is defferent from that expected...)\n" );
  return 0;
}
