#include <cure/cure_string.h>

int main(void)
{
  FILE *fp;
  char buf[BUFSIZ];

  fp = fopen( "test.txt", "r" );
  while( 1 ){
    if( !zFSkipDefaultComment( fp ) ||
        !fgets( buf, BUFSIZ, fp ) ) break;
    printf( "buf: %s", buf );
  }
  fclose( fp );
  return 0;
}
