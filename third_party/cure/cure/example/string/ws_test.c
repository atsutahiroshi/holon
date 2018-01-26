#include <cure/cure_string.h>

int main(void)
{
  FILE *fp;
  char buf[BUFSIZ];

  fp = fopen( "ws.txt", "r" );
  while( 1 ){
    zFSkipWS( fp );
    if( feof( fp ) ) break;
    printf( "%s\n", zFToken( fp, buf, BUFSIZ ) );
  }
  fclose( fp );
  return 0;
}
