#include <cure/cure_string.h>

#define TKNSIZ 5

int main(void)
{
  char tkn[TKNSIZ];
  char str[] = "token1\tTOKEN2 \"separated token 3\"    token4\n";
  FILE *fp;

  printf( "[file]\n" );
  fp = fopen( "test.txt", "r" );
  while( 1 ){
    if( !zFSkipDefaultComment( fp ) ||
        !zFToken( fp, tkn, TKNSIZ ) ) break;
    printf( "token-> %s\n", tkn );
  }
  fclose( fp );

  printf( "[string]\n" );
  while( zSToken( str, tkn, TKNSIZ ) )
    printf( "token-> %s\n", tkn );
  return 0;
}
