#include <cure/cure_string.h>

int main(void)
{
  char tkn[BUFSIZ];
  char str[] = "token1\tTOKEN2 \"separated token 3\"    token4\n";
  char expstr[] = "a=1, aa=2.0, XYZ=-1.0e-16";
  FILE *fp;

  printf( "[file]\n" );
  fp = fopen( "test.txt", "r" );
  while( 1 ){
    if( !zFSkipDefaultComment( fp ) ||
        !zFToken( fp, tkn, BUFSIZ ) ) break;
    printf( "token-> %s\n", tkn );
  }
  fclose( fp );

  printf( "[string]\n" );
  while( zSToken( str, tkn, BUFSIZ ) )
    printf( "token-> %s\n", tkn );

  printf( "[expression string]\n" );
  while( zSToken( expstr, tkn, BUFSIZ ) )
    printf( "token-> %s\n", tkn );
  return 0;
}
