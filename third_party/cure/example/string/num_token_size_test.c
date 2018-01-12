#include <cure/cure_string.h>

#define TKNSIZ 5

bool check_str(FILE *fp)
{
  char buf[BUFSIZ], tkn[TKNSIZ];

  if( !fgets( buf, BUFSIZ, fp ) ) return false;
  zCutNL( buf );
  printf( "%s\t->", buf );
  zSNumToken( buf, tkn, TKNSIZ );
  printf( "num:%s\tremainder:%s\n", tkn, buf );
  return true;
}

bool check_fp(FILE *fp)
{
  char tkn[BUFSIZ], buf[TKNSIZ];

  zFNumToken( fp, tkn, TKNSIZ );
  fgets( buf, BUFSIZ, fp );
  zCutNL( buf );
  printf( "num:%s\tremainder:%s\n", tkn, buf );
  return !zFSkipDelimiter( fp ) ? false : true;
}

int main(void)
{
  FILE *fp;

  fp = fopen( "num.txt", "r" );
  printf( "num. from str. test\n" );
  while( check_str( fp ) );

  rewind( fp );
  printf( "\nnum. from file test\n" );
  while( check_fp( fp ) );
  fclose( fp );
  return 0;
}
