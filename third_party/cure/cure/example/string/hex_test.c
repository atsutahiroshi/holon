#include <cure/cure_string.h>

void examine(char *str)
{
  printf( "%s -> %s\n", str, zBoolExpr( zStrIsHex( str ) ) );
}

int main(void)
{
  examine( "012345" );
  examine( "0123456789ab" );
  examine( "AbCdEf" );
  examine( "A b C d E f" );
  examine( "AbCdEfG" );
  examine( "hello world" );
  return 0;
}
