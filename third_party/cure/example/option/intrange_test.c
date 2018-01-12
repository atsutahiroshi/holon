#include <cure/cure_option.h>

void parse_int_range(char *expr)
{
  int from, to;

  zParseIntRange( expr, &from, &to );
  printf( "org:\"%s\" ->   %d - %d\n", expr, from, to );
}

int main(void)
{
  parse_int_range( "3" );
  parse_int_range( "3-7" );
  parse_int_range( "7-3" );
  parse_int_range( "-7" );
  parse_int_range( "3-" );
  parse_int_range( "-" );
  parse_int_range( "hoge" );
  parse_int_range( "hoge1-hoge2" );
  parse_int_range( "1hoge-2hoge" );
  return 0;
}
