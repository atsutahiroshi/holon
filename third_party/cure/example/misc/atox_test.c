#include <cure/cure_misc.h>

void test(char *str)
{
  int val;

  val = atox( str );
  printf( "%s -> %x(%d)\n", str, val, val );
}

int main(void)
{
  test( "11" );
  test( "ff" );
  test( "1f" );
  test( "1g2h3i" );
  return 0;
}
