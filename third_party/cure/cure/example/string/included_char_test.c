#include <cure/cure_string.h>

void isincludechar_test(char c, char *str)
{
  zIsIncludedChar( 'a', "bcdefg" ) ?
    printf( "%c is included in %s.\n", c, str ) :
    printf( "%c is not included in %s.\n", c, str );
}

int main(void)
{
  isincludechar_test( 'a', "bcdefg" );
  isincludechar_test( 'a', "bcdefga" );
  return 0;
}
