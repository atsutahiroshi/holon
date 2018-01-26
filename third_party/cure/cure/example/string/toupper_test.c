#include <cure/cure_string.h>

#define STR "sj;fKJ:AFLdDs;BD1Kfan"

int main(void)
{
  char str[BUFSIZ];

  printf( "orig ... %s\n", STR );
  zToUpper( STR, str );
  printf( "upper... %s\n", str );
  zToLower( str, str );
  printf( "lower... %s\n", str );
  return 0;
}
