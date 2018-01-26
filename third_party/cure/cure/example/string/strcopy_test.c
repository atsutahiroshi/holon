#include <cure/cure_string.h>

#define SIZE 10

int main(void)
{
  char str[SIZE];

  zStrCopy( str, "daa", SIZE ); printf( "%s\n", str );
  zStrCopy( str, "how are you?", SIZE ); printf( "%s\n", str );

  zStrCopy( str, "foo", SIZE ); printf( "%s\n", str );
  zStrCat( str, "baa", SIZE );  printf( "%s\n", str );
  zStrCat( str, "baa", SIZE );  printf( "%s\n", str );
  zStrCat( str, "baa", SIZE );  printf( "%s\n", str );
  return 0;
}
