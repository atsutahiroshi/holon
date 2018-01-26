#include <cure/cure_string.h>

int main(void)
{
  register int i;

  for( i=0; i<10; i++ ){
    zIndent( i );
    printf( "foo\n" );
  }
  return 0;
}
