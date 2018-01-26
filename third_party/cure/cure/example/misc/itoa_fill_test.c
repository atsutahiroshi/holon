#include <cure/cure_misc.h>

int main(void)
{
  char buf[BUFSIZ];
  register int i;

  for( i=1; i<=1000; i++ ){
    itoa_zerofill( i, 4, buf );
    printf( "%s\n", buf );
  }
  return 0;
}
