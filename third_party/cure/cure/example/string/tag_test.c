#include <cure/cure_string.h>

int main(void)
{
  char buf[BUFSIZ], str[BUFSIZ];

  zExtractTag( "[tag]", buf );
  printf( "%s\n", buf );
  strcpy( str, "[tag]" );
  zExtractTag( str, buf );
  printf( "%s\n", buf );
  return 0;
}
