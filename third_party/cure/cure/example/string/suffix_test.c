#include <cure/cure_string.h>

int main(void)
{
  char org[BUFSIZ], new[BUFSIZ];
  char *cp;

  strcpy( org, "foo" );
  zAddSuffix( org, "baa", new, BUFSIZ );
  zAddSuffix( new, "woo", new, BUFSIZ );
  zAddSuffix( new, "yeah", new, BUFSIZ );

  puts( org );
  puts( new );

  strcpy( org, "xxx.yyy.zzz" );
  printf( "%s\n", zGetSuffix( org ) );
  cp = zCutSuffix( org );
  printf( "%s . %s\n", org, cp );

  zReplaceSuffix( org, "www", new, BUFSIZ );
  puts( new );
  zReplaceSuffix( org, "www", new, BUFSIZ );
  puts( new );
  zReplaceSuffix( org, "zzz", new, BUFSIZ );
  puts( new );

  return 0;
}
