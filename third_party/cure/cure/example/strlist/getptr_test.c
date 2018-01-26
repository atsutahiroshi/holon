#include <cure/cure_strlist.h>

int main(int argc, char *argv[])
{
  zStrList list;
  char *str1, *str2, *str3;
  int i;

  zListInit( &list );
  for( i=0; i<argc; i++ )
    zStrListInsert( &list, argv[i] );
  zStrListFWrite( stdout, &list );
  zStrListGetPtr( &list, 3, &str1, &str2, &str3 );
  printf( "str1=%s\n", str1 );
  printf( "str2=%s\n", str2 );
  printf( "str3=%s\n", str3 );
  zStrListDestroy( &list );
  return 0;
}
