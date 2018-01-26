#include <cure/cure_strlist.h>

int main(int argc, char *argv[])
{
  zStrList list;
  int i;

  zListInit( &list );
  for( i=0; i<argc; i++ )
    zStrListInsert( &list, argv[i] );
  zStrListFWrite( stdout, &list );
  zStrListDestroy( &list );
  return 0;
}
