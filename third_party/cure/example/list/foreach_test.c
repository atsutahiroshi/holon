#include <cure/cure_list.h>

zListClass(int_list_t, int_list_cell_t, int);

void create_list(int_list_t *list)
{
  int_list_cell_t *cp;

  while( 1 ){
    if( !( cp = zAlloc( int_list_cell_t, 1 ) ) ){
      ZALLOCERROR( "int_list_cell" );
      exit( 1 );
    }
    printf( "input value(999 for terminate): " );
    scanf( "%d", &cp->data );
    if( cp->data == 999 ) break;
    zListInsertTail( list, cp );
  }
}

void show_list(int_list_t *list)
{
  int_list_cell_t *cp;
  register int i = 0;

  zListForEach( list, cp )
    printf( "%d:%d\n", i++, cp->data );
}

int main(void)
{
  int_list_t list;

  zListInit( &list );
  create_list( &list );
  show_list( &list );
  zListDestroy( int_list_cell_t, &list );
  return 0;
}
