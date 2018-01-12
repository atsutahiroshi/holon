#include <cure/cure_list.h>

zListClass(int_list_t, int_list_cell_t, int);

void list_test(void)
{
  register int i;
  int_list_t list;
  int_list_cell_t *cp;

  printf( "+++ list test +++\n\n" );
  zListInit( &list );
  while( 1 ){
    if( !( cp = zAlloc( int_list_cell_t, 1 ) ) ){
      ZALLOCERROR( "int_list_cell" );
      exit( 1 );
    }
    printf( "input value(999 for terminate): " );
    scanf( "%d", &cp->data );
    if( cp->data == 999 ) break;

    if( !zListIsEmpty(&list) && cp->data < zListTail(&list)->data ){
      printf( "<add tail>\n" );
      zListInsertTail( &list, cp );
    } else{
      printf( "<add head>\n" );
      zListInsertHead( &list, cp );
    }
    zListWrite( (zList*)&list );
  }
  for( i=0; !zListIsEmpty(&list); i++ ){
    zListDeleteTail( &list, &cp );
    printf( "<%d> value = %d\n", i, cp->data );
    zFree( cp );
  }
  zListWrite( (zList*)&list );
  zListDestroy( int_list_cell_t, &list );
  printf( "list test terminated.\n\n" );
}

void stack_test(void)
{
  register int i;
  int_list_t list;
  int_list_cell_t *cp;

  printf( "+++ stack test +++\n\n" );
  zListInit( &list );
  while( 1 ){
    if( !( cp = zAlloc( int_list_cell_t, 1 ) ) ){
      ZALLOCERROR( "int_list_cell" );
      exit( 1 );
    }
    printf( "input value(999 for terminate): " );
    scanf( "%d", &cp->data );
    if( cp->data == 999 ) break;

    printf( "<push>\n" );
    zStackPush( &list, cp );
    zListWrite( (zList*)&list );
  }
  for( i=0; !zListIsEmpty(&list); i++ ){
    zStackPop( &list, &cp );
    printf( "<%d> value = %d\n", i, cp->data );
    zFree( cp );
  }
  zListWrite( (zList*)&list );
  zListDestroy( int_list_cell_t, &list );
  printf( "stack test terminated.\n\n" );
}

void queue_test(void)
{
  register int i;
  int_list_t list;
  int_list_cell_t *cp;

  printf( "+++ queue test +++\n\n" );
  zListInit( &list );
  while( 1 ){
    if( !( cp = zAlloc( int_list_cell_t, 1 ) ) ){
      ZALLOCERROR( "int_list_cell" );
      exit( 1 );
    }
    printf( "input value(999 for terminate): " );
    scanf( "%d", &cp->data );
    if( cp->data == 999 ) break;

    printf( "<enqueue>\n" );
    zQueueEnqueue( &list, cp );
    zListWrite( (zList*)&list );
  }
  for( i=0; !zListIsEmpty(&list); i++ ){
    zQueueDequeue( &list, &cp );
    printf( "<%d> value = %d\n", i, cp->data );
    zFree( cp );
  }
  zListWrite( (zList*)&list );
  zListDestroy( int_list_cell_t, &list );
  printf( "queue test terminated.\n" );
}

int main(void)
{
  list_test();
  stack_test();
  queue_test();
  return 0;
}
