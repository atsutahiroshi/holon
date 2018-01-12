#include <cure/cure_ring.h>

#define N 12

zRingClass( int_ring_array_t, int );

void ira_write(int_ring_array_t *ira)
{
  register int i;

  printf( "head -> %d\n", ira->head );
  for( i=0; i<zRingNum(ira); i++ )
    printf( "[%d] %d\n", i, *zRingElem(ira,i) );
  printf( "\n" );
}

int main(void)
{
  int_ring_array_t ira;
  int i;

  zRingCreate( &ira, int, N );
  for( i=0; i<N; i++ )
    zRingSetElem( &ira, i, &i );
  ira_write( &ira );
  printf( ">>forward<<\n" );
  for( i=0; i<N; i++ ){
    zRingIncHead( &ira );
    ira_write( &ira );
    getchar();
  }
  printf( ">>backward<<\n" );
  for( i=0; i<N; i++ ){
    zRingDecHead( &ira );
    ira_write( &ira );
    getchar();
  }
  zRingDestroy( &ira );
  return 0;
}
