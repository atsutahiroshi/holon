#include <cure/cure_index.h>

int main(void)
{
  zIndex idx;

  idx = zIndexCreateList( 5, 0, 3, 2, 4, 5 );
  zIndexWrite( idx );
  printf( "head: %d\n", zIndexHead(idx) );
  printf( "neck: %d\n", zIndexNeck(idx) );
  printf( "tail: %d\n", zIndexTail(idx) );
  zIndexDestroy( idx );
  return 0;
}
