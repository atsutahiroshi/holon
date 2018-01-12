#include <cure/cure_index.h>

int main(void)
{
  zIndex idx;

  idx = zIndexCreate( 10 );
  printf( "<original index>\n" );
  zIndexWrite( idx );
  printf( "(move 3rd to 7th...)\n" );
  zIndexMove( idx, 2, 6 );
  printf( "<modified index>\n" );
  zIndexWrite( idx );
  printf( "(move 8th to 5th...)\n" );
  zIndexMove( idx, 7, 4 );
  printf( "<modified index>\n" );
  zIndexWrite( idx );
  zIndexDestroy( idx );
  return 0;
}
