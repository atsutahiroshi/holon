#include <cure/cure_index.h>

int main(void)
{
  zIndex idx;

  idx = zIndexCreate( 5 );
  printf( "<original index>\n" );
  zIndexWrite( idx );
  printf( "(swap 2nd and 4th elem...)\n" );
  zIndexSwap( idx, 1, 3 );
  printf( "<modified index>\n" );
  zIndexWrite( idx );
  zIndexDestroy( idx );
  return 0;
}
