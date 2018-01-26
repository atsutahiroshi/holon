#include <cure/cure_rrtab.h>

void tab_write(zRRBool *tab)
{
  int i, j;

  printf( "  |" );
  for( i=0; i<tab->num; i++ )
    printf( "%2d ", i );
  printf( "\n" );
  printf( "---" );
  for( i=0; i<tab->num; i++ )
    printf( "---" );
  printf( "\n" );
  for( i=0; i<tab->num; i++ ){
    printf( "%02d|", i );
    for( j=0; j<tab->num; j++ ){
      printf( " %c ", zRRBoolCheck(tab,i,j) ? 'T' : 'F' );
    }
    printf( "\n" );
  }
}

#define N 5

int main(void)
{
  zRRBool tab;
  uint i, j;

  zRRBoolAlloc( &tab, N );
  printf( ">>initialize\n" );
  tab_write( &tab );

  do{
    printf( "i=" ); if( scanf( "%d", &i ) != 1 );
    printf( "j=" ); if( scanf( "%d", &j ) != 1 );
    if( zRRBoolCheck( &tab, i, j ) )
      zRRBoolUnmark( &tab, i, j );
    else
      zRRBoolMark( &tab, i, j );
    printf( ">>marked\n" );
    tab_write( &tab );
  } while( i != 0 || j != 0 );

  zRRBoolFree( &tab );
  return 0;
}
