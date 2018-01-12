#include <cure/cure_misc.h>

#define BUGGY

#define SIZE1 100
#define SIZE2 1000

int main(void)
{
  register int i;
  int *buf1, *buf2;

  buf1 = zAlloc( int, SIZE1 );
#ifdef BUGGY
  buf2 = zAlloc( int, SIZE2 ); /* dummy */
#endif
  printf( "start from %p\n", buf1 );
  for( i=0; i<SIZE1; i++ )
    printf( "%d ", ( buf1[i] = i ) );
  printf( "\n" );
#ifdef BUGGY
  buf1 = zRealloc( buf1, int, SIZE2 );
#endif
  printf( "start from %p <- notice!\n", buf1 );
  for( i=0; i<SIZE1; i++ )
    printf( "%d ", buf1[i] );

  printf( "\n" );
  for( i=0; i<SIZE2; i++ ) buf1[i] = i;

  return 0;
}
