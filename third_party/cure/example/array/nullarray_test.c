#include <cure/cure_array.h>

zArrayClass( int_array_t, int );

int main(void)
{
  int_array_t array;

  zArrayAlloc( &array, int, 0 );
  zArrayFree( &array );
  return 0;
}
