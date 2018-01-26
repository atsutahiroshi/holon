#include <cure/cure_bit.h>

int main(void)
{
  ulong val = 0x6ae9;

  printf( "\nbit reverse test\n" );
  printf_bit( val, 16 );
  printf( ": <-- original value\n" );
  printf_bit( bit_reverse(val,8), 16 );
  printf( ": <-- reverse(LSB only)\n" );
  printf_bit( bit_reverse(val,16), 16 );
  printf( ": <-- reverse\n" );
  return 0;
}
