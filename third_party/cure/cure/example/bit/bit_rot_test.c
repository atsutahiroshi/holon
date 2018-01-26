#include <cure/cure_bit.h>

int main(void)
{
  ulong val = 0x6ae9;

  printf( "\nbit rotate test\n" );
  printf( "   " );
  printf_bit( val, 16 );
  printf( "     : <-- original value\n" );

  printf( "        " );
  printf_bit( bit_rotate(val,8,5), 16 );
  printf( ": <-- rotate 5 bit(LSB only)\n" );
  printf_bit( bit_rotate(val,8,-3), 16 );
  printf( "        : <-- rotate -3 bit(LSB only)\n" );
  printf( "        " );
  printf_bit( bit_rotate(val,16,5), 16 );
  printf( ": <-- rotate 5 bit\n" );
  printf_bit( bit_rotate(val,16,-3), 16 );
  printf( "        : <-- rotate -3 bit\n" );
  return 0;
}
