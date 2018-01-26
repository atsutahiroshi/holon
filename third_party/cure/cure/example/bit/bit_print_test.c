#include <cure/cure_bit.h>
#include <cure/cure_string.h>

int main(void)
{
  ubyte val1, val2, val3;

  val1 = 0xa;
  val2 = 0xaf;
  val3 = val1 ^ val2;
  printf( "val1          = " ); printf_bit( val1, 8 ); zEndl();
  printf( "val2          = " ); printf_bit( val2, 8 ); zEndl();
  printf( "val1 XOR val2 = " ); printf_bit( val3, 8 ); zEndl();
  return 0;
}
