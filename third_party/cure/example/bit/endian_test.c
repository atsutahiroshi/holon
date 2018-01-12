#include <cure/cure_misc.h>

void longlongtest(void)
{
  unsigned long long org, cnv;

  org = 0x0123456789abcdefLL;
  cnv = endian_reverse64( org );
  printf( "%16llX -> %16llX\n", org, cnv );
}

void longtest(void)
{
  ulong org, cnv;

  org = 0x01234567L;
  cnv = endian_reverse32( org );
  printf( "%8lX -> %8lX\n", org, cnv );
}

void wordtest(void)
{
  uword org, cnv;

  org = 0x0123L;
  cnv = endian_reverse16( org );
  printf( "%4hX -> %4hX\n", org, cnv );
}

int main(void)
{
  longlongtest();
  longtest();
  wordtest();
  return 0;
}
